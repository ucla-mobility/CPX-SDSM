#!/usr/bin/env python3
# mypy: ignore-errors
"""
Offline replay of sdsm_trust_perception's TrustEngine against a completed
run's rosBridgeMode="log" event file -- no rclpy, no live ROS graph, no UDP.

WHY THIS WORKS WITHOUT ROS
----------------------------
RosSDSMApp.cc's sendToRos() builds one JSON line (via buildSdsmJson()) per
TX/RX event and only branches on where it goes: BridgeMode::Log appends it to
a shared ofstream, BridgeMode::Live sends the same line over UDP. The file a
"log"-mode run writes (results/<logPrefix>-r<runNumber>-ros-events.jsonl) is
therefore byte-identical JSON to what veins_ros_bridge's live UDP path
decodes -- confirmed by reading RosSDSMApp.cc directly, not assumed. And
sdsm_trust_perception.trust_node's actual per-frame logic (_JudgeState, the
sim_time bucket-close-on-advance loop, TrustEngine.process_frame) barely
touches rclpy.Node -- only self.get_logger()/self.pub.publish() at the edges,
which this script replaces with plain prints and CSV rows. So the SAME
decode -> bucket -> TrustEngine path trust_node.py drives live, this script
drives from a file, at full CPU speed instead of real time.

This mirrors trust_node.py's _EgoState/_JudgeState/on_event/_flush closely
on purpose: the two should keep behaving identically as the live node
evolves. If trust_node.py's bucketing logic changes, update this to match.

THE SYNTHETIC CLOCK (why dt isn't real wall-clock here)
-----------------------------------------------------------
TrustEngine.process_frame() reads dt = now() - last_now() to drive
persistence/decay math (see TrustEngine.__init__'s `now` param and
_persistence_weight) -- in trust_node.py's live path this naturally lands
close to flush_interval_s because that's how often real flushes actually
fire in wall time. A batch replay processes an entire run in a few seconds
of real CPU time, so real wall-clock dt would be ~0 -- nothing like what a
live run produced and not what the persistence/decay math is calibrated
against. Instead this script hands TrustEngine a synthetic clock that
advances by exactly flush_interval_s each time it's called (once per
process_frame(), i.e. once per closed bucket): dt always equals
flush_interval_s, matching a live run's nominal cadence exactly, regardless
of how fast this script actually runs.

Usage:
  python analysis/replay_trust_verdicts.py results/Periodic/seed0/Periodic-r0-ros-events.jsonl
  python analysis/replay_trust_verdicts.py results/Periodic/seed0 --prefix Periodic-r0

Requires ros2_ws built and its overlay sourced first:
  cd ros2_ws && colcon build --symlink-install && source install/setup.bash

Output: <input-stem>-trust-verdicts.csv, one row per (judge, sender, flush),
matching sdsm_trust_interfaces/TrustVerdict.msg's fields. trusted=False rows
are what got filtered -- that count (or the weighted equivalent) is the
"how much does it filter out" number.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path
from typing import Optional

try:
    import numpy as np
except ImportError:
    print("ERROR: numpy not found. Activate the same Python env ros2_ws was built with.",
          file=sys.stderr)
    raise

try:
    from sdsm_trust_perception.global_trust_perception.pipeline import sdsm_codec as codec
    from sdsm_trust_perception.global_trust_perception.pipeline.caution_map import (
        CautionMap,
        Sightings,
        conflicts,
    )
    from sdsm_trust_perception.global_trust_perception.pipeline.persistent_reputation_tracker import (
        BATCH_SIZE,
        PersistentReputationTracker,
    )
    from sdsm_trust_perception.global_trust_perception.pipeline.trustworthy_perception import (
        FrameStats,
        OPPORTUNITY_RANGE_M,
        TrustEngine,
        VERDICT_DEFERRED,
        VERDICT_MATCHED,
        VERDICT_UNCORROBORATED,
    )
    from sdsm_trust_perception.global_trust_perception.trust_calculations.consistency import T_DEADLINE_S
    from sdsm_trust_perception.global_trust_perception.trust_calculations.reputation import HIGH_TRUST
    from sdsm_trust_perception.global_trust_perception.trust_calculations.consistency_checks.kinematic_checks import TrackData
    from sdsm_trust_perception.global_trust_perception.tracking.SORT.modified_sort_centroid import Sort
except ImportError as e:
    print(
        "ERROR: sdsm_trust_perception not importable.\n"
        "Build and source the ROS 2 overlay first:\n"
        "  cd ros2_ws && colcon build --symlink-install && source install/setup.bash\n"
        f"(underlying error: {e})",
        file=sys.stderr,
    )
    sys.exit(1)

_VERDICT_NAME = {
    VERDICT_MATCHED: "MATCHED",
    VERDICT_DEFERRED: "DEFERRED",
    VERDICT_UNCORROBORATED: "UNCORROBORATED",
}

DEFAULT_FLUSH_INTERVAL_S = 0.5
_NODE_RE = re.compile(r'\{"event":"(?:TX|RX)","node":(\d+)')
NEAR_RANGE_M = 150.0
_CSV_FIELDS = [
    "judge_node", "sender_node", "msg_cnt", "sim_time",
    "trusted", "r_old", "r_new", "tau", "n_total", "correct", "incorrect",
    "n_objects", "n_matched", "n_deferred", "n_uncorroborated",
    "n_admitted", "n_phantom", "n_phantom_admitted", "false_freq",
    "n_near", "n_near_admitted",
    "n_hidden", "n_hidden_solo", "n_hidden_admitted", "n_hidden_solo_admitted",
    "n_unverified", "n_near_unverified", "n_hidden_unverified", "n_hidden_solo_unverified",
    "n_phantom_unverified", "n_strikes",
]
# One row per (judge, flush) when --caution-map is on: the judge's caution layer, scored against
# ground truth (phantom / hidden-real ids). "hz" (hazard) = a reported phantom / hidden object the
# judge's own path would actually hit within the look-ahead (plain footprint, no caution margin).
_CAUTION_FIELDS = [
    "judge_node", "sim_time", "advisory", "n_confirmed", "n_caution",
    "n_caution_phantom", "n_caution_hidden", "n_caution_other",
    "n_crit_caution", "n_crit_caution_phantom", "n_crit_caution_hidden", "n_crit_caution_other",
    "n_crit_confirmed", "n_crit_caution_2s", "max_criticality",
    "hz_hidden", "hz_hidden_confirmed", "hz_hidden_caution", "hz_hidden_none",
    "hz_phantom", "hz_phantom_confirmed", "hz_phantom_caution", "hz_phantom_none",
]


class _FakeClock:
    """Advances by exactly `step` seconds each call -- see module docstring."""

    def __init__(self, step: float):
        self._t = 0.0
        self._step = step
        self._first = True

    def __call__(self) -> float:
        if self._first:
            self._first = False
            return self._t
        self._t += self._step
        return self._t


class _Event:
    """Plain stand-in for a ReceivedSdsm message -- same field names/meanings
    (see ReceivedSdsm.msg), built from one decoded JSONL line. No rosidl
    message needed here since nothing publishes/subscribes over ROS."""

    __slots__ = ("is_rx", "node", "sim_time", "latency", "sender_node", "sdsm", "local")

    def __init__(self, is_rx, node, sim_time, latency, sender_node, sdsm, local=None):
        self.local = local
        self.is_rx = is_rx
        self.node = node
        self.sim_time = sim_time
        self.latency = latency
        self.sender_node = sender_node
        self.sdsm = sdsm


def _event_from_line(line: str) -> Optional[_Event]:
    line = line.strip()
    if not line:
        return None
    d = json.loads(line)
    event = d["event"]
    if event not in ("TX", "RX"):
        return None
    is_rx = event == "RX"
    node = int(d["node"])
    sdsm = codec.sdsm_from_dict(d["sdsm"])
    if is_rx:
        latency = float(d.get("latency", 0.0))
        sender_node = int(d.get("sender", sdsm.source_id[0]))
    else:
        latency = 0.0
        sender_node = node
    return _Event(is_rx, node, float(d["time"]), latency, sender_node, sdsm,
                  d.get("local") if not is_rx else None)


class _EgoState:
    """Mirrors trust_node.py's _EgoState exactly -- see that class's
    docstring for why a judge keeps its last TX between flushes."""

    def __init__(self):
        self.positions: list = []
        self.dims: list = []
        self.headings: list = []
        self.scores: list = []
        self.labels: list = []
        self.equipment: int = 0
        self.object_ids: list = []   # evaluation only
        self._raw_positions: list = []
        self._velocities: list = []
        self._send_time: float = 0.0
        self._last_xy = None
        self._last_t = None
        self._self_heading: float = 360.0
        self._self_vel = (0.0, 0.0)

    def update_from(self, msg, send_time: float, local=None) -> None:
        if local:
            # The vehicle's FULL local view (not the broadcast list cut at the object cap).
            raw = [(float(o[1]), float(o[2]), 0.0) for o in local]
            vel = [(0.0, 0.0) if o[4] <= -900 else (float(o[3]) * math.cos(o[4]), float(o[3]) * math.sin(o[4]))
                   for o in local]
            self.dims = [codec.SELF_DIMS_M[:2] + (0.0,) for _ in local]
            self.headings = [360.0 if o[4] <= -900 else (90.0 - math.degrees(o[4])) % 360.0 for o in local]
            self.scores = [1.0] * len(local)
            self.labels = [codec.OBJ_TYPE_VEHICLE] * len(local)
            self.object_ids = [int(o[0]) for o in local]
        else:
            raw = codec.get_global_positions_of(msg)
            vel = codec.get_velocities_of(msg)
            self.dims = codec.get_dims_of(msg)
            self.headings = codec.get_headings_of(msg)
            self.scores = codec.get_local_scores_of(msg)
            self.labels = codec.get_labels_of(msg)
            self.object_ids = codec.get_object_ids_of(msg)
        self.equipment = codec.get_equipment_type_of(msg)

        # The judge's own vehicle is ego-known too. Its velocity and heading
        # come from its own motion between TXs (the wire carries none for the
        # sender); heading is kept while stationary.
        x, y, _ = codec.ref_pos_of(msg)
        if self._last_xy is not None and send_time > self._last_t:
            dx, dy = x - self._last_xy[0], y - self._last_xy[1]
            dt = send_time - self._last_t
            self._self_vel = (dx / dt, dy / dt)
            if math.hypot(dx, dy) > 0.1:
                self._self_heading = codec.bearing_deg(dx, dy)
        self._last_xy = (x, y)
        self._last_t = send_time
        me = codec.self_object_of(msg, self._self_heading)
        self._raw_positions = raw + [me['position']]
        self._velocities = list(vel) + [self._self_vel]
        self.dims.append(me['dims'])
        self.headings.append(me['heading'])
        self.scores.append(me['score'])
        self.labels.append(me['label'])
        self._send_time = send_time
        self.align_to(send_time)

    def align_to(self, t_ref: float) -> None:
        """Dead-reckon the held snapshot to the bucket's common instant."""
        self.positions = codec.advance_positions(
            self._raw_positions, self._velocities, t_ref - self._send_time)


class _JudgeState:
    """Mirrors trust_node.py's _JudgeState, with a synthetic clock (see
    module docstring) in place of the ROS wall clock."""

    def __init__(self, judge_id: int, db_path: str, flush_hz: float, deadline_s: float,
                 high_trust: float, admission: str, persist: int, opp_range: float,
                 extra: Optional[dict] = None, caution: Optional[CautionMap] = None):
        self.judge_id = judge_id
        self.caution = caution
        self.reputation_db = PersistentReputationTracker(db_path)
        self.engine = TrustEngine(self.reputation_db, flush_hz=flush_hz,
                                  deadline_s=deadline_s, now=_FakeClock(1.0 / flush_hz),
                                  high_trust=high_trust, admission=admission,
                                  contradiction_persist=persist, opportunity_range_m=opp_range,
                                  **(extra or {}))
        self.flush_dt = 1.0 / flush_hz
        self.trackers: dict[int, Sort] = {}
        self.ego = _EgoState()
        self.current_bucket: Optional[int] = None
        self.buffer: list = []
        self.frame_count = 0
        self.diag_late = 0

    def sort_for(self, sender_node: int) -> Sort:
        trk = self.trackers.get(sender_node)
        if trk is None:
            trk = Sort(dt=self.flush_dt)
            self.trackers[sender_node] = trk
        return trk

    def close(self):
        self.reputation_db.close()


class TrustReplayer:
    """Drives _JudgeState the same way trust_node.py's on_event/_flush do,
    reading events from an iterable instead of a ROS subscription."""

    def __init__(self, db_dir: Path, flush_interval_s: float = DEFAULT_FLUSH_INTERVAL_S,
                 deferred_deadline_s: float = T_DEADLINE_S, verbose: bool = False,
                 high_trust: float = HIGH_TRUST, admission: str = 'tiered',
                 persist: int = 1, opp_range: float = OPPORTUNITY_RANGE_M,
                 extra: Optional[dict] = None):
        extra = dict(extra or {})
        self.ignore_local = bool(extra.pop('ignore_local', False))
        self.caution_on = bool(extra.pop('caution_map', False))
        self.caution_margin = extra.pop('caution_margin', None)
        self.caution_rows: list[dict] = []
        self.extra = extra
        self.high_trust = high_trust
        self.admission = admission
        self.persist = persist
        self.opp_range = opp_range
        self.flush_interval_s = flush_interval_s
        self.deferred_deadline_s = deferred_deadline_s
        self.db_dir = db_dir
        self.db_dir.mkdir(parents=True, exist_ok=True)
        self.verbose = verbose
        self._judges: dict[int, _JudgeState] = {}
        self.rows: list[dict] = []

    def _judge(self, judge_id: int) -> _JudgeState:
        js = self._judges.get(judge_id)
        if js is None:
            db_path = str(self.db_dir / f"historical_reputations_{judge_id}.db")
            js = _JudgeState(judge_id, db_path, flush_hz=1.0 / self.flush_interval_s,
                             deadline_s=self.deferred_deadline_s, high_trust=self.high_trust,
                             admission=self.admission, persist=self.persist,
                             opp_range=self.opp_range, extra=self.extra,
                             caution=(CautionMap(**({} if self.caution_margin is None
                                                    else {'margin_m': self.caution_margin}))
                                      if self.caution_on else None))
            self._judges[judge_id] = js
        return js

    def on_event(self, event: _Event) -> None:
        judge_id = codec.envelope_receiver(event)
        js = self._judge(judge_id)
        bucket_idx = int(event.sim_time // self.flush_interval_s)

        if js.current_bucket is None:
            js.current_bucket = bucket_idx
        elif bucket_idx < js.current_bucket:
            js.diag_late += 1
            return
        elif bucket_idx > js.current_bucket:
            self._flush(js)
            js.current_bucket = bucket_idx
            js.buffer = []

        js.buffer.append(event)

    def _flush(self, js: _JudgeState) -> None:
        if not js.buffer:
            return
        js.frame_count += 1

        positions_by_agent, dims_by_agent, headings_by_agent = {}, {}, {}
        labels_by_agent, object_ids_by_agent, equipment_by_agent = {}, {}, {}
        source_id_map, ref_pos_by_agent, tracks_by_agent = {}, {}, {}
        raw_msg_by_agent, sim_time_by_agent = {}, {}
        velocities_by_agent = {}

        rx_by_sender: dict[int, _Event] = {}
        tx_event: Optional[_Event] = None
        for event in js.buffer:
            if event.is_rx:
                rx_by_sender[int(event.sender_node)] = event
            else:
                tx_event = event

        # Every detection is dead-reckoned to this one instant (the bucket's
        # end) so views taken up to a bucket apart can be clustered spatially.
        t_ref = (js.current_bucket + 1) * self.flush_interval_s
        if tx_event is not None:
            js.ego.update_from(tx_event.sdsm, tx_event.sim_time,
                               None if self.ignore_local else tx_event.local)
        js.ego.align_to(t_ref)

        for sender_node, event in rx_by_sender.items():
            msg = event.sdsm
            n = codec.num_detections_of(msg)
            positions = codec.get_global_positions_of(msg)
            dims = codec.get_dims_of(msg)
            headings = codec.get_headings_of(msg)
            velocities = codec.get_velocities_of(msg)
            positions = codec.advance_positions(
                positions, velocities, t_ref - (event.sim_time - event.latency))

            xy = (np.array([(p[0], p[1]) for p in positions], dtype=float)
                  if n else np.empty((0, 2)))
            vel = np.array(velocities, dtype=float) if n else None
            dim_arr = np.array(dims, dtype=float) if n else None
            tracks = js.sort_for(sender_node).update(xy, vel, dim_arr)

            positions_by_agent[sender_node] = positions
            velocities_by_agent[sender_node] = velocities
            dims_by_agent[sender_node] = dims
            headings_by_agent[sender_node] = headings
            labels_by_agent[sender_node] = codec.get_labels_of(msg)
            object_ids_by_agent[sender_node] = codec.get_object_ids_of(msg)
            equipment_by_agent[sender_node] = codec.get_equipment_type_of(msg)
            source_id_map[sender_node] = codec.sender_id_tuple(msg)
            raw_msg_by_agent[sender_node] = msg
            sim_time_by_agent[sender_node] = event.sim_time

            ref_x, ref_y, _ = codec.ref_pos_of(msg)
            ref_pos_by_agent[sender_node] = (ref_x, ref_y, event.sim_time, event.latency)

            tracks_by_agent[sender_node] = TrackData(
                track_ids=[int(t.id) for t in tracks],
                kalman_x=[float(t.kf.x[0, 0]) for t in tracks],
                kalman_y=[float(t.kf.x[1, 0]) for t in tracks],
                kalman_vx=[float(t.kf.x[4, 0]) for t in tracks],
                kalman_vy=[float(t.kf.x[5, 0]) for t in tracks],
            )

        try:
            stats = js.engine.process_frame(
                positions_by_agent, js.ego.positions,
                dims_by_agent, js.ego.dims,
                source_id_map,
                tracks_by_agent=tracks_by_agent if tracks_by_agent else None,
                ref_pos_by_agent=ref_pos_by_agent if ref_pos_by_agent else None,
                scores_by_agent=None,
                ego_scores=None,
                headings_by_agent=headings_by_agent if headings_by_agent else None,
                ego_headings=js.ego.headings if js.ego.headings else None,
                equipment_by_agent=equipment_by_agent if equipment_by_agent else None,
                ego_equipment=js.ego.equipment,
                classes_by_agent=labels_by_agent if labels_by_agent else None,
                ego_classes=js.ego.labels if js.ego.labels else None,
                object_ids_by_agent=object_ids_by_agent if object_ids_by_agent else None,
                ego_ref_pos=tuple(js.ego.positions[-1][:2]) if js.ego.positions else None,
            )
        except Exception as e:
            if self.verbose:
                print(f"  judge={js.judge_id} frame {js.frame_count} dropped: {e!r}",
                      file=sys.stderr)
            return

        js.id_reporters = {}
        for sender, m in raw_msg_by_agent.items():
            for oid in codec.get_object_ids_of(m):
                js.id_reporters.setdefault(int(oid), set()).add(sender)
        for s in stats:
            source_id = source_id_map[s.agent_id]
            js.reputation_db.record(source_id, s.r_new, js.frame_count, s.risk_persist)
            self._record_row(js, s, raw_msg_by_agent[s.agent_id], sim_time_by_agent[s.agent_id])
        if js.caution is not None:
            self._caution_step(js, stats, positions_by_agent, velocities_by_agent,
                               object_ids_by_agent, t_ref)

        if js.frame_count % BATCH_SIZE == 0:
            js.reputation_db.flush(js.frame_count)

    def _caution_step(self, js: _JudgeState, stats, pos_by, vel_by, oid_by, t_ref: float) -> None:
        """Fold this flush into the judge's caution layer and score it against ground truth."""
        if not js.ego.positions:
            return
        ego_xy = js.ego.positions[-1][:2]
        ego_v = js.ego._self_vel

        def gather(pairs) -> Sightings:
            if not pairs:
                return Sightings.empty()
            return Sightings(
                np.array([a for a, _ in pairs], dtype=int),
                np.array([int(oid_by[a][i]) for a, i in pairs], dtype=int),
                np.array([(pos_by[a][i][0], pos_by[a][i][1]) for a, i in pairs], dtype=float),
                np.array([(vel_by[a][i][0], vel_by[a][i][1]) for a, i in pairs], dtype=float))

        conf = gather([(s.agent_id, i) for s in stats for i in s.admitted])
        unv = gather([(s.agent_id, i) for s in stats for i in s.unverified])
        seen = (np.array([(p[0], p[1]) for p in js.ego.positions[:-1]], dtype=float)
                if len(js.ego.positions) > 1 else None)
        fm = js.caution.update(t_ref, ego_xy, ego_v, conf, unv, seen)

        # Evaluation-only ground truth: phantom ids (own 900000+node, shared 950000) and hidden
        # real obstacles (800000+k), all wrapped by the 16-bit object_id field.
        hidden_base = 800000 % 65536

        def kind(sender: int, oid: int) -> str:
            if oid in ((900000 + sender) % 65536, 950000 % 65536):
                return 'phantom'
            if hidden_base <= oid < hidden_base + 100:
                return 'hidden'
            return 'other'

        cnt = {'phantom': 0, 'hidden': 0, 'other': 0}
        crit = {'phantom': 0, 'hidden': 0, 'other': 0}
        marker_oids = set()
        for m in fm.markers:
            k = kind(m.sender, m.object_id)
            cnt[k] += 1
            crit[k] += int(m.critical)
            marker_oids.add(m.object_id)
        conf_oids = set(int(o) for o in conf.oid)

        # Hazards: reported phantom / hidden objects whose closest approach to the judge's own
        # path is inside the keep-out zone. Where did each end up: confirmed, caution, or absent?
        cm = js.caution
        hz = {'hidden': [0, 0, 0, 0], 'phantom': [0, 0, 0, 0]}   # total, confirmed, caution, none
        seen_oids = set()
        for sender, oids in oid_by.items():
            idx = [i for i, o in enumerate(oids)
                   if kind(sender, int(o)) != 'other' and int(o) not in seen_oids]
            if not idx:
                continue
            xy = np.array([(pos_by[sender][i][0], pos_by[sender][i][1]) for i in idx], dtype=float)
            v = np.array([(vel_by[sender][i][0], vel_by[sender][i][1]) for i in idx], dtype=float)
            # Plain footprint: would the ego actually hit it, ignoring any caution margin?
            hit, _, _ = conflicts(ego_xy, ego_v, xy, v, cm.half_len_m, cm.half_wid_m, cm.horizon_s)
            for j, i in enumerate(idx):
                o = int(oids[i])
                seen_oids.add(o)
                if not hit[j]:
                    continue
                k = kind(sender, o)
                hz[k][0] += 1
                hz[k][1 if o in conf_oids else 2 if o in marker_oids else 3] += 1

        self.caution_rows.append({
            "judge_node": js.judge_id, "sim_time": round(t_ref, 3), "advisory": fm.advisory,
            "n_confirmed": len(conf), "n_caution": len(fm.markers),
            "n_caution_phantom": cnt['phantom'], "n_caution_hidden": cnt['hidden'],
            "n_caution_other": cnt['other'],
            "n_crit_caution": sum(crit.values()), "n_crit_caution_phantom": crit['phantom'],
            "n_crit_caution_hidden": crit['hidden'], "n_crit_caution_other": crit['other'],
            "n_crit_confirmed": fm.n_critical_confirmed,
            "n_crit_caution_2s": sum(1 for m in fm.markers if m.critical and m.ttc_s <= 2.0),
            "max_criticality": round(max((m.criticality for m in fm.markers), default=0.0), 3),
            "hz_hidden": hz['hidden'][0], "hz_hidden_confirmed": hz['hidden'][1],
            "hz_hidden_caution": hz['hidden'][2], "hz_hidden_none": hz['hidden'][3],
            "hz_phantom": hz['phantom'][0], "hz_phantom_confirmed": hz['phantom'][1],
            "hz_phantom_caution": hz['phantom'][2], "hz_phantom_none": hz['phantom'][3],
        })

    def _record_row(self, js: _JudgeState, s: FrameStats, msg, sim_time: float) -> None:
        n_matched = sum(1 for v in s.verdict if v == VERDICT_MATCHED)
        n_deferred = sum(1 for v in s.verdict if v == VERDICT_DEFERRED)
        n_uncorr = sum(1 for v in s.verdict if v == VERDICT_UNCORROBORATED)
        obj_ids = codec.get_object_ids_of(msg)
        # Objects within NEAR_RANGE_M of the judge: the ones it could act on now.
        # Farther ones cannot be checked by anybody yet.
        if js.ego.positions:    # none until the judge's own first TX
            ex, ey = js.ego.positions[-1][0], js.ego.positions[-1][1]
            near_idx = [i for i, p in enumerate(codec.get_global_positions_of(msg))
                        if math.hypot(p[0] - ex, p[1] - ey) <= NEAR_RANGE_M]
        else:
            near_idx = []
        near_set = set(near_idx)
        # Evaluation-only: RosSDSMApp hiddenObjects are real static obstacles with id
        # 800000+k (wrapped by the 16-bit field). "Solo" = nobody else in this window
        # reports it, and neither does the judge: a true unique sighting.
        hidden_base = 800000 % 65536
        hidden_idx = {i for i, oid in enumerate(obj_ids)
                      if hidden_base <= oid % 65536 < hidden_base + 100}
        solo_idx = {i for i in hidden_idx
                    if js.id_reporters.get(int(obj_ids[i]), set()) == {s.agent_id}
                    and int(obj_ids[i]) not in js.ego.object_ids}
        # Evaluation-only: RosSDSMApp attackType="phantom" fabricates id 900000+node,
        # which the 16-bit object_id field wraps.
        phantom_id = (900000 + s.agent_id) % 65536

        def is_phantom(oid: int) -> bool:
            # own phantom, or the colluding attackers' shared fake (id 950000)
            return oid % 65536 in (phantom_id, 950000 % 65536)
        self.rows.append({
            "judge_node": js.judge_id,
            "sender_node": s.agent_id,
            "msg_cnt": codec.sequence_of(msg),
            "sim_time": round(sim_time, 3),
            "trusted": s.trusted,
            "r_old": round(s.r_old, 6),
            "r_new": round(s.r_new, 6),
            "tau": round(s.tau, 6),
            "n_total": round(s.n_total, 4),
            "correct": round(s.correct, 4),
            "incorrect": round(s.incorrect, 4),
            "n_objects": len(s.verdict),
            "n_matched": n_matched,
            "n_deferred": n_deferred,
            "n_uncorroborated": n_uncorr,
            "n_admitted": len(s.admitted),
            "false_freq": round(s.false_freq, 4),
            "n_near": len(near_idx),
            "n_near_admitted": sum(1 for i in s.admitted if i in near_set),
            "n_hidden": len(hidden_idx),
            "n_hidden_solo": len(solo_idx),
            "n_hidden_admitted": sum(1 for i in s.admitted if i in hidden_idx),
            "n_hidden_solo_admitted": sum(1 for i in s.admitted if i in solo_idx),
            "n_unverified": len(s.unverified),
            "n_near_unverified": sum(1 for i in s.unverified if i in near_set),
            "n_hidden_unverified": sum(1 for i in s.unverified if i in hidden_idx),
            "n_hidden_solo_unverified": sum(1 for i in s.unverified if i in solo_idx),
            "n_phantom_unverified": sum(1 for i in s.unverified if is_phantom(obj_ids[i])),
            "n_strikes": s.strikes,
            "n_phantom": sum(1 for oid in obj_ids if is_phantom(oid)),
            "n_phantom_admitted": sum(1 for i in s.admitted if is_phantom(obj_ids[i])),
        })

    def finish(self) -> None:
        """Flush every judge's last open bucket -- on_event only closes a
        bucket when a NEWER one arrives, so without this the tail <=
        flush_interval_s of every judge's stream is silently dropped. (Same
        fix cpx-mono-aa applied to trust_node.py's destroy_node() for the
        live path.)"""
        for js in self._judges.values():
            self._flush(js)
            js.close()


def replay_file(jsonl_path: Path, db_dir: Path, flush_interval_s: float,
                deferred_deadline_s: float, verbose: bool,
                high_trust: float = HIGH_TRUST, admission: str = 'tiered',
                persist: int = 1, opp_range: float = OPPORTUNITY_RANGE_M,
                extra: Optional[dict] = None, judge_stride: int = 1, judge_rem: int = 0,
                caution_rows: Optional[list] = None) -> list[dict]:
    replayer = TrustReplayer(db_dir, flush_interval_s, deferred_deadline_s, verbose, high_trust,
                             admission, persist, opp_range, extra)
    n_lines = n_events = n_errors = 0
    with jsonl_path.open("r", encoding="utf-8") as f:
        for line in f:
            n_lines += 1
            if judge_stride > 1:
                # Judges are independent: skip lines for unselected judges before any decoding.
                m = _NODE_RE.match(line)
                if m and int(m.group(1)) % judge_stride != judge_rem:
                    continue
            try:
                event = _event_from_line(line)
            except Exception as e:
                n_errors += 1
                if verbose:
                    print(f"  line {n_lines}: {e!r}", file=sys.stderr)
                continue
            if event is None:
                continue
            n_events += 1
            replayer.on_event(event)
    replayer.finish()
    if caution_rows is not None:
        caution_rows.extend(replayer.caution_rows)
    print(f"  {n_lines} lines, {n_events} TX/RX events, {n_errors} decode errors, "
          f"{len(replayer.rows)} verdict rows, {len(replayer._judges)} judges")
    return replayer.rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help="Path to a *-ros-events.jsonl file, or a results dir "
                                  "(with --prefix) containing one")
    ap.add_argument("--prefix", help="Run prefix, e.g. Periodic-r0 (only used if `path` is a dir)")
    ap.add_argument("--flush-interval", type=float, default=DEFAULT_FLUSH_INTERVAL_S)
    ap.add_argument("--deferred-deadline", type=float, default=T_DEADLINE_S)
    ap.add_argument("--high-trust", type=float, default=HIGH_TRUST,
                    help="Reputation at/above which every object is admitted without "
                         "corroboration; >1.0 requires corroboration for all (default %(default)s)")
    ap.add_argument("--out", help="Output CSV path (default: <input-stem>-trust-verdicts.csv)")
    ap.add_argument("--db-dir", help="Reputation DB dir (default: alongside the output CSV)")
    ap.add_argument("--admission", choices=("tiered", "probabilistic", "strict_unverified"), default="tiered",
                    help="Per-object admission policy (default %(default)s)")
    ap.add_argument("--persist", type=int, default=1,
                    help="Flushes a contradiction must persist before it counts (strict_unverified/probabilistic)")
    ap.add_argument("--opportunity-range", type=float, default=OPPORTUNITY_RANGE_M,
                    help="Metres within which a witness could have seen an object (strict_unverified/probabilistic)")
    ap.add_argument("--strike-penalty", type=float, default=0.0,
                    help="strict_unverified: reputation taken per persistent, strongly contradicted object (0 = off)")
    ap.add_argument("--strike-missed", type=float, default=2.0,
                    help="Witness weight that must have missed an object for a strike")
    ap.add_argument("--report-cap", type=int, default=0,
                    help="Objects per message at which senders truncate (e.g. 16); a sender at the cap is not counted as missing objects beyond its farthest reported one")
    ap.add_argument("--admit-judge-confirmed", action="store_true",
                    help="Admit objects the judge itself sees even while the sender fails the gate")
    ap.add_argument("--ignore-local", action="store_true",
                    help="Use the judge's truncated broadcast list instead of its full local view (for comparison)")
    ap.add_argument("--support-threshold", type=float, default=None,
                    help="Peer reputation mass needed to corroborate (default 1.0); with --peer-cap 0.6, 1.5 needs 3 peers")
    ap.add_argument("--strike-kinematic", action="store_true",
                    help="Let implausible motion count toward a strike (off by default: misfires at density)")
    ap.add_argument("--strike-missed-frac", type=float, default=0.0,
                    help="Also require this fraction of the witness weight in range to have missed the object")
    ap.add_argument("--peer-cap", type=float, default=1.0,
                    help="Cap on the corroboration mass any one peer can contribute (1.0 = uncapped)")
    ap.add_argument("--caution-map", action="store_true",
                    help="strict_unverified: turn unverified objects into caution markers (potential critical "
                         "vehicles with a larger keep-out) and write <stem>-caution.csv")
    ap.add_argument("--caution-margin", type=float, default=None,
                    help="Extra keep-out (m) around a caution marker (default 3.0)")
    ap.add_argument("--judge-stride", type=int, default=1,
                    help="Only replay judges with node %% stride == --judge-rem (independent judges; much faster on dense logs)")
    ap.add_argument("--judge-rem", type=int, default=0)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    p = Path(args.path)
    if p.is_dir():
        if not args.prefix:
            print("ERROR: --prefix required when `path` is a directory", file=sys.stderr)
            return 1
        jsonl_path = p / f"{args.prefix}-ros-events.jsonl"
    else:
        jsonl_path = p

    if not jsonl_path.exists():
        print(f"ERROR: {jsonl_path} not found. Was this run started with "
              f"rosBridgeMode=\"log\"?", file=sys.stderr)
        return 1

    out_path = Path(args.out) if args.out else jsonl_path.with_name(
        jsonl_path.stem.replace("-ros-events", "") + "-trust-verdicts.csv")
    db_dir = Path(args.db_dir) if args.db_dir else out_path.parent / "trust_reputation_db"

    if args.caution_map and args.admission != "strict_unverified":
        print("ERROR: --caution-map needs --admission strict_unverified", file=sys.stderr)
        return 1
    caution_rows: list = []
    print(f"Replaying {jsonl_path} ...")
    rows = replay_file(jsonl_path, db_dir, args.flush_interval, args.deferred_deadline, args.verbose,
                       args.high_trust, args.admission, args.persist, args.opportunity_range,
                       {'strike_penalty': args.strike_penalty, 'strike_missed_min': args.strike_missed,
                        'peer_support_cap': args.peer_cap,
                        'strike_missed_frac': args.strike_missed_frac,
                        'support_threshold': args.support_threshold,
                        'strike_use_kinematic': args.strike_kinematic,
                        'report_cap': args.report_cap,
                        'admit_judge_confirmed': args.admit_judge_confirmed,
                        'ignore_local': args.ignore_local,
                        'caution_map': args.caution_map,
                        'caution_margin': args.caution_margin},
                       args.judge_stride, args.judge_rem, caution_rows)

    if not rows:
        print("No verdict rows produced.", file=sys.stderr)
        return 1

    with out_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=_CSV_FIELDS)
        w.writeheader()
        w.writerows(rows)
    print(f"  -> {out_path}")
    if caution_rows:
        cpath = out_path.with_name(out_path.name.replace("-trust-verdicts.csv", "") + "-caution.csv")
        with cpath.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=_CAUTION_FIELDS)
            w.writeheader()
            w.writerows(caution_rows)
        print(f"  -> {cpath}")

    n_total = len(rows)
    n_untrusted = sum(1 for r in rows if not r["trusted"])
    print(f"  filter rate: {n_untrusted}/{n_total} verdicts NOT trusted "
          f"({100.0 * n_untrusted / n_total:.2f}%)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
