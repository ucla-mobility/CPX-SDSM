# mypy: ignore-errors
"""
Trust perception node: the second-layer trust check on SDSM traffic.

Subscribes to ReceivedSdsm events from veins_ros_bridge (one per TX/RX the
running Veins simulation reports) and, for EVERY simulated node that shows up
as an RX receiver, runs an independent TrustEngine judging what that node
received -- exactly as if each simulated vehicle carried its own onboard
trust module. Publishes one TrustVerdict per (judge, sender) per flush.

WHY ONE PROCESS RUNS MANY JUDGES, NOT ONE JUDGE PER PROCESS
-------------------------------------------------------------
CPX-Mono's agent.py is one ROS node per ego, because a real onboard trust
module lives on ONE vehicle. Here, RosSDSMApp funnels every simulated node's
TX and RX events through one shared UDP port (see veins_ros_bridge), so this
node already receives the whole scenario's traffic in one stream; spawning
one ROS node per simulated vehicle would need a discovery mechanism this
bridge doesn't have. Instead, ONE TrustEngine + one SORT tracker set is kept
per `node` id that appears in the stream (_JudgeState), so the whole scenario
is judged from every vehicle's own perspective in a single process -- the
trust verdicts differ per judge exactly as they would with one module per
vehicle, since each _JudgeState only ever sees events where it is `node`.

EGO SEMANTICS
--------------
A TX event with node=N is N's own broadcast: it becomes _JudgeState(N)'s ego
ground truth for that flush (mirrors CPX-Mono agent.py's sim-mode "ego echo",
where a node's own published SDSM feeds its own engine as ego truth). An RX
event with node=N is what N actually received from sdsm.source_id[0] after
the 802.11p channel model -- it feeds _JudgeState(N)'s engine as an
OTHER-agent report to judge. The same physical broadcast therefore appears
once as ego truth (in the sender's own _JudgeState) and once as an
other-agent report (in each receiver's _JudgeState) -- never both in the same
engine, so a node is never asked to corroborate itself.

FLUSH WINDOWING
-----------------
Bucketed by sim_time (RosSDSMApp's simTime(), shared across every node's
stream) rather than a ROS wall-clock timer: a batch OMNeT run's UDP arrival
rate has no fixed relationship to wall time, so a timer-driven flush would
either starve or over-fire depending on how fast the sim actually runs. Each
judge closes its own bucket independently, the first time an event for that
judge arrives with a later bucket index -- simpler than CPX-Mono agent.py's
close-budget/backlog machinery (no multi-host clock skew to absorb here: one
process, one shared sim clock), at the cost of not tolerating out-of-order
delivery across a bucket boundary. UDP is delivered same-host, same-process,
so reordering is not expected in practice; a late event past its bucket's
close is dropped and counted (self._diag_late), not silently misfiled into
the wrong frame.
"""

import logging
import math
import os
import traceback
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sdsm_trust_interfaces.msg import CautionMap as CautionMapMsg, ReceivedSdsm, TrustVerdict
from sdsm_trust_perception.global_trust_perception.pipeline import sdsm_codec as codec
from sdsm_trust_perception.global_trust_perception.pipeline.caution_map import (
    CAUTION_MARGIN_M,
    CautionMap,
    Sightings,
)
from sdsm_trust_perception.global_trust_perception.pipeline.persistent_reputation_tracker import (
    BATCH_SIZE,
    PersistentReputationTracker,
)
from sdsm_trust_perception.global_trust_perception.pipeline.trustworthy_perception import (
    FrameStats,
    TrustEngine,
    VERDICT_DEFERRED,
    VERDICT_MATCHED,
    VERDICT_UNCORROBORATED,
)
from sdsm_trust_perception.global_trust_perception.trust_calculations.consistency import T_DEADLINE_S
from sdsm_trust_perception.global_trust_perception.trust_calculations.consistency_checks.kinematic_checks import TrackData
from sdsm_trust_perception.global_trust_perception.tracking.SORT.modified_sort_centroid import Sort

_VERDICT_TO_WIRE = {
    VERDICT_MATCHED: TrustVerdict.VERDICT_MATCHED,
    VERDICT_DEFERRED: TrustVerdict.VERDICT_DEFERRED,
    VERDICT_UNCORROBORATED: TrustVerdict.VERDICT_UNCORROBORATED,
}

_DEFAULT_FLUSH_INTERVAL_S = 0.5
_LATE_WARN_EVERY = 50  # log every Nth dropped-late event, not each one


class _EgoState:
    """One judge's most recent own broadcast, held across flushes.

    A TX event only arrives on the flush where that node actually sent, but a
    judge should keep comparing against its last known ego truth on flushes
    where it stayed silent -- mirrors real onboard use, where a vehicle's own
    latest perception frame doesn't vanish between its own broadcast ticks.
    None until the first TX event for this judge arrives.
    """

    def __init__(self):
        self.positions: list = []
        self.dims: list = []
        self.headings: list = []
        self.scores: list = []
        self.labels: list = []
        self.equipment: int = 0
        self._raw_positions: list = []
        self._velocities: list = []
        self._send_time: float = 0.0
        self._last_xy = None
        self._last_t = None
        self._self_heading: float = 360.0
        self._self_vel = (0.0, 0.0)

    def update_from(self, msg, send_time: float) -> None:
        raw = codec.get_global_positions_of(msg)
        vel = codec.get_velocities_of(msg)
        self.dims = codec.get_dims_of(msg)
        self.headings = codec.get_headings_of(msg)
        self.scores = codec.get_local_scores_of(msg)
        self.labels = codec.get_labels_of(msg)
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
    """Everything one judging node's trust view needs, kept between flushes."""

    def __init__(self, judge_id: int, db_path: str, flush_hz: float, deadline_s: float,
                 now, admission: str = 'tiered', contradiction_persist: int = 1,
                 opportunity_range_m: float = 150.0, strike_penalty: float = 0.0,
                 strike_missed_min: float = 2.0, peer_support_cap: float = 1.0,
                 strike_missed_frac: float = 0.0, support_threshold: float = None,
                 strike_use_kinematic: bool = False, report_cap: int = 0,
                 admit_judge_confirmed: bool = False, caution: Optional[CautionMap] = None):
        self.judge_id = judge_id
        self.caution = caution
        self.reputation_db = PersistentReputationTracker(db_path)
        self.engine = TrustEngine(self.reputation_db, flush_hz=flush_hz,
                                  deadline_s=deadline_s, now=now, admission=admission,
                                  contradiction_persist=contradiction_persist,
                                  opportunity_range_m=opportunity_range_m,
                                  strike_penalty=strike_penalty, strike_missed_min=strike_missed_min,
                                  peer_support_cap=peer_support_cap,
                                  strike_missed_frac=strike_missed_frac,
                                  support_threshold=support_threshold,
                                  strike_use_kinematic=strike_use_kinematic,
                                  report_cap=report_cap,
                                  admit_judge_confirmed=admit_judge_confirmed)
        self.flush_dt = 1.0 / flush_hz
        self.trackers: dict[int, Sort] = {}   # sender_node -> Sort
        self.ego = _EgoState()
        self.current_bucket: Optional[int] = None
        self.buffer: list[ReceivedSdsm] = []
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


class TrustPerceptionNode(Node):

    def __init__(self):
        super().__init__('trust_perception_node')

        self.declare_parameter('events_topic', '/veins/sdsm_events')
        self.declare_parameter('verdicts_topic', '/veins/trust_verdicts')
        self.declare_parameter('flush_interval_s', _DEFAULT_FLUSH_INTERVAL_S)
        self.declare_parameter('deferred_deadline_s', T_DEADLINE_S)
        self.declare_parameter('db_dir', _default_db_dir())
        self.declare_parameter('admission', 'tiered')   # 'tiered' | 'probabilistic' | 'strict_unverified'
        self.declare_parameter('contradiction_persist', 1)
        self.declare_parameter('opportunity_range_m', 150.0)
        self.declare_parameter('strike_penalty', 0.0)
        self.declare_parameter('strike_missed_min', 2.0)
        self.declare_parameter('peer_support_cap', 1.0)
        self.declare_parameter('strike_missed_frac', 0.0)
        self.declare_parameter('support_threshold', 0.0)          # 0 = default (1.0)
        self.declare_parameter('strike_use_kinematic', False)
        self.declare_parameter('report_cap', 0)
        self.declare_parameter('admit_judge_confirmed', False)
        # strict_unverified only: publish unverified objects as caution markers (potential
        # critical vehicles with a larger keep-out) on caution_topic.
        self.declare_parameter('caution_map', False)
        self.declare_parameter('caution_topic', '/veins/caution_map')
        self.declare_parameter('caution_margin_m', CAUTION_MARGIN_M)

        events_topic = self.get_parameter('events_topic').value
        verdicts_topic = self.get_parameter('verdicts_topic').value
        self.flush_interval_s = float(self.get_parameter('flush_interval_s').value)
        if self.flush_interval_s <= 0.0:
            raise ValueError(f'flush_interval_s={self.flush_interval_s} must be > 0')
        self.deferred_deadline_s = float(self.get_parameter('deferred_deadline_s').value)
        self.db_dir = str(self.get_parameter('db_dir').value)
        self.admission = str(self.get_parameter('admission').value)
        self.contradiction_persist = int(self.get_parameter('contradiction_persist').value)
        self.opportunity_range_m = float(self.get_parameter('opportunity_range_m').value)
        self.strike_penalty = float(self.get_parameter('strike_penalty').value)
        self.strike_missed_min = float(self.get_parameter('strike_missed_min').value)
        self.peer_support_cap = float(self.get_parameter('peer_support_cap').value)
        self.strike_missed_frac = float(self.get_parameter('strike_missed_frac').value)
        _st = float(self.get_parameter('support_threshold').value)
        self.support_threshold = _st if _st > 0.0 else None
        self.strike_use_kinematic = bool(self.get_parameter('strike_use_kinematic').value)
        self.report_cap = int(self.get_parameter('report_cap').value)
        self.admit_judge_confirmed = bool(self.get_parameter('admit_judge_confirmed').value)
        self.caution_map = bool(self.get_parameter('caution_map').value)
        self.caution_margin_m = float(self.get_parameter('caution_margin_m').value)
        if self.caution_map and self.admission != 'strict_unverified':
            raise ValueError("caution_map needs admission='strict_unverified'")
        os.makedirs(self.db_dir, exist_ok=True)

        self._judges: dict[int, _JudgeState] = {}

        self.sub = self.create_subscription(
            ReceivedSdsm, events_topic, self.on_event, 200,
        )
        self.pub = self.create_publisher(TrustVerdict, verdicts_topic, 200)
        self.caution_pub = (self.create_publisher(
            CautionMapMsg, str(self.get_parameter('caution_topic').value), 200)
            if self.caution_map else None)

        self.get_logger().info(
            f'trust_perception_node up: {events_topic} -> {verdicts_topic}, '
            f'flush_interval={self.flush_interval_s * 1000:.0f}ms, '
            f'db_dir={self.db_dir}'
        )

    def _now(self) -> float:
        """Node clock in seconds -- only used by TrustEngine for dt between
        process_frame() CALLS (wall time between flushes, a real pipeline
        cost), never for scene time -- scene time throughout this node is
        sim_time from the events themselves. See TrustEngine's own `now`
        docstring in trustworthy_perception.py for why the two clocks are
        deliberately different quantities."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _judge(self, judge_id: int) -> _JudgeState:
        js = self._judges.get(judge_id)
        if js is None:
            db_path = os.path.join(self.db_dir, f'historical_reputations_{judge_id}.db')
            js = _JudgeState(judge_id, db_path, flush_hz=1.0 / self.flush_interval_s,
                             deadline_s=self.deferred_deadline_s, now=self._now,
                             admission=self.admission,
                             contradiction_persist=self.contradiction_persist,
                             opportunity_range_m=self.opportunity_range_m,
                             strike_penalty=self.strike_penalty,
                             strike_missed_min=self.strike_missed_min,
                             peer_support_cap=self.peer_support_cap,
                             strike_missed_frac=self.strike_missed_frac,
                             support_threshold=self.support_threshold,
                             strike_use_kinematic=self.strike_use_kinematic,
                             report_cap=self.report_cap,
                             admit_judge_confirmed=self.admit_judge_confirmed,
                             caution=(CautionMap(margin_m=self.caution_margin_m)
                                      if self.caution_map else None))
            self._judges[judge_id] = js
            self.get_logger().info(f'New judge node={judge_id}, db={db_path}')
        return js

    def on_event(self, event: ReceivedSdsm) -> None:
        judge_id = codec.envelope_receiver(event)
        js = self._judge(judge_id)
        bucket_idx = int(event.sim_time // self.flush_interval_s)

        if js.current_bucket is None:
            js.current_bucket = bucket_idx
        elif bucket_idx < js.current_bucket:
            js.diag_late += 1
            if js.diag_late % _LATE_WARN_EVERY == 1:
                self.get_logger().warning(
                    f'judge={judge_id}: dropped late event (bucket {bucket_idx} < '
                    f'current {js.current_bucket}, total late={js.diag_late})'
                )
            return
        elif bucket_idx > js.current_bucket:
            self._flush(js)
            js.current_bucket = bucket_idx
            js.buffer = []

        js.buffer.append(event)

    def _flush(self, js: _JudgeState) -> None:
        """Run one closed bucket through js.engine and publish its verdicts."""
        if not js.buffer:
            return
        js.frame_count += 1

        positions_by_agent: dict[int, list] = {}
        velocities_by_agent: dict[int, list] = {}
        dims_by_agent: dict[int, list] = {}
        headings_by_agent: dict[int, list] = {}
        labels_by_agent: dict[int, list] = {}
        object_ids_by_agent: dict[int, list] = {}
        equipment_by_agent: dict[int, int] = {}
        source_id_map: dict[int, tuple] = {}
        ref_pos_by_agent: dict[int, tuple] = {}
        tracks_by_agent: dict[int, TrackData] = {}
        raw_msg_by_agent: dict[int, object] = {}
        sim_time_by_agent: dict[int, float] = {}

        # One event per sender per bucket is kept -- the latest, mirroring
        # CPX-Mono agent.py's _latest_per_sender (a sender publishing faster
        # than the flush window, or jittering across a boundary, must not
        # enter fusion twice as if it contradicted itself).
        rx_by_sender: dict[int, ReceivedSdsm] = {}
        tx_event: Optional[ReceivedSdsm] = None
        for event in js.buffer:
            if event.is_rx:
                rx_by_sender[int(event.sender_node)] = event
            else:
                tx_event = event  # last TX this bucket wins

        # Every detection is dead-reckoned to this one instant (the bucket's
        # end) so views taken up to a bucket apart can be clustered spatially.
        t_ref = (js.current_bucket + 1) * self.flush_interval_s
        if tx_event is not None:
            js.ego.update_from(tx_event.sdsm, tx_event.sim_time)
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
            # recv_t: sim_time of this RX event -- the sim's own shared clock,
            # matching how sender_motion.SenderMotionHistory expects strictly
            # increasing timestamps per sender to derive dt/speed from.
            # latency: read directly from the event, already computed by the
            # sim in seconds (see ReceivedSdsm.msg) -- this pipeline runs on
            # one shared sim clock, not two independent hosts' wall clocks,
            # so there is no send/receive timestamp arithmetic to do here
            # (contrast CPX-Mono's send_latency_of, which reconstructs this
            # from two wall-clock stamps because it has no shared clock).
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
                scores_by_agent=None,   # no per-object local scores on this wire
                ego_scores=None,        # -- see sdsm_codec module docstring
                headings_by_agent=headings_by_agent if headings_by_agent else None,
                ego_headings=js.ego.headings if js.ego.headings else None,
                equipment_by_agent=equipment_by_agent if equipment_by_agent else None,
                ego_equipment=js.ego.equipment,
                classes_by_agent=labels_by_agent if labels_by_agent else None,
                ego_classes=js.ego.labels if js.ego.labels else None,
                object_ids_by_agent=object_ids_by_agent if object_ids_by_agent else None,
                ego_ref_pos=tuple(js.ego.positions[-1][:2]) if js.ego.positions else None,
            )
        except Exception:
            self.get_logger().error(
                f'judge={js.judge_id} frame {js.frame_count} dropped:\n'
                f'{traceback.format_exc()}'
            )
            return

        for s in stats:
            source_id = source_id_map[s.agent_id]
            js.reputation_db.record(source_id, s.r_new, js.frame_count, s.risk_persist)
            self._publish_verdict(js, s, raw_msg_by_agent[s.agent_id],
                                  sim_time_by_agent[s.agent_id])
        if js.caution is not None and js.ego.positions:
            self._publish_caution(js, stats, positions_by_agent, velocities_by_agent,
                                  object_ids_by_agent, t_ref)

        if js.frame_count % BATCH_SIZE == 0:
            js.reputation_db.flush(js.frame_count)

    def _publish_caution(self, js: _JudgeState, stats, pos_by, vel_by, oid_by, t_ref: float) -> None:
        """Fold this flush into the judge's caution layer and publish its markers."""
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
        fm = js.caution.update(t_ref, js.ego.positions[-1][:2], js.ego._self_vel, conf, unv, seen)

        out = CautionMapMsg()
        out.judge_node = js.judge_id
        out.sim_time = t_ref
        out.advisory = fm.advisory
        out.n_confirmed_critical = fm.n_critical_confirmed
        out.sender_node = [m.sender for m in fm.markers]
        out.object_id = [m.object_id for m in fm.markers]
        out.x = [m.x for m in fm.markers]
        out.y = [m.y for m in fm.markers]
        out.vx = [m.vx for m in fm.markers]
        out.vy = [m.vy for m in fm.markers]
        out.keepout_m = [m.keepout_m for m in fm.markers]
        out.critical = [m.critical for m in fm.markers]
        out.ttc_s = [m.ttc_s for m in fm.markers]
        out.dist_m = [m.dist_m for m in fm.markers]
        out.criticality = [m.criticality for m in fm.markers]
        self.caution_pub.publish(out)

    def _publish_verdict(self, js: _JudgeState, s: FrameStats, msg, sim_time: float) -> None:
        out = TrustVerdict()
        out.judge_node = js.judge_id
        out.sender_node = s.agent_id
        out.msg_cnt = codec.sequence_of(msg)
        out.sim_time = sim_time
        out.trusted = s.trusted
        out.r_old = s.r_old
        out.r_new = s.r_new
        out.tau = s.tau
        out.n_total = s.n_total
        out.correct = s.correct
        out.incorrect = s.incorrect
        out.verdict = [_VERDICT_TO_WIRE[v] for v in s.verdict]
        admitted_set = set(s.admitted)
        out.admitted = [i in admitted_set for i in range(len(s.verdict))]
        unverified_set = set(s.unverified)
        out.unverified = [i in unverified_set for i in range(len(s.verdict))]
        self.pub.publish(out)

        if s.trusted:
            status = f'{s.tier.upper()} R={s.r_old:.3f}->{s.r_new:.3f}'
        else:
            status = f'GATE FAIL R_eff={s.r_eff:.3f}<tau={s.tau:.3f}'
        self.get_logger().info(
            f'judge={js.judge_id} sender={s.agent_id} N={s.n_total:.2f} '
            f'C={s.correct:.2f} I={s.incorrect:.2f} kine={s.kine_flagged} '
            f'attr={s.attr_correct} unc={s.uncorroborated} [{status}]'
        )

    def destroy_node(self):
        # Flush each judge's last open bucket before closing its DB -- nothing
        # else ever triggers that final flush (on_event only closes a bucket
        # when a NEWER one arrives), so without this the last <=
        # flush_interval_s of every run would be silently dropped.
        for js in self._judges.values():
            self._flush(js)
            js.close()
        super().destroy_node()


def _default_db_dir() -> str:
    """<this package's share dir's sibling data/>, i.e. CPX-SDSM/ros2_ws/data/
    when COLCON_PREFIX_PATH is sourced; falls back to the working directory
    otherwise. Mirrors CPX-Mono agent.py's _default_db_path derivation."""
    prefix = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)[0]
    if prefix:
        ws_root = os.path.dirname(prefix)  # <ros2_ws>/install -> <ros2_ws>
        return os.path.join(ws_root, 'data', 'sdsm_trust_perception')
    return os.path.join(os.getcwd(), 'data', 'sdsm_trust_perception')


def main(args=None):
    logging.basicConfig(level=logging.INFO, format='%(name)s | %(message)s')
    rclpy.init(args=args)
    node = TrustPerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
