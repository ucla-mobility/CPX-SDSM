# Ported (pure Python, no ROS deps) from CPX-Mono's
# ros2/src/global_trust_perception/global_trust_perception/pipeline/trustworthy_perception.py
# — see that repo for the full design writeup.
#
# CHANGES FROM THE SOURCE FILE:
#   - imports repointed at this package (sdsm_trust_perception.* instead of
#     global_trust_perception.*)
#   - the mmcooper_fuse cross-agent spatial-matching stage IS ported (see
#     ../mmcooper_fuse): one reputation-weighted MS-PSF pass over ego + every
#     sender clusters all reports of the same real object, and each sender's
#     matched/ego_only/other_only buckets and its peer-witness reputation-mass
#     support come from those shared clusters, exactly as in the source. Not
#     ported: the phase-2 display fusion, the last_fusion viz hook and the
#     capture-time frame grouping (the replay/node bucket by receive time).
#   - the deferred ledger is keyed by the sender's reported object_id (falling
#     back to the SORT track id) instead of SORT's track id alone: on this wire
#     the id is a stable per-vehicle identity, while track ids flap and every
#     flap resets the grace clock.
#   - process_frame's dims_by_agent/ego_dims, headings_by_agent/ego_headings,
#     scores_by_agent/ego_scores, equipment_by_agent/ego_equipment,
#     classes_by_agent/ego_classes keep the exact same meaning and defaults
#     as the source file; sdsm_codec.py documents what CPX-SDSM's wire format
#     can and cannot actually supply for each.
"""
Trustworthy perception: the orchestrator (TrustEngine).

This module owns the per-frame pipeline but NOT the math behind any stage -
each calculation concern lives in its own module and is called from here:

    tracking/SORT/modified_sort_centroid.py - Kalman temporal tracking (track ids)
    consistency.py                        - verdict policy: matched/ego_only ->
                                            weighted (C, I, held); pen / threshold
    deferred.py                           - other_only grace/back-pay/expiry ledger
    reputation.py                         - dynamic threshold + reputation update

The engine is deliberately ROS-free: the trust_node decodes ReceivedSdsm
events into global (x, y, z) positions and forwards TrackData (from its own
embedded SORT tracker) before passing them here, so the whole trust pipeline
stays unit-testable without rclpy.

TWO-PHASE DESIGN (as in the source file): this engine runs the phase-1
"judging" fusion — ego + every agent, UNGATED, weighted by reputation — to
decide who to reward and who to dock. The phase-2 "output" fusion (over only
the admitted agents) is not run here; process_frame reports each agent's
admitted detection indices in FrameStats.admitted for a caller to act on.

PIPELINE PER FRAME (ego-centric, "ego" = the judging simulated node), for
each other agent:
    0. gate      : R_eff = R_old * F * V (kinematic freshness * persistence
                   penalty) vs tau(R_eff) decides whether this agent's DATA is
                   admitted to the OUTPUT this frame; F and V are gate-only —
                   Stage 4 and the DB only ever see raw R — and the pipeline
                   runs either way so a failing agent's reputation can still
                   recover. F comes from how far the sender's own tracked
                   speed could have carried it during THIS message's own
                   send-to-receive latency (reputation_multipliers.
                   sender_motion / .kinematic_freshness) — not from how long
                   since the sender's previous message, so a sender that was
                   silent for a while and then sends a message with
                   negligible latency reads as fully fresh.
    1. tracking  : stable track ids and Kalman state come from TrackData
                   (this engine's own SORT tracker, one per (judge, sender))
    2. matching  : one MS-PSF fusion over ego + every agent (ungated,
                   reputation-weighted); each agent's matched/ego_only/
                   other_only buckets are derived from the shared clusters,
                   and the other peers' reputation mass in a detection's
                   cluster drives corroboration
    3. consistency: matched/ego_only -> weighted (C, I) instantly (consistency.py);
                   other_only -> the deferred ledger (deferred.py), which
                   back-pays on corroboration and back-charges on expiry
    4. reputation : R_new from weighted (C, I); silent agents frozen
    5. admission : Stage-0 verdict in FrameStats.trusted gates the output; within
                   [tau, HIGH_TRUST) only corroborated objects pass, at/above
                   HIGH_TRUST everything passes (optimistic trust). R_new only
                   moves NEXT frame's gate.
"""

import logging
import math
import time
from typing import Callable, NamedTuple, Optional

import numpy as np

from sdsm_trust_perception.global_trust_perception.mmcooper_fuse.adapter import StreamInput, fuse
from sdsm_trust_perception.global_trust_perception.trust_calculations.consistency import (
    T_DEADLINE_S,
    corroboration_support,
    is_corroborated,
    weighted_ego_consistency,
)
from sdsm_trust_perception.global_trust_perception.trust_calculations.consistency_checks.attribute_checks import check_size_agreement
from sdsm_trust_perception.global_trust_perception.trust_calculations.consistency_checks.kinematic_checks import (
    KinematicHistory,
)
from sdsm_trust_perception.global_trust_perception.trust_calculations.deferred import LedgerDelta, PendingVerdicts
from sdsm_trust_perception.global_trust_perception.pipeline.persistent_reputation_tracker import (
    PersistentReputationTracker,
)
from sdsm_trust_perception.global_trust_perception.trust_calculations.reputation import (
    HIGH_TRUST,
    REPUTATION_DEFAULT,
    dynamic_threshold,
    reputation_update,
)
from sdsm_trust_perception.global_trust_perception.trust_calculations.reputation_multipliers.absence_decay import absence_decay
from sdsm_trust_perception.global_trust_perception.trust_calculations.reputation_multipliers.kinematic_freshness import (
    kinematic_freshness_factor,
)
from sdsm_trust_perception.global_trust_perception.trust_calculations.reputation_multipliers.persistence_penalty import (
    PersistencePenalty,
)
from sdsm_trust_perception.global_trust_perception.trust_calculations.reputation_multipliers.sender_motion import (
    SenderMotionHistory,
)

_log = logging.getLogger(__name__)


FRAME_WINDOW_MS = 500  # group SDSMs whose receive times fall within this
FRAME_FLUSH_HZ  = 2    # 1000 / FRAME_WINDOW_MS

# Cluster key for ego's own stream. Other agents are keyed by their integer
# agent_id, so a string can never collide.
_EGO_KEY = 'ego'

# --- Persistence-penalty tuning at the ENGINE's cadence -----------------------
# Trackers are fed once per flush (FRAME_FLUSH_HZ), not at the design doc's
# 25 fps module default, so the engine injects its own rate and sizes the
# memory in real time: accumulated risk halves V_HALF_LIFE_S seconds after
# drops stop (V_HALF_LIFE_S * flush_hz flushes -- 60 at the 2 Hz default).
V_HALF_LIFE_S = 30.0

# --- Stage 2b/2c diagnostic-only thresholds ------------------------------------
# KDS and SS are continuous and drive C/I directly (see Stage 3); these
# thresholds only decide what counts toward the logged kine_flagged/
# attr_correct diagnostics in FrameStats -- they do not gate anything.
# --- Probabilistic admission (opt-in: TrustEngine(admission='probabilistic')) ---
# An uncorroborated object is neither trusted nor damned by being uncorroborated:
# what matters is whether anyone COULD have witnessed it. A phantom in the
# middle of several vehicles that do not report it is contradicted; a real
# object out of everyone else's reach simply cannot be checked.
OPPORTUNITY_RANGE_M = 150.0    # a peer this close to an object could have seen it
ABSENCE_K = 1.0                # belief falls by exp(-K * missed-witness weight)
CONTRADICTED_MISSED_MIN = 1.0  # missed-witness weight that counts as contradicting
NEAR_REPORT_M = 10.0           # a witness reporting anything this close DID see it
                               # (its view just did not cluster: noise, timing)
KDS_IMPLAUSIBLE = 0.3          # kinematic score below this = physically implausible
P_ADMIT = 0.4                  # minimum belief to admit an uncorroborated object
LIAR_EWMA_ALPHA = 0.1          # per-flush weight of the newest frame in false_freq
LIAR_FREQ = 0.5                # false_freq at/above this = consistently false sender

# Fallback pairing after the fusion: an unmatched sender detection within this
# distance (m) of an unmatched judge detection is the same object. The fusion
# needs sub-metre agreement, but here both views are dead-reckoned beacon
# positions that routinely differ by a few metres (median ~3 m).
SOFT_MATCH_M = 6.0

# Stream key for the judge's own detections in the shared fusion (agent ids are
# node indices, so a string can never collide with one).
_EGO_KEY = 'ego'

_KDS_FLAG_THRESHOLD = 0.5
_SS_AGREE_THRESHOLD = 0.8

# Reputation gate on size-agreement credit: preserves the original
# check_size_agreement policy (other_rep > rep_gate required for 'correct'),
# now applied at the call site since attribute_checks.py is purely
# geometric. Below this, SS never contributes to matched_certainties.
_SIZE_REP_GATE = 0.70

# J2735 obj_type convention used elsewhere in this codebase: 1 = vehicle.
_VEHICLE_OBJ_TYPE = 1

# --- Per-detection verdict labels (Stage 5b, display/diagnostics only) --------
VERDICT_MATCHED        = 'matched'         # corroborated, by ego or by peers
VERDICT_DEFERRED       = 'deferred'        # held inside the ledger grace window
VERDICT_UNCORROBORATED = 'uncorroborated'  # grace window closed, being charged


def _vehicle_probs(classes: Optional[list], n: int) -> Optional[list]:
    """Per-detection P(vehicle) in {0.0, 1.0} from the raw reported obj_type
    label -- no statistics. See kinematic_checks module docstring.
    """
    if classes is None or len(classes) != n:
        return None
    return [1.0 if int(c) == _VEHICLE_OBJ_TYPE else 0.0 for c in classes]


class LedgerStats(NamedTuple):
    """The deferred-ledger contribution to one agent's frame."""

    pending: int
    backpaid: float
    backcharged: float
    settled_correct: int
    settled_incorrect: int
    support_max: float


class FrameStats(NamedTuple):
    """Per-agent result of one frame flush, for the node to log/publish."""

    agent_id: int
    n_total: float
    correct: float
    incorrect: float
    held: int
    s_frame: float
    r_old: float
    r_new: float
    tau: float
    trusted: bool
    kine_flagged: int
    attr_correct: int
    uncorroborated: int
    f_factor: float
    r_eff: float
    v_factor: float
    risk_persist: float
    ledger: LedgerStats
    tier: str
    admitted: tuple
    verdict: tuple
    matched_ego: tuple
    # probabilistic admission only (empty / 0.0 otherwise)
    p_obj: tuple = ()
    false_freq: float = 0.0
    # 'strict_unverified' admission only: objects from a trusted sender that nobody confirms
    # and nobody contradicts (a flag for the consumer, NOT in `admitted`)
    unverified: tuple = ()


class _Gate(NamedTuple):
    """Stage-0 verdict for one agent, private to process_frame."""

    r_old: float
    f: float
    v: float
    r_eff: float
    tau: float
    passed: bool


class TrustEngine:
    """
    Per-ego reputation tracker.

    Each judging node owns one TrustEngine. Every frame:
      - stable track ids and Kalman state come from this engine's own SORT
        tracker (fed by the trust_node from ReceivedSdsm events)
      - a sender's claim that an object EXISTS is taken at face value from the
        first frame it reports it: nothing here re-filters on tracker
        confirmation. The deferred ledger's grace window is the only delay
        before an uncorroborated report is judged.
    """

    def __init__(self, tracker: PersistentReputationTracker,
                 flush_hz: float = FRAME_FLUSH_HZ,
                 deadline_s: float = T_DEADLINE_S,
                 now: Callable[[], float] = time.monotonic,
                 high_trust: float = HIGH_TRUST,
                 admission: str = 'tiered',
                 contradiction_persist: int = 1,
                 opportunity_range_m: float = OPPORTUNITY_RANGE_M):
        self._now = now
        # Reputation at/above which ALL of a sender's objects are admitted without
        # corroboration (optimistic tier). Above 1.0 disables that tier, so every
        # admitted object must be corroborated regardless of the sender's standing.
        self._high_trust = float(high_trust)
        # 'tiered': HIGH_TRUST optimistic tier / corroborated-only below it.
        # 'probabilistic': per-object belief + consistent-liar filter (see
        # _object_beliefs); the optimistic tier is not used.
        # 'strict_unverified': strict corroborated-only `admitted`, plus an `unverified` class
        # (uncorroborated but uncontradicted, from a trusted sender) for the consumer.
        if admission not in ('tiered', 'probabilistic', 'strict_unverified'):
            raise ValueError(
                f"admission={admission!r} must be 'tiered', 'probabilistic' or 'strict_unverified'")
        self._admission = admission
        self._false_freq: dict[int, float] = {}
        # An object only counts as contradicted after being flagged this many
        # consecutive flushes (1 = immediately). Absent-witness evidence is noisy
        # (radio loss, timing), so a blip should not reject a real object.
        self._persist = max(1, int(contradiction_persist))
        self._streaks: dict[int, dict] = {}
        # How close a witness must be to an object to count as able to see it; set
        # this to the sensors' real range (a witness beyond it cannot contradict).
        self._opp_range = float(opportunity_range_m)
        if flush_hz <= 0.0:
            raise ValueError(f'flush_hz={flush_hz} must be > 0')
        self._flush_hz = float(flush_hz)
        self._tracker = tracker
        self.reputations: dict[int, float] = {}
        self._kin = KinematicHistory()
        self._sender_motion = SenderMotionHistory()
        self._last_t: Optional[float] = None
        self._persistence: dict[int, PersistencePenalty] = {}
        if deadline_s <= 0.0:
            raise ValueError(f'deadline_s={deadline_s} must be > 0')
        self._deadline_flushes = max(1, round(deadline_s * self._flush_hz))
        self._pending = PendingVerdicts(self._deadline_flushes)
        self._frame_idx = 0
        self._pending_risk_l: dict[int, tuple[float, float]] = {}

    def get_reputation(self, agent_id: int, source_id_str: Optional[str] = None) -> float:
        if agent_id not in self.reputations:
            seeded = None
            if source_id_str:
                row = self._tracker.get_last_reputation_with_ts(source_id_str)
                if row is not None:
                    r_last, ts_last, risk_l = row
                    gap_s = time.time() - ts_last
                    seeded = absence_decay(r_last, gap_s)
                    self._pending_risk_l[agent_id] = (risk_l, seeded)
                    _log.debug(
                        'Seeding agent=%s from DB: R_last=%.3f gap=%.1fs -> R_seeded=%.3f',
                        source_id_str, r_last, gap_s, seeded,
                    )
                elif self._tracker.get_last_reputation(source_id_str) is not None:
                    seeded = REPUTATION_DEFAULT
                    _log.warning(
                        'Agent %s has only pre-ts-migration history; '
                        'reseeding at default %.2f', source_id_str, seeded,
                    )
            if seeded is None:
                seeded = REPUTATION_DEFAULT
                if source_id_str:
                    self._tracker.record_initial(source_id_str, REPUTATION_DEFAULT)
            self.reputations[agent_id] = seeded
        return self.reputations[agent_id]

    def get_threshold(self, agent_id: int) -> float:
        """tau(R) - the gate this agent must currently clear."""
        return dynamic_threshold(self.get_reputation(agent_id))

    def is_trusted(self, agent_id: int) -> bool:
        """True iff the agent's current reputation clears its own dynamic threshold."""
        R = self.get_reputation(agent_id)
        return R >= dynamic_threshold(R)

    def weighted_score(self, agent_id: int, local_score: float) -> tuple[float, float, float]:
        """weighted_score = R(agent_id) * local_score. Returns (weighted, R, tau(R))."""
        R = self.get_reputation(agent_id)
        return R * local_score, R, dynamic_threshold(R)

    def _persistence_weight(self, agent_id: int, R_old: float) -> float:
        """Advance this agent's persistence tracker one flush; return V."""
        tracker = self._persistence.get(agent_id)
        if tracker is None:
            tracker = PersistencePenalty(half_life_s=V_HALF_LIFE_S,
                                         fps=self._flush_hz)
            self._persistence[agent_id] = tracker
            pending = self._pending_risk_l.pop(agent_id, None)
            if pending is not None:
                tracker.seed(*pending)
        return tracker.update(R_old)

    def _persistent(self, agent_id, keys, contradicted):
        """Apply the persistence rule: contradicted only after `_persist` consecutive
        flushes for the same object key (no keys -> no smoothing)."""
        if self._persist <= 1 or keys is None:
            return list(contradicted)
        prev = self._streaks.get(agent_id, {})
        cur, out = {}, []
        for key, c in zip(keys, contradicted):
            n = prev.get(key, 0) + 1 if c else 0
            cur[key] = n
            out.append(n >= self._persist)
        self._streaks[agent_id] = cur
        return out

    def _object_beliefs(self, agent_id, other_positions, kds, corroborated,
                        R_old, cluster_of, witnesses):
        """Per-object belief that the report is real, plus which are contradicted.

        Corroborated objects are certain (1.0). Any other object gets
        belief = R_old * kds * exp(-ABSENCE_K * missed), where `missed` is the
        reputation weight of witnesses (the judge counts 1.0) within
        OPPORTUNITY_RANGE_M of the object that did NOT report it (a witness that
        reports anything within NEAR_REPORT_M of it is not counted as missing). It is
        contradicted when that missed weight reaches CONTRADICTED_MISSED_MIN or
        its motion is implausible (kds < KDS_IMPLAUSIBLE); an object nobody
        could have seen is never contradicted.
        """
        beliefs, contradicted = [], []
        prior = min(max(R_old, 0.0), 1.0)
        for j, pos in enumerate(other_positions):
            if j in corroborated:
                beliefs.append(1.0)
                contradicted.append(False)
                continue
            reporters = cluster_of.get((agent_id, j), {})
            missed = 0.0
            for w, wx, wy, wkey, wxy in witnesses:
                if wkey == agent_id or wkey in reporters:
                    continue
                if math.hypot(pos[0] - wx, pos[1] - wy) > self._opp_range:
                    continue
                if len(wxy) and np.min(np.hypot(wxy[:, 0] - pos[0],
                                                wxy[:, 1] - pos[1])) <= NEAR_REPORT_M:
                    continue
                missed += w
            beliefs.append(prior * kds[j] * math.exp(-ABSENCE_K * missed))
            contradicted.append(missed >= CONTRADICTED_MISSED_MIN or kds[j] < KDS_IMPLAUSIBLE)
        return beliefs, contradicted

    def process_frame(self,
                      positions_by_agent: dict,
                      ego_positions: list,
                      dims_by_agent: Optional[dict] = None,
                      ego_dims: Optional[list] = None,
                      source_id_map: Optional[dict] = None,
                      tracks_by_agent: Optional[dict] = None,
                      ref_pos_by_agent: Optional[dict] = None,
                      scores_by_agent: Optional[dict] = None,
                      ego_scores: Optional[list] = None,
                      headings_by_agent: Optional[dict] = None,
                      ego_headings: Optional[list] = None,
                      equipment_by_agent: Optional[dict] = None,
                      ego_equipment: int = 0,
                      classes_by_agent: Optional[dict] = None,
                      ego_classes: Optional[list] = None,
                      object_ids_by_agent: Optional[dict] = None,
                      ego_ref_pos: Optional[tuple] = None) -> list[FrameStats]:
        """
        Run the trust pipeline over one frame. Same parameter meanings as the
        source file (see module docstring's CHANGES note for what differs),
        plus object_ids_by_agent, which only keys the deferred ledger (matching
        is spatial, by fusion). Missing/ragged -> the SORT track ids are used.
        ego_ref_pos is the judge's own (x, y); it counts as a witness for the
        probabilistic admission's absence evidence.
        """
        self._frame_idx += 1
        now = self._now()
        dt = (now - self._last_t) if self._last_t is not None else 0.5
        self._last_t = now

        _log.debug('Stage 1 [ego]   dets=%d', len(ego_positions))

        if not positions_by_agent:
            return []

        # --- STAGE 0 (gate precheck) --------------------------------------
        gate_info: dict[int, _Gate] = {}
        for agent_id in positions_by_agent:
            sid_tuple = (source_id_map or {}).get(agent_id)
            sid_str = ' '.join(f'{b:02x}' for b in sid_tuple) if sid_tuple else None
            R_old = self.get_reputation(agent_id, sid_str)
            ref = (ref_pos_by_agent or {}).get(agent_id)
            if ref is not None:
                ref_x, ref_y, recv_t, latency_s = ref
                speed = self._sender_motion.update(agent_id, ref_x, ref_y, recv_t)
                try:
                    F = kinematic_freshness_factor(speed, latency_s)
                except ValueError as exc:
                    _log.warning(
                        'Stage 0 [agent=%d]  unusable freshness measurement, '
                        'F=1.0 this frame: %s', agent_id, exc,
                    )
                    F = 1.0
            else:
                F = 1.0
            V = self._persistence_weight(agent_id, R_old)
            R_eff = R_old * F * V
            tau_gate = dynamic_threshold(R_eff)
            gate_passed = R_eff >= tau_gate
            gate_info[agent_id] = _Gate(R_old, F, V, R_eff, tau_gate, gate_passed)
            _log.debug(
                'Stage 0 [agent=%d]  gate precheck: R_old=%.3f F=%.3f V=%.3f '
                'R_eff=%.3f %s tau=%.3f -> %s',
                agent_id, R_old, F, V, R_eff, '>=' if gate_passed else '<', tau_gate,
                'ADMIT' if gate_passed else 'WITHHOLD (still scored)',
            )

        # --- STAGE 1b (kinematic) -----------------------------------------
        kds_by_agent: dict[int, list] = {}
        for agent_id, other_positions in positions_by_agent.items():
            agent_track_data = (tracks_by_agent or {}).get(agent_id)
            agent_vehicle_probs = _vehicle_probs(
                (classes_by_agent or {}).get(agent_id), len(other_positions),
            )
            if agent_track_data:
                kds_by_agent[agent_id] = self._kin.score_and_update(
                    agent_id, agent_track_data, _to_xy_array(other_positions), dt,
                    headings=(headings_by_agent or {}).get(agent_id),
                    vehicle_probs=agent_vehicle_probs,
                )
            else:
                kds_by_agent[agent_id] = [1.0] * len(other_positions)
            if len(kds_by_agent[agent_id]) != len(other_positions):
                kds_by_agent[agent_id] = [1.0] * len(other_positions)

        # --- STAGE 2 (matching): ONE MS-PSF fusion across ego + all agents,
        # UNGATED and weighted by reputation (ego = 1). Each cluster is
        # {stream_key: detection_idx}; per-agent buckets are derived from it
        # below, and corroboration_support() turns that membership into the
        # reputation mass behind each detection. Judging everyone (not just
        # gate-passers) is deliberate: an agent can't be scored against a
        # consensus it was excluded from forming.
        streams = [StreamInput(
            key=_EGO_KEY, positions=ego_positions, dims=ego_dims,
            headings=ego_headings, scores=ego_scores,
            modality=int(ego_equipment), reliability=1.0, labels=ego_classes,
        )]
        streams += [
            StreamInput(
                key=aid,
                positions=positions_by_agent[aid],
                dims=(dims_by_agent or {}).get(aid),
                headings=(headings_by_agent or {}).get(aid),
                scores=(scores_by_agent or {}).get(aid),
                modality=int((equipment_by_agent or {}).get(aid, 0)),
                reliability=gate_info[aid].r_old,
                kds=kds_by_agent[aid],
                labels=(classes_by_agent or {}).get(aid),
            )
            for aid in positions_by_agent
        ]
        clusters = fuse(streams, _EGO_KEY).clusters
        support = corroboration_support(
            clusters,
            {st.key: st.reliability for st in streams},
            {st.key: _row_or_default(st.scores, len(st.positions), 1.0) for st in streams},
            _EGO_KEY,
        )
        _log.debug('Stage 2 [all]  ego + %d agents -> %d consensus clusters',
                   len(positions_by_agent), len(clusters))

        ego_score_list = _row_or_default(ego_scores, len(ego_positions), 1.0)

        cluster_of: dict = {}
        witnesses: list = []
        if self._admission in ('probabilistic', 'strict_unverified'):
            for c in clusters:
                for key, det in c.items():
                    cluster_of[(key, det)] = c
            if ego_ref_pos is not None:
                witnesses.append((1.0, ego_ref_pos[0], ego_ref_pos[1], _EGO_KEY,
                                  _to_xy_array(ego_positions)))
            for aid, gi in gate_info.items():
                ref = (ref_pos_by_agent or {}).get(aid)
                if ref is not None and gi.passed:
                    witnesses.append((gi.r_old, ref[0], ref[1], aid,
                                      _to_xy_array(positions_by_agent[aid])))

        stats = []
        for agent_id, other_positions in positions_by_agent.items():
            R_old, F, V, R_eff, tau_gate, gate_passed = gate_info[agent_id]

            agent_track_data = (tracks_by_agent or {}).get(agent_id)
            other_ids = (object_ids_by_agent or {}).get(agent_id) or []

            # --- STAGE 2 cont.: derive this agent's buckets from the clusters -
            matched = [(c[_EGO_KEY], c[agent_id]) for c in clusters
                       if _EGO_KEY in c and agent_id in c]
            matched_ego = {i for i, _ in matched}
            matched_other = {j for _, j in matched}
            # Soft pairing of what the fusion left unmatched (nearest first, 1-to-1).
            free_i = [i for i in range(len(ego_positions)) if i not in matched_ego]
            free_j = [j for j in range(len(other_positions)) if j not in matched_other]
            if free_i and free_j:
                cand = sorted(
                    (math.hypot(ego_positions[i][0] - other_positions[j][0],
                                ego_positions[i][1] - other_positions[j][1]), i, j)
                    for i in free_i for j in free_j)
                for d, i, j in cand:
                    if d > SOFT_MATCH_M:
                        break
                    if i in matched_ego or j in matched_other:
                        continue
                    matched.append((i, j))
                    matched_ego.add(i)
                    matched_other.add(j)
            ego_only = [i for i in range(len(ego_positions)) if i not in matched_ego]
            other_only = [j for j in range(len(other_positions)) if j not in matched_other]
            _log.debug(
                'Stage 2 [agent=%d]  matched=%d  ego_only=%d  other_only=%d',
                agent_id, len(matched), len(ego_only), len(other_only),
            )

            # --- STAGE 2b (kinematic): continuous KDS discount --------------
            kds = kds_by_agent[agent_id]
            kine_flagged = sum(1 for _, oi in matched if kds[oi] < _KDS_FLAG_THRESHOLD)

            # --- STAGE 2c (attribute): continuous size-agreement score (SS) -
            other_dims_list = (dims_by_agent or {}).get(agent_id)
            ss_scores = check_size_agreement(matched, ego_dims or [], other_dims_list or [])
            attr_correct = sum(
                1 for ss in ss_scores
                if ss >= _SS_AGREE_THRESHOLD and R_old > _SIZE_REP_GATE
            )

            # --- STAGE 2d (corroboration): reputation-mass support per det ----
            # Support = sum of R * certainty over the OTHER peers in each
            # detection's cluster (ego excluded -- ego presence means matched,
            # not other_only). A detection is corroborated when its support
            # clears the threshold, so k low-R colluders cannot mutually
            # corroborate their ghosts, and a sender never vouches for itself.
            other_score_list = _row_or_default(
                (scores_by_agent or {}).get(agent_id), len(other_positions), 1.0
            )
            support_by_det = support.get(agent_id, {})
            corroborated = {
                j for j in range(len(other_positions))
                if j in matched_other or is_corroborated(support_by_det.get(j, 0.0))
            }
            uncorroborated = sum(
                1 for j in other_only
                if not is_corroborated(support_by_det.get(j, 0.0))
            )
            support_max = max((support_by_det.get(j, 0.0) for j in other_only), default=0.0)

            # --- STAGE 3 (consistency) ---------------------------------------
            matched_certainties = [
                other_score_list[oi] * kds[oi] for _, oi in matched
            ]
            ego_only_certainties = [ego_score_list[i] for i in ego_only]
            C, I, held = weighted_ego_consistency(matched_certainties, ego_only_certainties)

            track_ids = list(agent_track_data.track_ids) if agent_track_data else None
            if track_ids is not None and len(track_ids) != len(other_positions):
                track_ids = None
            # The ledger's grace clock must survive tracker re-IDs: a SORT track
            # that flaps to a new id resets the clock (a "death" inside the
            # window is forgiven), so an uncorroborated ghost would never reach
            # its deadline. This port already trusts the reported object_id for
            # matching, so the ledger is keyed by it too when the sender supplied
            # one per detection; SORT track ids remain the fallback.
            if len(other_ids) == len(other_positions) and other_positions:
                track_ids = list(other_ids)

            if track_ids is not None:
                ledger_obs = [
                    (track_ids[j], other_score_list[j], other_score_list[j] * kds[j],
                     support_by_det.get(j, 0.0))
                    for j in other_only
                ]
                vindicated_tids = [track_ids[oi] for _, oi in matched]
                delta = self._pending.observe(
                    agent_id, self._frame_idx, ledger_obs, vindicated_tids
                )
            else:
                held += len(other_only)
                delta = LedgerDelta()

            C += delta.correct
            I += delta.incorrect
            held += delta.pending
            N_total = C + I
            ledger_stats = LedgerStats(
                pending=delta.pending, backpaid=delta.correct,
                backcharged=delta.incorrect, settled_correct=delta.settled_correct,
                settled_incorrect=delta.settled_incorrect, support_max=support_max,
            )

            # --- STAGE 4 (reputation) ----------------------------------------
            R_new, s_frame = reputation_update(R_old, C, I, N_total, held)
            self.reputations[agent_id] = R_new

            # --- STAGE 5 (admission) ------------------------------------------
            trusted = gate_passed
            p_obj, false_freq = (), self._false_freq.get(agent_id, 0.0)
            contradicted = []
            unverified = ()
            if self._admission == 'strict_unverified':
                p_list, contradicted = self._object_beliefs(
                    agent_id, other_positions, kds, corroborated, R_old,
                    cluster_of, witnesses)
                p_obj = tuple(p_list)
                raw_contra = contradicted
                contradicted = self._persistent(agent_id, track_ids, raw_contra)
                if gate_passed:
                    unverified = tuple(
                        j for j in range(len(other_positions))
                        if j not in corroborated and not contradicted[j]
                        and (p_list[j] >= P_ADMIT or raw_contra[j]))
            if self._admission == 'probabilistic':
                p_list, contradicted = self._object_beliefs(
                    agent_id, other_positions, kds, corroborated, R_old,
                    cluster_of, witnesses)
                strike = 1.0 if any(contradicted) else 0.0
                false_freq = ((1.0 - LIAR_EWMA_ALPHA) * false_freq
                              + LIAR_EWMA_ALPHA * strike)
                self._false_freq[agent_id] = false_freq
                p_obj = tuple(p_list)
                trusted = gate_passed and false_freq < LIAR_FREQ
            if not trusted:
                tier, admitted = 'reject', ()
            elif self._admission == 'probabilistic':
                tier = 'probabilistic'
                admitted = tuple(
                    j for j in range(len(other_positions))
                    if j in corroborated or (not contradicted[j] and p_obj[j] >= P_ADMIT))
            elif self._admission == 'strict_unverified':
                tier, admitted = 'strict_unverified', tuple(sorted(corroborated))
            elif R_old >= self._high_trust:
                tier, admitted = 'optimistic', tuple(range(len(other_positions)))
            else:
                tier, admitted = 'corroborated', tuple(sorted(corroborated))

            verdicts = []
            for j in range(len(other_positions)):
                if j in corroborated:
                    verdicts.append(VERDICT_MATCHED)
                elif (track_ids is not None
                      and self._pending.status(agent_id, track_ids[j]) == 'expired'):
                    verdicts.append(VERDICT_UNCORROBORATED)
                else:
                    verdicts.append(VERDICT_DEFERRED)

            stats.append(FrameStats(
                agent_id=agent_id, n_total=N_total, correct=C, incorrect=I,
                held=held, s_frame=s_frame, r_old=R_old, r_new=R_new,
                tau=tau_gate, trusted=trusted, kine_flagged=kine_flagged,
                attr_correct=attr_correct, uncorroborated=uncorroborated,
                f_factor=F, r_eff=R_eff, v_factor=V,
                risk_persist=self._persistence[agent_id].risk_persist,
                ledger=ledger_stats, tier=tier, admitted=admitted,
                verdict=tuple(verdicts), matched_ego=tuple(sorted(matched_ego)),
                p_obj=p_obj, false_freq=false_freq, unverified=unverified,
            ))
        return stats


def _to_xy_array(positions: list) -> np.ndarray:
    """Convert list of (x,y,z) or (x,y) tuples to (N,2) float array."""
    if not positions:
        return np.empty((0, 2))
    return np.array([(p[0], p[1]) for p in positions], dtype=float)


def _row_or_default(seq: Optional[list], n: int, default) -> list:
    """Length-n list, substituting default when seq is absent or ragged."""
    if seq is None or len(seq) != n:
        return [default] * n
    return list(seq)
