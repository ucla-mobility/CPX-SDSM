"""
Trustworthy perception: the orchestrator (TrustEngine).

This module owns the per-frame pipeline but NOT the math behind any stage -
each calculation concern lives in its own module and is called from here:

    global_trust_tracker/tracker_node.py - Kalman temporal tracking (track ids)
    mmcooper_fuse/adapter.py              - cross-agent spatial matching + support
                                            (MS-PSF phase-1 fusion)
    consistency.py                        - verdict policy: matched/ego_only ->
                                            weighted (C, I, held); pen / threshold
    deferred.py                           - other_only grace/back-pay/expiry ledger
    reputation.py                         - dynamic threshold + reputation update

The engine is deliberately ROS-free: agent.py decodes SDSM messages into
global (x, y, z) positions and forwards TrackData (from the tracker node's
TrackUpdate topic) before passing them here, so the whole trust pipeline stays
unit-testable without rclpy.

TWO-PHASE DESIGN: this engine runs the phase-1 "judging" fusion — ego + every
agent, UNGATED, weighted by reputation — to decide who to reward and who to
dock. The phase-2 "output" fusion (over only the admitted agents) is not run
here; process_frame reports each agent's admitted detection indices in
FrameStats.admitted and the node fuses/forwards that set separately.

PIPELINE PER FRAME (ego-centric), for each other agent:
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
                   (tracker node output)
    2. matching  : ONE MS-PSF phase-1 fusion across ego + all agents; each
                   agent's matched/ego_only/other_only buckets are derived from
                   the shared clusters, and reputation-weighted support per
                   detection drives corroboration
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
import time
from typing import Callable, NamedTuple, Optional

import numpy as np

from global_trust_perception.trust_calculations.consistency import (
    T_DEADLINE_S,
    corroboration_support,
    is_corroborated,
    weighted_ego_consistency,
)
from global_trust_perception.trust_calculations.consistency_checks.attribute_checks import check_size_agreement
from global_trust_perception.trust_calculations.consistency_checks.kinematic_checks import (
    KinematicHistory,
)
from global_trust_perception.trust_calculations.deferred import LedgerDelta, PendingVerdicts
from global_trust_perception.mmcooper_fuse.adapter import StreamInput, fuse
from global_trust_perception.pipeline.persistent_reputation_tracker import (
    PersistentReputationTracker,
)
from global_trust_perception.trust_calculations.reputation import (
    HIGH_TRUST,
    REPUTATION_DEFAULT,
    dynamic_threshold,
    reputation_update,
)
from global_trust_perception.trust_calculations.reputation_multipliers.absence_decay import absence_decay
from global_trust_perception.trust_calculations.reputation_multipliers.kinematic_freshness import (
    kinematic_freshness_factor,
)
from global_trust_perception.trust_calculations.reputation_multipliers.persistence_penalty import (
    PersistencePenalty,
)
from global_trust_perception.trust_calculations.reputation_multipliers.sender_motion import (
    SenderMotionHistory,
)

_log = logging.getLogger(__name__)


FRAME_WINDOW_MS = 500  # group SDSMs whose receive times fall within this
FRAME_FLUSH_HZ  = 2    # 1000 / FRAME_WINDOW_MS

# Cluster key for ego's own stream in the phase-1 fusion. Other agents are keyed
# by their integer agent_id, so a string can never collide.
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
_KDS_FLAG_THRESHOLD = 0.5
_SS_AGREE_THRESHOLD = 0.8

# Reputation gate on size-agreement credit: preserves the original
# check_size_agreement policy (other_rep > rep_gate required for 'correct'),
# now applied at the call site since attribute_checks.py is purely
# geometric. Below this, SS never contributes to matched_certainties.
_SIZE_REP_GATE = 0.70

# J2735 obj_type convention used elsewhere in this codebase (scene_node.py,
# sim_world.py): 1 = vehicle.
_VEHICLE_OBJ_TYPE = 1

# --- Per-detection verdict labels (Stage 5b, display/diagnostics only) --------
# Strings, matching FrameStats.tier's convention rather than magic integers.
# The compact wire encoding is the ROS codec's business (pipeline/
# trust_verdicts.py), which keeps this engine ROS-free -- nothing here may
# import a message type.
VERDICT_MATCHED        = 'matched'         # corroborated, by ego or by peers
VERDICT_DEFERRED       = 'deferred'        # held inside the ledger grace window
VERDICT_UNCORROBORATED = 'uncorroborated'  # grace window closed, being charged


def _vehicle_probs(classes: Optional[list], n: int) -> Optional[list]:
    """Per-detection P(vehicle) in {0.0, 1.0} from the raw reported obj_type
    label -- no statistics. SDSM doesn't carry a genuine per-class
    probability vector, only a hard label (obj_type) plus a general
    detection-confidence scalar (obj_local_scores) that measures something
    different (whether an object exists at all) from classification
    confidence. An earlier version of this function treated that scalar as
    if it graded classification uncertainty, mirroring
    mmcooper_fuse.fusion.build_class_probs's synthesis for c_i -- an
    unjustified proxy once traced back to what the scalar actually measures,
    so this trusts the reported label directly instead. Float-typed (not
    bool) so an affine floor (P(vehicle) = beta + (1-beta)*hard_indicator,
    disclosed as a deliberate addition, not the paper's) could be added here
    later without changing the interface.

    Returns None (score_and_update then assumes P(vehicle)=1 for every
    detection, the same conservative default used before class data was
    wired into this pipeline) if class labels aren't available or ragged.
    """
    if classes is None or len(classes) != n:
        return None
    return [1.0 if int(c) == _VEHICLE_OBJ_TYPE else 0.0 for c in classes]


class LedgerStats(NamedTuple):
    """The deferred-ledger contribution to one agent's frame, grouped so the
    growing set of ledger diagnostics is one FrameStats field, not five loose
    ones (keeps FrameStats' public contract stable as the ledger evolves)."""

    pending: int             # other_only tracks still held this frame
    backpaid: float          # credit added to C by ledger settlements this frame
    backcharged: float       # penalty added to I by ledger settlements this frame
    settled_correct: int     # tracks settled Correct this frame (diag)
    settled_incorrect: int   # tracks settled/charged Incorrect this frame (diag)
    support_max: float       # max peer reputation-mass support among this
                             # agent's other_only detections (diag)


class FrameStats(NamedTuple):
    """Per-agent result of one frame flush, for the node to log."""

    agent_id: int
    n_total: float
    correct: float
    incorrect: float
    held: int       # seen-but-not-scored this frame (forgiven misses / ledger)
    s_frame: float
    r_old: float
    r_new: float
    tau: float      # tau(R_eff) — the gate checked at Stage 0 this frame
    trusted: bool   # Stage-0 gate: R_eff >= tau(R_eff). False = this agent's
                    # objects are withheld from the OUTPUT this frame; its
                    # reputation still updates normally
    kine_flagged: int  # matched pairs demoted by kinematic check (logged only)
    attr_correct: int  # pairs where size agreed and rep gate passed (logged only)
    uncorroborated: int  # other_only detections whose peer support did not
                         # clear the threshold this frame (fed to the ledger)
    f_factor: float      # kinematic freshness factor F for this frame's
                         # message(s); 1.0 when no sender position data was
                         # provided, or when the latency measurement was
                         # unusable. Transient: F is applied only at the
                         # Stage-0 gate, never to r_new/DB
    r_eff: float         # R_old * f_factor * v_factor — the effective reputation
                         # the Stage-0 gate actually compared against tau(r_eff)
    v_factor: float      # persistence-penalty weight V; 1.0 while this agent's
                         # reputation is not falling. Gate-only like f_factor —
                         # never applied to r_new/DB; the memory of past drops
                         # lives in the tracker's risk accumulator, not in R
    risk_persist: float  # the persistence channel risk_L in [0, 1] after this
                         # frame (observability: how much drop-debt is banked)
    ledger: LedgerStats  # deferred other_only verdict contribution this frame
    tier: str            # output tier: 'reject' | 'corroborated' | 'optimistic'
    admitted: tuple      # this agent's detection indices admitted to the OUTPUT
                         # (phase-2) fusion this frame, per the tier policy
    verdict: tuple       # per-detection VERDICT_* label, parallel to this
                         # agent's detections. DISPLAY ONLY -- no stage reads
                         # it; it reports what the stages above already decided
    matched_ego: tuple   # ego detection indices this agent corroborated this
                         # frame, so a viewer can colour ego's OWN boxes
                         # without re-deriving cluster membership itself


class _Gate(NamedTuple):
    """Stage-0 verdict for one agent, private to process_frame.

    Named fields instead of a bare tuple: the corroboration pass reads
    .passed for OTHER agents long after the per-agent unpack, and a
    positional index there is a silent-transposition hazard.
    """

    r_old: float
    f: float        # kinematic freshness factor for this frame's message(s)
    v: float        # persistence-penalty weight from this agent's tracker
    r_eff: float    # r_old * f * v — what the gate actually compares
    tau: float      # dynamic_threshold(r_eff)
    passed: bool


class TrustEngine:
    """
    Per-ego reputation tracker.

    Each agent node owns one TrustEngine. Every frame:
      - stable track ids and Kalman state arrive as TrackData (from the tracker
        node via agent.py); when absent, other_only detections are simply held
        (no track id => no ledger grading)
      - all streams (ego + every agent) feed one MS-PSF phase-1 clustering;
        per-agent buckets and witness counts are derived from the clusters
      - a sender's claim that an object EXISTS is taken at face value from the
        first frame it reports it: the local pipeline has already dropped what
        its sender was unsure of, so nothing here re-filters on tracker
        confirmation. The deferred ledger's grace window is the only delay
        before an uncorroborated report is judged.
    """

    def __init__(self, tracker: PersistentReputationTracker,
                 flush_hz: float = FRAME_FLUSH_HZ,
                 deadline_s: float = T_DEADLINE_S,
                 now: Callable[[], float] = time.monotonic):
        # WHERE TIME COMES FROM. `now` supplies the seconds this engine measures
        # dt with, injected rather than read from the `time` module so the
        # caller decides which clock the pipeline lives on -- agent.py passes
        # the ROS node clock, which under use_sim_time follows the BAG's clock
        # rather than the wall. That is what makes a judgement independent of
        # playback rate: at `ros2 bag play --rate 0.5` the wall takes twice as
        # long, but scene time (and therefore every window derived from it)
        # advances identically, so slowing playback down to watch it cannot
        # change what the engine decides. The default keeps this module
        # ROS-free and every non-ROS caller on the monotonic wall clock.
        self._now = now
        # The persistence penalty decays once per flush, so its half-life is
        # only correct in wall-clock terms if it knows the real flush rate --
        # a caller flushing at 20 Hz while this defaulted to 2 would stretch
        # V_HALF_LIFE_S tenfold. Injected rather than read from the module
        # constant so the engine's cadence is the caller's to state.
        if flush_hz <= 0.0:
            raise ValueError(f'flush_hz={flush_hz} must be > 0')
        self._flush_hz = float(flush_hz)
        self._tracker = tracker
        self.reputations: dict[int, float] = {}
        self._kin = KinematicHistory()
        self._sender_motion = SenderMotionHistory()
        self._last_t: Optional[float] = None
        self._persistence: dict[int, PersistencePenalty] = {}
        # deferred verdicts for uncorroborated other_only detections.
        # The ledger counts in flushes, so the wall-clock window is converted
        # here using this engine's own rate -- the same reason flush_hz is
        # injected for the persistence penalty above.
        if deadline_s <= 0.0:
            raise ValueError(f'deadline_s={deadline_s} must be > 0')
        self._deadline_flushes = max(1, round(deadline_s * self._flush_hz))
        self._pending = PendingVerdicts(self._deadline_flushes)
        self._frame_idx = 0  # monotonic flush counter for the ledger's deadlines
        # The most recent frame's fusion, held for the READ-ONLY viz path and
        # nothing else -- see the last_fusion property. Assigned, never copied:
        # a judged frame builds this object regardless, so holding the
        # reference for one frame costs an attribute store and no compute. A
        # peerless frame is the exception -- see fuse_solo_frames below.
        self._last_fusion = None
        # Opt-in: also fuse frames that carry no peer, for the viz path only
        # (see the peerless branch in process_frame). Off by default so a
        # headless run pays nothing; the agent turns it on while something is
        # subscribed to the fused scene.
        self.fuse_solo_frames = False
        # (risk_l, last_r) loaded from DB in get_reputation(), consumed once by
        # _persistence_weight() when the tracker is first constructed.
        # last_r is set to R_seeded so the first update() produces dnorm = 0.
        self._pending_risk_l: dict[int, tuple[float, float]] = {}

    def get_reputation(self, agent_id: int, source_id_str: Optional[str] = None) -> float:
        if agent_id not in self.reputations:
            seeded = None
            if source_id_str:
                row = self._tracker.get_last_reputation_with_ts(source_id_str)
                if row is not None:
                    r_last, ts_last, risk_l = row
                    # DELIBERATELY the wall clock, not self._now: this measures
                    # how long ago a reputation was WRITTEN TO DISK, which is
                    # real elapsed time between runs (possibly days) and has
                    # nothing to do with scene time inside any one recording.
                    # A sim clock here would read a fresh bag as "no time has
                    # passed since last week's run".
                    gap_s = time.time() - ts_last
                    seeded = absence_decay(r_last, gap_s)
                    self._pending_risk_l[agent_id] = (risk_l, seeded)
                    _log.debug(
                        'Seeding agent=%s from DB: R_last=%.3f gap=%.1fs -> R_seeded=%.3f',
                        source_id_str, r_last, gap_s, seeded,
                    )
                elif self._tracker.get_last_reputation(source_id_str) is not None:
                    # Legacy agent: history exists but predates the ts column,
                    # so the absence gap is unknowable. POLICY: reset to the
                    # neutral default rather than trusting stale history of
                    # unknown age. No record_initial here — the agent is
                    # already registered; a duplicate initial row would fake
                    # a brand-new agent on top of real history.
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
        """tau(R) - the gate this agent must currently clear (reputation.dynamic_threshold)."""
        return dynamic_threshold(self.get_reputation(agent_id))

    def is_trusted(self, agent_id: int) -> bool:
        """True iff the agent's current reputation clears its own dynamic threshold.

        Uses raw R only: freshness is a per-message property and applies solely
        inside process_frame's Stage-0 gate, not to this agent-level view.
        """
        R = self.get_reputation(agent_id)
        return R >= dynamic_threshold(R)

    def weighted_score(self, agent_id: int, local_score: float) -> tuple[float, float, float]:
        """
        weighted_score = R(agent_id) * local_score

        Returns (weighted, R, tau(R)) so the caller can log all three.
        """
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

    @property
    def last_fusion(self):
        """The most recent frame's FusionResult, or None -- VIZ ONLY.

        On a frame with no peer this is ego's own detections fused alone, when
        fuse_solo_frames is set; otherwise such a frame leaves it None.

        Exists so a visualizer can draw the fused boxes without re-running (or
        re-deriving) the fusion. It is NOT part of the trust decision and
        nothing in the production path may read it: the decision consumes
        `clusters`, while the geometry, scores and contributors here are the
        display derivations fuse() computes anyway (mmcooper_fuse/adapter.py).

        Returned by reference, not copied, so it costs nothing to keep -- and
        must therefore be treated as read-only by the caller.

        TEMPORALLY COUPLED: valid only for the frame just processed. It is
        reset to None at the top of every process_frame, so a caller that reads
        it late gets None instead of a previous frame's boxes. Read it
        immediately after the process_frame call that produced it.
        """
        return self._last_fusion

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
                      ego_classes: Optional[list] = None) -> list[FrameStats]:
        """
        Run the trust pipeline over one frame.

        positions_by_agent : dict[agent_id -> list of global (x,y,z)] from OTHER agents
        ego_positions      : list of global (x,y,z) this car detected
        dims_by_agent      : dict[agent_id -> list of (w,l,h) in metres]
        ego_dims           : list of (w,l,h) in metres matching ego_positions
        source_id_map      : dict[agent_id -> 4-tuple] full SDSM source_id per agent
        tracks_by_agent    : dict[agent_id -> TrackData] for other agents.
                             Ego passes none of its own: ego gets no
                             self-check (kinematic scoring has only ever
                             gated OTHER agents' corroboration of ego, see
                             Stage 1b) and is never graded by the ledger, so
                             ego track state has no consumer here.
        ref_pos_by_agent   : dict[agent_id -> (ref_x, ref_y, recv_t, latency_s)]
                             this frame's sender reference position, ego's
                             receive wall-clock time (feeds sender_motion's
                             speed estimate), and this message's own
                             send-to-receive latency (feeds the blind-distance
                             calculation) — see reputation_multipliers.
                             sender_motion / .kinematic_freshness. Missing
                             agent (or None dict) means F=1, i.e. fully fresh
                             — so callers without this data get today's
                             behaviour.
        scores_by_agent    : dict[agent_id -> list of local certainty [0,1]] per
                             detection. Missing -> 1.0 (fully certain), so callers
                             without scores get uniform-confidence behaviour.
        ego_scores         : ego's per-detection certainty (miss penalties weigh
                             pen(ego_score)); missing -> 1.0.
        headings_by_agent  : dict[agent_id -> list of heading degrees] per
                             detection; missing -> 0 (axis-aligned box).
        ego_headings       : ego's per-detection headings; missing -> 0.
        equipment_by_agent : dict[agent_id -> equipment_type int] -> MS-PSF
                             modality; missing -> 0.
        ego_equipment      : ego's equipment_type; default 0.
        classes_by_agent   : dict[agent_id -> list of J2735 obj_type int] per
                             detection. Feeds StreamInput.labels (MS-PSF's own
                             class-probability vector c_i, eq 3.19) and, via
                             _vehicle_probs, the hard P(vehicle) gate in
                             KinematicHistory.score_and_update (trusts the
                             reported label directly, no statistics -- see
                             _vehicle_probs) -- missing -> P(vehicle)=1 for
                             every detection (the same conservative default
                             used before class data was wired into this
                             pipeline), and StreamInput.labels falls back to
                             its own default (class 0 for all).
        ego_classes        : ego's per-detection J2735 obj_type; missing -> 0.

        When tracks_by_agent is None (e.g. in tests or before the tracker node
        has published), other_only detections are simply held (no stable track
        id => no ledger grading).
        """
        self._frame_idx += 1
        # Cleared up front so last_fusion is never a previous frame's answer:
        # a caller that reads it after a frame which never reached the fusion
        # stage gets None rather than something stale to publish.
        self._last_fusion = None
        now = self._now()
        dt = (now - self._last_t) if self._last_t is not None else 0.5
        self._last_t = now

        # Logged before the early return below: with a single sender (ego echo
        # only) there are no peers to judge and no later stage runs, so this is
        # the one line that shows ingest and tracking are alive.
        _log.debug('Stage 1 [ego]   dets=%d', len(ego_positions))

        # Ego's own stream, built here rather than at stage 2 so the peerless
        # branch below can fuse it alone. Stage 2 uses this same object, so the
        # judged fusion and the solo one cannot drift apart.
        ego_stream = StreamInput(
            key=_EGO_KEY, positions=ego_positions, dims=ego_dims,
            headings=ego_headings, scores=ego_scores,
            modality=int(ego_equipment), reliability=1.0,
            labels=ego_classes,
        )

        if not positions_by_agent:
            # No peer in this bucket: nothing to judge, so the trust path ends
            # here exactly as before -- no gate, no ledger, no reputation move.
            # The VIZ path still needs an answer. Without one, last_fusion
            # stays None, fused_objects.build() encodes num_objects=0 and the
            # viewer DELETEs the whole fused layer -- so every bucket a peer's
            # message misses blinks the boxes off and the next one back on.
            # Ego alone fuses to ego's own boxes, which IS what ego acts on
            # with nothing to corroborate, and it is CURRENT rather than the
            # previous frame held over.
            if self.fuse_solo_frames:
                self._last_fusion = fuse([ego_stream], _EGO_KEY)
            return []

        # --- STAGE 0 (gate precheck), for EVERY agent before anything else:
        # effective reputation R_eff = R_old * F * V (kinematic freshness *
        # persistence penalty) vs its threshold tau(R_eff). Decides whether
        # each agent's DATA is admitted downstream this frame, and defines the
        # witness set for corroboration (only gate-passing agents can
        # corroborate). Both multipliers are gate-only — they never touch
        # self.reputations, Stage 4, or the DB. F is per-message (a sender
        # whose tracked speed times this message's own transmission latency
        # implies a large blind distance costs only this frame's admission);
        # V carries memory of sustained reputation FALLS in its own risk
        # accumulator, so an on-off attacker stays penalised between hits
        # even while raw R rebounds.
        # The full pipeline below runs regardless, so reputation keeps
        # updating and a failing agent can earn its way back.
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
                    # A latency past the clock-skew tolerance says this node's
                    # own measurement is unusable -- it is not a verdict about
                    # the sender, which did nothing differently. So it falls
                    # back to the same neutral F the no-ref branch below uses
                    # ("no usable freshness evidence this frame") rather than
                    # propagating: left as a raise it kills the flush timer's
                    # callback and with it the whole node, costing every
                    # downstream view the rest of the run to punish one bad
                    # sample. Measured on a clean run, 151 of 151 messages
                    # landed at +0.5..+17 ms, so this is a rare tail and not a
                    # bias -- WARNING, not debug, so a systematic timestamping
                    # regression still announces itself as a stream of these
                    # rather than hiding as one neutral frame.
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

        # --- STAGE 1b (kinematic): per-detection KDS from Kalman track
        # history (kinematic_checks.score_and_update), computed BEFORE Stage 2
        # since KDS is now a fusion input (eq 2.8's orientation weighting),
        # not a post-hoc filter -- it has to exist before fuse() runs. Ego
        # gets no self-check: kinematic scoring has only ever gated OTHER
        # agents' corroboration of ego, never ego's own detections, so ego's
        # KDS defaults to neutral (1.0), consistent with its reliability=1.0.
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

        # --- STAGE 2 (matching): ONE MS-PSF phase-1 fusion across ego + all
        # agents, UNGATED and weighted by reputation (ego = 1). Each cluster is
        # {stream_key: detection_idx}; per-agent buckets are derived from it
        # below, and corroboration_support() turns that membership into the
        # reputation mass behind each detection. Judging everyone (not just
        # gate-passers) is deliberate: an agent can't be scored against consensus
        # if it was excluded from forming it.
        streams = [ego_stream]
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
        fusion = fuse(streams, _EGO_KEY)
        # Held for the viz path only (see last_fusion). The trust decision below
        # reads `clusters` and nothing else; fusion's box geometry, scores and
        # contributors are display derivations fuse() computes and discards.
        self._last_fusion = fusion
        clusters = fusion.clusters
        # Value the consensus grouping as corroboration (trust policy, so it
        # lives in consistency). Derived from `streams` rather than the caller's
        # raw dicts so the reliabilities and certainties are exactly the ones
        # the fusion itself saw.
        support = corroboration_support(
            clusters,
            {s.key: s.reliability for s in streams},
            {s.key: _row_or_default(s.scores, len(s.positions), 1.0) for s in streams},
            _EGO_KEY,
        )
        _log.debug(
            'Stage 2 [all]  ego + %d agents -> %d consensus clusters',
            len(positions_by_agent), len(clusters),
        )

        ego_score_list = _row_or_default(ego_scores, len(ego_positions), 1.0)

        stats = []
        for agent_id, other_positions in positions_by_agent.items():
            R_old, F, V, R_eff, tau_gate, gate_passed = gate_info[agent_id]

            agent_track_data = (tracks_by_agent or {}).get(agent_id)

            # --- STAGE 2 cont.: derive this agent's buckets from the clusters -
            matched = [(c[_EGO_KEY], c[agent_id]) for c in clusters
                       if _EGO_KEY in c and agent_id in c]
            matched_ego   = {e for e, _ in matched}
            matched_other = {o for _, o in matched}
            ego_only   = [i for i in range(len(ego_positions)) if i not in matched_ego]
            other_only = [j for j in range(len(other_positions)) if j not in matched_other]
            _log.debug(
                'Stage 2 [agent=%d]  matched=%d  ego_only=%d  other_only=%d',
                agent_id, len(matched), len(ego_only), len(other_only),
            )

            # --- STAGE 2b (kinematic): continuous KDS discount, not demotion --
            # Matched pairs stay matched (KDS was already consumed as a
            # fusion input at Stage 2, upstream); a low-KDS detection counts
            # for less corroboration below instead of being excluded outright.
            kds = kds_by_agent[agent_id]
            kine_flagged = sum(1 for _, oi in matched if kds[oi] < _KDS_FLAG_THRESHOLD)
            _log.debug(
                'Stage 2b [agent=%d]  kine_flagged(<%.2f)=%d  matched=%d',
                agent_id, _KDS_FLAG_THRESHOLD, kine_flagged, len(matched),
            )

            # --- STAGE 2c (attribute): continuous size-agreement score (SS) --
            # SS is diagnostic-only, same as the retired boolean verdict was
            # (see consistency_checks/README.md's "does not currently feed
            # into reputation scoring") -- it is NEVER multiplied into
            # matched_certainties below. Reputation gating now lives here
            # (attribute_checks.py is purely geometric) purely to decide what
            # counts toward the attr_correct diagnostic, preserving the
            # original policy of only crediting size agreement once
            # other_rep clears _SIZE_REP_GATE.
            other_dims_list = (dims_by_agent or {}).get(agent_id)
            ss_scores = check_size_agreement(matched, ego_dims or [], other_dims_list or [])
            attr_correct = sum(
                1 for ss in ss_scores
                if ss >= _SS_AGREE_THRESHOLD and R_old > _SIZE_REP_GATE
            )
            _log.debug(
                'Stage 2c [agent=%d]  attr_correct=%d/%d',
                agent_id, attr_correct, len(matched),
            )

            # --- STAGE 2d (corroboration): reputation-mass support per det ----
            # Support = sum of R * certainty over the OTHER peers in each
            # detection's phase-1 cluster (ego excluded — ego presence means
            # matched, not other_only). A detection is corroborated when its
            # support clears the threshold. This replaces binary witness-counting
            # with reputation-weighted mass, so k low-R colluders cannot mutually
            # corroborate their ghosts.
            support_by_det = support.get(agent_id, {})
            other_score_list = _row_or_default(
                (scores_by_agent or {}).get(agent_id), len(other_positions), 1.0
            )

            matched_other_final = {o for _, o in matched}
            corroborated = {
                j for j in range(len(other_positions))
                if j in matched_other_final or is_corroborated(support_by_det.get(j, 0.0))
            }
            uncorroborated = sum(
                1 for j in other_only
                if not is_corroborated(support_by_det.get(j, 0.0))
            )
            support_max = max((support_by_det.get(j, 0.0) for j in other_only), default=0.0)
            _log.debug(
                'Stage 2d [agent=%d]  uncorroborated=%d  support_max=%.3f',
                agent_id, uncorroborated, support_max,
            )

            # --- STAGE 3 (consistency): matched/ego_only weighted instantly;
            # other_only routed through the deferred ledger ------------------
            # KDS discounts how much a matched pair counts as corroborating
            # evidence -- this is the ONLY place it touches reputation; a low
            # KDS here lowers this frame's C, which lowers S_frame and
            # therefore R_new (reputation_update below). Its use in fusion
            # (mspsf's orientation weighting) is separate and never reaches
            # this computation. SS does NOT enter here -- see Stage 2c.
            matched_certainties = [
                other_score_list[oi] * kds[oi] for _, oi in matched
            ]
            ego_only_certainties = [ego_score_list[i] for i in ego_only]
            C, I, held = weighted_ego_consistency(matched_certainties, ego_only_certainties)

            # A stable track id is required to grade other_only over time; without
            # track data every other_only is simply held (no ledger).
            track_ids = list(agent_track_data.track_ids) if agent_track_data else None
            if track_ids is not None and len(track_ids) != len(other_positions):
                track_ids = None

            if track_ids is not None:
                # All other_only enter the ledger from their first sighting: a
                # sender's claim that an object exists is taken at face value
                # the moment it makes it, and the ledger's grace window is the
                # only delay before that claim is judged.
                # KDS discounts the CREDIT term only (other_score * kds), same
                # formula as the matched path -- the ledger's raw certainty
                # term stays undiscounted so a kinematically implausible ghost
                # is never penalised more softly than an equally-fabricated
                # but "smooth" one (see deferred.py's _Entry docstring).
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
                held += len(other_only)     # no track ids -> all held, no grading
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
            _log.debug(
                'Stage 3 [agent=%d]  C=%.3f  I=%.3f  held=%d  -> N_total=%.3f '
                '(ledger +C=%.3f +I=%.3f pend=%d)',
                agent_id, C, I, held, N_total,
                delta.correct, delta.incorrect, delta.pending,
            )

            # --- STAGE 4 (reputation): frame score + reputation update --------
            # held is passed so the baseline drift can tell "sent nothing" from
            # "sent plenty, all still inside the ledger's grace window" -- both
            # give N_total == 0, and only the first should pull R to neutral.
            R_new, s_frame = reputation_update(R_old, C, I, N_total, held)
            self.reputations[agent_id] = R_new
            _log.debug(
                'Stage 4 [agent=%d]  S_frame=%.3f  R: %.3f -> %.3f',
                agent_id, s_frame, R_old, R_new,
            )

            # --- STAGE 5 (admission): tier policy on the OUTPUT ---------------
            # trusted carries the Stage-0 decision; R_new only moves NEXT frame's
            # gate. The output tier decides which detections leave the pipeline:
            #   reject      : gate failed -> nothing
            #   optimistic  : R_old >= HIGH_TRUST -> every detection (proven
            #                 sensor acts immediately, no corroboration wait)
            #   corroborated: [tau, HIGH_TRUST) -> only corroborated detections
            #                 (a fresh R=0.5 Sybil's ghosts are muted at once,
            #                  while still being graded by the ledger above)
            if not gate_passed:
                tier, admitted = 'reject', ()
            elif R_old >= HIGH_TRUST:
                tier, admitted = 'optimistic', tuple(range(len(other_positions)))
            else:
                tier, admitted = 'corroborated', tuple(sorted(corroborated))
            _log.debug(
                'Stage 5 [agent=%d]  tier=%s admitted=%d/%d  (F=%.3f V=%.3f '
                'R_eff=%.3f; next frame: R_new=%.3f vs tau=%.3f)',
                agent_id, tier, len(admitted), len(other_positions),
                F, V, R_eff, R_new, dynamic_threshold(R_new),
            )
            # --- STAGE 5b (display): per-detection verdict label ---------------
            # Diagnostics only -- no stage reads this back. Derived from state
            # the stages above already computed, plus the ledger's own view of
            # each track, so a displayed verdict can never disagree with what
            # actually drove reputation. Runs after the ledger's observe() so
            # a track that expired THIS frame already reads as expired.
            verdicts = []
            for j in range(len(other_positions)):
                if j in corroborated:
                    verdicts.append(VERDICT_MATCHED)
                elif (track_ids is not None
                      and self._pending.status(agent_id, track_ids[j]) == 'expired'):
                    verdicts.append(VERDICT_UNCORROBORATED)
                else:
                    # Held: inside the grace window, or untracked (no ledger).
                    verdicts.append(VERDICT_DEFERRED)

            stats.append(FrameStats(
                agent_id=agent_id, n_total=N_total, correct=C, incorrect=I,
                held=held, s_frame=s_frame, r_old=R_old, r_new=R_new,
                tau=tau_gate, trusted=gate_passed, kine_flagged=kine_flagged,
                attr_correct=attr_correct, uncorroborated=uncorroborated,
                f_factor=F, r_eff=R_eff, v_factor=V,
                risk_persist=self._persistence[agent_id].risk_persist,
                ledger=ledger_stats, tier=tier, admitted=admitted,
                verdict=tuple(verdicts), matched_ego=tuple(sorted(matched_ego)),
            ))
        return stats


def _to_xy_array(positions: list) -> np.ndarray:
    """Convert list of (x,y,z) or (x,y) tuples to (N,2) float array."""
    if not positions:
        return np.empty((0, 2))
    return np.array([(p[0], p[1]) for p in positions], dtype=float)


def _row_or_default(seq: Optional[list], n: int, default) -> list:
    """Length-n list, substituting default when seq is absent or ragged.

    Mirrors the codec's defensive decode: a missing or mismatched per-detection
    list must never misindex against the position list it parallels.
    """
    if seq is None or len(seq) != n:
        return [default] * n
    return list(seq)
