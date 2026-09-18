"""
Integration tests: kinematic freshness factor F at the Stage-0 gate of
process_frame.

test_kinematic_freshness.py covers the F formula in isolation and
test_sender_motion.py covers the speed estimation in isolation; these tests
prove the WIRING -- that F reaches the gate, that it changes only the gate,
and that the stored reputation trajectory is provably identical with and
without it.

Scenario mirrors test_basic_pipeline.py: ego = agent 1 sees two fixed boxes,
neighbour = agent 2 reports positions, and we drive process_frame directly.
The neighbour always AGREES (reports both boxes), so any withheld verdict in
these tests is attributable to freshness alone, never to bad data.

Latency values are derived from the kinematic_freshness module's own
constants (GRACE_M, RHO_DIST, F_FLOOR, R_STAR), not pinned literals.

Producing an exact, controllable blind distance: most tests here install
_FixedSpeed in place of the engine's real SenderMotionHistory, so
blind_distance = SPEED * latency is controlled purely by the latency passed
in ref_pos_by_agent, independent of any Kalman convergence behaviour (which
is what test_sender_motion.py covers). A couple of sanity tests use the real
SenderMotionHistory to prove the end-to-end wiring for a genuinely fresh
sender.
"""

import math

from global_trust_perception.pipeline.persistent_reputation_tracker import PersistentReputationTracker
from global_trust_perception.trust_calculations.reputation import DELTA, REPUTATION_DEFAULT, dynamic_threshold
from global_trust_perception.trust_calculations.reputation_multipliers.kinematic_freshness import (
    D_THRESHOLD_M,
    F_FLOOR,
    GRACE_M,
    R_STAR,
    RHO_DIST,
    kinematic_freshness_factor,
)
from global_trust_perception.pipeline.trustworthy_perception import TrustEngine


EGO = [(3.0, 5.0, 0.0), (4.0, 2.0, 0.0)]
BOTH = [(3.0, 5.0, 0.0), (4.0, 2.0, 0.0)]   # neighbour agrees on everything
OTHER = 2

SPEED = 5.0   # fixed stubbed speed; blind_distance = SPEED * latency


class _FixedSpeed:
    """Test double for SenderMotionHistory: always returns SPEED regardless
    of position/time, so latency alone controls blind_distance."""

    def update(self, agent_id, ref_x, ref_y, recv_t):
        return SPEED


# Blind distance at which F reaches F_FLOOR exactly (solve
# RHO_DIST^(dist/D_THRESHOLD_M) == F_FLOOR for dist).
_DIST_FLOOR = GRACE_M + D_THRESHOLD_M * math.log(F_FLOOR) / math.log(RHO_DIST)

# A blind distance deep past the floor: F == F_FLOOR < R_STAR for any sender.
STALE_DIST = _DIST_FLOOR * 2
# A blind distance safely inside the grace band: F = 1 exactly.
FRESH_DIST = GRACE_M / 2
# Mid graded band (between grace and the floor point), reputation-sensitive zone.
MID_DIST = (GRACE_M + _DIST_FLOOR) / 2

STALE_LATENCY = STALE_DIST / SPEED
FRESH_LATENCY = FRESH_DIST / SPEED
MID_LATENCY = MID_DIST / SPEED

# Agreement frames until R_old first clears the gate crossover (R climbs by
# DELTA per perfect frame from REPUTATION_DEFAULT) — derived, not pinned, so
# retuning DELTA or the tau curve moves these with it.
FRAMES_TO_GATE = math.ceil((R_STAR - REPUTATION_DEFAULT) / DELTA)
FRAMES_WELL_ABOVE = FRAMES_TO_GATE + 2


def _engine() -> TrustEngine:
    return TrustEngine(PersistentReputationTracker(':memory:'))


def _stub_engine() -> TrustEngine:
    engine = _engine()
    engine._sender_motion = _FixedSpeed()
    return engine


def _frame(engine, latency=None):
    kwargs = {}
    if latency is not None:
        kwargs['ref_pos_by_agent'] = {OTHER: (0.0, 0.0, 1000.0, latency)}
    stats = engine.process_frame({OTHER: BOTH}, EGO, {}, [], **kwargs)
    return stats[0]


def _climb(engine, frames: int):
    """Agreement frames with no sender-position data: R rises 0.1 per frame."""
    for _ in range(frames):
        _frame(engine)


# --- wiring basics -------------------------------------------------------------

def test_no_ref_pos_dict_means_f_one():
    s = _frame(_engine())
    assert s.f_factor == 1.0
    assert s.r_eff == s.r_old


def test_agent_missing_from_dict_means_f_one():
    engine = _engine()
    stats = engine.process_frame({OTHER: BOTH}, EGO, {}, [],
                                 ref_pos_by_agent={99: (1.0, 2.0, 1000.0, 0.5)})
    assert stats[0].f_factor == 1.0


def test_sender_first_message_is_fully_fresh():
    """No prior state to estimate speed from yet -- same convention as
    SenderMotionHistory.update()'s first-sighting passthrough. Uses the
    REAL SenderMotionHistory (not the fixed-speed stub) to prove this
    end-to-end."""
    engine = _engine()
    stats = engine.process_frame({OTHER: BOTH}, EGO, {}, [],
                                 ref_pos_by_agent={OTHER: (500.0, 500.0, 1000.0, 5.0)})
    assert stats[0].f_factor == 1.0


def test_fresh_message_identical_to_no_ref_pos():
    s = _frame(_stub_engine(), latency=FRESH_LATENCY)
    assert s.f_factor == 1.0
    assert s.r_eff == s.r_old


def test_f_factor_and_r_eff_reported():
    engine = _stub_engine()
    _climb(engine, FRAMES_WELL_ABOVE)
    s = _frame(engine, latency=MID_LATENCY)
    assert s.f_factor == kinematic_freshness_factor(SPEED, MID_LATENCY)
    assert s.r_eff == s.r_old * s.f_factor
    assert s.tau == dynamic_threshold(s.r_eff)


# --- the gate uses R_eff ---------------------------------------------------------

def test_diverged_message_withheld_despite_high_reputation():
    """Deep past the floor point, even a well-reputed agent is withheld.

    Structural, not calibration: F == F_FLOOR < R_STAR there (F_FLOOR < R_STAR
    is asserted at kinematic_freshness.py import), so r_eff = r_old * F < R_STAR
    for ANY r_old <= 1.
    """
    engine = _stub_engine()
    _climb(engine, FRAMES_WELL_ABOVE)
    assert _frame(engine).trusted          # sanity: fresh frame passes

    s = _frame(engine, latency=STALE_LATENCY)
    assert s.r_old > R_STAR                # reputation alone would pass
    assert s.r_eff < R_STAR                # ...but the blind distance pushes below R*
    assert not s.trusted


def test_gate_verdict_always_matches_formula():
    """Across reputations (stranger, at-gate, well above) and all three
    latency regimes, the engine's verdict equals the independently
    computed R_eff >= tau(R_eff). This pins the WIRING without pinning
    calibration — retuning constants moves both sides together. (The
    calibration itself has a labelled canary in test_kinematic_freshness.py.)
    """
    for climb in [0, FRAMES_TO_GATE, FRAMES_WELL_ABOVE]:
        for latency in [FRESH_LATENCY, MID_LATENCY, STALE_LATENCY]:
            engine = _stub_engine()
            _climb(engine, climb)
            s = _frame(engine, latency=latency)
            assert s.r_eff == s.r_old * kinematic_freshness_factor(SPEED, latency)
            assert s.trusted == (s.r_eff >= dynamic_threshold(s.r_eff))


# --- F never touches stored reputation -------------------------------------------

def test_r_new_identical_with_and_without_divergence():
    """Two engines, same data; one always at a stale latency. R trajectories
    must match exactly while the trusted verdicts diverge — F gates, never
    scores."""
    plain, stale = _engine(), _stub_engine()
    verdicts_differ = False
    for _ in range(FRAMES_WELL_ABOVE + 1):
        s_plain = _frame(plain)
        s_stale = _frame(stale, latency=STALE_LATENCY)
        assert s_stale.r_new == s_plain.r_new
        assert s_stale.s_frame == s_plain.s_frame
        if s_stale.trusted != s_plain.trusted:
            verdicts_differ = True
    assert verdicts_differ                 # the gate DID diverge along the way


def test_stored_reputation_is_raw_after_diverged_frame():
    engine = _stub_engine()
    _climb(engine, FRAMES_WELL_ABOVE)
    s = _frame(engine, latency=STALE_LATENCY)
    # engine state holds raw R_new, not the discounted r_eff
    assert engine.get_reputation(OTHER) == s.r_new
    assert engine.get_reputation(OTHER) > s.r_eff


def test_divergence_leaves_no_mark_on_next_frame():
    """A diverged frame costs ONLY that frame: the next fresh frame gates on
    undiscounted reputation again."""
    engine = _stub_engine()
    _climb(engine, FRAMES_WELL_ABOVE)
    assert not _frame(engine, latency=STALE_LATENCY).trusted
    s = _frame(engine)                     # fresh again (no ref_pos_by_agent)
    assert s.f_factor == 1.0
    assert s.trusted
