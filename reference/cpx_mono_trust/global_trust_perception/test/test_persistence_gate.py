"""
Integration tests: persistence-penalty weight V at the Stage-0 gate.

test_persistence_penalty.py covers the V math in isolation; these tests prove
the WIRING — that drops in raw reputation reach the per-agent tracker, that V
discounts the gate (composing with kinematic freshness F), that the on-off
attacker the design targets is actually withheld while its raw reputation
looks healthy, and that V never touches the stored reputation trajectory.

Scenario mirrors test_basic_pipeline.py: ego = agent 1 sees two fixed boxes,
neighbour = agent 2 reports positions. Under the current rule a miss is forgiven,
so a drop is induced by an 'attack' frame instead: an uncorroborated ghost the
deferred ledger back-charges once its grace window (T_DEADLINE_FLUSHES) passes. A 'good' frame reports both real boxes (matched, S = +1). Because an
intermittent ghost dies inside the grace window and is forgiven, the on-off
attack has to alternate in BLOCKS, not single frames.

Producing an exact, controllable blind distance for the freshness-composes
tests: see test_freshness_gate.py's module docstring -- _engine() installs
_FixedSpeed in place of the real SenderMotionHistory, so blind_distance =
SPEED * latency is controlled purely by the latency passed in
ref_pos_by_agent.
"""

import math

import pytest

from global_trust_perception.trust_calculations.consistency import T_DEADLINE_FLUSHES as DEADLINE
from global_trust_perception.trust_calculations.consistency_checks.kinematic_checks import TrackData

from global_trust_perception.pipeline.persistent_reputation_tracker import PersistentReputationTracker
from global_trust_perception.trust_calculations.reputation import (
    dynamic_threshold,
    reputation_update,
    trust_fixed_point,
)
from global_trust_perception.trust_calculations.reputation_multipliers.kinematic_freshness import (
    D_THRESHOLD_M,
    F_FLOOR,
    GRACE_M,
    RHO_DIST,
    kinematic_freshness_factor,
)
from global_trust_perception.trust_calculations.reputation_multipliers.persistence_penalty import (
    beta_from_half_life,
)
from global_trust_perception.pipeline.trustworthy_perception import (
    FRAME_FLUSH_HZ,
    TrustEngine,
    V_HALF_LIFE_S,
)


EGO = [(3.0, 5.0, 0.0), (4.0, 2.0, 0.0)]
BOTH = [(3.0, 5.0, 0.0), (4.0, 2.0, 0.0)]   # neighbour agrees on everything
OTHER = 2
WARMUP_FRAMES = 5                            # engine frames run first (no rep touched)

SPEED = 5.0   # fixed stubbed speed; blind_distance = SPEED * latency


class _FixedSpeed:
    """Test double for SenderMotionHistory: always returns SPEED regardless
    of position/time, so latency alone controls blind_distance."""

    def update(self, agent_id, ref_x, ref_y, recv_t):
        return SPEED


# Blind distance at which F reaches F_FLOOR exactly (solve
# RHO_DIST^(dist/D_THRESHOLD_M) == F_FLOOR for dist).
_DIST_FLOOR = GRACE_M + D_THRESHOLD_M * math.log(F_FLOOR) / math.log(RHO_DIST)

# Mid graded band: F strictly between the floor and 1.
MID_LATENCY = ((GRACE_M + _DIST_FLOOR) / 2) / SPEED


def _engine() -> TrustEngine:
    engine = TrustEngine(PersistentReputationTracker(':memory:'))
    engine._sender_motion = _FixedSpeed()
    for _ in range(WARMUP_FRAMES):
        engine.process_frame({}, EGO, {}, [])
    return engine


def _ref_pos_kwargs(latency):
    if latency is None:
        return {}
    return {'ref_pos_by_agent': {OTHER: (0.0, 0.0, 1000.0, latency)}}


def _frame(engine, positions, latency=None):
    kwargs = _ref_pos_kwargs(latency)
    stats = engine.process_frame({OTHER: positions}, EGO, {}, [], **kwargs)
    return stats[0]


GHOST = [(50.0, 50.0, 0.0)]     # object ego never sees; the neighbour's fabrication


def _tracked(points: list) -> TrackData:
    """Stationary TrackData for each reported point (stable ids)."""
    return TrackData(
        track_ids=list(range(len(points))),
        kalman_x=[p[0] for p in points], kalman_y=[p[1] for p in points],
        kalman_vx=[0.0] * len(points), kalman_vy=[0.0] * len(points),
    )


def _report(engine, positions, latency=None):
    """One frame: neighbour reports positions as tracked objects (ledger active)."""
    kwargs = _ref_pos_kwargs(latency)
    return engine.process_frame(
        {OTHER: positions}, EGO, {}, [],
        tracks_by_agent={OTHER: _tracked(positions)}, **kwargs)[0]


def _attack(engine, latency=None):
    """Drop-inducing frame: an uncorroborated ghost, back-charged once
    its grace window passes."""
    return _report(engine, GHOST, latency)


def _good(engine, latency=None):
    """Reputation-earning frame: both real boxes, matched -> Correct."""
    return _report(engine, BOTH, latency)


# --- wiring basics -------------------------------------------------------------

def test_clean_agent_v_stays_one_and_gate_unchanged():
    """Rising reputation never trips the penalty: V = 1, r_eff = r_old."""
    engine = _engine()
    for _ in range(6):
        s = _frame(engine, BOTH)
        assert s.v_factor == 1.0
        assert s.risk_persist == 0.0
        assert s.r_eff == s.r_old


def test_drop_reaches_tracker_and_discounts_gate():
    engine = _engine()
    for _ in range(DEADLINE + 1):            # grace window, then the back-charge drop
        _attack(engine)
    s = _attack(engine)                      # next frame's gate reflects the drop
    assert s.v_factor < 1.0
    assert s.risk_persist > 0.0
    assert s.r_eff == s.r_old * s.v_factor   # F = 1 here
    assert s.tau == dynamic_threshold(s.r_eff)


def test_v_composes_with_freshness():
    """r_eff = r_old * F * V with both multipliers strictly below 1."""
    engine = _engine()
    for _ in range(DEADLINE + 2):            # into the V < 1 regime
        _attack(engine)
    s = _attack(engine, latency=MID_LATENCY)
    assert s.f_factor == kinematic_freshness_factor(SPEED, MID_LATENCY)
    assert 0.0 < s.f_factor < 1.0
    assert 0.0 < s.v_factor < 1.0
    assert s.r_eff == s.r_old * s.f_factor * s.v_factor


def test_gate_verdict_always_matches_formula():
    """Across attack, recovery and diverged frames the verdict equals the
    independently computed R_eff >= tau(R_eff)."""
    engine = _engine()
    script = [([], None), ([], None), (BOTH, None), ([], MID_LATENCY),
              (BOTH, None), (BOTH, None), ([], None)]
    for positions, latency in script:
        s = _frame(engine, positions, latency=latency)
        assert s.r_eff == s.r_old * s.f_factor * s.v_factor
        assert s.trusted == (s.r_eff >= dynamic_threshold(s.r_eff))


# --- V never touches stored reputation -------------------------------------------

def test_stored_reputation_stays_raw_while_v_penalises():
    """r_new must equal the raw (C, I) update — no V baked into scoring —
    and the engine's stored state must hold that raw value."""
    engine = _engine()
    for _ in range(DEADLINE + 1):            # reach the charging regime
        _attack(engine)
    for _ in range(3):
        s = _attack(engine)
        expected_raw, _ = reputation_update(s.r_old, s.correct, s.incorrect,
                                            s.n_total)
        assert s.r_new == expected_raw
        assert engine.get_reputation(OTHER) == s.r_new
    assert s.risk_persist > 0.0              # the penalty state DID accumulate


# --- the on-off attacker: the reason V exists --------------------------------------

def test_onoff_attacker_withheld_while_raw_reputation_passes():
    """A burst of sustained ghosting banks drop-debt; a recovered raw R that
    per-frame gating would pass must still be withheld by that banked risk —
    the on-off attack per-frame scoring cannot catch. (Single-frame ghosts die
    inside the grace window and are forgiven, so the attack runs in blocks.)

    The attack burst is real; only the recovered reputation is seeded, at the
    gate crossover trust_fixed_point() — the exact R where raw gating flips to
    passing. Seeding is what makes this deterministic: R moves in DELTA steps,
    and the withheld band [R*, R*/V) is narrower than one step, so whether a
    freely-climbing R happens to land inside it is an artefact of the step grid
    (it did under dynamic tau, it does not under the static threshold) rather
    than a property of the persistence gate.

    Binding to trust_fixed_point() rather than a tuned constant keeps the
    assertion valid under BOTH threshold regimes: at R = R* raw gating passes
    by definition, and any V < 1 puts r_eff = R*·V below R*. Under a static tau
    that is below the constant; under dynamic tau, tau is decreasing, so
    tau(r_eff) > tau(R*) = R* > r_eff. Withheld either way.
    """
    engine = _engine()
    for _ in range(4):
        _good(engine)                        # climb: R 0.5 -> ~1.0
    for _ in range(DEADLINE + 4):            # attack burst: banks persistence risk
        _attack(engine)

    # Just above the crossover: trust_fixed_point() bisects from below, which
    # lands an ulp short when tau is a constant (the fixed point is exactly on
    # the step). The same +eps nudge test_kinematic_freshness uses to sit on
    # the passing side of the gate.
    r_seed = min(1.0, trust_fixed_point() + 1e-6)
    engine.reputations[OTHER] = r_seed       # recovered to the gate crossover
    s = _good(engine)

    assert s.r_old == r_seed
    assert s.r_old >= dynamic_threshold(s.r_old)   # raw gating would ADMIT
    assert s.v_factor < 1.0                        # but drop-debt is banked
    assert s.r_eff < s.r_old
    assert not s.trusted                           # so the gate WITHHOLDS


def test_attacker_recovers_after_sustained_good_behaviour():
    """Once the attack stops, risk decays with V_HALF_LIFE_S and the agent
    earns back both its weight and its admission — floored, not fatal."""
    engine = _engine()
    for _ in range(4):
        _good(engine)
    for _ in range(DEADLINE + 4):            # sustained ghost banks risk
        _attack(engine)
    s_attacked = _good(engine)
    assert s_attacked.v_factor < 1.0

    # ~4 risk half-lives of clean frames at the flush cadence
    clean = int(4 * V_HALF_LIFE_S * FRAME_FLUSH_HZ)
    for _ in range(clean):
        s = _good(engine)
    assert s.v_factor > s_attacked.v_factor
    assert s.v_factor > 0.95
    assert s.trusted
    assert s.r_old > trust_fixed_point()     # and passes on raw merit again


# --- absence gap: risk is not decayed and the gap is not scored as a drop ----------

def test_absence_gap_does_not_decay_risk_and_is_not_scored_as_a_drop():
    engine = _engine()
    for _ in range(4):
        _good(engine)
    for _ in range(DEADLINE + 6):            # sustained ghost banks persistence risk
        _attack(engine)
    _good(engine)                            # ghost settles; good frames from here
    s_before = _good(engine)                 # only decay risk by one EMA step
    risk_before = s_before.risk_persist
    assert risk_before > 0.0

    # A good frame introduces no drop (dnorm = 0: gain clips to zero) and no gap,
    # so risk must decay by exactly one EMA step — never by a wall-clock gap factor.
    s = _good(engine)
    beta_l = beta_from_half_life(V_HALF_LIFE_S, float(FRAME_FLUSH_HZ))
    assert s.risk_persist == pytest.approx(risk_before * beta_l, rel=1e-9)
