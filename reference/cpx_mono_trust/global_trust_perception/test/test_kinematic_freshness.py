"""
Unit tests for reputation_multipliers.kinematic_freshness.

All assertions use the module's own constants (GRACE_M, R_STAR, ...) rather
than pinned literals, so retuning a constant does not break tests -- but
breaking a structural property (the floor, the eventual-unconditional
guarantee) does.

Properties under test:
  1. Grace window: F = 1.0 exactly for speed*latency <= GRACE_M (including
     any latency when speed == 0)
  2. Graded band: F strictly decreasing past the grace window
  3. Eventual-unconditional guarantee: a sufficiently blind-distant sender
     fails the gate even for a perfect reputation (R=1, R_eff=F), because
     F -> F_FLOOR and F_FLOOR < R_STAR
  4. Floor: F never below F_FLOOR, floor reached and held at extreme blind
     distance, floor < R_STAR (or guarantee 3 would be void)
  5. Clock skew: small negative latency clamps to fresh; beyond tolerance raises
  6. Negative speed raises (a caller bug -- speed is a magnitude)
  7. Module constants are internally consistent (parameter validation moved
     to import time)
"""

import math

import pytest

from global_trust_perception.trust_calculations.reputation import dynamic_threshold
from global_trust_perception.trust_calculations.reputation_multipliers.kinematic_freshness import (
    CLOCK_SKEW_TOLERANCE_S,
    D_THRESHOLD_M,
    F_FLOOR,
    GRACE_M,
    R_STAR,
    RHO_DIST,
    kinematic_freshness_factor,
)

# Blind distance at which F reaches F_FLOOR exactly (solve
# RHO_DIST^(dist/D_THRESHOLD_M) == F_FLOOR for dist).
_DIST_FLOOR = GRACE_M + D_THRESHOLD_M * math.log(F_FLOOR) / math.log(RHO_DIST)

SPEED = 5.0   # an arbitrary fixed nonzero speed used to convert a
              # blind-distance target into a latency: latency = dist / SPEED


def _factor_at_distance(dist_m: float) -> float:
    return kinematic_freshness_factor(SPEED, dist_m / SPEED)


# --- 1. Grace window ----------------------------------------------------------

def test_zero_speed_fully_fresh_regardless_of_latency():
    for latency in [0.0, 1.0, 1000.0]:
        assert kinematic_freshness_factor(0.0, latency) == 1.0


def test_zero_latency_fully_fresh_regardless_of_speed():
    for speed in [0.0, 5.0, 100.0]:
        assert kinematic_freshness_factor(speed, 0.0) == 1.0


def test_blind_distance_at_grace_fully_fresh():
    assert _factor_at_distance(GRACE_M) == 1.0


def test_blind_distance_just_under_grace_fully_fresh():
    assert _factor_at_distance(GRACE_M * 0.99) == 1.0


def test_blind_distance_just_over_grace_penalised():
    assert _factor_at_distance(GRACE_M * 1.01) < 1.0


# --- 2. Graded band -----------------------------------------------------------

def test_strictly_decreasing_in_graded_band():
    """Sample the open interval (grace, dist_floor): each step must strictly drop."""
    n = 8
    step = (_DIST_FLOOR - GRACE_M) / (n + 1)
    distances = [GRACE_M + i * step for i in range(1, n + 1)]
    factors = [_factor_at_distance(d) for d in distances]
    for a, b in zip(factors, factors[1:]):
        assert a > b


def test_graded_band_matches_closed_form():
    """Inside the band (above floor), F is exactly rho^((dist - grace) / D)."""
    dist = (GRACE_M + _DIST_FLOOR) / 2.0
    expected = RHO_DIST ** ((dist - GRACE_M) / D_THRESHOLD_M)
    assert _factor_at_distance(dist) == pytest.approx(expected, rel=1e-12)


# --- 3. Eventual-unconditional guarantee ----------------------------------------

def test_sufficiently_blind_distant_perfect_sender_fails_gate():
    """For blind distance well past where F floors, R_eff = 1.0 * F < R_STAR
    always.

    Structural, not calibration: F -> F_FLOOR as blind distance grows
    (RHO_DIST < 1), and F_FLOOR < R_STAR is asserted at import, so this
    holds for ANY RHO_DIST/F_FLOOR/R_STAR combination the module accepts --
    it doesn't depend on a specific cutoff distance.
    """
    for dist in [_DIST_FLOOR * 1.5, _DIST_FLOOR * 10, 500.0]:
        f = _factor_at_distance(dist)
        r_eff = 1.0 * f
        assert r_eff < R_STAR
        assert r_eff < dynamic_threshold(r_eff)   # the actual gate comparison


# --- 4. Floor -------------------------------------------------------------------

def test_floor_below_r_star():
    """Structural invariant: a floor above R_STAR would void the eventual-
    unconditional-failure guarantee."""
    assert F_FLOOR < R_STAR


def test_f_never_below_floor():
    for dist in [0.0, GRACE_M, _DIST_FLOOR, 10.0, 100.0, 1e6]:
        assert _factor_at_distance(dist) >= F_FLOOR


def test_floor_reached_and_held():
    """Where rho^((dist-grace)/D) sinks below the floor, F pins at exactly F_FLOOR."""
    assert _factor_at_distance(_DIST_FLOOR * 1.01) == F_FLOOR
    assert _factor_at_distance(1e6) == F_FLOOR   # no underflow to 0.0


def test_f_never_exceeds_one():
    for dist in [0.0, GRACE_M, 2.0, _DIST_FLOOR, 100.0]:
        assert _factor_at_distance(dist) <= 1.0


# --- 5. Clock skew --------------------------------------------------------------

def test_small_negative_skew_clamped_to_fresh():
    assert kinematic_freshness_factor(SPEED, -CLOCK_SKEW_TOLERANCE_S / 2) == 1.0


def test_skew_at_tolerance_boundary_clamped():
    assert kinematic_freshness_factor(SPEED, -CLOCK_SKEW_TOLERANCE_S) == 1.0


def test_skew_beyond_tolerance_raises():
    with pytest.raises(ValueError):
        kinematic_freshness_factor(SPEED, -CLOCK_SKEW_TOLERANCE_S * 1.5)


def test_large_negative_latency_raises():
    with pytest.raises(ValueError):
        kinematic_freshness_factor(SPEED, -1.0)


# --- 6. Negative speed -----------------------------------------------------------

def test_negative_speed_raises():
    with pytest.raises(ValueError):
        kinematic_freshness_factor(-1.0, 1.0)


# --- 7. Constant consistency (validation happens at import) ----------------------

def test_module_constants_consistent():
    """The invariants the import-time validation enforces, restated here so a
    future edit weakening that validation still has a failing test."""
    assert GRACE_M >= 0.0
    assert 0.0 <= F_FLOOR < 1.0
    assert CLOCK_SKEW_TOLERANCE_S >= 0.0
    assert 0.0 < RHO_DIST < 1.0
    assert D_THRESHOLD_M > 0.0
    assert 0.0 < R_STAR < 1.0
