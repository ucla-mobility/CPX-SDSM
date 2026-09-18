"""
Unit tests for reputation_multipliers.absence_decay.

Invariants under test:
  1. No gap -> R unchanged
  2. Gap within grace period -> R unchanged
  3. Above-baseline R decays toward baseline once past grace
  4. Below-baseline R is never modified (silence does not reward)
  5. R at exactly baseline is unchanged
  6. After one month, above-baseline R is within 2 % of baseline
  7. Decay is monotonically increasing with gap_s past the grace period
  8. R_decayed never exceeds R_last (never upgrades reputation)
  9. ValueError on gap_s < 0 or R_last outside [0, 1]
"""

import math

import pytest

from global_trust_perception.trust_calculations.reputation import REPUTATION_DEFAULT
from global_trust_perception.trust_calculations.reputation_multipliers.absence_decay import (
    GRACE_GAP_S,
    TAU_GAP_S,
    absence_decay,
)

BASELINE = REPUTATION_DEFAULT   # 0.5
ONE_MONTH_S = 30 * 24 * 3600   # 2_592_000 s


# --- Invariant 1: no gap leaves R unchanged ----------------------------------

def test_zero_gap_above_baseline():
    assert absence_decay(0.9, 0.0) == pytest.approx(0.9, abs=1e-9)


def test_zero_gap_below_baseline():
    assert absence_decay(0.2, 0.0) == pytest.approx(0.2, abs=1e-9)


def test_zero_gap_at_baseline():
    assert absence_decay(BASELINE, 0.0) == pytest.approx(BASELINE, abs=1e-9)


# --- Invariant 2: gap within (or at) grace period leaves R unchanged ---------

def test_within_grace_unchanged():
    assert absence_decay(0.9, GRACE_GAP_S - 1.0) == pytest.approx(0.9, abs=1e-9)


def test_at_grace_boundary_unchanged():
    assert absence_decay(0.9, GRACE_GAP_S) == pytest.approx(0.9, abs=1e-9)


def test_just_past_grace_decays():
    r = absence_decay(0.9, GRACE_GAP_S + 1.0)
    assert BASELINE < r < 0.9


# --- Invariant 3: above-baseline R decays toward baseline past grace ---------

def test_above_baseline_decays():
    r = absence_decay(0.9, GRACE_GAP_S + 3600.0)
    assert BASELINE < r < 0.9


def test_above_baseline_half_life():
    # Test pure exponential math with grace_s=0 to isolate formula correctness.
    half_life_s = TAU_GAP_S * math.log(2)
    r = absence_decay(0.9, half_life_s, grace_s=0.0)
    assert r == pytest.approx(BASELINE + (0.9 - BASELINE) * 0.5, rel=1e-6)


def test_grace_zero_restores_immediate_decay():
    # With grace disabled, a 1-hour gap should decay (original pre-grace behavior).
    r = absence_decay(0.9, 3600.0, grace_s=0.0)
    assert BASELINE < r < 0.9


# --- Invariant 4: below-baseline R is never changed --------------------------

def test_below_baseline_untouched_short_gap():
    assert absence_decay(0.1, GRACE_GAP_S + 3600.0) == pytest.approx(0.1, abs=1e-9)


def test_below_baseline_untouched_long_gap():
    assert absence_decay(0.1, ONE_MONTH_S) == pytest.approx(0.1, abs=1e-9)


def test_below_baseline_untouched_very_low():
    assert absence_decay(0.0, ONE_MONTH_S) == pytest.approx(0.0, abs=1e-9)


# --- Invariant 5: R at exactly baseline is unchanged -------------------------

def test_at_baseline_unchanged():
    assert absence_decay(BASELINE, ONE_MONTH_S) == pytest.approx(BASELINE, abs=1e-9)


# --- Invariant 6: one month brings above-baseline R within 2 % of baseline --

def test_one_month_high_rep_near_baseline():
    r = absence_decay(0.9, ONE_MONTH_S)
    assert abs(r - BASELINE) < 0.02


def test_one_month_moderate_rep_near_baseline():
    r = absence_decay(0.7, ONE_MONTH_S)
    assert abs(r - BASELINE) < 0.02


# --- Invariant 7: monotonically increasing toward baseline past grace --------

def test_monotone_toward_baseline():
    gaps = [GRACE_GAP_S + 3600 * i for i in range(1, 8)]
    rs = [absence_decay(0.9, g) for g in gaps]
    for a, b in zip(rs, rs[1:]):
        assert a > b


# --- Invariant 8: R_decayed never exceeds R_last (no upgrade) ---------------

def test_never_upgrades_above_baseline():
    for r_last in [0.51, 0.6, 0.75, 0.9, 1.0]:
        for gap in [0.0, GRACE_GAP_S, ONE_MONTH_S]:
            assert absence_decay(r_last, gap) <= r_last + 1e-9


def test_never_upgrades_below_baseline():
    for r_last in [0.0, 0.1, 0.3, 0.49]:
        for gap in [0.0, GRACE_GAP_S, ONE_MONTH_S]:
            assert absence_decay(r_last, gap) <= r_last + 1e-9


# --- Invariant 9: error cases ------------------------------------------------

def test_negative_gap_raises():
    with pytest.raises(ValueError):
        absence_decay(0.9, -1.0)


def test_r_last_above_one_raises():
    with pytest.raises(ValueError):
        absence_decay(1.01, 0.0)


def test_r_last_below_zero_raises():
    with pytest.raises(ValueError):
        absence_decay(-0.01, 0.0)


def test_tau_zero_raises():
    with pytest.raises(ValueError):
        absence_decay(0.9, 100.0, tau_gap_s=0.0)
