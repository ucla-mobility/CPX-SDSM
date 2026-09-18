"""
Unit tests for consistency_checks.attribute_checks.check_size_agreement.

SS is now continuous and purely geometric (CooperFuse eq 2.5: min/max ratio
per axis, reduced across axes via min -- see the module docstring).
Reputation gating no longer lives in this module; it moved to the call site
in trustworthy_perception.py, so these tests only pin the size-ratio math.
Pure Python.
"""

from global_trust_perception.trust_calculations.consistency_checks.attribute_checks import (
    check_size_agreement,
)

DIMS = (2.0, 4.5, 1.5)  # (w, l, h) -- the sim's vehicle box
PAIR = [(0, 0)]


def _ss(ego_dims, other_dims):
    return check_size_agreement(PAIR, [ego_dims], [other_dims])[0]


# --- the core ratio ----------------------------------------------------------------

def test_identical_dims_score_one():
    assert _ss(DIMS, DIMS) == 1.0


def test_width_off_by_factor_two_scores_half():
    """min/max ratio for a 2x width mismatch: 2.0/4.0 == 0.5 (verified
    against the actual module)."""
    assert _ss((2.0, 4.5, 1.5), (4.0, 4.5, 1.5)) == 0.5


def test_length_off_by_factor_two_scores_half():
    assert _ss((2.0, 4.5, 1.5), (2.0, 9.0, 1.5)) == 0.5


def test_worst_axis_bounds_the_score():
    """Width agrees exactly, length is off by 2x -> SS takes the worse
    (length) axis, per the min-across-axes reduction."""
    assert _ss((2.0, 4.5, 1.5), (2.0, 9.0, 1.5)) == 0.5


def test_ratio_is_order_independent():
    """min/max is symmetric: doesn't matter which side is bigger."""
    assert _ss((2.0, 4.5, 1.5), (4.0, 4.5, 1.5)) == _ss((4.0, 4.5, 1.5), (2.0, 4.5, 1.5))


def test_height_ignored():
    """Only width and length are compared; height may differ wildly."""
    assert _ss((2.0, 4.5, 1.5), (2.0, 4.5, 99.0)) == 1.0


def test_near_zero_dims_count_as_agreement():
    """Both-near-zero guard: malformed dims must not divide by zero."""
    assert _ss((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)) == 1.0


# --- structural cases ---------------------------------------------------------------

def test_out_of_range_index_neutral():
    """A pair pointing past either dims list scores neutral (1.0), not a crash."""
    assert check_size_agreement([(0, 5)], [DIMS], [DIMS]) == [1.0]


def test_empty_pairs():
    assert check_size_agreement([], [DIMS], [DIMS]) == []


def test_scores_parallel_to_pairs():
    """One score per pair, in order, judged independently."""
    ego = [DIMS, DIMS]
    other = [DIMS, (DIMS[0] * 2, DIMS[1] * 2, DIMS[2])]
    scores = check_size_agreement([(0, 0), (1, 1)], ego, other)
    assert scores == [1.0, 0.5]
