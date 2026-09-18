"""
Unit tests for consistency_checks.kinematic_checks.KinematicHistory.

KDS is now continuous -- a minimum-jerk quintic fit (KDS_jerk) times a
turning-radius ratio (KDS_curvature), see the module docstring for what's
ported from CooperFuse eq 2.2-2.4 vs. approximated -- so these tests pin
qualitative properties (exact zero-energy cases, monotonicity, structural
behaviour, the vehicle-class gate) rather than hand-derived numeric
boundaries. A few thresholds are pinned to values verified by running the
actual module (see comments). Needs numpy only -- runs anywhere.
"""

import numpy as np

from global_trust_perception.trust_calculations.consistency_checks.kinematic_checks import (
    KinematicHistory,
    TrackData,
    _KDS_A_MAX_MS2,
    _KDS_NOISE_M,
    _min_jerk_energy_1d,
    _unexplained_tolerance_m,
)

AGENT = 2
DT = 0.5   # production frame period
# trustworthy_perception._KDS_FLAG_THRESHOLD, restated rather than imported so
# these tests keep needing numpy only (the pipeline module pulls in ROS).
_FLAG_THRESHOLD = 0.5


# --- closed-form jerk energy matches an independent fit-and-integrate reference
# The module computes the minimum-jerk energy in one closed-form step (no 3x3
# solve, no quadrature). This pins that formula against a reference that fits
# the quintic numerically (np.linalg.solve) and integrates the squared jerk on
# a fine grid -- the equivalence check that retires the transcription-error risk
# the earlier numerical solve was there to avoid.

def _reference_energy(p0, v0, p1, v1, T, n=20001):
    A = np.array([
        [T ** 3, T ** 4, T ** 5],
        [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
        [6 * T, 12 * T ** 2, 20 * T ** 3],
    ])
    rhs = np.array([p1 - (p0 + v0 * T), v1 - v0, 0.0])
    c3, c4, c5 = np.linalg.solve(A, rhs)
    t = np.linspace(0.0, T, n)
    jerk = 6.0 * c3 + 24.0 * c4 * t + 60.0 * c5 * (t ** 2)
    return float(np.trapz(jerk ** 2, t))


def test_min_jerk_energy_matches_fit_and_integrate_reference():
    rng = np.random.default_rng(1)
    for _ in range(5000):
        p0, v0, p1, v1 = rng.normal(0, 20, 4)
        T = rng.uniform(0.05, 1.0)
        ref = _reference_energy(p0, v0, p1, v1, T)
        got = _min_jerk_energy_1d(p0, v0, p1, v1, T)
        # fine-grid trapz on a quartic still carries a little discretization
        # error; the closed form is exact, so allow a small relative tolerance.
        assert abs(got - ref) <= 1e-4 * max(1.0, abs(ref))


def test_min_jerk_energy_zero_for_constant_velocity():
    # Constant-velocity continuation (p1 = p0 + v*T, v1 = v0) needs no jerk.
    assert _min_jerk_energy_1d(0.0, 15.0, 7.5, 15.0, 0.5) == 0.0
    assert _min_jerk_energy_1d(10.0, 0.0, 10.0, 0.0, 0.5) == 0.0


def _track(tid, x, y, vx, vy):
    """TrackData for a single detection with the given Kalman state."""
    return TrackData(
        track_ids=[tid],
        kalman_x=[x], kalman_y=[y], kalman_vx=[vx], kalman_vy=[vy],
    )


def _xy(x, y):
    return np.array([[x, y]], dtype=float)


# --- warm-up and degenerate inputs ---------------------------------------------

def test_first_frame_always_neutral():
    kin = KinematicHistory()
    assert kin.score_and_update(AGENT, _track(0, 0, 0, 0, 0), _xy(0, 0), DT) == [1.0]


def test_unknown_track_id_always_neutral():
    kin = KinematicHistory()
    kin.score_and_update(AGENT, _track(0, 0, 0, 0, 0), _xy(0, 0), DT)
    # different track id next frame: no history for it
    assert kin.score_and_update(AGENT, _track(9, 50, 50, 0, 0), _xy(50, 50), DT) == [1.0]


def test_dt_too_small_cannot_judge():
    kin = KinematicHistory()
    kin.score_and_update(AGENT, _track(0, 0, 0, 0, 0), _xy(0, 0), DT)
    assert kin.score_and_update(AGENT, _track(0, 99, 99, 0, 0), _xy(99, 99), 0.0) == [1.0]


def test_empty_frame():
    kin = KinematicHistory()
    empty = TrackData([], [], [], [], [])
    assert kin.score_and_update(AGENT, empty, np.empty((0, 2)), DT) == []


# --- the core score: energy grows with implausibility ----------------------------

def test_exact_constant_velocity_continuation_scores_perfect():
    """Reporting exactly where constant-velocity motion predicts costs zero
    jerk energy -> KDS == 1.0 exactly (verified: straight continuation at
    v=15 m/s, dt=0.5s gives KDS=1.0)."""
    kin = KinematicHistory()
    vx = 15.0
    kin.score_and_update(AGENT, _track(0, 0, 0, vx, 0), _xy(0, 0), DT)
    kds = kin.score_and_update(
        AGENT, _track(0, vx * DT, 0, vx, 0), _xy(vx * DT, 0), DT
    )
    assert kds == [1.0]


def test_stationary_object_at_rest_scores_perfect():
    kin = KinematicHistory()
    kin.score_and_update(AGENT, _track(0, 10, 5, 0, 0), _xy(10, 5), DT)
    kds = kin.score_and_update(AGENT, _track(0, 10, 5, 0, 0), _xy(10, 5), DT)
    assert kds == [1.0]


def test_larger_deviation_scores_lower():
    """Monotonicity: a bigger unexplained displacement costs more jerk
    energy -> lower KDS."""
    def score_for_offset(offset):
        kin = KinematicHistory()
        kin.score_and_update(AGENT, _track(0, 10, 5, 0, 0), _xy(10, 5), DT)
        return kin.score_and_update(
            AGENT, _track(0, 10, 5, 0, 0), _xy(10 + offset, 5), DT
        )[0]

    small = score_for_offset(0.5)
    mid = score_for_offset(2.0)
    large = score_for_offset(20.0)
    assert 0.0 <= large < mid < small <= 1.0


# --- the tolerance is stated in metres and tracks the frame period ------------
# The energy scale is derived per-dt from _unexplained_tolerance_m rather than
# fixed, so that what is actually being chosen is a displacement. These pin the
# property that broke when it was a constant: a fixed J0 fixes an energy, and
# energy carries T^-5, so the tolerated displacement drifted as T^2.5.

def _unexplained_score(offset, dt):
    """KDS for a stationary track reported `offset` metres from where it was."""
    kin = KinematicHistory()
    kin.score_and_update(AGENT, _track(0, 0, 0, 0, 0), _xy(0, 0), dt)
    return kin.score_and_update(AGENT, _track(0, 0, 0, 0, 0), _xy(offset, 0), dt)[0]


def test_tolerance_is_hit_at_exactly_half_score_at_every_frame_period():
    """KDS_jerk == 0.5 at d == _unexplained_tolerance_m(dt), by construction,
    whatever dt is -- the property a fixed J0 could not hold."""
    for dt in (0.05, 0.118, 0.15, 0.5, 1.0):
        d_tol = _unexplained_tolerance_m(dt, _KDS_NOISE_M, _KDS_A_MAX_MS2)
        assert abs(_unexplained_score(d_tol, dt) - 0.5) < 1e-9


def test_tolerated_displacement_grows_with_frame_period():
    """Longer between looks, more real motion an unmodelled acceleration
    explains -- so the tolerance must widen, never narrow."""
    tols = [_unexplained_tolerance_m(dt, _KDS_NOISE_M, _KDS_A_MAX_MS2)
            for dt in (0.05, 0.118, 0.15, 0.5, 1.0)]
    assert all(a < b for a, b in zip(tols, tols[1:]))
    # ...and never below the noise floor, however short the period.
    assert _unexplained_tolerance_m(1e-3, _KDS_NOISE_M, _KDS_A_MAX_MS2) >= _KDS_NOISE_M


def test_wire_quantisation_step_is_not_flagged():
    """THE REGRESSION. SDSM positions arrive on a 0.1 m grid
    (sdsm_units.OFFSET_UNIT_M), so an honest, perfectly tracked object steps a
    whole unit for free. Under the previous fixed J0 that scored 0.089 at the
    sender frame period and was demoted; 64% of matched pairs on clean
    recorded data went the same way."""
    for dt in (0.118, 0.15):
        assert _unexplained_score(0.1, dt) > _FLAG_THRESHOLD
    # a full 0.2 m of quantisation plus centroid jitter still survives
    assert _unexplained_score(0.2, 0.118) > _FLAG_THRESHOLD


def test_single_frame_teleport_still_flagged_at_short_frame_period():
    """Widening the tolerance must not cost the check its actual job: a jump
    no bounded acceleration explains is still demoted, at the same short dt
    where the honest quantisation step now passes."""
    for jump in (0.5, 1.0, 2.0, 5.0):
        assert _unexplained_score(jump, 0.118) < _FLAG_THRESHOLD


def test_readme_worked_example_teleport_scores_near_zero():
    """Established agreement then a 50+ m unexplained jump: verified this
    scenario scores exactly 0.0 in float64 (energy is many orders of
    magnitude past the point where exp(-J/J0) underflows)."""
    kin = KinematicHistory()
    kin.score_and_update(AGENT, _track(0, 11, 5, 1, 0), _xy(11, 5), DT)
    kds = kin.score_and_update(AGENT, _track(0, 40, 80, 1, 0), _xy(40, 80), DT)
    assert kds[0] < 1e-6


# --- curvature ratio: shortest Dubins path length vs. straight-line distance
# (Dubins is the forward-only special case of Reed-Shepp -- see module
# docstring for why reversal/C|C|C words were left out) --------------------

# A 150-degree turn point reached from (0,0) heading +x at 15 m/s, dt=0.5s --
# same displacement magnitude as continuing straight ahead, but far enough
# off-axis that a bounded-turning-radius vehicle needs real extra distance
# to reorient. J2735 heading 300 deg corresponds to math yaw 150 deg (the
# direction of travel to this point); J2735 90 deg corresponds to yaw 0
# (straight ahead, matching the initial heading).
_TURN_XY = (-6.495, 3.75)
_HEADING_STRAIGHT_DEG = 90.0
_HEADING_ALIGNED_WITH_TURN_DEG = 300.0


def test_straight_with_matching_heading_scores_perfect():
    """A straight line with a heading that matches the direction of travel
    the whole way needs no curvature detour: KDS_curvature == 1.0 exactly,
    same as the jerk term for this scenario -> KDS == 1.0."""
    kin = KinematicHistory()
    kin.score_and_update(AGENT, _track(0, 0, 0, 15, 0), _xy(0, 0), DT,
                         headings=[_HEADING_STRAIGHT_DEG], vehicle_probs=[1.0])
    kds = kin.score_and_update(
        AGENT, _track(0, 7.5, 0, 15, 0), _xy(7.5, 0), DT,
        headings=[_HEADING_STRAIGHT_DEG], vehicle_probs=[1.0],
    )
    assert kds == [1.0]


def test_curvature_soft_gated_by_vehicle_probability():
    """Same 150-degree-turn scenario, reported heading consistent with
    having actually turned: P(vehicle)=1 applies the full curvature term,
    P(vehicle)=0 skips it entirely, and P(vehicle)=0.5 lands exactly between
    the two -- a graded blend, not a binary switch (verified: 8.20e-18 /
    2.23e-17 / 3.65e-17 respectively). vehicle_probs=None defaults to the
    P(vehicle)=1 case, the same conservative default used before class data
    was wired into this pipeline."""
    def score_for(vehicle_probs):
        kin = KinematicHistory()
        kin.score_and_update(AGENT, _track(0, 0, 0, 15, 0), _xy(0, 0), DT,
                             headings=[_HEADING_STRAIGHT_DEG], vehicle_probs=[1.0])
        return kin.score_and_update(
            AGENT, _track(0, *_TURN_XY, 15, 0), _xy(*_TURN_XY), DT,
            headings=[_HEADING_ALIGNED_WITH_TURN_DEG], vehicle_probs=vehicle_probs,
        )[0]

    certain_vehicle = score_for([1.0])
    uncertain = score_for([0.5])
    certain_non_vehicle = score_for([0.0])
    assert certain_non_vehicle > uncertain > certain_vehicle > 0.0
    assert score_for(None) == certain_vehicle


def test_missing_heading_is_neutral_curvature():
    """Without a reported heading, the curvature term can't judge anything
    and stays neutral (1.0) -- score matches the P(vehicle)=0 case exactly,
    using only the jerk term."""
    def score_for(headings):
        kin = KinematicHistory()
        kin.score_and_update(AGENT, _track(0, 0, 0, 15, 0), _xy(0, 0), DT,
                             headings=[_HEADING_STRAIGHT_DEG], vehicle_probs=[1.0])
        return kin.score_and_update(
            AGENT, _track(0, *_TURN_XY, 15, 0), _xy(*_TURN_XY), DT,
            headings=headings, vehicle_probs=[1.0],
        )[0]

    assert score_for(None) == score_for(None)  # sanity: deterministic
    no_heading = score_for(None)
    unavailable_sentinel = score_for([360.0])  # J2735 "unavailable"
    assert no_heading == unavailable_sentinel
    assert no_heading > score_for([_HEADING_ALIGNED_WITH_TURN_DEG])


def test_stale_heading_scores_lower_than_updated_heading():
    """Reporting the OLD heading (as if it never updated after the turn)
    implies an even more awkward Dubins reorientation than reporting the
    heading consistent with having actually turned -- verified: stale
    7.03e-18 < updated 8.20e-18."""
    def score_for(reported_heading):
        kin = KinematicHistory()
        kin.score_and_update(AGENT, _track(0, 0, 0, 15, 0), _xy(0, 0), DT,
                             headings=[_HEADING_STRAIGHT_DEG], vehicle_probs=[1.0])
        return kin.score_and_update(
            AGENT, _track(0, *_TURN_XY, 15, 0), _xy(*_TURN_XY), DT,
            headings=[reported_heading], vehicle_probs=[1.0],
        )[0]

    stale = score_for(_HEADING_STRAIGHT_DEG)          # still "facing" old direction
    updated = score_for(_HEADING_ALIGNED_WITH_TURN_DEG)  # facing the turn direction
    assert 0.0 < stale < updated


def test_curvature_score_neutral_when_stationary():
    """No reliable heading to judge a turn against when the track wasn't
    moving -- curvature term contributes nothing, only jerk energy does."""
    kin = KinematicHistory()
    kin.score_and_update(AGENT, _track(0, 0, 0, 0, 0), _xy(0, 0), DT,
                         vehicle_probs=[1.0])
    kds = kin.score_and_update(
        AGENT, _track(0, 0.5, 0, 0, 0), _xy(0.5, 0), DT, vehicle_probs=[1.0]
    )
    assert kds[0] > 0.0  # not zeroed out by a spurious curvature violation


# --- state handling ---------------------------------------------------------------

def test_prediction_uses_kalman_state_not_reported_position():
    """The saved state is the Kalman-corrected values from TrackData, not the
    raw det_xy: pass mismatching values and verify the score follows the
    Kalman side."""
    kin = KinematicHistory()
    # Kalman says (0,0) even though the raw report was (100, 100)
    kin.score_and_update(AGENT, _track(0, 0.0, 0.0, 0.0, 0.0), _xy(100, 100), DT)
    # next report near the KALMAN prediction (0,0) scores well (verified 0.825
    # for this exact 0.5 m offset: half the 0.95 m tolerance at dt=0.5s)...
    kds = kin.score_and_update(
        AGENT, _track(0, 0.5, 0.0, 0.0, 0.0), _xy(0.5, 0.0), DT
    )
    assert kds[0] > 0.8
    # ...whereas near the previous RAW report it would have scored ~0.


def test_agents_isolated():
    """Track ids are per-agent: agent 3's track 0 has no history from agent 2."""
    kin = KinematicHistory()
    kin.score_and_update(2, _track(0, 0, 0, 0, 0), _xy(0, 0), DT)
    assert kin.score_and_update(3, _track(0, 99, 99, 0, 0), _xy(99, 99), DT) == [1.0]


def test_stale_track_state_dropped_when_absent():
    """A track missing from one frame loses its saved state (new_state only
    carries current tracks), so its reappearance cannot be judged."""
    kin = KinematicHistory()
    kin.score_and_update(AGENT, _track(0, 0, 0, 0, 0), _xy(0, 0), DT)
    empty = TrackData([], [], [], [], [])
    kin.score_and_update(AGENT, empty, np.empty((0, 2)), DT)
    assert kin.score_and_update(
        AGENT, _track(0, 99, 99, 0, 0), _xy(99, 99), DT
    ) == [1.0]
