"""
Kinematic-Dynamic Consistency Score (KDS): approximates CooperFuse Ch. 2
Sec 2.3.4 (Zheng, "Cooperative Perception for Safer Smart Intersections").

Between frames, each tracked object's reported position is checked against a
minimum-jerk trajectory fit from the previous Kalman state (position +
velocity) to the reported current position, times a bounded-curvature
plausibility ratio derived from real Dubins path geometry (car-like, bounded
turning radius). The result is a continuous score, KDS, in (0, 1].

FROM THE PAPER (eq 2.2-2.4):
  - minimize the integral of squared jerk (eq 2.3) subject to endpoint
    states x(0) in X_track (the previous tracked state) and x(T) = the
    candidate's current state (eq 2.4).
  - the energy -> score conversion is explicitly left unspecified by the
    paper ("the resulting energy value is converted into a kinematic and
    dynamic consistency score"); here it is KDS_jerk = exp(-J / J0(T)),
    with the energy scale J0 stated as a tolerance on unexplained
    displacement and derived per frame period -- see THE TOLERANCE below.

APPROXIMATED, NOT PORTED (also unspecified by the paper: solver, polynomial
order, waypoint sampling):
  - the paper additionally threads the path through Reed-Shepp waypoints
    x(t_i) in X_RS, i.e. a true nonholonomic path plan reachable with
    forward AND backward motion. This module instead computes the shortest
    DUBINS path connecting the previous heading to the candidate's reported
    heading -- Dubins is the forward-only special case of Reed-Shepp (no
    reversal / "three-point-turn" maneuvers). Reversal words were left out
    deliberately, not for lack of trying: their tangent-point geometry is
    materially harder to derive correctly than the CSC words used here, and
    could not be verified with the same confidence (the 4 CSC words below
    were checked by simulating the actual arc-line-arc path across 2000
    random pose pairs and confirming it lands on the goal pose, with zero
    failures; an equivalent check for the reversal words was not achieved).
    KDS_curvature = straight_line_distance / shortest_Dubins_length, a ratio
    in (0, 1] -- 1.0 when a straight line already satisfies both heading
    constraints, falling as more curvature-constrained detour is required.
    KDS_curvature is a curvature PROXY, not itself part of the jerk
    integral -- it is not the same quantity as J in eq 2.3-2.4, just a
    separate term motivated by the same nonholonomic constraint the paper's
    Reed-Shepp waypoints enforce. The jerk integral is fully captured by
    KDS_jerk alone.
  - KDS = KDS_jerk * KDS_curvature_eff: the two are kept as separate
    multiplicative terms rather than summed into one energy, because they
    live on very different natural scales (jerk energy grows as 1/T^5 and
    reaches 1e5-1e8 over the frame periods this runs at; the curvature is
    dimensionless and bounded in (0, 1]) and summing them would let one
    silently swamp the other.

Flagged detections lose weight rather than being dropped: callers multiply
KDS into the fusion blend (orientation only) and into how much a matched
pair counts as corroborating evidence, rather than hard-gating either.

CLASS GATING IS A HARD SWITCH ON THE REPORTED LABEL, NOT STATISTICS.
An earlier version of this module blended KDS_curvature continuously by
P(vehicle), synthesized from a class-probability vector built out of the
reported obj_type label plus a general detection-confidence scalar
(obj_local_scores). That scalar measures whether an object exists, not how
confident the classifier is about WHICH class it is -- SDSM does not carry
a genuine per-class probability vector, so that synthesis was an
unjustified proxy dressed up as a soft distribution. This module now trusts
the reported label directly:

    KDS_curvature_eff = 1 - P(vehicle) * (1 - KDS_curvature_raw)
    P(vehicle) in {0.0, 1.0}, from the caller's raw obj_type label

The formula is unchanged (and still float-typed, not bool, so an affine
floor -- P(vehicle) = beta + (1-beta)*hard_indicator, a deliberate addition
disclosed as not-Zheng's, not something derived from the paper -- could be
layered in later without an interface change) -- what changed is that
P(vehicle) is no longer synthesized from data that doesn't actually measure
classification confidence.
"""

import math
from typing import NamedTuple, Optional

import numpy as np

# --- THE TOLERANCE: design choices not specified by the paper -------------
# The energy -> score map is stated as a TOLERANCE ON UNEXPLAINED DISPLACEMENT
# and the energy scale derived from it, rather than the other way round.
#
# The two are the same map. Under the constant-velocity boundary conditions
# used in score_and_update (v1 = v0, the previous velocity is all that is
# known), the closed form below collapses to J = 720*d^2/T^5, where
# d = |p1 - p0 - v0*T| is the displacement constant-velocity motion does not
# explain. So for any tolerance d_tol(T),
#
#     exp(-J / J0(T))  ==  2 ** -(d / d_tol(T))**2,
#     J0(T) = 720 * d_tol(T)**2 / (T**5 * ln 2)
#
# and KDS_jerk is exactly 0.5 at d = d_tol(T), by construction.
#
# WHY J0 IS NOT A CONSTANT. Fixing J0 fixes an ENERGY, and energy carries
# T^-5, so the displacement a fixed J0 tolerates scales as T^2.5 -- a factor
# of 411 between dt=0.5s and dt=0.15s. The previous constant (130000.0, tuned
# for a 2 m tolerance at dt=0.5s) therefore tolerated 9.7 cm at the replay
# pipeline's actual flush period, and 5.4 cm at the senders' true frame
# period -- BELOW the 0.1 m grid SDSM positions arrive on
# (sdsm_units.OFFSET_UNIT_M). An honest, perfectly tracked object stepping one
# unit on the wire scored 0.089 and was demoted; on clean recorded data with
# no attackers the check demoted 64% of matched pairs. Expressed as a velocity
# error the same constant swung from 0.13 m/s at dt=0.05s to 11.2 m/s at
# dt=1.0s, which is not a threshold anyone can reason about.
#
# The two terms below are the two reasons a truthful report misses its
# constant-velocity prediction, and they scale differently in T:
_KDS_NOISE_M = 0.20                # error that does NOT shrink with T: the
                                   # 0.1 m wire grid plus centroid jitter. It
                                   # is there whether you look after 50 ms or
                                   # after a second.
_KDS_A_MAX_MS2 = 6.0               # hard braking / aggressive accel for a
                                   # car. The 0.5*a*T^2 term is what an
                                   # unmodelled but BOUNDED acceleration can
                                   # add on top of constant velocity -- real
                                   # physics, so it scales as T^2.
# Resulting tolerance stays in a 1.6-4 m/s band across dt = 0.05..1.0 s, and
# still scores a 0.5 m single-frame teleport at 0.05 and a 2 m one at 0.0.
_MIN_TURN_RADIUS_M = 5.0           # Dubins turning radius, car-like agent
_MIN_DT_S = 1e-3                   # below this, the quintic solve is ill-conditioned
_MIN_SPEED_FOR_HEADING = 1e-3      # below this, velocity direction is unreliable

# J2735 heading sentinel: 28800 * 0.0125 deg = 360 -> "unavailable" (matches
# mmcooper_fuse/adapter.py's _HEADING_UNAVAILABLE_DEG).
_HEADING_UNAVAILABLE_DEG = 360.0


class TrackData(NamedTuple):
    """Per-detection track state forwarded from a TrackUpdate message.

    All lists are parallel to the corresponding agent's detection list.
    Populated by agent.py from the tracker node's TrackUpdate; passed into
    TrustEngine.process_frame so the trust pipeline stays ROS-free.
    """
    track_ids: list
    kalman_x: list
    kalman_y: list
    kalman_vx: list
    kalman_vy: list


def _min_jerk_energy_1d(p0: float, v0: float, p1: float, v1: float, T: float) -> float:
    """integral_0^T p'''(t)^2 dt for the minimum-jerk quintic p(t) connecting
    (p0, v0) to (p1, v1) over [0, T] with zero acceleration at both ends
    (p(0)=p0, p'(0)=v0, p''(0)=0, p(T)=p1, p'(T)=v1, p''(T)=0).

    Closed form, in one step: both the quintic coefficients and the integral
    of the squared jerk (a quartic in t) have exact solutions, so there is no
    3x3 solve and no numerical quadrature. The expression was derived
    symbolically (sympy) from those same boundary conditions and is pinned in
    test_kinematic_checks against a direct fit-and-integrate reference across
    random inputs, which is what retires the transcription-error risk that
    motivated the earlier numerical solve.
    """
    return 48.0 * (
        4.0 * T ** 2 * v0 ** 2 + 7.0 * T ** 2 * v0 * v1 + 4.0 * T ** 2 * v1 ** 2
        + 15.0 * T * p0 * (v0 + v1) - 15.0 * T * p1 * (v0 + v1)
        + 15.0 * (p0 - p1) ** 2
    ) / T ** 5


def _unexplained_tolerance_m(dt: float, noise_m: float, a_max_ms2: float) -> float:
    """Unexplained displacement at which KDS_jerk is exactly 0.5, in metres.

    See THE TOLERANCE above for where the two terms come from.
    """
    return noise_m + 0.5 * a_max_ms2 * dt * dt


def _jerk_energy_scale(dt: float, noise_m: float, a_max_ms2: float) -> float:
    """J0 for this frame period: the energy at which KDS_jerk reaches 0.5.

    Derived from the displacement tolerance rather than tuned directly, so
    the quantity actually being chosen is in metres. Only defined for
    dt > 0; callers gate on _MIN_DT_S before reaching it.
    """
    d_tol = _unexplained_tolerance_m(dt, noise_m, a_max_ms2)
    return 720.0 * d_tol * d_tol / (dt ** 5 * math.log(2.0))


def _yaw_from_j2735(heading_deg) -> Optional[float]:
    """J2735 heading (deg, 0=North, clockwise) -> math yaw (rad, 0=+x East,
    counter-clockwise). Returns None for missing/unavailable headings rather
    than fabricating a yaw of 0 -- unlike adapter.py's _yaw_of (which
    defaults unavailable to 0 for display geometry), a consistency check
    must not silently judge curvature against a heading it doesn't have.
    """
    if heading_deg is None or float(heading_deg) >= _HEADING_UNAVAILABLE_DEG:
        return None
    return math.radians(90.0 - float(heading_deg))


def _mod2pi(angle: float) -> float:
    """Wrap an angle to [0, 2*pi)."""
    return angle % (2.0 * math.pi)


def _left_center(x: float, y: float, theta: float, r: float) -> tuple[float, float]:
    return (x - r * math.sin(theta), y + r * math.cos(theta))


def _right_center(x: float, y: float, theta: float, r: float) -> tuple[float, float]:
    return (x + r * math.sin(theta), y - r * math.cos(theta))


def _dubins_csc_length(x0: float, y0: float, th0: float,
                       x1: float, y1: float, th1: float,
                       r: float, turn0: str, turn1: str) -> Optional[float]:
    """Length of one CSC (circle-straight-circle) Dubins word connecting two
    oriented poses at turning radius r. turn0/turn1 in {'L', 'R'} pick which
    way the vehicle turns at the start/goal.

    Returns None if this particular word is not geometrically realisable for
    this pair of poses -- only possible for the opposite-handed words (one
    'L' one 'R'), when the two turning circles are closer than 2r and no
    internal tangent line exists between them.
    """
    C1 = _left_center(x0, y0, th0, r) if turn0 == 'L' else _right_center(x0, y0, th0, r)
    C2 = _left_center(x1, y1, th1, r) if turn1 == 'L' else _right_center(x1, y1, th1, r)
    D = math.hypot(C2[0] - C1[0], C2[1] - C1[1])
    if D < 1e-9:
        return 0.0
    phi0 = math.atan2(C2[1] - C1[1], C2[0] - C1[0])

    if turn0 == turn1:
        # Same-handed (external tangent): for equal radii the tangent line
        # is parallel to the centre-centre line, and its length is D itself.
        phi_options = [(phi0, D)]
    else:
        # Opposite-handed (internal tangent): the tangent line crosses
        # between the circles. Its angular offset from the centre-centre
        # line is beta = asin(2r/D) (right triangle: hypotenuse D, opposite
        # side 2r), and its length is the adjacent side, D*cos(beta) --
        # NOT D (verified numerically; using D directly here was an earlier
        # bug in this derivation, caught by simulating the resulting path).
        if D < 2.0 * r:
            return None
        beta = math.asin(min(1.0, 2.0 * r / D))
        seg = D * math.cos(beta)
        phi_options = [(phi0 + beta, seg), (phi0 - beta, seg)]

    best = None
    for phi_t, seg in phi_options:
        # Tangent point angle (circle-relative): the vehicle's velocity
        # direction at circle-relative angle alpha is (alpha - 90 deg) for a
        # clockwise/right circle and (alpha + 90 deg) for a counter-
        # clockwise/left circle; solving velocity == phi_t gives alpha.
        a1 = phi_t + math.pi / 2 if turn0 == 'R' else phi_t - math.pi / 2
        a2 = phi_t + math.pi / 2 if turn1 == 'R' else phi_t - math.pi / 2
        start_a = th0 + math.pi / 2 if turn0 == 'R' else th0 - math.pi / 2
        goal_a = th1 + math.pi / 2 if turn1 == 'R' else th1 - math.pi / 2
        arc1 = r * _mod2pi(start_a - a1) if turn0 == 'R' else r * _mod2pi(a1 - start_a)
        arc2 = r * _mod2pi(a2 - goal_a) if turn1 == 'R' else r * _mod2pi(goal_a - a2)
        total = arc1 + seg + arc2
        if best is None or total < best:
            best = total
    return best


_DUBINS_WORDS = (('R', 'R'), ('L', 'L'), ('R', 'L'), ('L', 'R'))


def _dubins_path_length(x0: float, y0: float, th0: float,
                        x1: float, y1: float, th1: float, r: float) -> float:
    """Shortest of the 4 Dubins CSC words connecting two oriented poses.
    Always resolves to a finite value: the same-handed words (RSR, LSL) are
    geometrically realisable for any pair of poses."""
    lengths = [
        _dubins_csc_length(x0, y0, th0, x1, y1, th1, r, t0, t1)
        for t0, t1 in _DUBINS_WORDS
    ]
    return min(L for L in lengths if L is not None)


def _curvature_score(px: float, py: float, th0: float,
                     rx: float, ry: float, th1: float,
                     min_turn_radius_m: float) -> float:
    """Ratio in (0, 1]: straight-line distance over the shortest Dubins path
    length connecting (px, py, th0) to (rx, ry, th1) at the given turning
    radius. 1.0 when a straight line already satisfies both heading
    constraints; falls as more curvature-constrained detour is required to
    reconcile the reported heading change (a Dubins path can never be
    shorter than the straight-line distance, so this ratio is always in
    (0, 1])."""
    dist = math.hypot(rx - px, ry - py)
    if dist < 1e-6:
        return 1.0  # no displacement to judge
    L = _dubins_path_length(px, py, th0, rx, ry, th1, min_turn_radius_m)
    if L < 1e-9:
        return 1.0
    return min(1.0, dist / L)


class KinematicHistory:
    """
    Stores per-agent, per-track (x, y, vx, vy) between frames.

    After each Sort.update() call, pass the returned TrackData and the raw
    detection positions to score_and_update(). It returns a continuous KDS
    per detection: 1.0 = fully consistent, lower = less kinematically
    plausible given the previous tracked state.
    """

    def __init__(self,
                noise_m: float = _KDS_NOISE_M,
                a_max_ms2: float = _KDS_A_MAX_MS2,
                min_turn_radius_m: float = _MIN_TURN_RADIUS_M):
        # Replaces the previous single `j0`: the energy scale is now derived
        # per frame period from these two, so a fixed J0 is no longer a
        # meaningful thing to hand in. See THE TOLERANCE above.
        self.noise_m = noise_m
        self.a_max_ms2 = a_max_ms2
        self.min_turn_radius_m = min_turn_radius_m
        # agent_id -> track_id -> (x, y, vx, vy) saved after Sort.update()
        self._prev: dict[int, dict[int, tuple[float, float, float, float]]] = {}

    def score_and_update(self,
                         agent_id: int,
                         track_data: TrackData,
                         det_xy: np.ndarray,
                         dt: float,
                         headings: Optional[list] = None,
                         vehicle_probs: Optional[list] = None) -> list[float]:
        """
        Score each detection for this agent by how much jerk energy a
        minimum-jerk trajectory would need to explain moving from the
        previous Kalman state to the reported position, times a Dubins
        curvature-plausibility ratio (soft-gated by vehicle-class
        probability) if a reported heading is available.

        track_data    : TrackData parallel to det_xy (from the tracker node)
        det_xy        : (N, 2) array of raw reported [x, y] positions, metres
        dt            : seconds elapsed since the previous call for this agent
        headings      : optional J2735 heading degrees per detection. Without
                        it (or with the unavailable sentinel), the curvature
                        term is neutral (1.0) -- can't judge a heading change
                        without a heading.
        vehicle_probs : optional per-detection P(this is a vehicle), in
                        {0.0, 1.0} in practice -- the caller's raw obj_type
                        label, not a statistical synthesis (see module
                        docstring). Float-typed rather than bool so an
                        affine floor could be added upstream later without
                        an interface change. None -> P(vehicle)=1 for every
                        detection, the same conservative default used
                        before class data was wired into this pipeline.

        Returns list[float] per detection: KDS in (0, 1]. New tracks (no
        previous-frame history) or dt too small always return the neutral
        1.0 -- can't judge, assume consistent.
        """
        prev = self._prev.get(agent_id, {})
        results: list[float] = []
        new_state: dict[int, tuple[float, float, float, float]] = {}

        # One frame period, so one energy scale for every detection here.
        j0 = (_jerk_energy_scale(dt, self.noise_m, self.a_max_ms2)
              if dt > _MIN_DT_S else 0.0)

        for i in range(len(track_data.track_ids)):
            tid = track_data.track_ids[i]
            rx, ry = float(det_xy[i][0]), float(det_xy[i][1])
            p_vehicle = 1.0 if vehicle_probs is None else float(vehicle_probs[i])
            th1 = _yaw_from_j2735(headings[i]) if headings is not None else None

            if tid in prev and dt > _MIN_DT_S:
                px, py, vx, vy = prev[tid]
                J = (_min_jerk_energy_1d(px, vx, rx, vx, dt)
                     + _min_jerk_energy_1d(py, vy, ry, vy, dt))
                kds_jerk = math.exp(-J / j0)

                speed = math.hypot(vx, vy)
                if th1 is not None and speed > _MIN_SPEED_FOR_HEADING:
                    th0 = math.atan2(vy, vx)
                    kds_curv_raw = _curvature_score(
                        px, py, th0, rx, ry, th1, self.min_turn_radius_m
                    )
                else:
                    kds_curv_raw = 1.0  # no reliable heading to judge against

                kds_curv_eff = 1.0 - p_vehicle * (1.0 - kds_curv_raw)
                kds = kds_jerk * kds_curv_eff
            else:
                kds = 1.0  # no history, or dt too small -> assume consistent

            results.append(kds)
            # Save Kalman-corrected state (post-update) for next frame
            new_state[tid] = (
                float(track_data.kalman_x[i]),
                float(track_data.kalman_y[i]),
                float(track_data.kalman_vx[i]),
                float(track_data.kalman_vy[i]),
            )

        self._prev[agent_id] = new_state
        return results
