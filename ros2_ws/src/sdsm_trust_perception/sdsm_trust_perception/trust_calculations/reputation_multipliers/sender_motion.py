# Ported verbatim (pure Python, no ROS deps) from CPX-Mono's
# ros2/src/global_trust_perception/global_trust_perception/trust_calculations/reputation_multipliers/sender_motion.py
# — see that repo for the full design writeup. No logic changed.
"""
Sender speed tracking, feeding the kinematic freshness factor.

Tracks each sender's own reported reference position (SDSM ref_pos, decoded
to local (x, y) metres by sdsm_codec) across frames with a small dedicated
Kalman filter (state [x, y, vx, vy], measurement [x, y]), and returns the
current smoothed speed estimate -- what reputation_multipliers.
kinematic_freshness multiplies by this message's own send-to-receive latency
to bound how far the sender could have moved since this message was recorded
(blind_distance = speed * latency).

Deliberately NOT based on how long since the sender's PREVIOUS message: an
earlier version of this factor predicted this message's position from the
last one and penalised the residual, which breaks down after any silence
gap longer than a constant-velocity model stays meaningful (a sender quiet
for three minutes then reporting again would look "diverged" purely from
the stale extrapolation, even though the new message itself has negligible
transmission latency and is telling the truth about where the vehicle is
right now). Tracking speed only -- never predicting or comparing position --
sidesteps that: a long gap since the last sample just means the velocity
estimate hasn't been refreshed in a while, not that the CURRENT message is
suspect.

This is deliberately independent of any SORT-style tracking pipeline: it
operates purely on fields already present on every raw SDSM broadcast
(the decoded ref position, plus the judging node's own receive timestamp),
so it has no dependency on a tracker existing, being reachable, or behaving
any particular way.

A dedicated filter, not a reused one: KalmanCentroidTracker (SORT's
per-object tracker) assumes a fixed unit timestep between update() calls,
which does not hold here -- senders broadcast at irregular intervals -- so
this filter rebuilds its state-transition matrix F (and scales process noise
Q) from the actual measured dt every call instead of assuming one.
"""

import numpy as np
from filterpy.kalman import KalmanFilter

# --- Kalman tuning ------------------------------------------------------------
# Initial state uncertainty: velocity is seeded at 0 (unknown), so its slice
# of P starts wider than position's, letting the first few updates correct it
# quickly rather than clinging to the zero guess.
_POSITION_NOISE_SCALE     = 10.0
_INITIAL_VEL_UNCERTAINTY  = 10.0   # extra P multiplier on top of the above

# Process noise on velocity, scaled by dt each call: velocity is modelled as
# near-constant between messages, so this should be small relative to the
# measurement noise below.
_PROCESS_NOISE_VEL = 0.01

# Measurement noise on x, y: matches typical V2X-grade GNSS position jitter.
_MEASUREMENT_NOISE_POS = 1.0


class SenderMotionHistory:
    """
    Per-agent Kalman filter over a sender's own reference position, tracking
    speed only.

    update() must be called with each sender's messages in receive order;
    out-of-order or duplicate timestamps (dt <= 0) are skipped rather than
    fed to the filter, since a non-positive dt can't be predicted over --
    the last valid speed estimate is returned unchanged in that case.
    """

    def __init__(self):
        self._filters: dict[int, KalmanFilter] = {}
        self._last_t: dict[int, float] = {}

    def _new_filter(self, x: float, y: float) -> KalmanFilter:
        kf = KalmanFilter(dim_x=4, dim_z=2)
        kf.x = np.array([[x], [y], [0.0], [0.0]])
        kf.F = np.eye(4)   # [0,2] and [1,3] (the dt terms) filled in per call
        kf.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)
        kf.P *= _POSITION_NOISE_SCALE
        kf.P[2:, 2:] *= _INITIAL_VEL_UNCERTAINTY
        kf.R *= _MEASUREMENT_NOISE_POS
        kf.Q = np.zeros((4, 4))   # [2,2] and [3,3] filled in per call
        return kf

    @staticmethod
    def _speed(kf: KalmanFilter) -> float:
        return float(np.hypot(kf.x[2, 0], kf.x[3, 0]))

    def update(self, agent_id: int, ref_x: float, ref_y: float, recv_t: float) -> float:
        """
        Advance this sender's filter one step; return the current smoothed
        speed estimate in m/s.

        First-ever message for this agent_id: no prior samples to estimate
        velocity from, so the filter is seeded here and 0.0 (unknown speed,
        treated as stationary) is returned.

        Out-of-order or duplicate messages (recv_t <= the last one seen for
        this agent_id) are skipped: a non-positive dt can't be predicted
        over, so the last known speed estimate is returned unchanged.
        """
        kf = self._filters.get(agent_id)
        if kf is None:
            self._filters[agent_id] = self._new_filter(ref_x, ref_y)
            self._last_t[agent_id] = recv_t
            return 0.0

        dt = recv_t - self._last_t[agent_id]
        if dt <= 0.0:
            return self._speed(kf)

        kf.F[0, 2] = dt
        kf.F[1, 3] = dt
        q = _PROCESS_NOISE_VEL * dt
        kf.Q[2, 2] = q
        kf.Q[3, 3] = q

        kf.predict()
        kf.update(np.array([[ref_x], [ref_y]]))
        self._last_t[agent_id] = recv_t
        return self._speed(kf)
