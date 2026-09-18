# Ported verbatim (pure Python, no ROS deps) from CPX-Mono's
# ros2/src/global_trust_perception/global_trust_perception/trust_calculations/reputation_multipliers/kinematic_freshness.py
# — see that repo for the full design writeup. Only the reputation import
# was repointed at this package; no logic changed.
"""
Kinematic freshness factor F for the Stage-0 trust gate.

Replaces the original time-based message-latency freshness (decay purely on
send-to-receive delay) with a hybrid: how far the sender's own tracked speed
could have carried it during THIS message's own transmission latency.

    blind_distance_m = speed_ms * latency_s
    effective_distance = max(0, blind_distance_m - GRACE_M)
    F = max(F_FLOOR, RHO_DIST ** (effective_distance / D_THRESHOLD_M))

speed_ms comes from reputation_multipliers.sender_motion.SenderMotionHistory
(a smoothed estimate of the sender's own speed, from consecutive reference
positions). latency_s comes from the sdsm_codec's send_latency_of (the
message's own send stamp vs. ego's receive wall-clock) -- send/receive
timestamps are TRUSTED inputs here, a separate concern from this factor
(spoofing a stamp is not what this gates against; see below).

WHY LATENCY, NOT GAP-SINCE-LAST-MESSAGE
----------------------------------------
An earlier version of this factor predicted this message's position from
the sender's PREVIOUS one and penalised the residual. That breaks down after
any silence longer than a constant-velocity model stays meaningful: a sender
quiet for three minutes then reporting again would look "diverged" purely
from extrapolating over a meaningless gap, even though the new message
itself has negligible transmission latency and is telling the truth about
where the vehicle is right now. Multiplying tracked speed by THIS message's
own latency sidesteps that entirely -- it only depends on how long this
specific snapshot took to arrive, never on how long since the previous one.

WHY THIS DOESN'T CATCH A SPOOFED TIMESTAMP
-------------------------------------------
A dishonest sender can claim a small send-to-receive latency the same way
it could under the pure time-based version. That is deliberately out of
scope for this factor: timestamps are assumed true here, since verifying
them is a different trust concern (Stage 1b's kinematic_checks.py judges
physical plausibility of reported OBJECT positions against prior track
history, which is much harder to fake without a physically coherent lie).
This factor's job is narrower: given trusted timing, bound how stale the
POSITION information in this message can be.

THE FLOOR
---------
Same role F_FLOOR played in every version of this factor: bounds F for any
consumer other than the gate, prevents underflow to literal 0.0 at extreme
blind distance, and guarantees a sufficiently blind-distant sender ALWAYS
fails the gate no matter its reputation (F_FLOOR < R_STAR is asserted at
import, so R_eff = R * F_FLOOR <= F_FLOOR < R_STAR for any R <= 1).
"""

from sdsm_trust_perception.global_trust_perception.trust_calculations.reputation import trust_fixed_point

# --- Constants ---------------------------------------------------------------

# Flat allowance for negligible blind distance (speed-estimate noise,
# quantisation) before the decay starts. A stationary or near-stationary
# sender (speed ~= 0) never accumulates meaningful blind distance regardless
# of latency, so this mostly guards against GPS/estimate jitter at low speed.
GRACE_M: float = 1.0

# Decay base per D_THRESHOLD_M of effective blind distance (hand-tuned, same
# role RHO_FRESH played in the time-based version): 0.5 means F halves every
# D_THRESHOLD_M metres past the grace band. Must be in (0, 1).
RHO_DIST: float = 0.5

# Scale of the exponential decay: how many metres of effective blind
# distance it takes for F to fall by one factor of RHO_DIST.
D_THRESHOLD_M: float = 5.0

# Lower bound on F. Must stay below R_STAR (asserted at import) -- same
# invariant every previous version of this factor relied on.
F_FLOOR: float = 0.5

# Maximum negative latency treated as cross-host clock skew and clamped to
# 0. GPS-disciplined V2X clocks disagree by far less; anything beyond this
# is a timestamping bug, not skew, and raises ValueError. (Only guards
# against measurement noise in the trusted timestamps -- see module
# docstring: WHY THIS DOESN'T CATCH A SPOOFED TIMESTAMP.)
CLOCK_SKEW_TOLERANCE_S: float = 0.010

# --- Import-time validation of the tuned constants ---------------------------
# Validated ONCE here rather than on every call: kinematic_freshness_factor
# deliberately has no tuning parameters, so the only way to retune is to edit
# these constants, and the only place they can be wrong is at import.
if GRACE_M < 0.0:
    raise ValueError(f'GRACE_M={GRACE_M} must be >= 0')
if not (0.0 < RHO_DIST < 1.0):
    raise ValueError(f'RHO_DIST={RHO_DIST} must be in (0, 1)')
if D_THRESHOLD_M <= 0.0:
    raise ValueError(f'D_THRESHOLD_M={D_THRESHOLD_M} must be > 0')
if not (0.0 <= F_FLOOR < 1.0):
    raise ValueError(f'F_FLOOR={F_FLOOR} must be in [0, 1)')
if CLOCK_SKEW_TOLERANCE_S < 0.0:
    raise ValueError(f'CLOCK_SKEW_TOLERANCE_S={CLOCK_SKEW_TOLERANCE_S} must be >= 0')

# The gate crossover R* = tau(R*), imported from the tau math rather than
# hardcoded so this floor tracks any retuning of the threshold curve.
R_STAR: float = trust_fixed_point()

assert F_FLOOR < R_STAR, (
    f'F_FLOOR={F_FLOOR} must stay below R_STAR={R_STAR:.4f}; '
    'see module docstring (THE FLOOR)'
)

# -----------------------------------------------------------------------------


def kinematic_freshness_factor(speed_ms: float, latency_s: float) -> float:
    """Return F from how far speed_ms * latency_s could have carried the sender.

    Reads the module constants directly -- no tuning parameters beyond the
    two arguments. Retuning happens by editing the constants (GRACE_M,
    RHO_DIST, D_THRESHOLD_M, F_FLOOR, CLOCK_SKEW_TOLERANCE_S), validated
    once at import.

    Args:
        speed_ms: the sender's current tracked speed in m/s (sender_motion.
            SenderMotionHistory.update()). Must be >= 0.
        latency_s: this message's send-to-receive latency in seconds
            (sdsm_codec.send_latency_of()). Small negatives
            (>= -CLOCK_SKEW_TOLERANCE_S) are clamped to 0 as cross-host
            clock skew; more negative raises.

    Returns:
        F in [F_FLOOR, 1]. Exactly 1.0 when speed_ms * latency_s <= GRACE_M
        (including whenever speed_ms == 0, regardless of latency). Approaches
        F_FLOOR (< R_STAR, gate-failing for every sender) as the blind
        distance grows, since RHO_DIST < 1.

    Raises:
        ValueError: if speed_ms < 0 (a caller bug -- speed is a magnitude),
                    or latency_s < -CLOCK_SKEW_TOLERANCE_S (a timestamping
                    bug, not skew).
    """
    if speed_ms < 0.0:
        raise ValueError(f'speed_ms={speed_ms} must be >= 0')
    if latency_s < -CLOCK_SKEW_TOLERANCE_S:
        raise ValueError(
            f'latency_s={latency_s} below -{CLOCK_SKEW_TOLERANCE_S}: '
            'negative beyond clock-skew tolerance indicates a timestamping bug'
        )

    latency_s = max(0.0, latency_s)   # clamp tolerated skew to "arrived now"
    blind_distance_m = speed_ms * latency_s
    effective_distance = max(0.0, blind_distance_m - GRACE_M)
    return max(F_FLOOR, RHO_DIST ** (effective_distance / D_THRESHOLD_M))
