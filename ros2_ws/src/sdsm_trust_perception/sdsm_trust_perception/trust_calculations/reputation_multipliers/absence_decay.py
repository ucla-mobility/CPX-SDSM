# Ported verbatim (pure Python, no ROS deps) from CPX-Mono's
# ros2/src/global_trust_perception/global_trust_perception/trust_calculations/reputation_multipliers/absence_decay.py
# — see that repo for the full design writeup. Only the reputation import was
# repointed at this package; no logic changed.
"""
Absence-gap decay multiplier for inter-session reputation seeding.

When a previously seen agent reappears after a gap (no frames processed
during the absence), its stored reputation is decayed toward baseline
before being used as the starting point for the new session.

Formula (applied once at re-seed time):
    eff_gap_s = max(0, gap_s - GRACE_GAP_S)
    R_seeded = baseline + (R_last - baseline) * exp(-eff_gap_s / TAU_GAP_S)

Grace period (GRACE_GAP_S = 259_200 s = 3 days): reputation is held fixed
for the first 3 days of absence. Decay only begins once that window elapses,
so a brief offline period carries no penalty.

Downward-only: only reputations above baseline (0.5) decay. A reputation
already below baseline is returned unchanged — silence must not reward a
distrusted agent by pulling it toward neutral.

Timescale calibration (TAU_GAP_S = 576_000 s):
    half-life of the above-baseline deviation ≈ 4.6 days (measured from grace end)
    after 30 days total absence (27 days effective): deviation × exp(-27d / 6.66d) ≈ 0.017
    i.e. R=0.9 unseen for one month → R_seeded ≈ 0.507
"""

import math

from sdsm_trust_perception.trust_calculations.reputation import R_MAX, R_MIN, REPUTATION_DEFAULT

# --- Constants ---------------------------------------------------------------

# Time constant in seconds for the exponential decay of above-baseline
# reputation during an inter-session absence. Calibrated so that one month
# (~2.6 M s) of absence reduces the deviation from baseline by ~98.9 %.
# Adjust this constant to change the month-scale decay speed:
#   smaller TAU_GAP_S → faster forgetting
#   larger  TAU_GAP_S → slower forgetting
# Half-life = TAU_GAP_S * ln(2) ≈ 399_000 s ≈ 4.6 days.
TAU_GAP_S: float = 576_000.0

# Grace window in seconds before absence decay begins. An agent returning
# within this window is reseeded at its exact stored reputation with no
# penalty. Decay starts only for absences longer than this.
# 3 days × 86_400 s/day = 259_200 s.
GRACE_GAP_S: float = 259_200.0

# -----------------------------------------------------------------------------


def absence_decay(
    R_last: float,
    gap_s: float,
    baseline: float = REPUTATION_DEFAULT,
    tau_gap_s: float = TAU_GAP_S,
    grace_s: float = GRACE_GAP_S,
) -> float:
    """Return the reputation to seed for an agent returning after gap_s seconds.

    Args:
        R_last:    Most recent reputation stored in the DB for this agent.
        gap_s:     Seconds elapsed since that record was written (time.time()
                   at re-appearance minus ts of the last DB row).
        baseline:  Neutral reputation value. Default REPUTATION_DEFAULT (0.5).
        tau_gap_s: Exponential decay time constant in seconds. Default TAU_GAP_S.
        grace_s:   Grace window in seconds. No decay applied for absences at or
                   below this duration. Default GRACE_GAP_S (3 days).

    Returns:
        R_seeded in [0, 1]. Equal to R_last when gap_s <= grace_s or
        R_last <= baseline. Approaches baseline asymptotically as gap_s grows
        beyond grace_s; never crosses baseline from above (downward-only).

    Raises:
        ValueError: if gap_s < 0, R_last outside [0, 1], or tau_gap_s <= 0.
    """
    if gap_s < 0.0:
        raise ValueError(f'gap_s={gap_s} must be >= 0')
    if not (R_MIN <= R_last <= R_MAX):
        raise ValueError(f'R_last={R_last} outside [{R_MIN},{R_MAX}]')
    if tau_gap_s <= 0.0:
        raise ValueError(f'tau_gap_s={tau_gap_s} must be > 0')

    if R_last <= baseline:
        return R_last

    eff_gap_s = max(0.0, gap_s - grace_s)
    R_seeded = baseline + (R_last - baseline) * math.exp(-eff_gap_s / tau_gap_s)
    return max(R_MIN, min(R_MAX, R_seeded))
