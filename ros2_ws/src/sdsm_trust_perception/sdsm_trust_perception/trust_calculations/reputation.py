# Ported (pure Python, no ROS deps) from CPX-Mono's
# ros2/src/global_trust_perception/global_trust_perception/trust_calculations/reputation.py
# — see that repo for the full design writeup.
#
# ONE DELIBERATE DEVIATION FROM THE SOURCE FILE: dynamic_threshold() there
# currently has its real formula commented out and unconditionally
# `return 0.6` ("temporary: fixed threshold in place of dynamic tau(R)"), so
# every sender is gated at a flat 0.6 regardless of reputation. That stub is
# NOT ported here — this copy restores the documented formula below, since a
# threshold that never adapts to reputation is not a meaningful second-layer
# trust check. If CPX-Mono's stub was intentional/WIP, reconcile before
# treating the two pipelines as equivalent.
"""
Reputation scoring: the per-agent trust value and its dynamics.

Pure, dependency-free formulas for
  - dynamic_threshold(R) : the acceptance threshold tau as a function of an
                           agent's reputation R, and
  - reputation_update(...) : how R moves given a frame's correct/incorrect
                             counts (plus Solution-C decay on empty frames).

Reputation R is in [0, 1], default 0.5 for unknown agents. These functions
know nothing about detections, matching, or messages - they operate purely
on R and integer counts, so they stay trivially unit-testable.
"""


# --- Dynamic threshold tau(R) = tau_min + (tau_max - tau_min) * (1 - R^gamma)
TAU_MIN = 0.30   # threshold floor (even perfectly reputable agents)
TAU_MAX = 0.90   # threshold for zero-reputation strangers
GAMMA   = 2.0    # strictness penalty; >1 = stricter, =1 = linear

# --- Defaults
REPUTATION_DEFAULT = 0.5  # initial R for unknown agents; decay baseline

# --- Optimistic-trust tier bound (output admission, phase 2)
# An agent at or above HIGH_TRUST has earned immediate belief: its detections
# pass to the fused output even without cross-agent corroboration, so a proven
# sensor's late-breaking object (a pedestrian stepping out of an occlusion)
# acts without waiting for a second witness. Agents in [tau, HIGH_TRUST) only
# pass their CORROBORATED objects; uncorroborated ones are muted from the output
# (but still graded), which mutes a fresh R=0.5 Sybil's ghosts immediately.
HIGH_TRUST = 0.95

# --- Reputation clip bounds
R_MIN, R_MAX = 0.0, 1.0

# --- Reputation update constants
DELTA      = 0.10   # How fast R moves per valid data frame
DECAY_RATE = 0.003  # Solution C: drift toward baseline per empty frame

def dynamic_threshold(R: float,
                      tau_min: float = TAU_MIN,
                      tau_max: float = TAU_MAX,
                      gamma: float = GAMMA) -> float:
    """
    tau_dynamic(R) = tau_min + (tau_max - tau_min) * (1 - R^gamma)

    - R=1 (perfectly reputable) -> tau_min
    - R=0 (stranger)            -> tau_max
    - gamma controls curve shape.

    Raises ValueError if R is outside [0,1] or the hyperparameters are
    inconsistent (gamma <= 0, tau_min > tau_max). Internal callers always
    pass an already-clipped R; out-of-range input means a pipeline bug,
    so fail loudly instead of silently clamping.
    """
    if not (R_MIN <= R <= R_MAX):
        raise ValueError(f'R={R} outside [{R_MIN},{R_MAX}]')
    if gamma <= 0.0:
        raise ValueError(f'gamma={gamma} must be > 0')
    if tau_min > tau_max:
        raise ValueError(f'tau_min={tau_min} > tau_max={tau_max}')
    return tau_min + (tau_max - tau_min) * (1.0 - R ** gamma)


def trust_fixed_point(tau_min: float = TAU_MIN,
                      tau_max: float = TAU_MAX,
                      gamma: float = GAMMA) -> float:
    """
    R* solving R = dynamic_threshold(R): the trust-gate crossover.

    The Stage-0 gate admits an agent when R >= tau(R). Because tau is
    strictly decreasing in R, that inequality is equivalent to R >= R*,
    where R* is the unique fixed point computed here: the single reputation
    at which an agent exactly meets its own threshold. Consumers that need
    "the reputation value above which the gate passes" (e.g. the kinematic
    freshness floor derivation in reputation_multipliers.kinematic_freshness)
    should call this instead of hardcoding the crossover, so they track any
    retuning of tau_min / tau_max / gamma automatically.

    g(R) = R - tau(R) is strictly increasing with g(0) = -tau_max < 0 and
    g(1) = 1 - tau_min > 0 (given tau_min < 1), so bisection converges
    unconditionally; 60 iterations put the error below 1e-18.

    Raises ValueError if tau_min >= 1 (no fixed point would exist in [0,1])
    or if the parameters are rejected by dynamic_threshold.
    """
    if tau_min >= 1.0:
        raise ValueError(f'tau_min={tau_min} must be < 1 for a fixed point to exist')
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = (lo + hi) / 2.0
        if mid >= dynamic_threshold(mid, tau_min, tau_max, gamma):
            hi = mid
        else:
            lo = mid
    return (lo + hi) / 2.0


def reputation_update(R_old: float,
                      C: float,
                      I: float,
                      N_total: float,
                      held: float = 0.0,
                      delta: float = DELTA,
                      baseline: float = REPUTATION_DEFAULT,
                      decay_rate: float = DECAY_RATE) -> tuple[float, float]:
    """
    S_frame = (C - I) / N_total      in [-1, 1]
    R_new   = clip(R_old + delta * S_frame, 0, 1)

    Per spec: 'false positives rate' = (C - I) / N_total. The term FPR is
    used loosely here - it's really a per-frame correctness score.

    C, I, N_total are WEIGHTED counts (floats): a confident agreement or a
    confident mistake weighs more than a hesitant one, and back-paid /
    back-charged ledger settlements land here as fractional contributions.
    N_total = C + I keeps S_frame in [-1, 1] since C, I >= 0, so tau / delta
    need no retuning; the integer-count callers of before remain valid
    (int is a float), so this is a type-widening only.

    Solution C: if N_total == 0 (agent sent nothing scorable this frame), R
    drifts toward baseline instead of staying frozen:
        R_new = R_old + decay_rate * (baseline - R_old)
    so stale reputations reset to neutral over time.

    HELD SUSPENDS THAT DRIFT. N_total == 0 conflates two situations the
    caller can tell apart and this function could not:

        held == 0   the agent really did send nothing scorable. Drifting a
                    stale reputation back to neutral is the intent.
        held > 0    the agent sent plenty and none of it has been ADJUDICATED
                    yet -- every detection is inside the deferred ledger's
                    grace window, or is an ego_only miss it is not charged
                    for. It is participating, and R holds.

    Decaying the second case penalises an agent for having its evidence held,
    which is the pipeline's own latency and not a fact about the agent. It
    also scales the wrong way: LENGTHENING the grace window (so honest
    reports are not charged as ghosts too early) leaves detections pending
    for longer, which produced MORE N_total == 0 frames and so dragged
    reputation toward baseline harder -- the fix for one failure quietly
    driving another.

    Returns: (R_new, S_frame). S_frame is 0.0 on an empty frame, whether or
    not the drift was suspended.

    Raises ValueError on negative counts or R_old outside [0,1].
    """
    if C < 0 or I < 0 or N_total < 0:
        raise ValueError(f'negative counts: C={C} I={I} N_total={N_total}')
    if held < 0:
        raise ValueError(f'negative held={held}')
    if not (R_MIN <= R_old <= R_MAX):
        raise ValueError(f'R_old={R_old} outside [{R_MIN},{R_MAX}]')

    if N_total <= 0:
        if held > 0:
            return R_old, 0.0        # sub judice: participating, not yet judged
        R_new = R_old + decay_rate * (baseline - R_old)
        return max(R_MIN, min(R_MAX, R_new)), 0.0

    s_frame = (C - I) / float(N_total)
    R_new = R_old + delta * s_frame
    R_new = max(R_MIN, min(R_MAX, R_new))
    return R_new, s_frame
