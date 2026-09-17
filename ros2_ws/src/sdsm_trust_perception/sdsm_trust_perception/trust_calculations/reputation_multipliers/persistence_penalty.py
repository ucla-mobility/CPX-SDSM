# Ported verbatim (pure Python, no ROS deps) from CPX-Mono's
# ros2/src/global_trust_perception/global_trust_perception/trust_calculations/reputation_multipliers/persistence_penalty.py
# — see that repo for the full design writeup. Only the reputation import was
# repointed at this package; no logic changed.
"""
Persistence-penalty weight V for the Stage-0 trust gate.

V in [V_FLOOR, 1] punishes agents whose reputation keeps FALLING -- the
signature of an on-off attacker that behaves well most frames and injects
bad data on a duty cycle, banking enough reputation between hits that the
raw score R never stays low. Like the freshness factor F, V is applied ONLY
at the gate comparison (R_eff = R * F * V vs tau(R_eff)) and never modifies
the stored reputation: the persistent state that carries the memory of past
misbehaviour is the risk accumulator inside each tracker, not R itself.
(Feeding V back into stored R would make drops amplify themselves: a drop
lowers V, which would lower R, which would register as a further drop.)

The math (design doc: CPX-Mono visualizations/persistence_penalty.html,
rescaled from its 0-100 display units to this pipeline's R in [0, 1]):

  1. d_t     = R_{t-1} - R_t                     signed change; positive = drop
  2. dnorm_t = clip(d_t / d_range - deadzone, 0, 1)
               gains and sub-dead-zone jitter count as zero
  3. risk_L  = beta_L * risk_L + (1 - beta_L) * dnorm^alpha
               persistence channel: a normalized leaky EMA in [0, 1] that
               rises under sustained drops and bleeds off when they stop
  4. risk_r  = sum_{k<K} beta^k dnorm_{t-k}^alpha / sum_{k<K} beta^k
               recent-severity channel: how bad have drops been lately
  5. V_t     = clip(1 - lambda_r * risk_r^(1/alpha)
                      - lambda_L * risk_L^(1/alpha), V_floor, 1)

DERIVED, NOT TUNED
------------------
Two coefficients follow the same single-source-of-truth pattern as RHO_DIST
in kinematic_freshness.py -- you tune the meaningful quantity and the raw
coefficient is computed from it:

  beta_L   = 2 ^ (-1 / (FPS * PERSIST_HALF_LIFE_S))
    The knob is a real-time half-life: how long after drops stop until the
    accumulated risk halves. A raw beta_L slider cannot resolve long
    memories (a one-hour half-life is beta_L ~= 0.9999923).

  lambda_L = (1 - V_TARGET) / (DUTY_BAD_FRACTION * DUTY_SEVERITY) ^ (1/alpha)
    Anchored to a duty cycle declared intolerable: an attacker bad
    DUTY_BAD_FRACTION of frames at severity DUTY_SEVERITY converges to
    V = V_TARGET. Defaults: bad 30% of the time at full severity -> V = 0.5.

SELECTIVITY AND THE ONE REAL TRADE-OFF
--------------------------------------
A sustained slide saturates risk_L at 1 while a lone dip contributes only
(1 - beta_L), so persistence is punished S_L = (1 - beta_L)^(-1/alpha) times
harder than a single drop of the same step size. Raising the half-life
improves S_L but lengthens the rebound half-life m_1/2 = alpha*ln2/(-ln
beta_L) (= N_half exactly at alpha = 1) -- longer memory means slower
forgiveness. That is the only real trade-off in the design.

THE FLOOR
---------
V_FLOOR = 0.4 sits below the gate crossover R_STAR (see
kinematic_freshness.py) so a fully-floored agent fails the gate even at
R = 1, and below V_TARGET = 0.5 so the duty-cycle anchor above is actually
reachable (a floor above the target would clip the promised
convergence). The floor
keeps the worst case recoverable: once drops stop, risk decays and V climbs
back off the floor.

THE DEAD-ZONE
-------------
The tracker is fed R_old every processed frame, including empty frames where
Solution-C decay drifts R toward baseline by up to DECAY_RATE * (R - 0.5)
<= 0.0015 per frame -- a tiny perpetual "drop". DROP_DEADZONE = 0.01
(normalized; 0.004 raw at D_RANGE = 0.4) swallows that drift plus scoring
jitter, so only genuine misbehaviour feeds either channel.

GAPS AND COLD STARTS
--------------------
risk_L and the recent window are NOT decayed or cleared during absence: only
frames where the agent is actually observed advance the EMA. seed(risk_l,
last_r) initialises both the persisted risk channel and the last-R baseline
from DB; setting last_r = R_seeded (the absence-decayed reputation) makes the
first post-restart update() produce dnorm = 0, so the cross-restart R
difference is not scored as a drop.
cold_start_debias corrects the EMA's warm-up bias (risk / (1 - beta_L^t));
it becomes load-bearing only if risk_L is ever persisted per-ID across runs,
which persistent_reputation_tracker does via reputation_log (see
PersistencePenalty.seed).
"""

from collections import deque

from sdsm_trust_perception.trust_calculations.reputation import trust_fixed_point

# --- Constants ---------------------------------------------------------------

# Frames per second the tracker is fed at. Converts PERSIST_HALF_LIFE_S into
# a frame count; the flush tick is assumed to match. Tuning note: if the
# publish rate changes significantly, revisit this (same caveat as
# GRACE_M in kinematic_freshness.py).
FPS: float = 25.0

# Real-time half-life of the persistence accumulator: how long after drops
# stop until accumulated risk halves. THE memory knob -- BETA_L is derived.
PERSIST_HALF_LIFE_S: float = 0.5

# Drop scale in R units: the per-frame fall that counts as maximally severe
# (dnorm = 1). Set near a real bad-day drop, not the literal worst case;
# 0.4 is four max-severity scoring frames (DELTA = 0.10) back to back.
D_RANGE: float = 0.4

# Normalized dead-zone: drops below DROP_DEADZONE * D_RANGE (0.004 raw) are
# treated as no drop. Sized to swallow Solution-C empty-frame decay (<=
# 0.0015/frame) -- see THE DEAD-ZONE above.
DROP_DEADZONE: float = 0.01

# Drop-shape exponent. 1 = linear; < 1 punishes many-small over one-big
# drops; > 1 the reverse.
ALPHA: float = 1.0

# Weighting falloff inside the recent-severity window (higher = older drops
# in the window count more evenly) and the window length in frames.
RECENT_BETA: float = 0.80
RECENT_WINDOW_K: int = 6

# How hard recent severity pulls V down. Kept small next to LAMBDA_L: the
# persistence channel is the point of this module; the recent channel only
# adds a prompt sting on the drop frame itself.
LAMBDA_R: float = 0.10

# The duty-cycle anchor LAMBDA_L is derived from: an attacker bad
# DUTY_BAD_FRACTION of frames at normalized severity DUTY_SEVERITY is pulled
# to V = V_TARGET at convergence.
DUTY_BAD_FRACTION: float = 0.30
DUTY_SEVERITY: float = 1.0
V_TARGET: float = 0.5

# Minimum weight. Must stay below both R_STAR (so a floored agent fails the
# gate at any R -- checked at import) and V_TARGET (so the anchor above is
# reachable). See THE FLOOR.
V_FLOOR: float = 0.4

# --- Pure helpers ------------------------------------------------------------


def beta_from_half_life(half_life_s: float, fps: float = FPS) -> float:
    """Return the EMA decay beta_L whose half-life is half_life_s at fps.

    beta_L = 2 ^ (-1 / N_half) with N_half = fps * half_life_s frames, so the
    accumulator halves over exactly that span. The soft memory span is
    N_eff = 1 / (1 - beta_L) ~= 1.44 * N_half.

    Raises ValueError if half_life_s <= 0 or fps <= 0.
    """
    if half_life_s <= 0.0:
        raise ValueError(f'half_life_s={half_life_s} must be > 0')
    if fps <= 0.0:
        raise ValueError(f'fps={fps} must be > 0')
    return 2.0 ** (-1.0 / (fps * half_life_s))


def lambda_l_from_duty_cycle(duty_bad: float,
                             severity: float,
                             v_target: float,
                             alpha: float) -> float:
    """Return lambda_L anchored to an intolerable duty cycle.

    Solve V = 1 - lambda_L * risk^(1/alpha) = v_target at the converged risk
    of an attacker bad duty_bad of frames at severity c:  risk -> duty_bad *
    c^alpha, hence

        lambda_L = (1 - v_target) / (duty_bad * severity) ^ (1/alpha)

    Defaults (0.30, 1.0, 0.5, alpha=1) give lambda_L ~= 1.67.

    Raises ValueError if duty_bad or severity is outside (0, 1], v_target is
    outside (0, 1), or alpha <= 0.
    """
    if not (0.0 < duty_bad <= 1.0):
        raise ValueError(f'duty_bad={duty_bad} must be in (0, 1]')
    if not (0.0 < severity <= 1.0):
        raise ValueError(f'severity={severity} must be in (0, 1]')
    if not (0.0 < v_target < 1.0):
        raise ValueError(f'v_target={v_target} must be in (0, 1)')
    if alpha <= 0.0:
        raise ValueError(f'alpha={alpha} must be > 0')
    return (1.0 - v_target) / (duty_bad * severity) ** (1.0 / alpha)


def cold_start_debias(risk: float, beta_l: float, frames_observed: int) -> float:
    """Return the warm-up-corrected risk estimate risk / (1 - beta_l^t).

    A leaky EMA started at 0 underestimates during its first frames; after t
    frames the maximum reachable risk is 1 - beta_l^t, so dividing by that
    debiases the estimate (t=1 recovers the single observed dnorm^alpha
    exactly; large t leaves risk unchanged). Only needed if risk_L is
    persisted per-ID across separate encounters.

    Raises ValueError if frames_observed < 1, risk is outside [0, 1], or
    beta_l is outside (0, 1).
    """
    if frames_observed < 1:
        raise ValueError(f'frames_observed={frames_observed} must be >= 1')
    if not (0.0 <= risk <= 1.0):
        raise ValueError(f'risk={risk} outside [0, 1]')
    if not (0.0 < beta_l < 1.0):
        raise ValueError(f'beta_l={beta_l} must be in (0, 1)')
    return risk / (1.0 - beta_l ** frames_observed)


# --- Derived constants (single source of truth: the knobs above) --------------

BETA_L: float = beta_from_half_life(PERSIST_HALF_LIFE_S, FPS)
LAMBDA_L: float = lambda_l_from_duty_cycle(DUTY_BAD_FRACTION, DUTY_SEVERITY,
                                           V_TARGET, ALPHA)

# --- Import-time validation of the tuned constants ----------------------------
# Same pattern as freshness.py: the defaults are validated once here; the
# constructor re-validates because callers may inject their own tuning.
if not (0.0 < D_RANGE <= 1.0):
    raise ValueError(f'D_RANGE={D_RANGE} must be in (0, 1]: R lives in [0, 1]')
if not (0.0 <= DROP_DEADZONE < 1.0):
    raise ValueError(f'DROP_DEADZONE={DROP_DEADZONE} must be in [0, 1)')
if not (0.0 < RECENT_BETA <= 1.0):
    raise ValueError(f'RECENT_BETA={RECENT_BETA} must be in (0, 1]')
if RECENT_WINDOW_K < 1:
    raise ValueError(f'RECENT_WINDOW_K={RECENT_WINDOW_K} must be >= 1')
if LAMBDA_R < 0.0:
    raise ValueError(f'LAMBDA_R={LAMBDA_R} must be >= 0')
if not (0.0 < V_FLOOR <= 1.0):
    raise ValueError(f'V_FLOOR={V_FLOOR} must be in (0, 1]')

assert LAMBDA_R < LAMBDA_L, (
    f'LAMBDA_R={LAMBDA_R} must stay below LAMBDA_L={LAMBDA_L:.4f}: '
    'the persistence channel is the point of this module'
)
assert V_FLOOR < V_TARGET, (
    f'V_FLOOR={V_FLOOR} must stay below V_TARGET={V_TARGET}: '
    'a floor above the target clips the duty-cycle anchor'
)

# The gate crossover R* = tau(R*), imported from the tau math rather than
# hardcoded (same derivation kinematic_freshness.py uses for F_FLOOR), so
# this floor tracks any retuning of the threshold curve.
R_STAR: float = trust_fixed_point()

assert V_FLOOR < R_STAR, (
    f'V_FLOOR={V_FLOOR} must stay below R_STAR={R_STAR:.4f}; '
    'see module docstring (THE FLOOR)'
)

# -----------------------------------------------------------------------------


class PersistencePenalty:
    """Per-agent tracker for the persistence-penalty weight V.

    One instance per agent id, owned by the engine alongside its reputation.
    Feed update(R_old) once per processed frame; apply the returned V at the
    gate only. Call rejoin(gap_s) once when the agent reappears after an
    absence. All state is private; risk_persist is exposed read-only for
    logging and tests.
    """

    def __init__(self,
                 d_range: float = D_RANGE,
                 deadzone: float = DROP_DEADZONE,
                 alpha: float = ALPHA,
                 half_life_s: float = PERSIST_HALF_LIFE_S,
                 fps: float = FPS,
                 recent_beta: float = RECENT_BETA,
                 recent_window_k: int = RECENT_WINDOW_K,
                 lambda_r: float = LAMBDA_R,
                 lambda_l: float = LAMBDA_L,
                 v_floor: float = V_FLOOR):
        """Validate and store the tuning; see module constants for semantics.

        Raises ValueError on any out-of-range parameter (fail loudly: a bad
        tuning value is a caller bug, not something to clamp silently).
        """
        if d_range <= 0.0:
            raise ValueError(f'd_range={d_range} must be > 0')
        if not (0.0 <= deadzone < 1.0):
            raise ValueError(f'deadzone={deadzone} must be in [0, 1)')
        if alpha <= 0.0:
            raise ValueError(f'alpha={alpha} must be > 0')
        if not (0.0 < recent_beta <= 1.0):
            raise ValueError(f'recent_beta={recent_beta} must be in (0, 1]')
        if recent_window_k < 1:
            raise ValueError(f'recent_window_k={recent_window_k} must be >= 1')
        if lambda_r < 0.0:
            raise ValueError(f'lambda_r={lambda_r} must be >= 0')
        if lambda_l < 0.0:
            raise ValueError(f'lambda_l={lambda_l} must be >= 0')
        if not (0.0 < v_floor <= 1.0):
            raise ValueError(f'v_floor={v_floor} must be in (0, 1]')

        self._beta_l = beta_from_half_life(half_life_s, fps)  # validates both
        self._fps = fps
        self._d_range = d_range
        self._deadzone = deadzone
        self._alpha = alpha
        self._recent_beta = recent_beta
        self._lambda_r = lambda_r
        self._lambda_l = lambda_l
        self._v_floor = v_floor

        # Recent-channel denominator: the FULL geometric sum, also while the
        # window is not yet filled (missing history counts as zero drops), so
        # a drop on frame 1 scores the same as at steady state.
        self._w = sum(recent_beta ** k for k in range(recent_window_k))

        self._risk_l = 0.0                                # risk_L in [0, 1]
        self._recent: deque[float] = deque(maxlen=recent_window_k)
        self._last_r: float | None = None                 # None = no baseline

    @property
    def risk_persist(self) -> float:
        """The persistence channel risk_L in [0, 1] (read-only)."""
        return self._risk_l

    def seed(self, risk_l: float, last_r: float) -> None:
        """Seed state from reputation_log after a process restart.

        Sets risk_L to the persisted value and last-R baseline to last_r so
        the first update() after restart computes dnorm against the correct
        baseline. Pass last_r = R_seeded (the absence-decayed reputation that
        becomes R_old on the first post-restart frame) to make that update()
        produce dnorm = 0, preventing the cross-restart R difference from
        being scored as a drop.

        Only call on a freshly constructed tracker before the first update().
        cold_start_debias is not applied; frames_observed is not stored and
        the bias is negligible for trackers that ran long enough to be persisted.

        Raises ValueError for out-of-range inputs.
        """
        if not (0.0 <= risk_l <= 1.0):
            raise ValueError(f'risk_l={risk_l} outside [0, 1]')
        if not (0.0 <= last_r <= 1.0):
            raise ValueError(f'last_r={last_r} outside [0, 1]')
        self._risk_l = risk_l
        self._last_r = last_r

    def update(self, r_calc: float) -> float:
        """Advance one frame on this agent's raw reputation; return V.

        Args:
            r_calc: This frame's base reputation R_old in [0, 1] (raw, BEFORE
                    any multiplier -- V is derived from the raw trajectory).

        Returns:
            V in [v_floor, 1] to apply at the gate this frame. The first call
            after construction or rejoin() has no baseline, contributes
            dnorm = 0, and penalises only via whatever risk_L already holds.

        Raises:
            ValueError: if r_calc is outside [0, 1].
        """
        if not (0.0 <= r_calc <= 1.0):
            raise ValueError(f'r_calc={r_calc} outside [0, 1]')

        if self._last_r is None:
            dnorm = 0.0
        else:
            d = (self._last_r - r_calc) / self._d_range - self._deadzone
            dnorm = min(1.0, max(0.0, d))
        self._last_r = r_calc

        da = dnorm ** self._alpha
        self._risk_l = self._beta_l * self._risk_l + (1.0 - self._beta_l) * da

        self._recent.appendleft(da)   # maxlen evicts the (K+1)-th oldest
        risk_r = sum(self._recent_beta ** k * d
                     for k, d in enumerate(self._recent)) / self._w

        inv_a = 1.0 / self._alpha
        v = (1.0
             - self._lambda_r * risk_r ** inv_a
             - self._lambda_l * self._risk_l ** inv_a)
        return min(1.0, max(self._v_floor, v))
