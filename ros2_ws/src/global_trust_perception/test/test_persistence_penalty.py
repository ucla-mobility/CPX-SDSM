"""
Executable specification for the persistence-penalty weight V.

STATUS: TDD -- global_trust_perception.trust_calculations.reputation_multipliers.persistence_penalty
does NOT exist yet. Every test here fails at collection (ImportError) until it
is implemented. The contract below is transcribed from the interactive design
doc `visualizations/persistence_penalty.html` (commits 5c31e98 + a74ac9f),
rescaled from its 0-100 display units to this pipeline's R in [0, 1].

The math being specified (numbers refer to the equations in the HTML):
  1. d_t       = R_{t-1} - R_t                       (positive = drop)
  2. dnorm_t   = clip(d_t / D_range - eps, 0, 1)     (gains and dead-zone -> 0)
  3. risk_L    = beta_L * risk_L + (1 - beta_L) * dnorm^alpha   (leaky EMA, [0,1])
  3a. beta_L   = 2 ** (-1 / (fps * T_half))          (derived from a half-life)
  4. risk_r    = sum_{k<K} beta^k dnorm_{t-k}^alpha / sum_{k<K} beta^k
  5. V_t       = clip(1 - lambda_r * risk_r^(1/alpha)
                        - lambda_L * risk_L^(1/alpha), V_floor, 1)
  6. R_t       = R_calc * F * V_t   (V is one more multiplier next to F)
  lambda_L is anchored: lambda_L = (1 - V_target) / (duty_bad * severity)^(1/alpha)
  cold-start debias:        risk_hat = risk_L / (1 - beta_L ** frames_observed)

Expected module API (what this file imports):

  Module-level constants (defaults; validated once, like freshness.py):
    FPS                   frames per second (25.0)
    PERSIST_HALF_LIFE_S   T_half, the real-time persistence memory knob
    BETA_L                DERIVED: beta_from_half_life(PERSIST_HALF_LIFE_S, FPS)
    D_RANGE               drop scale in R units, in (0, 1]
    DROP_DEADZONE         eps, normalized dead-zone in [0, 1)
    ALPHA                 drop-shape exponent, > 0
    RECENT_BETA           recent-window decay, in (0, 1]
    RECENT_WINDOW_K       recent-window length K, >= 1
    LAMBDA_R              recent-channel weight, >= 0 (kept small vs LAMBDA_L)
    DUTY_BAD_FRACTION, DUTY_SEVERITY, V_TARGET   the duty-cycle anchor
    LAMBDA_L              DERIVED: lambda_l_from_duty_cycle(anchor..., ALPHA)
    V_FLOOR               minimum weight, in (0, 1]

  Pure helpers:
    beta_from_half_life(half_life_s, fps) -> float
    lambda_l_from_duty_cycle(duty_bad, severity, v_target, alpha) -> float
    cold_start_debias(risk, beta_l, frames_observed) -> float

  Stateful per-agent tracker (one instance per agent id):
    PersistencePenalty(d_range=..., deadzone=..., alpha=..., half_life_s=...,
                       fps=..., recent_beta=..., recent_window_k=...,
                       lambda_r=..., lambda_l=..., v_floor=...)
      .update(r_calc) -> V     feed this frame's base reputation (R in [0,1],
                               BEFORE V); returns the weight V for this frame.
                               The very first call has no previous R, so it
                               contributes dnorm = 0 and returns 1.0 (given
                               empty history). Raises ValueError outside [0,1].
      .risk_persist            read-only risk_L in [0, 1] (observability)
      .seed(risk_l, last_r)    initialise from persisted state after restart;
                               last_r should be R_seeded so the first update()
                               produces dnorm = 0 (no false drop from the
                               absence_decay discontinuity).

Like test_kinematic_freshness.py, assertions prefer the module's own constants and
relational properties over pinned literals, so retuning does not break tests
but breaking a structural property does.
"""

import math

import pytest

from global_trust_perception.trust_calculations.reputation_multipliers.persistence_penalty import (
    ALPHA,
    BETA_L,
    D_RANGE,
    DROP_DEADZONE,
    DUTY_BAD_FRACTION,
    DUTY_SEVERITY,
    FPS,
    LAMBDA_L,
    LAMBDA_R,
    PERSIST_HALF_LIFE_S,
    RECENT_BETA,
    RECENT_WINDOW_K,
    V_FLOOR,
    V_TARGET,
    PersistencePenalty,
    beta_from_half_life,
    cold_start_debias,
    lambda_l_from_duty_cycle,
)


# --- Fixtures / helpers ---------------------------------------------------------

# Explicit baseline parameters for behavioural tests: every knob is passed, so
# the assertions do not silently depend on the module's default tuning.
BASE = dict(
    d_range=0.4,
    deadzone=0.0,
    alpha=1.0,
    half_life_s=2.0,       # N_half = 50 frames at 25 fps
    fps=25.0,
    recent_beta=0.8,
    recent_window_k=6,
    lambda_r=0.1,
    lambda_l=1.67,
    v_floor=0.05,
)


def make(**overrides) -> PersistencePenalty:
    params = dict(BASE)
    params.update(overrides)
    return PersistencePenalty(**params)


def run(tracker: PersistencePenalty, rs) -> list:
    """Feed a reputation trajectory; return the V_t series."""
    return [tracker.update(r) for r in rs]


def reference_v(rs, *, d_range, deadzone, alpha, half_life_s, fps,
                recent_beta, recent_window_k, lambda_r, lambda_l, v_floor):
    """Line-for-line transcription of simulate() in persistence_penalty.html.

    Independent oracle for the exactness test: dn[0] = 0, the recent-channel
    denominator is the FULL geometric sum W even while t < K (missing history
    counts as zero drops), and both channels are re-raised by 1/alpha in V.
    """
    beta_l = 2.0 ** (-1.0 / (fps * half_life_s))
    w = sum(recent_beta ** k for k in range(recent_window_k))
    dn = [0.0]
    for prev, cur in zip(rs, rs[1:]):
        dn.append(min(1.0, max(0.0, (prev - cur) / d_range - deadzone)))
    risk_l = 0.0
    out = []
    for t in range(len(rs)):
        risk_l = beta_l * risk_l + (1.0 - beta_l) * dn[t] ** alpha
        num = sum(recent_beta ** k * dn[t - k] ** alpha
                  for k in range(recent_window_k) if t - k >= 0)
        v = (1.0
             - lambda_r * (num / w) ** (1.0 / alpha)
             - lambda_l * risk_l ** (1.0 / alpha))
        out.append(min(1.0, max(v_floor, v)))
    return out


# --- 1. Module constants: derivations and internal consistency -------------------

def test_beta_l_is_derived_from_the_half_life():
    """beta_L is not a free knob: it must equal 2^(-1/(fps * T_half))."""
    assert BETA_L == pytest.approx(
        beta_from_half_life(PERSIST_HALF_LIFE_S, FPS), rel=1e-12)


def test_lambda_l_is_derived_from_the_duty_cycle_anchor():
    """lambda_L comes from the intolerable-duty-cycle anchor, not hand-tuning."""
    assert LAMBDA_L == pytest.approx(
        lambda_l_from_duty_cycle(DUTY_BAD_FRACTION, DUTY_SEVERITY,
                                 V_TARGET, ALPHA), rel=1e-12)


def test_module_constants_consistent():
    assert FPS == 25.0                      # canary: fixed in the design doc
    assert PERSIST_HALF_LIFE_S > 0.0
    assert 0.0 < BETA_L < 1.0
    assert 0.0 < D_RANGE <= 1.0             # R lives in [0, 1]
    assert 0.0 <= DROP_DEADZONE < 1.0
    assert ALPHA > 0.0
    assert 0.0 < RECENT_BETA <= 1.0
    assert RECENT_WINDOW_K >= 1
    assert LAMBDA_R >= 0.0
    assert LAMBDA_L >= 0.0
    assert LAMBDA_R < LAMBDA_L              # persistence channel dominates
    assert 0.0 < V_FLOOR <= 1.0
    assert 0.0 < DUTY_BAD_FRACTION <= 1.0
    assert 0.0 < DUTY_SEVERITY <= 1.0
    assert 0.0 < V_TARGET < 1.0


# --- 2. Pure helpers --------------------------------------------------------------

def test_beta_from_half_life_closed_form():
    assert beta_from_half_life(2.0, 25.0) == pytest.approx(
        2.0 ** (-1.0 / 50.0), rel=1e-12)


def test_beta_from_half_life_halves_over_exactly_n_half_frames():
    beta_l = beta_from_half_life(2.0, 25.0)
    assert beta_l ** 50 == pytest.approx(0.5, rel=1e-9)


@pytest.mark.parametrize('half_life_s,fps', [(0.0, 25.0), (-1.0, 25.0),
                                             (2.0, 0.0), (2.0, -25.0)])
def test_beta_from_half_life_rejects_nonpositive(half_life_s, fps):
    with pytest.raises(ValueError):
        beta_from_half_life(half_life_s, fps)


def test_lambda_l_duty_cycle_anchor_example():
    """The design doc's worked default: 30% duty at full severity -> V 0.5."""
    assert lambda_l_from_duty_cycle(0.30, 1.0, 0.5, 1.0) == pytest.approx(
        0.5 / 0.3, rel=1e-9)


def test_lambda_l_duty_cycle_general_formula():
    duty, sev, v_target, alpha = 0.2, 0.5, 0.7, 2.0
    expected = (1.0 - v_target) / (duty * sev) ** (1.0 / alpha)
    assert lambda_l_from_duty_cycle(duty, sev, v_target, alpha) == pytest.approx(
        expected, rel=1e-9)


@pytest.mark.parametrize('duty,sev,v_target,alpha', [
    (0.0, 1.0, 0.5, 1.0),     # zero duty cycle: anchor undefined
    (0.3, 0.0, 0.5, 1.0),     # zero severity: anchor undefined
    (0.3, 1.0, 1.0, 1.0),     # V_target must be < 1
    (0.3, 1.0, -0.1, 1.0),
    (0.3, 1.0, 0.5, 0.0),     # alpha must be > 0
])
def test_lambda_l_duty_cycle_rejects_bad_anchor(duty, sev, v_target, alpha):
    with pytest.raises(ValueError):
        lambda_l_from_duty_cycle(duty, sev, v_target, alpha)


def test_cold_start_debias_formula():
    assert cold_start_debias(0.05, 0.9, 1) == pytest.approx(0.5, rel=1e-9)


def test_cold_start_debias_makes_first_frame_unbiased():
    """After one observed drop d, risk = (1-beta)*d; debias at t=1 recovers d."""
    beta_l, d = 0.97, 0.42
    assert cold_start_debias((1.0 - beta_l) * d, beta_l, 1) == pytest.approx(
        d, rel=1e-9)


def test_cold_start_debias_converges_to_identity():
    assert cold_start_debias(0.3, 0.9, 500) == pytest.approx(0.3, rel=1e-6)


def test_cold_start_debias_of_maximal_risk_is_one():
    """risk after t all-max frames is 1 - beta^t, so the debias caps at 1."""
    beta_l, t = 0.95, 7
    risk_max = 1.0 - beta_l ** t
    assert cold_start_debias(risk_max, beta_l, t) == pytest.approx(1.0, rel=1e-9)


@pytest.mark.parametrize('risk,beta_l,t', [
    (0.1, 0.9, 0),      # no frames observed: division by zero
    (-0.1, 0.9, 3),     # risk outside [0, 1]
    (1.1, 0.9, 3),
    (0.1, 0.0, 3),      # beta_l outside (0, 1)
    (0.1, 1.0, 3),
])
def test_cold_start_debias_rejects_bad_input(risk, beta_l, t):
    with pytest.raises(ValueError):
        cold_start_debias(risk, beta_l, t)


# --- 3. Drop normalization: gains, dead-zone, cap ---------------------------------

def test_first_frame_is_neutral_even_at_low_reputation():
    """No previous R means no drop: the first update never penalises."""
    assert make().update(0.1) == 1.0


def test_no_drops_no_penalty():
    """Steady and rising trajectories keep V at exactly 1 and risk at 0."""
    t = make()
    vs = run(t, [0.5, 0.5, 0.6, 0.7, 0.9, 0.9, 1.0])
    assert vs == [1.0] * len(vs)
    assert t.risk_persist == 0.0


def test_jitter_below_deadzone_is_ignored():
    """The steady scenario: small oscillation never trips either channel."""
    t = make(deadzone=0.06, d_range=0.4, lambda_l=10.0, lambda_r=1.0)
    seed, r, rs = 3, 0.96, []
    for _ in range(200):
        seed = (seed * 9301 + 49297) % 233280
        r = min(1.0, max(0.93, r + (seed / 233280 - 0.5) * 0.03))
        rs.append(r)          # step size <= 0.015 < deadzone * d_range = 0.024
    assert run(t, rs) == [1.0] * len(rs)


def test_drop_exactly_at_deadzone_is_ignored():
    # Exactly-representable binary fractions: 0.125/0.5 - 0.25 is 0.0 exactly,
    # so this boundary case is a true equality, not a float coincidence.
    t = make(deadzone=0.25, d_range=0.5, lambda_l=10.0)
    t.update(0.75)
    assert t.update(0.75 - 0.125) == 1.0
    assert t.risk_persist == 0.0


def test_drop_just_over_deadzone_penalises():
    t = make(deadzone=0.25, d_range=0.5, lambda_l=10.0)
    t.update(0.75)
    assert t.update(0.75 - 0.1875) < 1.0


def test_drops_beyond_d_range_saturate_at_one():
    """dnorm clips at 1: a 2x-range drop scores the same as a 1x-range drop."""
    a = make(d_range=0.3)
    run(a, [0.8, 0.5])       # drop = d_range exactly
    b = make(d_range=0.3)
    run(b, [0.9, 0.3])       # drop = 2 * d_range
    assert a.risk_persist == pytest.approx(b.risk_persist, rel=1e-12)


def test_gain_frame_equivalent_to_flat_frame():
    """Gains clip to zero: recovering R must not differ from holding R."""
    gains = make()
    flats = make()
    v_g = run(gains, [0.9, 0.5, 0.9, 0.9, 0.9])
    v_f = run(flats, [0.9, 0.5, 0.5, 0.5, 0.5])
    assert v_g == pytest.approx(v_f, rel=1e-12)
    assert gains.risk_persist == pytest.approx(flats.risk_persist, rel=1e-12)


# --- 4. Single sharp drop: dip then rebound ---------------------------------------

def test_sharp_drop_dips_then_recovers_monotonically():
    t = make()
    vs = run(t, [0.9, 0.5] + [0.5] * 100)
    assert vs[0] == 1.0
    assert vs[1] < 1.0                       # penalty lands on the drop frame
    for a, b in zip(vs[1:], vs[2:]):
        assert b >= a                        # strictly recovering afterwards
    assert vs[-1] > vs[1]


def test_rebound_half_life_is_n_half_frames():
    """Once drops stop, risk_L halves over exactly N_half = fps * T_half frames."""
    n_half = int(BASE['fps'] * BASE['half_life_s'])       # 50
    t = make()
    run(t, [0.9, 0.5])                       # one full-severity drop (d = d_range)
    risk_after_drop = t.risk_persist
    assert risk_after_drop > 0.0
    run(t, [0.5] * n_half)
    assert t.risk_persist == pytest.approx(risk_after_drop / 2.0, rel=1e-9)


# --- 5. Sustained decline: saturation and selectivity -----------------------------

def test_sustained_decline_saturates_risk_below_one():
    """40 consecutive max drops: risk_L = 1 - beta_L^40, bounded by 1."""
    t = make(d_range=0.02, half_life_s=0.2)              # N_half = 5
    beta_l = beta_from_half_life(0.2, 25.0)
    rs = [1.0 - 0.02 * i for i in range(41)]             # -d_range every frame
    run(t, rs)
    assert t.risk_persist == pytest.approx(1.0 - beta_l ** 40, rel=1e-9)
    assert t.risk_persist < 1.0


def test_persistence_selectivity_ratio_s_l():
    """A sustained slide is punished ~S_L = 1/(1-beta_L) harder than one dip
    of the same per-frame size (alpha = 1) -- the design's core property."""
    beta_l = beta_from_half_life(0.2, 25.0)
    single = make(d_range=0.02, half_life_s=0.2)
    run(single, [1.0, 0.98])
    sustained = make(d_range=0.02, half_life_s=0.2)
    run(sustained, [1.0 - 0.02 * i for i in range(41)])
    s_l = 1.0 / (1.0 - beta_l)
    assert sustained.risk_persist / single.risk_persist == pytest.approx(
        s_l * (1.0 - beta_l ** 40), rel=1e-9)
    assert sustained.risk_persist / single.risk_persist > 0.9 * s_l


# --- 6. On-off attack: the reason this system exists ------------------------------

def test_onoff_attacker_punished_far_beyond_single_dip():
    """Periodic hits accumulate; V never fully recovers between hits, and the
    end-state penalty dwarfs that of a single identical hit long ago."""
    params = dict(BASE, half_life_s=20.0, lambda_r=0.0, lambda_l=2.0)
    period = [0.5] + [0.9] * 19                          # one -0.4 hit per 20 frames

    onoff = PersistencePenalty(**params)
    v_onoff = run(onoff, [0.9] + period * 20)

    single = PersistencePenalty(**params)
    v_single = run(single, [0.9] + period + [0.9] * 380)

    assert v_onoff[-1] < v_single[-1]
    assert (1.0 - v_onoff[-1]) > 5.0 * (1.0 - v_single[-1])
    assert max(v_onoff[300:]) < 0.999        # depressed even between hits


def test_duty_cycle_anchor_drives_v_to_v_target():
    """An attacker at the anchor duty cycle (30% of frames, full severity)
    converges to V ~= V_target -- the calibration lambda_L promises."""
    duty, v_target = 0.3, 0.5
    lam = lambda_l_from_duty_cycle(duty, 1.0, v_target, 1.0)
    t = make(half_life_s=80.0, lambda_r=0.0, lambda_l=lam,
             recent_window_k=1, v_floor=0.05)
    # 3 full-severity drops per 10 frames (rises clip to 0, holds contribute 0)
    period = [0.5, 0.9, 0.5, 0.9, 0.5, 0.9, 0.9, 0.9, 0.9, 0.9]
    vs = run(t, [0.9] + period * 3000)       # ~10 EMA time constants
    last_period = vs[-10:]
    assert sum(last_period) / 10.0 == pytest.approx(v_target, abs=0.02)


# --- 7. Drop-shape exponent alpha --------------------------------------------------

def test_alpha_below_one_punishes_many_small_over_one_big():
    small = make(alpha=0.5, d_range=0.5, half_life_s=1.0)
    run(small, [1.0 - 0.05 * i for i in range(11)])      # ten 0.05 drops
    big = make(alpha=0.5, d_range=0.5, half_life_s=1.0)
    run(big, [1.0, 0.5] + [0.5] * 9)                     # one 0.5 drop, same length
    assert small.risk_persist > big.risk_persist


def test_alpha_above_one_punishes_one_big_over_many_small():
    small = make(alpha=2.0, d_range=0.5, half_life_s=1.0)
    run(small, [1.0 - 0.05 * i for i in range(11)])
    big = make(alpha=2.0, d_range=0.5, half_life_s=1.0)
    run(big, [1.0, 0.5] + [0.5] * 9)
    assert big.risk_persist > small.risk_persist


# --- 8. Recent-severity channel -----------------------------------------------------

def test_recent_channel_reacts_immediately_and_expires_after_window():
    """With persistence off, an isolated drop penalises for exactly K frames."""
    k = 4
    t = make(lambda_l=0.0, lambda_r=1.0, recent_beta=0.8, recent_window_k=k)
    vs = run(t, [0.9, 0.7] + [0.7] * (k + 3))
    assert vs[1] < 1.0                       # reacts on the drop frame itself
    assert all(v < 1.0 for v in vs[1:1 + k])  # penalised while in the window
    assert all(v == 1.0 for v in vs[1 + k:])  # exactly 1 once it slides out


def test_recent_channel_uses_full_window_normalisation():
    """Early frames divide by the full geometric sum W (missing history = 0),
    so a drop at t=1 scores the same as the identical drop at steady state."""
    k, beta = 6, 0.8
    early = make(lambda_l=0.0, lambda_r=1.0, recent_beta=beta, recent_window_k=k)
    v_early = run(early, [0.9, 0.7])[-1]
    late = make(lambda_l=0.0, lambda_r=1.0, recent_beta=beta, recent_window_k=k)
    v_late = run(late, [0.9] * 50 + [0.7])[-1]
    assert v_early == pytest.approx(v_late, rel=1e-12)
    w = sum(beta ** i for i in range(k))
    expected = 1.0 - (0.2 / BASE['d_range']) / w
    assert v_early == pytest.approx(expected, rel=1e-9)


# --- 9. Full-pipeline exactness against the design doc's simulator -----------------

def test_matches_reference_simulator_exactly():
    """Mixed-parameter two-hit trajectory (alpha != 1, dead-zone on, one drop
    clipping at dnorm = 1) must reproduce simulate() from the HTML to 1e-9."""
    params = dict(
        d_range=0.5, deadzone=0.02, alpha=1.3, half_life_s=1.0, fps=25.0,
        recent_beta=0.8, recent_window_k=3, lambda_r=0.15, lambda_l=1.2,
        v_floor=0.1,
    )
    rs = [0.90, 0.70, 0.75, 0.55, 0.55, 0.80, 0.20, 0.35, 0.35]
    t = PersistencePenalty(**params)
    expected = reference_v(rs, **params)
    for r, e in zip(rs, expected):
        assert t.update(r) == pytest.approx(e, rel=1e-9)


# --- 10. Floor and bounds invariants -------------------------------------------------

def test_v_never_leaves_floor_one_band_and_risk_stays_normalised():
    """Adversarial full-range oscillation with harsh weights: V in [floor, 1]
    and risk_L in [0, 1] at every single step."""
    t = make(d_range=0.2, lambda_l=5.0, lambda_r=1.0, v_floor=0.3)
    rs = [1.0, 0.0] * 100
    for r in rs:
        v = t.update(r)
        assert 0.3 <= v <= 1.0
        assert 0.0 <= t.risk_persist <= 1.0


def test_worst_case_pins_at_exactly_the_floor():
    t = make(d_range=0.1, lambda_l=50.0, v_floor=0.25)
    vs = run(t, [1.0, 0.0] * 50)
    assert min(vs) == 0.25


def test_floor_keeps_v_recoverable():
    """After a floored streak, stability lifts V back off the floor."""
    t = make(d_range=0.1, lambda_l=50.0, v_floor=0.25, half_life_s=0.2)
    run(t, [1.0, 0.0] * 20)
    vs = run(t, [0.9] * 400)
    assert vs[-1] > 0.25


# --- 11. Rejoin: gap decay across absences -------------------------------------------

def test_seed_does_not_score_the_cross_restart_drop():
    """seed(risk_l, last_r=R_seeded) must not penalise the absence_decay
    discontinuity: the first update() after restart produces dnorm = 0
    regardless of how much lower R_seeded is than the pre-restart R."""
    dropped = make(lambda_l=10.0)
    run(dropped, [0.9, 0.9])
    risk_before = dropped.risk_persist
    dropped.seed(risk_before, 0.3)            # last_r = R_seeded (much lower)
    v_low = dropped.update(0.3)               # R_old = R_seeded on first frame

    control = make(lambda_l=10.0)
    run(control, [0.9, 0.9])
    control.seed(risk_before, 0.9)            # last_r = R_seeded (same as before)
    v_flat = control.update(0.9)

    assert v_low == pytest.approx(v_flat, rel=1e-12)
    assert dropped.risk_persist == pytest.approx(control.risk_persist, rel=1e-12)


def test_seed_preserves_risk_and_recent_window():
    """seed() must not decay risk_L or clear the recent window."""
    t = make()
    run(t, [0.9, 0.5, 0.5])
    risk_before = t.risk_persist
    t.seed(risk_before, 0.5)
    assert t.risk_persist == pytest.approx(risk_before, rel=1e-12)


# --- 12. Input validation --------------------------------------------------------------

@pytest.mark.parametrize('r_bad', [-0.1, 1.1, math.inf, -math.inf])
def test_update_rejects_out_of_range_reputation(r_bad):
    with pytest.raises(ValueError):
        make().update(r_bad)


@pytest.mark.parametrize('overrides', [
    dict(d_range=0.0),
    dict(d_range=-0.5),
    dict(deadzone=-0.01),
    dict(deadzone=1.0),
    dict(alpha=0.0),
    dict(alpha=-1.0),
    dict(half_life_s=0.0),
    dict(half_life_s=-2.0),
    dict(fps=0.0),
    dict(fps=-25.0),
    dict(recent_beta=0.0),
    dict(recent_beta=1.5),
    dict(recent_window_k=0),
    dict(lambda_r=-0.1),
    dict(lambda_l=-1.0),
    dict(v_floor=0.0),
    dict(v_floor=1.5),
])
def test_constructor_rejects_invalid_parameters(overrides):
    with pytest.raises(ValueError):
        make(**overrides)
