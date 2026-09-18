"""
End-to-end pipeline tests: drive TrustEngine.process_frame as a mini live sim
and assert the exact per-frame values (S_frame, R, tau, trusted verdict).

These are integration tests - they exercise the full chain
    SORT tracking -> cross-agent matching -> consistency -> reputation -> tau
through the real process_frame, not isolated formula units (those live in
test_consistency.py, test_kinematic_freshness.py, test_absence_decay.py, etc.).

SCENARIO (shared by every test)
-------------------------------
- 2 agents: ego = agent 1, neighbour = agent 2.
- Ego ALWAYS sees the same 2 objects (the boxes), at fixed global positions.
- Each test varies only what agent 2 reports, and we watch agent 2's
  reputation evolve in ego's TrustEngine.

WHY THE WARM-UP MATTERS
-----------------------
ego_only items (things ego sees that a neighbour misses) are no longer
penalised — they all go to held regardless of track confirmation. The warm-up
still confirms ego's tracks so that kinematic demotion and attribute checks
work correctly in later tests that exercise matched pairs.

Assertions use plain `assert` (pytest rewrites these for readable failures);
no third-party assertion library is required.
"""

from global_trust_perception.pipeline.persistent_reputation_tracker import PersistentReputationTracker
from global_trust_perception.trust_calculations.reputation import DECAY_RATE, REPUTATION_DEFAULT
from global_trust_perception.pipeline.trustworthy_perception import TrustEngine


def _make_tracker() -> PersistentReputationTracker:
    return PersistentReputationTracker(':memory:')


# Ego's two boxes, as global (x, y, z) - far enough apart (~3.16 m) that SORT
# never confuses them and MS-PSF never cross-matches them.
EGO = [(3.0, 5.0, 0.0), (4.0, 2.0, 0.0)]
BOX_A = [(3.0, 5.0, 0.0)]                    # neighbour sees box A only
BOTH = [(3.0, 5.0, 0.0), (4.0, 2.0, 0.0)]    # neighbour sees both boxes

OTHER = 2                                    # neighbour agent id
WARMUP_FRAMES = 5                            # engine frames run before the measured ones


# --- rounding helpers (float math -> stable comparisons) -----------------
def r3(v: float) -> float:
    return round(v, 3)


def r4(v: float) -> float:
    return round(v, 4)


# --- mini-sim drivers ----------------------------------------------------
def _warmup(engine: TrustEngine) -> None:
    """Advance the engine a few frames without touching any reputation."""
    for _ in range(WARMUP_FRAMES):
        engine.process_frame({}, EGO, {}, [])


def _frame(engine: TrustEngine, other_positions: list):
    """Run one measured frame where neighbour reports other_positions.

    Returns the single FrameStats for the neighbour (or None on a frame the
    engine treats as empty, e.g. neighbour dropped off entirely).
    """
    stats = engine.process_frame({OTHER: other_positions}, EGO, {}, [])
    return stats[0] if stats else None


def _fresh() -> TrustEngine:
    engine = TrustEngine(_make_tracker())
    _warmup(engine)
    return engine


# =========================================================================
# TEST 1: neighbour sees NONE of the 2 objects ego sees.
# Both ego_only items -> held (no penalty). N_total = 0 -> Solution-C drift.
# At the baseline (R = 0.5) Solution-C is a no-op: R stays at 0.5 every frame.
# =========================================================================
def test_neighbour_misses_all_objects():
    engine = _fresh()

    for _ in range(3):
        s = _frame(engine, [])
        assert s.correct == 0
        assert s.incorrect == 0
        assert s.held == 2           # both ego_only -> held, not charged
        assert s.n_total == 0
        assert r3(s.s_frame) == 0.0
        assert r3(s.r_new) == 0.5   # unchanged at baseline
        # assert r3(s.tau) == 0.75  # dynamic tau(0.5)
        assert r3(s.tau) == 0.6     # static threshold
        assert not s.trusted

    assert r3(engine.get_reputation(OTHER)) == 0.5
    # assert r3(engine.get_threshold(OTHER)) == 0.75  # dynamic tau(0.5)
    assert r3(engine.get_threshold(OTHER)) == 0.6
    assert not engine.is_trusted(OTHER)             # 0.5 < 0.6


# =========================================================================
# TEST 2: neighbour sees ALL objects ego sees, then drops off at count=3.
# The gate lags one frame (it checks R_old), so R climbing doesn't flip the
# verdict until the frame AFTER the crossing:
#   count=1: S=1, R=0.6, gate: 0.5 < tau=0.6         -> not trusted
#   count=2: S=1, R=0.7, gate: 0.6 >= tau=0.6        -> trusted
#   (dynamic tau: count=1 gate 0.5 < tau(0.5)=0.75, count=2 gate
#    0.6 < tau(0.6)=0.684 -> still not trusted; the static threshold admits
#    one frame earlier)
#   count=3: neighbour drops off entirely -> reputation FROZEN at 0.7
#            (the silent-decay loop was removed: history is tracked in the
#            reputation DB instead, and reappearance reseeds from it)
# =========================================================================
def test_neighbour_agrees_then_drops_off():
    engine = _fresh()

    # count=1
    s = _frame(engine, BOTH)
    assert s.correct == 2
    assert s.incorrect == 0
    assert s.n_total == 2
    assert r3(s.s_frame) == 1.0
    assert r3(s.r_new) == 0.6
    # assert r3(s.tau) == 0.75       # dynamic tau(R_old=0.5)
    assert r3(s.tau) == 0.6          # gate checked against R_old=0.5
    assert not s.trusted             # 0.5 < 0.6

    # count=2
    s = _frame(engine, BOTH)
    assert r3(s.s_frame) == 1.0
    assert r3(s.r_new) == 0.7
    # assert r3(s.tau) == 0.684      # dynamic tau(R_old=0.6)
    # assert not s.trusted           # dynamic: 0.6 < 0.684
    assert r3(s.tau) == 0.6
    assert s.trusted                 # 0.6 >= 0.6; R_new only moves NEXT frame's gate

    # count=3: neighbour drops off entirely -> empty frame -> R frozen.
    # process_frame returns [] here, so read state via the getters.
    out = engine.process_frame({}, EGO, {}, [])
    assert out == []
    assert r4(engine.get_reputation(OTHER)) == 0.7
    # assert r4(engine.get_threshold(OTHER)) == 0.606  # dynamic tau(0.7)
    assert r4(engine.get_threshold(OTHER)) == 0.6
    assert engine.is_trusted(OTHER)          # 0.7 >= 0.6


# =========================================================================
# TEST 3: neighbour sees 1 of the 2 objects (box A), every frame.
# matched=1 (C=1); missed box B is ego_only -> held (not charged).
# N_total=1, S=+1 -> R climbs by 0.1/frame.
# =========================================================================
def test_neighbour_sees_half_reputation_climbs():
    engine = _fresh()

    # frame 1: R_old=0.5, tau=0.6 (dynamic: tau(0.5)=0.75)
    s = _frame(engine, BOX_A)
    assert s.correct == 1
    assert s.incorrect == 0
    assert s.held == 1              # box B ego_only -> held
    assert s.n_total == 1
    assert r3(s.s_frame) == 1.0
    assert r3(s.r_new) == 0.6
    # assert r3(s.tau) == 0.75      # dynamic tau(0.5)
    assert r3(s.tau) == 0.6
    assert not s.trusted            # 0.5 < 0.6

    # frame 2: R_old=0.6, tau=0.6 -> already trusted under the static threshold
    # (dynamic: tau(0.6)=0.684, so 0.6 < 0.684 and trust waited one more frame)
    s = _frame(engine, BOX_A)
    assert r3(s.s_frame) == 1.0
    assert r3(s.r_new) == 0.7
    # assert r3(s.tau) == 0.684     # dynamic tau(0.6)
    # assert not s.trusted          # dynamic: 0.6 < 0.684
    assert r3(s.tau) == 0.6
    assert s.trusted                # 0.6 >= 0.6

    # frame 3: R_old=0.7, tau=0.6 -> stays trusted
    s = _frame(engine, BOX_A)
    assert r3(s.s_frame) == 1.0
    assert r3(s.r_new) == 0.8
    # assert r3(s.tau) == 0.606     # dynamic tau(0.7)
    assert r3(s.tau) == 0.6
    assert s.trusted                # 0.7 >= 0.6

    assert r3(engine.get_reputation(OTHER)) == 0.8
    # assert r3(engine.get_threshold(OTHER)) == 0.516  # dynamic tau(0.8)
    assert r3(engine.get_threshold(OTHER)) == 0.6


# =========================================================================
# TEST 4: trust boundary crossing on the way UP; silent frames cause only
# a slow Solution-C drift, not a sharp fall.
#
# trusted := R_old >= tau (Stage-0 gate). Static threshold: crossover 0.6.
# (dynamic tau: fixed point R* ~ 0.648)
#
# Climb  : neighbour agrees (S=+1), R rises 0.1/frame; crosses the threshold
#          at R_old=0.6 (dynamic: crossed R* between R_old=0.6 and R_old=0.7).
# Silence: neighbour goes quiet -> ego_only items held, N_total=0, Solution-C
#          drift: R_new = R_old + 0.003*(0.5-R_old). From R=0.9 this is a
#          tiny decrease (~0.0012/frame); trusted stays True for many frames.
# =========================================================================
def test_trust_boundary_crossing():
    engine = TrustEngine(_make_tracker())

    # --- climb: (expected R_new, expected trusted) per agreement frame ---
    # flip False -> True at frame 2, the first frame entered with R_old=0.6.
    # (dynamic tau flipped at frame 3, the first entered with R_old=0.7:
    #  climb = [(0.6, False), (0.7, False), (0.8, True), (0.9, True)])
    climb = [(0.6, False), (0.7, True), (0.8, True), (0.9, True)]
    for exp_r, exp_trusted in climb:
        s = _frame(engine, BOTH)
        assert r3(s.s_frame) == 1.0
        assert r3(s.r_new) == exp_r
        assert s.trusted == exp_trusted

    # --- silent frames: ego_only -> held, N_total=0, Solution-C slow drift ---
    # R barely moves (~0.0012/frame from 0.9); agent stays trusted throughout.
    for exp_r in [0.899, 0.898, 0.896, 0.895]:
        s = _frame(engine, [])
        assert s.incorrect == 0
        assert s.s_frame == 0.0
        assert r3(s.r_new) == exp_r
        assert s.trusted               # well above the 0.6 threshold

    assert engine.is_trusted(OTHER)


# =========================================================================
# TEST 5: Solution-C decay branch (N_total == 0) observed AWAY from baseline.
#
# The ghost-limitation test only sees this branch at R = 0.5 where the decay
# is a mathematical no-op. Here the agent is present-but-unscoreable (sends
# only a held ghost while ego sees nothing) at R = 0.9 and R = 0.2, so the
# drift toward baseline — including its SIGN in both directions — is pinned.
# Deleting the decay line or flipping its sign fails these assertions.
# =========================================================================
GHOST_POS = [(7.0, 7.0, 0.0)]   # object only the neighbour reports


def _held_only_frame(engine: TrustEngine):
    """Agent present, everything held: ghost from neighbour, ego blind."""
    stats = engine.process_frame({OTHER: GHOST_POS}, [], {}, [])
    return stats[0]


def test_decay_drifts_down_toward_baseline_from_above():
    engine = TrustEngine(_make_tracker())
    for _ in range(4):
        _frame(engine, BOTH)                       # R climbs to 0.9
    s = _held_only_frame(engine)
    assert (s.n_total, s.correct, s.incorrect, s.held) == (0, 0, 0, 1)
    assert s.s_frame == 0.0
    assert r4(s.r_new) == r4(0.9 + DECAY_RATE * (REPUTATION_DEFAULT - 0.9))
    assert s.r_new < s.r_old                       # decays DOWN from above


def test_decay_drifts_up_toward_baseline_from_below():
    engine = _fresh()                              # warm-up confirms ego tracks
    engine.reputations[OTHER] = 0.2                # seed below baseline directly:
                                                   # ego_only misses no longer sink
                                                   # R, so set the low start here
    s = _held_only_frame(engine)
    assert s.n_total == 0
    assert r4(s.r_new) == r4(0.2 + DECAY_RATE * (REPUTATION_DEFAULT - 0.2))
    assert s.r_new > s.r_old                       # decays UP from below


def test_decay_converges_monotonically():
    engine = TrustEngine(_make_tracker())
    for _ in range(4):
        _frame(engine, BOTH)                       # R = 0.9
    gaps = []
    for _ in range(5):
        s = _held_only_frame(engine)
        gaps.append(abs(s.r_new - REPUTATION_DEFAULT))
    assert all(a > b for a, b in zip(gaps, gaps[1:]))   # strictly approaching 0.5
