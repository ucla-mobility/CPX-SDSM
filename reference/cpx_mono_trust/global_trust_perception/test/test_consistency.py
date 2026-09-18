"""
Unit tests for the verdict POLICY in consistency.py: the weighted
matched/ego_only routing plus pen() and is_corroborated(). Pure counting +
arithmetic, runs anywhere (no numpy / shapely).

Policy under test (from the module docstring):
    matched                        -> Correct, weighted by the reporter's score c
    ego_only (any certainty)       -> held (agents not penalised for what ego sees)
    other_only                     -> NOT here (settled by the deferred ledger)

pen(c) = beta + (1-beta)*c is the affine floor: >= beta always, increasing in c,
so a hesitant mistake costs more than a hesitant correct earns.
"""

from global_trust_perception.trust_calculations.consistency import (
    PENALTY_FLOOR_BETA as BETA,
    SUPPORT_THRESHOLD_THETA as THETA,
    corroboration_support,
    is_corroborated,
    pen,
    weighted_ego_consistency,
)


def approx(a, b, tol=1e-9):
    return abs(a - b) < tol


# --- pen(): affine floor -------------------------------------------------

def test_pen_endpoints():
    assert approx(pen(0.0), BETA)      # no under-confidence dodge
    assert approx(pen(1.0), 1.0)       # confident lie costs the maximum


def test_pen_monotonic_in_confidence():
    assert pen(0.2) < pen(0.5) < pen(0.9)


def test_pen_hesitant_mistake_costs_more_than_hesitant_correct():
    # correct earns c; incorrect costs pen(c). At any c the mistake costs more.
    for c in (0.0, 0.1, 0.3, 0.5, 0.9):
        assert pen(c) > c


def test_pen_clamps_out_of_range():
    assert approx(pen(-1.0), BETA)
    assert approx(pen(2.0), 1.0)


# --- corroboration_support: R * certainty mass over peers ----------------
# Clusters are {stream_key: det_idx} as the phase-1 fusion emits them; these
# tests feed them directly, so the policy is pinned independently of the
# geometry that produced the grouping (test_adapter covers that half).

EGO = 'ego'


def test_support_excludes_ego():
    """Ego presence makes a detection matched, not other_only, so ego never
    counts as a corroborator — the matched path scores it instead."""
    support = corroboration_support(
        [{EGO: 0, 2: 0}], {EGO: 1.0, 2: 0.8}, {}, EGO)
    assert support[2][0] == 0.0


def test_support_is_reputation_mass_of_peers():
    """Missing scores default to certainty 1.0, so support is plain R mass."""
    support = corroboration_support(
        [{2: 0, 3: 0}], {2: 0.8, 3: 0.6}, {}, EGO)
    assert approx(support[2][0], 0.6)   # peer 3's reputation
    assert approx(support[3][0], 0.8)   # peer 2's reputation


def test_support_is_weighted_by_peer_certainty():
    """A peer's vote is R * c: a hesitant witness corroborates less than a
    confident one at the same reputation."""
    support = corroboration_support(
        [{2: 0, 3: 0}], {2: 0.8, 3: 0.6}, {2: [0.5], 3: [1.0]}, EGO)
    assert approx(support[2][0], 0.6)   # peer 3: 0.6 * 1.0
    assert approx(support[3][0], 0.4)   # peer 2: 0.8 * 0.5


def test_lone_detection_has_no_support():
    support = corroboration_support([{2: 0}], {2: 0.8}, {}, EGO)
    assert support[2][0] == 0.0


def test_low_rep_colluders_stay_below_threshold():
    """The collusion-resistance property: support is reputation MASS, so k
    low-R agents never corroborate each other however many they are."""
    cluster = {2: 0, 3: 0, 4: 0}
    rep = {2: 0.3, 3: 0.3, 4: 0.3}
    support = corroboration_support([cluster], rep, {}, EGO)
    assert not is_corroborated(support[2][0])   # 0.6 < THETA = 1.0


def test_unknown_peer_contributes_nothing():
    """A key absent from the reliability map is treated as zero reputation,
    never as a silent default."""
    support = corroboration_support([{2: 0, 9: 0}], {2: 0.8}, {}, EGO)
    assert support[2][0] == 0.0


# --- is_corroborated: reputation-mass threshold --------------------------

def test_is_corroborated_threshold():
    assert not is_corroborated(THETA - 1e-6)
    assert is_corroborated(THETA)
    assert is_corroborated(THETA + 0.5)


# --- weighted_ego_consistency --------------------------------------------

def test_all_empty():
    assert weighted_ego_consistency([], []) == (0.0, 0.0, 0)


def test_matched_weighted_by_certainty():
    C, I, held = weighted_ego_consistency([0.9, 0.8, 1.0], [])
    assert approx(C, 2.7) and I == 0.0 and held == 0


def test_ego_only_is_held():
    C, I, held = weighted_ego_consistency([], [1.0, 0.5])
    assert C == 0.0 and I == 0.0 and held == 2


def test_mixed_frame():
    C, I, held = weighted_ego_consistency(
        [0.9, 0.7],                       # matched
        [1.0, 0.5],                       # both ego_only -> held
    )
    assert approx(C, 1.6) and I == 0.0 and held == 2


def test_n_total_excludes_held():
    C, I, held = weighted_ego_consistency([1.0], [1.0])
    assert approx(C + I, 1.0)   # only the matched detection counts
    assert held == 1


def test_ego_only_all_held_regardless_of_confidence():
    C, I, held = weighted_ego_consistency([1.0, 1.0], [1.0, 1.0])
    assert C == 2.0 and I == 0.0 and held == 2
