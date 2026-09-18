"""
Equivalence tests for two candidate rewrites of the mspsf hot path, plus a
golden-scene characterization of ``fuse``.

Neither rewrite is currently in mmcooper_fuse. Both are kept here in full
(the ``_new_*`` functions) alongside a verbatim copy of the shipped body they
would replace (the ``_ref_*`` functions), so the equivalence argument stays
tested and reviewable until they are either applied or abandoned:

  fusion.mspsf kappa           computed on IoU-gated pairs instead of
                               materialising an (n, n, C) tensor
  fusion.mspsf support gather  one batched np.nonzero + contiguous slices
                               instead of a per-row boolean mask

Neither changes an algorithm -- only how the same quantity is evaluated, and
both are asserted bit-for-bit with array_equal. They feed discrete decisions
-- kappa gates the edge set, support drives ``max(unpicked, key=...)`` during
seed selection -- where a one-ulp drift could pick a different seed and change
cluster composition. If an edit makes either merely approximate, that IS a
behavioural change and these tests should fail rather than be relaxed.

Because the rewrites are not applied, the ``_ref_*`` copies are what
``fusion.py`` actually runs today; test_mspsf_blocks_match_inside_the_real_call
rebuilds both from real mspsf inputs so the copies cannot silently drift from
the module.

The scalar leaf rewrites of corners_from_pose, pose_from_corners and
_axial_circular_mean (Tier 0a) were removed from this file along with the code
they pinned, and are not covered here.

Requires the MS-PSF fusion path (shapely/scipy) -- runs in the workspace
container.
"""

import math

import numpy as np
import pytest

from global_trust_perception.mmcooper_fuse.adapter import StreamInput, fuse
from global_trust_perception.mmcooper_fuse.fusion import (
    FusionConfig,
    build_class_probs,
    mspsf,
)
from global_trust_perception.mmcooper_fuse.geometry import (
    compute_self_iou_mat,
    corners_from_pose,
)

EGO = 'ego'


def _ulp_tol(magnitude, ulps=8):
    """Absolute tolerance worth `ulps` float32 steps at the given magnitude.

    A fixed absolute tolerance is meaningless here: one float32 ulp is 1e-7 at
    a coordinate of 1 m but 6e-5 at 1000 m, so the bound has to scale with the
    numbers being compared or it either passes vacuously or fails spuriously.
    """
    return float(ulps * np.spacing(np.float32(max(abs(float(magnitude)), 1.0))))


# ---------------------------------------------------------------------------
# Verbatim copies of the shipped mspsf blocks
# ---------------------------------------------------------------------------

def _ref_kappa(class_probs, n):
    kappa = np.minimum(class_probs[:, None, :], class_probs[None, :, :]).sum(axis=2)
    np.fill_diagonal(kappa, 1.0)
    return kappa


def _ref_support(edge, affinity, scores, n, alpha):
    support = np.zeros(n, dtype=np.float32)
    for i in range(n):
        mask = edge[i].copy()
        mask[i] = False
        if not np.any(mask):
            support[i] = alpha * scores[i]
            continue
        nbr_aff = affinity[i, mask]
        nbr_scores = scores[mask]
        degree = float(nbr_aff.sum())
        raw = float((nbr_aff * nbr_scores).sum() / (degree + 1e-9))
        strength = min(1.0, degree)
        support[i] = alpha * scores[i] + (1.0 - alpha) * strength * raw
    return support


# ---------------------------------------------------------------------------
# Candidate replacements for the two mspsf blocks above. Not applied in
# fusion.py; test_mspsf_blocks_match_inside_the_real_call runs both against
# real mspsf inputs so the _ref_ copies cannot quietly drift from the module.
# ---------------------------------------------------------------------------

def _new_kappa(class_probs, n, iou, cfg):
    gate_threshold = min(
        cfg.mspsf_tau_graph,
        cfg.mspsf_tau_fuse,
        cfg.mspsf_tau_fuse * (1.0 - cfg.mspsf_delta_loc),
    )
    kappa = np.zeros((n, n), dtype=np.float32)
    ki, kj = np.nonzero(np.triu(iou >= gate_threshold, 1))
    if len(ki):
        pair_kappa = np.minimum(class_probs[ki], class_probs[kj]).sum(axis=1)
        kappa[ki, kj] = pair_kappa
        kappa[kj, ki] = pair_kappa
    np.fill_diagonal(kappa, 1.0)
    return kappa


def _new_support(edge, affinity, scores, n, alpha):
    neighbours = edge.copy()
    np.fill_diagonal(neighbours, False)
    nbr_rows, nbr_cols = np.nonzero(neighbours)
    bounds = np.zeros(n + 1, dtype=np.int64)
    np.cumsum(np.bincount(nbr_rows, minlength=n), out=bounds[1:])
    nbr_affinity = affinity[nbr_rows, nbr_cols]
    nbr_scores_flat = scores[nbr_cols]

    support = np.zeros(n, dtype=np.float32)
    for i in range(n):
        lo, hi = bounds[i], bounds[i + 1]
        if lo == hi:
            support[i] = alpha * scores[i]
            continue
        nbr_aff = nbr_affinity[lo:hi]
        nbr_scores = nbr_scores_flat[lo:hi]
        degree = float(nbr_aff.sum())
        raw = float((nbr_aff * nbr_scores).sum() / (degree + 1e-9))
        strength = min(1.0, degree)
        support[i] = alpha * scores[i] + (1.0 - alpha) * strength * raw
    return support


def _random_graph(rng, n):
    """Symmetric edge/affinity pair with a True diagonal, as mspsf builds."""
    density = float(rng.uniform(0.0, 1.0))
    edge = rng.uniform(0.0, 1.0, (n, n)) < density
    edge = np.triu(edge, 1)
    edge = edge | edge.T
    np.fill_diagonal(edge, True)
    affinity = (rng.uniform(0.0, 1.5, (n, n)) * edge).astype(np.float32)
    affinity = np.triu(affinity, 1)
    affinity = affinity + affinity.T
    np.fill_diagonal(affinity, 1.0)
    return edge, affinity


def _random_symmetric_iou(rng, n):
    """Symmetric, exact-1.0 diagonal, mostly sparse -- as compute_self_iou_mat
    guarantees, and as the kappa gate's correctness argument relies on."""
    m = rng.uniform(0.0, 1.0, (n, n)).astype(np.float32)
    m[rng.uniform(0.0, 1.0, (n, n)) < 0.85] = 0.0
    m = np.triu(m, 1)
    m = m + m.T
    np.fill_diagonal(m, 1.0)
    return m


# ---------------------------------------------------------------------------
# mspsf kappa block  (EXACT where reachable)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('num_classes', [1, 2, 3, 4, 8, 16])
def test_kappa_matches_reference_across_class_counts(num_classes):
    """Wherever kappa can be READ the two forms agree bit-for-bit; the gated
    form deliberately leaves unreachable entries at 0."""
    rng = np.random.default_rng(100 + num_classes)
    cfg = FusionConfig()
    gate = min(cfg.mspsf_tau_graph, cfg.mspsf_tau_fuse,
               cfg.mspsf_tau_fuse * (1.0 - cfg.mspsf_delta_loc))
    for _ in range(40):
        n = int(rng.integers(1, 40))
        cp = rng.uniform(0.0, 1.0, (n, num_classes)).astype(np.float32)
        iou = _random_symmetric_iou(rng, n)
        want = _ref_kappa(cp, n)
        got = _new_kappa(cp, n, iou, cfg)
        reachable = iou >= gate
        assert np.array_equal(got[reachable], want[reachable])
        assert np.array_equal(np.diag(got), np.diag(want))
        assert got.dtype == want.dtype == np.float32


@pytest.mark.parametrize('tau_graph,tau_fuse,delta_loc', [
    (0.05, 0.30, 0.0),     # shipped defaults
    (0.30, 0.05, 0.0),     # tau_fuse BELOW tau_graph
    (0.05, 0.30, 0.9),     # delta_loc relaxes cross-agent tau below tau_graph
    (0.50, 0.50, 0.5),     # both high, cross-agent tau halved
    (0.0, 0.0, 0.0),       # everything admitted
])
def test_kappa_gate_is_safe_for_any_config(tau_graph, tau_fuse, delta_loc):
    """The gate must bound every threshold kappa is compared against, not just
    the shipped one -- otherwise a reachable entry would read a 0 the dense
    form never produced, and the edge set would change."""
    cfg = FusionConfig(mspsf_tau_graph=tau_graph, mspsf_tau_fuse=tau_fuse,
                       mspsf_delta_loc=delta_loc)
    rng = np.random.default_rng(77)
    for _ in range(40):
        n = int(rng.integers(2, 30))
        cp = rng.uniform(0.0, 1.0, (n, 4)).astype(np.float32)
        iou = _random_symmetric_iou(rng, n)
        got = _new_kappa(cp, n, iou, cfg)
        want = _ref_kappa(cp, n)
        for tau in (tau_graph, tau_fuse, tau_fuse * (1.0 - delta_loc)):
            readable = iou >= tau
            assert np.array_equal(got[readable], want[readable]), \
                f'entries reachable at tau={tau} differ'


def test_kappa_single_detection():
    cfg = FusionConfig()
    cp = np.array([[0.3, 0.7]], dtype=np.float32)
    iou = np.ones((1, 1), dtype=np.float32)
    assert np.array_equal(_new_kappa(cp, 1, iou, cfg), _ref_kappa(cp, 1))


# ---------------------------------------------------------------------------
# mspsf support gather  (EXACT)
# ---------------------------------------------------------------------------

def test_support_matches_reference_bit_for_bit():
    rng = np.random.default_rng(16)
    alpha = FusionConfig().mspsf_alpha
    for _ in range(400):
        n = int(rng.integers(1, 60))
        edge, affinity = _random_graph(rng, n)
        scores = rng.uniform(0.0, 1.0, n).astype(np.float32)
        got = _new_support(edge, affinity, scores, n, alpha)
        want = _ref_support(edge, affinity, scores, n, alpha)
        assert np.array_equal(got, want), 'support is not bit-identical'
        assert got.dtype == want.dtype == np.float32


def test_support_uniform_scene_is_bit_identical():
    """The adversarial case for this rewrite. Every detection identical means
    every support value ties exactly, so seed selection is decided by
    max()'s first-wins rule -- one ulp of drift there picks a different seed.
    A whole-row reduction does drift here; the batched gather does not."""
    alpha = FusionConfig().mspsf_alpha
    for n in (2, 12, 200):
        edge = np.ones((n, n), dtype=bool)
        affinity = np.ones((n, n), dtype=np.float32)
        scores = np.full(n, 0.7, dtype=np.float32)
        got = _new_support(edge, affinity, scores, n, alpha)
        want = _ref_support(edge, affinity, scores, n, alpha)
        assert np.array_equal(got, want)
        assert int(np.argmax(got)) == int(np.argmax(want))


def test_support_fully_isolated_graph_takes_the_old_early_exit_value():
    """Every detection isolated: the `if not np.any(mask)` branch, now
    expressed as an empty slice."""
    alpha = FusionConfig().mspsf_alpha
    n = 12
    edge = np.eye(n, dtype=bool)
    affinity = np.eye(n, dtype=np.float32)
    scores = np.linspace(0.0, 1.0, n).astype(np.float32)
    got = _new_support(edge, affinity, scores, n, alpha)
    assert np.array_equal(got, _ref_support(edge, affinity, scores, n, alpha))
    # ...and the value really is the early-exit expression. Built element by
    # element on purpose: `alpha * scores[i]` multiplies a python float by a
    # numpy SCALAR and promotes to float64 before the float32 store, whereas
    # `alpha * scores` on the whole array stays in float32 and rounds
    # differently in the last ulp. That is the same scalar-vs-array casting
    # trap these tests exist to catch, so the expectation is spelled out the
    # way the original branch computed it.
    expected = np.zeros(n, dtype=np.float32)
    for i in range(n):
        expected[i] = alpha * scores[i]
    assert np.array_equal(got, expected)


def test_support_zero_affinity_and_zero_score_graphs():
    alpha = FusionConfig().mspsf_alpha
    n = 15
    edge = np.ones((n, n), dtype=bool)
    for affinity, scores in (
        (np.zeros((n, n), dtype=np.float32), np.ones(n, dtype=np.float32)),
        (np.ones((n, n), dtype=np.float32), np.zeros(n, dtype=np.float32)),
    ):
        assert np.array_equal(_new_support(edge, affinity, scores, n, alpha),
                              _ref_support(edge, affinity, scores, n, alpha))


def test_support_single_detection():
    alpha = FusionConfig().mspsf_alpha
    edge = np.array([[True]])
    affinity = np.array([[1.0]], dtype=np.float32)
    scores = np.array([0.8], dtype=np.float32)
    assert np.array_equal(_new_support(edge, affinity, scores, 1, alpha),
                          _ref_support(edge, affinity, scores, 1, alpha))


def test_mspsf_blocks_match_inside_the_real_call():
    """Guard against the extracted copies above drifting from fusion.py.

    Rebuilds kappa and support from real mspsf inputs and checks the reference
    forms still agree -- so if fusion.py's blocks change without this file
    changing, the copies stop being representative and the sweeps above stop
    meaning anything.
    """
    rng = np.random.default_rng(17)
    cfg = FusionConfig()
    n = 24
    centres = rng.uniform(-30.0, 30.0, (n, 2))
    boxes = np.stack([
        corners_from_pose(cx, cy, 4.5, 2.0, float(rng.uniform(-math.pi, math.pi)))
        for cx, cy in centres
    ]).astype(np.float32)
    scores = rng.uniform(0.3, 1.0, n).astype(np.float32)
    classes = rng.integers(0, 4, n)
    class_probs = build_class_probs(classes, scores, 4)
    agents = rng.integers(0, 3, n)
    modalities = rng.integers(0, 3, n)

    iou = compute_self_iou_mat(boxes, dist_threshold=cfg.mspsf_distance_threshold)
    bonus = (
        cfg.mspsf_gamma_agent * (agents[:, None] != agents[None, :])
        + cfg.mspsf_gamma_modality * (modalities[:, None] != modalities[None, :])
    )
    affinity = iou * (1.0 + bonus).astype(np.float32)

    kappa_new = _new_kappa(class_probs, n, iou, cfg)
    kappa_ref = _ref_kappa(class_probs, n)
    edge_new = (iou >= cfg.mspsf_tau_graph) & (kappa_new >= cfg.mspsf_kappa_thr)
    edge_ref = (iou >= cfg.mspsf_tau_graph) & (kappa_ref >= cfg.mspsf_kappa_thr)
    np.fill_diagonal(edge_new, True)
    np.fill_diagonal(edge_ref, True)
    assert np.array_equal(edge_new, edge_ref), 'gated kappa changed the edge set'

    assert np.array_equal(
        _new_support(edge_new, affinity, scores, n, cfg.mspsf_alpha),
        _ref_support(edge_ref, affinity, scores, n, cfg.mspsf_alpha),
    )

    out_boxes, out_scores, out_cp, groups = mspsf(
        boxes, scores, class_probs, agents, modalities, cfg,
        agent_reliabilities=np.array([1.0, 0.6, 0.3], dtype=np.float32),
        kds_scores=rng.uniform(0.2, 1.0, n).astype(np.float32),
        vehicle_probs=(classes == 1).astype(np.float32),
    )
    assert sum(len(g) for g in groups) == n
    assert len(out_boxes) == len(out_scores) == len(out_cp) == len(groups)


# ---------------------------------------------------------------------------
# End-to-end characterization
# ---------------------------------------------------------------------------

def _golden_scene():
    """Shared objects, ego-only and peer-only detections, mixed classes (so
    kappa is non-trivial), varied kds/headings/scores, distinct modalities and
    reliabilities -- chosen so every branch of the fused-output path runs."""
    rng = np.random.default_rng(20260728)
    truth = [(0.0, 0.0), (7.0, 1.5), (20.0, -4.0), (33.0, 9.0),
             (41.0, 2.0), (55.0, -11.0), (60.0, 5.0), (72.0, 0.0)]

    def obs(idx, jitter, z=0.7):
        return [(truth[i][0] + rng.normal(0, jitter),
                 truth[i][1] + rng.normal(0, jitter), z) for i in idx]

    return [
        StreamInput(EGO, obs([0, 1, 2, 3, 4], 0.25),
                    dims=[(2.0, 4.5, 1.5), (1.9, 4.4, 1.5), (0.8, 0.8, 1.8),
                          (2.1, 4.6, 1.5), (2.5, 6.0, 2.2)],
                    headings=[10.0, 95.0, 360.0, 182.0, 271.0],
                    scores=[0.95, 0.80, 0.60, 0.99, 0.55],
                    labels=[1, 1, 2, 1, 3],
                    modality=0, reliability=1.0),
        StreamInput(2, obs([0, 1, 2, 5, 6], 0.30),
                    dims=[(2.0, 4.5, 1.5), (2.0, 4.5, 1.5), (0.9, 0.9, 1.7),
                          (2.0, 4.5, 1.5), (2.2, 5.0, 1.6)],
                    headings=[12.0, 92.0, 0.0, 45.0, 300.0],
                    scores=[0.90, 0.70, 0.65, 0.88, 0.77],
                    labels=[1, 1, 2, 1, 1],
                    kds=[0.9, 0.4, 1.0, 0.75, 0.2],
                    modality=1, reliability=0.62),
        StreamInput(3, obs([0, 3, 4, 6, 7], 0.28),
                    dims=[(2.1, 4.4, 1.5), (2.0, 4.5, 1.5), (2.4, 5.9, 2.1),
                          (2.2, 5.1, 1.6), (1.0, 1.0, 1.9)],
                    headings=[8.0, 180.0, 268.0, 298.0, 360.0],
                    scores=[0.85, 0.93, 0.50, 0.81, 0.71],
                    labels=[1, 1, 3, 1, 2],
                    kds=[0.55, 0.95, 0.3, 1.0, 0.6],
                    modality=2, reliability=0.41),
        StreamInput(4, obs([1, 7], 0.22),
                    dims=[(1.8, 4.3, 1.4), (1.1, 1.1, 1.9)],
                    headings=[97.0, 15.0],
                    scores=[0.66, 0.44],
                    labels=[1, 2],
                    kds=[0.8, 0.5],
                    modality=1, reliability=0.15),
    ]


# Captured from the pre-optimization code (commit 66358980) and reproduced
# exactly by the optimized path. Cluster ORDER is pinned as well as membership:
# it is the order fused boxes are emitted in, and the seed loop's max() over a
# set makes it sensitive to any change in component discovery order.
_GOLDEN_CLUSTERS = [
    {2: 0, 3: 0, 'ego': 0}, {2: 1, 4: 0, 'ego': 1}, {2: 2, 'ego': 2},
    {3: 1, 'ego': 3}, {3: 2, 'ego': 4}, {2: 3}, {2: 4, 3: 3}, {3: 4, 4: 1},
]
_GOLDEN_CONTRIBUTORS = [
    ['ego', 2, 3], ['ego', 2, 4], ['ego', 2], ['ego', 3], ['ego', 3],
    [2], [2, 3], [3, 4],
]
# Re-baselined when adapter.fuse() stopped reading StreamInput.dims in
# declaration order: it took (w, l, h) as (length, width, height), so every
# box went into the fusion 90 degrees off. Correcting it moves the fused
# SCORES (an overlap-weighted quantity) by <= 7e-4 and re-poses the fused
# boxes, while leaving `clusters`, `contributors`, `labels` and `heights`
# byte-for-byte unchanged -- which is what matters, because clusters are the
# only thing the trust decision consumes. Scores and boxes are display
# derivations, read by pipeline/visualization.py and the fused-scene codec
# and by nothing else.
_GOLDEN_SCORES = [
    0.8648552894592285, 0.6056128740310669, 0.42025622725486755,
    0.951040506362915, 0.31950053572654724, 0.774399995803833,
    0.6479565501213074, 0.48417410254478455,
]
_GOLDEN_LABELS = [1, 1, 2, 1, 3, 1, 1, 2]
_GOLDEN_HEIGHTS = [
    1.5, 1.4925731430378613, 1.7598205402215525, 1.5,
    2.1728476825479217, 1.5, 1.6, 1.9,
]
# Re-baselined with _GOLDEN_SCORES above: same box, correctly oriented.
_GOLDEN_FIRST_BOX = [
    [-0.11399754881858826, 2.432110071182251],
    [1.8704042434692383, 2.0719878673553467],
    [1.0702142715454102, -2.3373477458953857],
    [-0.9141874313354492, -1.9772255420684814],
]


def test_fuse_reproduces_pre_optimization_output():
    r = fuse(_golden_scene(), EGO)
    assert [dict(sorted(c.items(), key=lambda kv: str(kv[0])))
            for c in r.clusters] == _GOLDEN_CLUSTERS
    assert r.contributors == _GOLDEN_CONTRIBUTORS
    assert [int(x) for x in r.fused_labels] == _GOLDEN_LABELS
    assert r.fused_scores == pytest.approx(_GOLDEN_SCORES, abs=1e-6)
    assert r.fused_heights == pytest.approx(_GOLDEN_HEIGHTS, abs=1e-6)
    assert np.allclose(r.fused_boxes[0], _GOLDEN_FIRST_BOX,
                       atol=_ulp_tol(10.0), rtol=0)


def test_fuse_is_deterministic_across_repeated_calls():
    a = fuse(_golden_scene(), EGO)
    b = fuse(_golden_scene(), EGO)
    assert a.clusters == b.clusters
    assert np.array_equal(np.stack(a.fused_boxes), np.stack(b.fused_boxes))
    assert a.fused_scores == b.fused_scores


def test_fuse_handles_empty_and_single_stream_scenes():
    assert fuse([], EGO).clusters == []
    assert fuse([StreamInput(EGO, [], [])], EGO).clusters == []
    solo = fuse([StreamInput(EGO, [(1.0, 2.0, 0.5)], [(2.0, 4.5, 1.5)])], EGO)
    assert solo.clusters == [{EGO: 0}]
