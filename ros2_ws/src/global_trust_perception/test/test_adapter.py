"""
Tests for mmcooper_fuse.adapter.fuse: cross-agent clustering, the boundary that
replaced matching.match_all_agents.

Geometry conventions under test:
  - StreamInput.dims are (w, l, h) in metres (get_dims_of's native order); the
    adapter builds rotated BEV boxes and clusters overlapping ones across streams
  - clusters are {stream_key: det_idx}, at most one detection per stream
  - contributors lead with ego when ego contributed

What that membership is WORTH as corroboration is trust policy, not geometry:
it lives in consistency.corroboration_support and is tested in
test_consistency.py.

Requires the MS-PSF fusion path (shapely/scipy) — runs in the workspace container.
"""

from global_trust_perception.mmcooper_fuse.adapter import StreamInput, fuse

EGO = 'ego'
DIMS = (2.0, 4.5, 1.5)      # (w, l, h) — the sim's vehicle box
FAR = 50.0                  # separation guaranteeing zero IoU


def _sets(clusters):
    return {frozenset(c.items()) for c in clusters}


def test_empty_streams():
    assert fuse([], EGO).clusters == []


def test_far_apart_stay_separate():
    r = fuse([
        StreamInput(EGO, [(0.0, 0.0, 0.0)], [DIMS], reliability=1.0),
        StreamInput(2, [(FAR, FAR, 0.0)], [DIMS], reliability=0.8),
    ], EGO)
    assert _sets(r.clusters) == {frozenset({(EGO, 0)}), frozenset({(2, 0)})}


def test_same_object_clusters_with_ego():
    pos = [(10.0, 10.0, 0.0)]
    r = fuse([
        StreamInput(EGO, pos, [DIMS], reliability=1.0),
        StreamInput(2, pos, [DIMS], reliability=0.8),
    ], EGO)
    assert _sets(r.clusters) == {frozenset({(EGO, 0), (2, 0)})}
    assert r.contributors[0][0] == EGO       # ego leads the contributor list


def test_peers_cluster_on_an_object_ego_missed():
    """Two peers reporting the same object ego never saw share one cluster.
    This is the membership consistency.corroboration_support values as peer
    support; the weighting itself is pinned in test_consistency."""
    pos = [(10.0, 10.0, 0.0)]
    r = fuse([
        StreamInput(EGO, [(0.0, 0.0, 0.0)], [DIMS], reliability=1.0),
        StreamInput(2, pos, [DIMS], reliability=0.8),
        StreamInput(3, pos, [DIMS], reliability=0.6),
    ], EGO)
    assert _sets(r.clusters) == {frozenset({(EGO, 0)}), frozenset({(2, 0), (3, 0)})}


def test_shuffled_order_indices_are_per_stream():
    a = (0.0, 0.0, 0.0)
    b = (FAR, FAR, 0.0)
    r = fuse([
        StreamInput(EGO, [a, b], [DIMS, DIMS], reliability=1.0),
        StreamInput(2, [b, a], [DIMS, DIMS], reliability=0.8),   # reversed
    ], EGO)
    assert _sets(r.clusters) == {
        frozenset({(EGO, 0), (2, 1)}),
        frozenset({(EGO, 1), (2, 0)}),
    }


def test_heading_separates_crossing_boxes():
    """Two boxes at the same centre but perpendicular headings overlap far less
    than axis-aligned ones would; a 90-degree crossing pair must NOT fuse."""
    pos = [(0.0, 0.0, 0.0)]
    long_thin = (1.0, 8.0, 1.5)   # (w, l, h): long in its heading direction
    r = fuse([
        StreamInput(EGO, pos, [long_thin], headings=[90.0], reliability=1.0),
        StreamInput(2, pos, [long_thin], headings=[0.0], reliability=0.8),
    ], EGO)
    # perpendicular long-thin boxes share little area -> two singleton clusters
    assert len(r.clusters) == 2
