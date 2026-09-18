"""
Sort min_hits confirmation gate tests.

Moved from global_trust_perception when Sort was extracted to this package.
Verifies Sort's own track confirmation timing directly, without involving
the trust pipeline.

appearance:  1     2     3     4     5 ...
hit_streak:  0     1     2     3     4
confirmed:   no    no    no    YES   YES
"""
import numpy as np
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from global_trust_tracker.SORT.modified_SORT_centroid import Sort

A = np.array([[3.0, 5.0]])
B = np.array([[4.0, 2.0]])


def test_track_unconfirmed_for_first_three_frames():
    s = Sort(min_hits=3)
    for _ in range(3):
        tracks = s.update(A)
        assert len(tracks) == 1
        assert not tracks[0].confirmed


def test_track_confirms_on_fourth_frame():
    s = Sort(min_hits=3)
    for _ in range(3):
        s.update(A)
    tracks = s.update(A)
    assert tracks[0].confirmed


def test_fading_track_never_confirms():
    s = Sort(min_hits=3)
    s.update(A)
    # object disappears — track stales out
    for _ in range(s.max_age + 1):
        s.update(np.empty((0, 2)))
    # A reappears — fresh track, back to unconfirmed
    tracks = s.update(A)
    assert not tracks[0].confirmed


def test_incorrect_route_held_then_charged():
    """A persistent miss is unconfirmed (held) for 3 frames, confirmed on frame 4.

    This is the Sort half of the min-hits test. The trust pipeline half
    (held -> incorrect transition) is exercised in test_basic_pipeline.py
    using explicit TrackData.
    """
    s = Sort(min_hits=3)
    for _ in range(3):
        tracks = s.update(B)
        assert not tracks[0].confirmed

    tracks = s.update(B)
    assert tracks[0].confirmed


def test_correct_route_track_also_confirms_after_min_hits():
    """Matched (agreed) objects confirm on the same schedule as missed ones."""
    s = Sort(min_hits=3)
    for i in range(5):
        tracks = s.update(A)
        assert tracks[0].confirmed == (i >= 3)
