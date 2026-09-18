"""
Unit tests for the fused-scene diagnostic channel.

Two seams:
  - TrustEngine.last_fusion: the read-only handle on the frame just processed
  - fused_objects: the codec that puts its geometry on the wire

The channel exists so a visualization can draw the CONSENSUS estimate beside
the inputs that produced it. Everything it carries is display derivation that
fuse() already computes and discards, so the tests below also pin the property
that makes it free: reading it must not perturb the trust decision.

The codec tests need sdsm_interfaces (container-only); the engine tests are
pure.
"""

import math

from global_trust_perception.mmcooper_fuse.geometry import corners_from_pose
from global_trust_perception.pipeline import fused_objects as fobj
from global_trust_perception.pipeline.persistent_reputation_tracker import (
    PersistentReputationTracker,
)
from global_trust_perception.pipeline.trustworthy_perception import TrustEngine


class _Fusion:
    """Minimal stand-in for FusionResult's display fields."""

    def __init__(self, boxes, centers_z, heights, scores, labels):
        self.fused_boxes = boxes
        self.fused_centers_z = centers_z
        self.fused_heights = heights
        self.fused_scores = scores
        self.fused_labels = labels


def _engine(tmp_path, name='r.db'):
    return TrustEngine(PersistentReputationTracker(str(tmp_path / name)))


# --- codec ---------------------------------------------------------------

def test_build_recovers_pose_from_bev_corners():
    """corners -> (center, size, yaw) is the exact inverse of corners_from_pose."""
    yaw = math.radians(30.0)
    fusion = _Fusion(
        boxes=[corners_from_pose(10.0, -3.0, 5.0, 2.0, yaw)],
        centers_z=[1.25], heights=[1.8], scores=[0.77], labels=[1],
    )

    (position, dims, got_yaw, score, label), = fobj.boxes_of(
        fobj.build(fusion, (2, 0, 0, 0)))

    assert position == (10.0, -3.0, 1.25)
    # size_x is the length along the heading, size_y the width.
    assert round(dims[0], 4) == 5.0
    assert round(dims[1], 4) == 2.0
    assert round(dims[2], 4) == 1.8
    assert round(got_yaw, 6) == round(yaw, 6)
    assert round(score, 4) == 0.77
    assert label == 1


def test_build_tolerates_an_absent_or_empty_fusion():
    """A frame that never fused encodes as an empty scene, not an exception.

    The viewer then clears rather than holding the previous frame's boxes
    forever, which is the whole reason last_fusion is reset per frame.
    """
    assert fobj.build(None, (1, 0, 0, 0)).num_objects == 0
    assert fobj.build(
        _Fusion([], [], [], [], []), (1, 0, 0, 0)).num_objects == 0


def test_topic_round_trips_the_ego_id():
    """topic()/ego_id_from_topic() are inverses; foreign topics decode to None."""
    assert fobj.ego_id_from_topic(fobj.topic(7)) == 7
    assert fobj.ego_id_from_topic('/perception/global_trustworthiness/sdsm') is None


# --- engine handle -------------------------------------------------------

def test_last_fusion_is_none_before_any_frame(tmp_path):
    """Nothing to publish before the first flush."""
    assert _engine(tmp_path).last_fusion is None


def test_last_fusion_is_populated_and_not_carried_across_frames(tmp_path):
    """It describes the frame just processed, never an older one.

    A stale handle would publish last frame's boxes against this frame's
    inputs — boxes that no longer correspond to anything on screen.
    """
    engine = _engine(tmp_path)

    engine.process_frame({2: [(1.0, 2.0, 0.0)]}, [(1.1, 2.1, 0.0)])
    first = engine.last_fusion
    assert first is not None
    assert len(first.fused_boxes) == len(first.fused_scores)

    # A frame with nothing to fuse must not leave the previous answer behind.
    engine.process_frame({}, [])
    assert engine.last_fusion is None or not engine.last_fusion.fused_boxes


def test_solo_frame_fuses_ego_alone_when_enabled(tmp_path):
    """A frame with no peer still describes ego, so the viewer has something current.

    Without this the peerless frames — every bucket a peer's message misses —
    publish an empty scene, and the viewer deletes and re-adds the whole fused
    layer, which reads as a blink.
    """
    engine = _engine(tmp_path)
    engine.fuse_solo_frames = True

    engine.process_frame({}, [(1.0, 2.0, 0.0), (8.0, 1.0, 0.0)])

    assert engine.last_fusion is not None
    assert len(engine.last_fusion.fused_boxes) == 2


def test_solo_frame_is_not_fused_by_default(tmp_path):
    """Off unless a viewer asked for it: a headless run pays nothing."""
    engine = _engine(tmp_path)

    engine.process_frame({}, [(1.0, 2.0, 0.0)])

    assert engine.last_fusion is None


def test_solo_fusion_does_not_change_the_trust_decision(tmp_path):
    """Fusing peerless frames is viz-only: identical reputations either way.

    The peerless branch returns before the gate, the ledger and the reputation
    update, so turning it on may add boxes to the screen and nothing else.
    """
    positions = {2: [(1.0, 2.0, 0.0), (8.0, 1.0, 0.0)]}
    ego = [(1.1, 2.1, 0.0)]

    plain = _engine(tmp_path, 'plain.db')
    solo = _engine(tmp_path, 'solo.db')
    solo.fuse_solo_frames = True

    for _ in range(5):
        # A peerless frame between judged ones: the case the flag changes.
        plain.process_frame({}, ego)
        solo.process_frame({}, ego)
        assert ([s.r_new for s in plain.process_frame(positions, ego)]
                == [s.r_new for s in solo.process_frame(positions, ego)])


def test_reading_last_fusion_does_not_change_the_trust_decision(tmp_path):
    """The viz handle is inert: identical reputations whether or not it is read.

    This is the property that lets the fused channel be free — it is a
    reference to what fuse() already built, never an extra computation and
    never an input to the decision.
    """
    positions = {2: [(1.0, 2.0, 0.0), (8.0, 1.0, 0.0)]}
    ego = [(1.1, 2.1, 0.0)]

    untouched = _engine(tmp_path, 'untouched.db')
    observed = _engine(tmp_path, 'observed.db')

    for _ in range(5):
        quiet = untouched.process_frame(positions, ego)
        watched = observed.process_frame(positions, ego)
        _ = observed.last_fusion       # the only difference between the two
        assert [s.r_new for s in quiet] == [s.r_new for s in watched]
