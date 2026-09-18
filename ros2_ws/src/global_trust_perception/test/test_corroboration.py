"""
Integration tests for Stage 2d corroboration in process_frame, now driven by
REPUTATION-WEIGHTED support instead of binary witness counting.

For an other_only detection, support = sum of R over the OTHER peers in its
phase-1 fusion cluster (ego excluded). FrameStats.uncorroborated counts
other_only detections whose support does not clear SUPPORT_THRESHOLD_THETA
this frame. The security property under test: k low-R colluders cannot
corroborate each other's ghost, because support is reputation MASS, not a
head count.

Requires the MS-PSF fusion path (shapely/scipy) — runs in the workspace
container. Reputations are seeded directly via engine.reputations to keep the
scenarios short.
"""

from global_trust_perception.trust_calculations.consistency import SUPPORT_THRESHOLD_THETA as THETA
from global_trust_perception.trust_calculations.consistency_checks.kinematic_checks import TrackData
from global_trust_perception.pipeline.persistent_reputation_tracker import PersistentReputationTracker
from global_trust_perception.pipeline.trustworthy_perception import TrustEngine

EGO = [(3.0, 5.0, 0.0), (4.0, 2.0, 0.0)]
BOTH = [(3.0, 5.0, 0.0), (4.0, 2.0, 0.0)]
GHOST = [(50.0, 50.0, 0.0)]      # far from everything real -> other_only

A2, A3 = 2, 3


def _engine() -> TrustEngine:
    return TrustEngine(PersistentReputationTracker(':memory:'))


def _by_agent(stats) -> dict:
    return {s.agent_id: s for s in stats}


def _tracks(points: list) -> TrackData:
    """Stationary tracks with stable ids, so the ledger grades these detections."""
    return TrackData(
        track_ids=list(range(len(points))),
        kalman_x=[p[0] for p in points], kalman_y=[p[1] for p in points],
        kalman_vx=[0.0] * len(points), kalman_vy=[0.0] * len(points),
    )


def test_lone_ghost_is_uncorroborated():
    """A ghost no peer reports has support 0 -> uncorroborated."""
    engine = _engine()
    engine.reputations[A2] = 0.8
    engine.reputations[A3] = 0.8
    stats = _by_agent(engine.process_frame({A2: GHOST, A3: BOTH}, EGO))
    assert stats[A2].uncorroborated == 1
    assert stats[A3].uncorroborated == 0   # A3's objects are ego-matched


def test_ghost_corroborated_by_high_rep_peer():
    """The same ghost reported by a second HIGH-reputation peer clears the
    threshold (support = peer R >= theta), so neither is uncorroborated."""
    engine = _engine()
    engine.reputations[A2] = THETA
    engine.reputations[A3] = THETA
    stats = _by_agent(engine.process_frame({A2: GHOST, A3: GHOST}, EGO))
    assert stats[A2].uncorroborated == 0
    assert stats[A3].uncorroborated == 0


def test_low_rep_colluders_cannot_corroborate():
    """Two low-R agents reporting the same ghost: each one's support is only the
    other's reputation (< theta), so the fabrication stays uncorroborated. This
    is the collusion-resistance property reputation-mass support buys."""
    engine = _engine()
    engine.reputations[A2] = 0.3
    engine.reputations[A3] = 0.3
    assert 0.3 < THETA
    stats = _by_agent(engine.process_frame({A2: GHOST, A3: GHOST}, EGO))
    assert stats[A2].uncorroborated == 1
    assert stats[A3].uncorroborated == 1


def test_lone_reporter_ghost_is_uncorroborated():
    """With no peer present at all, a ghost is uncorroborated (the fairness
    over transient occlusion is handled by the ledger's grace window, not by
    suppressing the flag)."""
    engine = _engine()
    engine.reputations[A2] = 0.8
    stats = engine.process_frame({A2: GHOST}, EGO)
    assert stats[0].uncorroborated == 1


def test_peer_corroboration_credits_on_the_first_frame_reported():
    """An other_only object a reputable peer also reports is settled CORRECT on
    the very frame it first appears -- existence is taken at the sender's word,
    so a brand-new track earns its credit immediately instead of waiting out a
    tracker warm-up before the ledger will judge it."""
    engine = _engine()
    engine.reputations[A2] = THETA
    engine.reputations[A3] = THETA
    tracks = {A2: _tracks(GHOST), A3: _tracks(GHOST)}
    stats = _by_agent(engine.process_frame({A2: GHOST, A3: GHOST}, EGO,
                                           tracks_by_agent=tracks))
    assert stats[A2].correct > 0 and stats[A2].ledger.pending == 0
    assert stats[A2].ledger.settled_correct == 1
    assert stats[A2].incorrect == 0.0


def test_ego_matched_objects_never_uncorroborated():
    """Objects ego also saw are matched, not other_only, so they are never
    counted uncorroborated regardless of peer support."""
    engine = _engine()
    engine.reputations[A2] = 0.2
    stats = engine.process_frame({A2: BOTH}, EGO)
    assert stats[0].uncorroborated == 0
