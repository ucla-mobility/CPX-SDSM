"""
A persistent ghost IS eventually penalised.

This replaces the old KNOWN-LIMITATION test (test_confirmed_ghost_limitation),
which pinned the behaviour where an other_only detection was forgiven forever.
The deferred ledger (deferred.PendingVerdicts) closes that gap: an uncorroborated
ghost is HELD during a grace window (occlusion is plausible), then back-charged
as Incorrect once the deadline passes and charged every frame after. The window
runs from the ghost's FIRST sighting — the sender's claim that the object exists
is taken at face value the moment it makes it.

Requires the MS-PSF fusion path (shapely/scipy) — runs in the workspace container.
"""

from global_trust_perception.trust_calculations.consistency import T_DEADLINE_FLUSHES as DEADLINE
from global_trust_perception.trust_calculations.consistency_checks.kinematic_checks import TrackData
from global_trust_perception.pipeline.persistent_reputation_tracker import PersistentReputationTracker
from global_trust_perception.pipeline.trustworthy_perception import TrustEngine

GHOST = [(50.0, 50.0, 0.0)]   # object ONLY the neighbour ever reports
OTHER = 2


def _ghost_track() -> TrackData:
    return TrackData(
        track_ids=[0],
        kalman_x=[50.0], kalman_y=[50.0], kalman_vx=[0.0], kalman_vy=[0.0],
    )


def _engine() -> TrustEngine:
    return TrustEngine(PersistentReputationTracker(':memory:'))


def test_ghost_held_during_grace_then_penalised():
    engine = _engine()
    engine.reputations[OTHER] = 0.8   # start reputable so the drop is visible

    # --- grace window: the ghost is held, never charged --------------------
    for _ in range(DEADLINE):
        s = engine.process_frame(
            {OTHER: GHOST}, [],
            tracks_by_agent={OTHER: _ghost_track()},
        )[0]
        assert s.incorrect == 0
        assert s.held >= 1
        assert s.ledger.pending == 1

    # --- deadline passes: the held frames are back-charged as Incorrect ----
    r_before = engine.get_reputation(OTHER)
    s = engine.process_frame(
        {OTHER: GHOST}, [],
        tracks_by_agent={OTHER: _ghost_track()},
    )[0]
    assert s.incorrect > 0
    assert s.ledger.backcharged > 0
    assert engine.get_reputation(OTHER) < r_before


def test_ghost_drains_reputation_when_persistent():
    engine = _engine()
    engine.reputations[OTHER] = 0.8
    for _ in range(DEADLINE + 6):
        engine.process_frame(
            {OTHER: GHOST}, [],
            tracks_by_agent={OTHER: _ghost_track()},
        )
    # a pure ghoster is driven well below its starting reputation
    assert engine.get_reputation(OTHER) < 0.5


def test_charge_lands_a_deadline_after_first_sighting():
    """The clock runs from the ghost's FIRST sighting: nothing delays the charge
    beyond the grace window itself."""
    engine = _engine()
    engine.reputations[OTHER] = 0.8
    r_before = engine.get_reputation(OTHER)

    charged_frame = None
    for f in range(DEADLINE + 2):
        s = engine.process_frame(
            {OTHER: GHOST}, [],
            tracks_by_agent={OTHER: _ghost_track()},
        )[0]
        if s.incorrect > 0 and charged_frame is None:
            charged_frame = f

    assert charged_frame == DEADLINE             # f=0 is the first sighting
    assert engine.get_reputation(OTHER) < r_before
