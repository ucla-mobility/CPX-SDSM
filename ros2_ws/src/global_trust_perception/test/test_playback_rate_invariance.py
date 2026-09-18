"""Playback rate must not change a judgement.

Slowing `ros2 bag play --rate` is a VIEWING aid: the scene is the same scene,
watched more slowly, so the verdicts it produces have to be identical. That
holds only if every time-derived quantity is measured on the clock the SCENE
advances on -- the ROS node clock under use_sim_time, fed by the bag -- rather
than on the wall.

Two mechanisms deliver that, and only one of them is testable without ROS:

  the flush timer   Node.create_timer runs on the node clock, so under
                    use_sim_time it fires per unit of BAG time. That is what
                    makes the deferred ledger's deadline (counted in flushes)
                    rate-invariant, and it is the dominant effect -- at
                    --rate 0.5 on wall time you get twice as many flushes per
                    scene second, so tracks expire after half as much scene.
                    Node-level, exercised by the demo script's use_sim_time
                    config, not reachable from here.

  the engine's dt   TrustEngine reads `now` for the dt it feeds to KDS. This
                    file pins that seam: the engine must take its dt from the
                    injected clock and nowhere else, which is what lets
                    agent.py put it on scene time.

Deliberately NOT asserted here: that two full scenarios at different rates
produce equal reputation traces. An honest agent saturates at R_MAX under
either clock, so such a test passes whether or not the seam works -- it would
pin nothing while looking like it pinned everything.
"""

import time

from global_trust_perception.pipeline.persistent_reputation_tracker import (
    PersistentReputationTracker,
)
from global_trust_perception.pipeline.trustworthy_perception import TrustEngine

OTHER = 2
EGO = [(3.0, 5.0, 0.0)]


class _FakeClock:
    """Advances `step` seconds per read, from a recognisable origin."""

    def __init__(self, step: float, start: float = 1000.0):
        self._t = start
        self._step = step

    def __call__(self) -> float:
        self._t += self._step
        return self._t


def _engine(now):
    return TrustEngine(PersistentReputationTracker(':memory:'), now=now)


def test_engine_reads_dt_from_the_injected_clock():
    """The engine's frame clock is the caller's, not time.monotonic()."""
    clock = _FakeClock(step=0.25)
    engine = _engine(clock)
    engine.process_frame({OTHER: EGO}, EGO)
    assert engine._last_t == 1000.25          # our clock, not the wall
    engine.process_frame({OTHER: EGO}, EGO)
    assert engine._last_t == 1000.50          # advanced by OUR step


def test_scene_clock_and_wall_clock_give_different_dt():
    """Under --rate 0.5 the wall advances twice as far per scene frame. The
    engine must be able to see the scene interval, which is the whole reason
    the clock is injected rather than read from the time module."""
    scene = _FakeClock(step=0.5)
    wall_at_half_rate = _FakeClock(step=1.0)   # same scene frame, 2x wall time

    e_scene, e_wall = _engine(scene), _engine(wall_at_half_rate)
    e_scene.process_frame({OTHER: EGO}, EGO)
    e_scene.process_frame({OTHER: EGO}, EGO)
    e_wall.process_frame({OTHER: EGO}, EGO)
    e_wall.process_frame({OTHER: EGO}, EGO)

    # 0.5 s of scene per frame either way, but the wall-read engine believes
    # 1.0 s elapsed -- the divergence use_sim_time removes.
    assert e_scene._last_t - 1000.0 == 1.0
    assert e_wall._last_t - 1000.0 == 2.0


def test_default_clock_is_the_wall_so_non_ros_callers_are_unaffected():
    """The injection must not change behaviour for callers that do not pass a
    clock -- tests, benchmarks and any non-ROS driver stay on monotonic time."""
    engine = TrustEngine(PersistentReputationTracker(':memory:'))
    before = time.monotonic()
    engine.process_frame({OTHER: EGO}, EGO)
    after = time.monotonic()
    assert before <= engine._last_t <= after
