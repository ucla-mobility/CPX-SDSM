"""
Unit tests for TrustEngine's injected flush rate.

The persistence penalty decays once per flush, not once per second: its
half-life is expressed as a frame count, ``fps * half_life_s``. So the engine
has to be told the rate its caller actually flushes at, or the wall-clock
half-life silently scales with the flush period -- an agent flushing at 20 Hz
while the engine assumed the 2 Hz default would hold accumulated risk ten
times longer than V_HALF_LIFE_S promises.

Requires sdsm_interfaces (container-only), same as the other pipeline tests.
"""

import pytest

from global_trust_perception.pipeline.persistent_reputation_tracker import (
    PersistentReputationTracker,
)
from global_trust_perception.pipeline.trustworthy_perception import (
    FRAME_FLUSH_HZ,
    TrustEngine,
    V_HALF_LIFE_S,
)
from global_trust_perception.trust_calculations.reputation_multipliers.persistence_penalty import (
    beta_from_half_life,
)

AGENT = 2


def _tracker_for(flush_hz=None):
    """The PersistencePenalty an engine builds on its first weighted frame.

    _persistence_weight constructs one lazily per agent, so a single call is
    what makes the engine's configured rate observable.
    """
    db = PersistentReputationTracker(':memory:')
    engine = (TrustEngine(db) if flush_hz is None
              else TrustEngine(db, flush_hz=flush_hz))
    engine._persistence_weight(AGENT, 0.5)
    return engine._persistence[AGENT]


def test_default_rate_is_the_module_constant():
    """Callers that do not state a rate keep the 2 Hz production behaviour."""
    assert _tracker_for()._fps == float(FRAME_FLUSH_HZ)


@pytest.mark.parametrize('flush_hz', [2.0, 5.0, 20.0, 100.0])
def test_configured_rate_reaches_the_persistence_tracker(flush_hz):
    assert _tracker_for(flush_hz)._fps == flush_hz


@pytest.mark.parametrize('flush_hz', [2.0, 20.0])
def test_decay_constant_follows_the_configured_rate(flush_hz):
    assert _tracker_for(flush_hz)._beta_l == beta_from_half_life(
        V_HALF_LIFE_S, flush_hz)


def test_wall_clock_half_life_is_preserved_across_rates():
    """The point of injecting the rate: risk halves after V_HALF_LIFE_S
    seconds whatever the flush period, which means ten times as many flushes
    at 20 Hz as at 2 Hz -- not the same count over a tenth of the time."""
    for flush_hz in (2.0, 20.0):
        tracker = _tracker_for(flush_hz)
        flushes_to_half = V_HALF_LIFE_S * flush_hz
        assert tracker._beta_l ** flushes_to_half == pytest.approx(0.5)


@pytest.mark.parametrize('bad', [0.0, -1.0])
def test_nonpositive_rate_is_rejected(bad):
    """Fail loudly at construction rather than dividing by zero later."""
    with pytest.raises(ValueError):
        TrustEngine(PersistentReputationTracker(':memory:'), flush_hz=bad)
