"""
Unit tests for reputation_multipliers.sender_motion.SenderMotionHistory.

update() returns a smoothed SPEED estimate (not a position residual): the
factor this feeds (kinematic_freshness) multiplies tracked speed by this
message's own transmission latency, deliberately independent of how long
since the sender's previous message (see sender_motion.py's module
docstring for why).

Needs numpy and filterpy only -- runs anywhere.
"""

import pytest

from global_trust_perception.trust_calculations.reputation_multipliers.sender_motion import (
    SenderMotionHistory,
)

AGENT = 7
DT = 0.5   # production frame period


def test_first_message_returns_zero_speed():
    sm = SenderMotionHistory()
    assert sm.update(AGENT, 10.0, 5.0, 100.0) == 0.0


def test_repeated_stationary_report_stays_at_zero_speed():
    sm = SenderMotionHistory()
    sm.update(AGENT, 10.0, 5.0, 100.0)
    speed = sm.update(AGENT, 10.0, 5.0, 100.0 + DT)
    assert speed == 0.0


def test_constant_velocity_speed_converges_to_true_speed():
    """Over several messages of true constant-velocity motion, the filter's
    speed estimate should converge to within 5% of the actual speed."""
    sm = SenderMotionHistory()
    true_speed = 5.0
    t = 100.0
    sm.update(AGENT, 0.0, 0.0, t)
    speed = 0.0
    for i in range(1, 10):
        t += DT
        speed = sm.update(AGENT, true_speed * DT * i, 0.0, t)
    assert speed == pytest.approx(true_speed, rel=0.05)


def test_agents_isolated():
    sm = SenderMotionHistory()
    sm.update(2, 0.0, 0.0, 100.0)
    sm.update(2, 100.0, 100.0, 100.5)   # agent 2 has an established fast history
    # agent 3's first message is unaffected by agent 2's state
    assert sm.update(3, 500.0, 500.0, 100.5) == 0.0


def test_nonpositive_dt_returns_last_known_speed_without_advancing_state():
    """An out-of-order or duplicate receive time can't be predicted over, so
    it's skipped rather than fed to the filter -- the last known speed
    estimate is returned unchanged."""
    sm = SenderMotionHistory()
    sm.update(AGENT, 0.0, 0.0, 100.0)
    speed_before = sm.update(AGENT, 2.5, 0.0, 100.0 + DT)   # establishes a speed

    # duplicate timestamp: skipped, last known speed returned unchanged
    assert sm.update(AGENT, 999.0, 999.0, 100.0 + DT) == speed_before
    # out-of-order (earlier) timestamp: also skipped
    assert sm.update(AGENT, 999.0, 999.0, 99.0) == speed_before
