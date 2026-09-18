"""
Unit tests for the freshness seam of the perception_message codec:
build() stamping real wall-clock send time, and send_latency_of()
decoding a wrap-aware send->receive latency.

Requires sdsm_interfaces (container-only), same as the pipeline tests.
"""

import time

import pytest

from global_trust_perception.pipeline import perception_message as pmsg
from global_trust_perception.trust_calculations.reputation_multipliers.kinematic_freshness import (
    CLOCK_SKEW_TOLERANCE_S,
    kinematic_freshness_factor,
)

_DAY_S = 86_400


def _msg_sent_at(t_send_s: float) -> pmsg.Message:
    """A minimal message whose send stamp encodes t_send_s (epoch seconds)."""
    msg = pmsg.Message()
    msg.sdsm_time_of_day_ms = int((t_send_s % _DAY_S) * 1000)
    return msg


# --- build() stamps the real clock ---------------------------------------------

def test_build_stamps_current_wall_clock():
    before = time.time()
    msg = pmsg.build(1, counter=0)
    after = time.time()
    # latency of a just-built message against 'now' must be ~0, not the old
    # synthetic counter value (counter=0 used to stamp 0 = midnight)
    lat = pmsg.send_latency_of(msg, after)
    assert 0.0 <= lat <= (after - before) + 0.01


def test_build_stamp_independent_of_counter():
    """The stamp is the clock, not the tick counter."""
    now = time.time()
    lat_a = pmsg.send_latency_of(pmsg.build(1, counter=0), now + 0.5)
    lat_b = pmsg.send_latency_of(pmsg.build(1, counter=9999), now + 0.5)
    assert lat_a == pytest.approx(lat_b, abs=0.01)


def test_build_stamps_day_of_month():
    # capture the day before AND after: build() may straddle UTC midnight
    day_before = time.gmtime().tm_mday
    msg = pmsg.build(1, counter=0)
    day_after = time.gmtime().tm_mday
    assert msg.sdsm_day in {day_before, day_after}


# --- send_latency_of decoding ---------------------------------------------------

def test_plain_latency():
    t = 1_700_000_000.0                      # arbitrary epoch instant
    msg = _msg_sent_at(t)
    assert pmsg.send_latency_of(msg, t + 0.3) == pytest.approx(0.3, abs=1e-6)


def test_zero_latency():
    t = 1_700_000_000.0
    assert pmsg.send_latency_of(_msg_sent_at(t), t) == pytest.approx(0.0, abs=1e-6)


def test_midnight_wraparound():
    """Sent 23:59:59.9, received 00:00:00.1 next day: +0.2 s, not -86399.8 s."""
    midnight = (1_700_000_000 // _DAY_S + 1) * _DAY_S
    msg = _msg_sent_at(midnight - 0.1)
    lat = pmsg.send_latency_of(msg, midnight + 0.1)
    assert lat == pytest.approx(0.2, abs=1e-6)


def test_small_negative_skew_preserved():
    """A sender clock slightly ahead yields a small negative latency, which
    kinematic_freshness_factor then clamps (within tolerance) rather than
    raising."""
    t = 1_700_000_000.0
    skew = CLOCK_SKEW_TOLERANCE_S / 2
    msg = _msg_sent_at(t + skew)
    lat = pmsg.send_latency_of(msg, t)
    assert lat == pytest.approx(-skew, abs=1e-6)
    assert kinematic_freshness_factor(5.0, lat) == 1.0   # end-to-end: clamped, fully fresh


def test_negative_skew_wraps_at_midnight_too():
    """Sender ahead of receiver ACROSS midnight still decodes as small skew.

    Tolerance is 2 ms: the wire stamp quantizes to whole ms (int truncation)
    and float64 carries ~1 ms of slop at epoch-second magnitudes.
    """
    midnight = (1_700_000_000 // _DAY_S + 1) * _DAY_S
    msg = _msg_sent_at(midnight + 0.004)     # sender already past midnight
    lat = pmsg.send_latency_of(msg, midnight - 0.004)
    assert lat == pytest.approx(-0.008, abs=2e-3)
    assert -CLOCK_SKEW_TOLERANCE_S <= lat < 0.0
