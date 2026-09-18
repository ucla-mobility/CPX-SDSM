"""
Unit tests for grouping frames by CAPTURE time rather than by arrival.

Two agents watching one instant do not finish describing it at the same
moment: each runs its own perception chain first, and a denser point cloud
takes longer. On the scene_01 pair those chains finish ~59 ms apart (~33 ms
for the infrastructure, ~92 ms for the vehicle). Grouped by ARRIVAL, that
spread decides which frame a report lands in, so the fix was to widen the
window until both fit -- but a window is also the span of real time a frame
claims was simultaneous, and widening it makes honest reports of a moving
object fail to associate. One knob, two jobs, pulling opposite ways as speed
rises.

Grouping by capture time separates them: `flush_interval_s` states how wide
an instant is, `close_budget_s` absorbs the chain spread. This file pins the
seam that makes it work -- that a sender's own reported capture lag is enough
to recover WHEN its frame describes, independently of when it arrived.

NOT covered here: AgentNode._take_closed_bucket, which needs a live node.
What it adds on top of this seam is bookkeeping (which bucket has closed,
what to do with a late arrival); the arithmetic that has to be right for any
of it to mean anything is below.

Requires sdsm_interfaces (container-only), same as the other pipeline tests.
"""

from global_trust_perception.pipeline import perception_message as pmsg
from global_trust_perception.pipeline.agent import _capture_time_of

# The measured chain latencies of the scene_01 pair (see the demo script).
INFRA_CHAIN_S = 0.033
VEHICLE_CHAIN_S = 0.092


def _msg(num_objects: int, lag_ms: int) -> pmsg.Message:
    """A message carrying num_objects detections, all stamped with lag_ms."""
    msg = pmsg.Message()
    msg.num_objects = num_objects
    for i in range(num_objects):
        msg.obj_measurement_time_ms[i] = lag_ms
    return msg


def _entry(msg: pmsg.Message, recv_scene: float) -> tuple:
    """What on_message buffers: (message, recv_wall, recv_scene)."""
    return (msg, 0.0, recv_scene)


def test_lag_decodes_from_ms_to_seconds():
    """The wire field is milliseconds; the pipeline works in seconds."""
    assert pmsg.capture_lag_of(_msg(1, 92)) == 0.092


def test_empty_message_reports_no_lag():
    """A message with no detections has no capture instant to be late from."""
    assert pmsg.capture_lag_of(_msg(0, 500)) == 0.0


def test_unset_field_reads_as_sent_at_capture():
    """A sender that never sets the field must not be read as time-travelling.

    Zero is the pre-existing wire value, so this is what makes the change
    backwards compatible: an old sender groups by arrival exactly as before,
    rather than being assigned some invented lag.
    """
    assert pmsg.capture_lag_of(_msg(3, 0)) == 0.0
    assert _capture_time_of(_entry(_msg(3, 0), 10.0)) == 10.0


def test_one_instant_aligns_across_unequal_chains():
    """THE POINT: same capture instant, different chains, same bucket.

    Both agents observe at t=10.0. The infrastructure's chain takes 33 ms and
    the vehicle's 92 ms, so their messages arrive 59 ms apart -- the spread
    that forced the flush window wide. Recovered capture times must agree
    exactly, so no window width is needed to pair them at all.
    """
    infra = _entry(_msg(2, round(INFRA_CHAIN_S * 1000)), 10.0 + INFRA_CHAIN_S)
    vehicle = _entry(_msg(4, round(VEHICLE_CHAIN_S * 1000)), 10.0 + VEHICLE_CHAIN_S)

    assert _capture_time_of(infra) == _capture_time_of(vehicle)


def test_distinct_instants_stay_distinct():
    """Alignment must not collapse genuinely different instants together.

    The guard against "fix the spread by making everything simultaneous": two
    frames captured 200 ms apart stay 200 ms apart, whatever their chains did.
    """
    early = _entry(_msg(1, round(VEHICLE_CHAIN_S * 1000)), 10.0 + VEHICLE_CHAIN_S)
    late = _entry(_msg(1, round(INFRA_CHAIN_S * 1000)), 10.2 + INFRA_CHAIN_S)

    assert round(_capture_time_of(late) - _capture_time_of(early), 6) == 0.2


def test_capture_time_precedes_arrival():
    """A frame cannot be captured after it was received."""
    entry = _entry(_msg(1, round(VEHICLE_CHAIN_S * 1000)), 10.0)
    assert _capture_time_of(entry) < 10.0
