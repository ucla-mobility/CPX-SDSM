"""
Unit tests for the input/output message split in the perception_message codec.

The two wire formats share their whole object payload and differ only in the
trust annotation: the input carries the sender's per-object local certainty,
the output carries the judging ego's singular global score. Because they are
two separate .msg files, that shared payload can DRIFT -- a field added to the
input and forgotten on the output would silently stop being rebroadcast. The
parity test below is the guard against exactly that.

Requires sdsm_interfaces (container-only), same as the pipeline tests.
"""

import pytest

from global_trust_perception.pipeline import perception_message as pmsg


# Every field the two messages are supposed to have in common. Adding a field
# to the object payload means adding it here AND to both .msg files.
_SHARED_FIELDS = (
    'msg_cnt', 'source_id', 'equipment_type',
    'ref_pos_x', 'ref_pos_y', 'ref_pos_z',
    'sdsm_day', 'sdsm_time_of_day_ms',
    'num_objects',
    'obj_type', 'object_id',
    'offset_x', 'offset_y', 'offset_z',
    'obj_width', 'obj_length', 'obj_height',
    'obj_speed', 'obj_heading', 'obj_measurement_time_ms',
    'local_score',
)


def _field_types(message_cls) -> dict:
    """field name -> ROS type string, via an instance (works either binding)."""
    return message_cls().get_fields_and_field_types()


# --- the two formats must not drift apart --------------------------------------

def test_shared_payload_fields_have_identical_types():
    in_types = _field_types(pmsg.Message)
    out_types = _field_types(pmsg.OutputMessage)
    for field in _SHARED_FIELDS:
        assert field in in_types, f'{field} missing from the input message'
        assert field in out_types, f'{field} missing from the output message'
        assert in_types[field] == out_types[field], (
            f'{field} type drifted: input={in_types[field]!r} '
            f'output={out_types[field]!r}'
        )


def test_no_unlisted_shared_fields():
    """A new field on BOTH messages must be added to _SHARED_FIELDS too, so the
    parity check above keeps covering the whole payload."""
    common = set(_field_types(pmsg.Message)) & set(_field_types(pmsg.OutputMessage))
    assert common == set(_SHARED_FIELDS)


def test_trust_annotation_is_what_differs():
    in_types = _field_types(pmsg.Message)
    out_types = _field_types(pmsg.OutputMessage)
    # per-object local certainty is input-only: no meaning once judged
    assert 'obj_local_scores' in in_types
    assert 'obj_local_scores' not in out_types
    # the ego's global score is output-only
    assert 'global_score' not in in_types
    assert 'global_score' in out_types


# --- with_global_score / global_score_of ----------------------------------------

def test_object_payload_carried_through_unchanged():
    msg = pmsg.build(agent_id=5, counter=3)          # the RSU sees the VRU
    assert msg.num_objects > 0, 'fixture needs a sender that detects something'
    out = pmsg.with_global_score(msg, 0.83)

    assert out.num_objects == msg.num_objects
    assert list(out.source_id) == list(msg.source_id)
    assert out.msg_cnt == msg.msg_cnt
    assert out.equipment_type == msg.equipment_type
    assert (out.ref_pos_x, out.ref_pos_y, out.ref_pos_z) == (
        msg.ref_pos_x, msg.ref_pos_y, msg.ref_pos_z)
    assert out.sdsm_day == msg.sdsm_day
    assert out.sdsm_time_of_day_ms == msg.sdsm_time_of_day_ms

    n = msg.num_objects
    for field in ('obj_type', 'object_id', 'offset_x', 'offset_y', 'offset_z',
                  'obj_width', 'obj_length', 'obj_height', 'obj_speed',
                  'obj_heading', 'obj_measurement_time_ms'):
        assert list(getattr(out, field))[:n] == list(getattr(msg, field))[:n], field


def test_global_score_round_trips():
    out = pmsg.with_global_score(pmsg.build(agent_id=5, counter=0), 0.83)
    assert pmsg.global_score_of(out) == pytest.approx(0.83)


def test_sender_local_score_passes_through():
    """Both scores survive: a consumer can weigh them itself, which it could
    not do if only their product were sent."""
    msg = pmsg.build(agent_id=5, counter=0)
    out = pmsg.with_global_score(msg, 0.83)
    assert out.local_score == pytest.approx(msg.local_score)
    assert pmsg.global_score_of(out) == pytest.approx(0.83)


def test_output_is_the_output_type():
    out = pmsg.with_global_score(pmsg.build(agent_id=5, counter=0), 0.5)
    assert isinstance(out, pmsg.OutputMessage)
    assert not isinstance(out, pmsg.Message)
