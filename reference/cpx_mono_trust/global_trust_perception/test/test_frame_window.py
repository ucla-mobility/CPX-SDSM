"""
Unit tests for the frame-window dedup that runs before decoding in flush_frame.

``_latest_per_sender`` enforces the invariant the decode loop assumes: at most
one message per sender per flush window. Without it the list-valued fields
(positions, dims, scores, headings, labels) concatenated every message a
sender landed in the window, while the scalar fields (raw_msg_by_agent,
equipment_by_agent, source_id_map) kept only the last. A sender that doubled
up was therefore scored on two frames of detections but had its verdict
stamped with only the later frame's counter, and the same physical object
entered fusion twice at two timestamps.

Senders double up whenever they publish faster than the flush timer, or when
one message jitters across a window boundary -- which stops being exotic once
the flush window shrinks from 500 ms to 50 ms.

Requires sdsm_interfaces (container-only), same as the other pipeline tests.
"""

from global_trust_perception.pipeline import perception_message as pmsg
from global_trust_perception.pipeline.agent import _latest_per_sender

EGO = 1


def _entry(sender: int, counter: int, recv_t: float):
    """A buffer entry identifiable by sender and counter, paired with recv_t.

    Mirrors what on_message appends: (message, arrival wall-clock time).
    """
    msg = pmsg.Message()
    # via the codec's own encoder, so this does not restate the
    # "agent id goes in byte 0" convention private to perception_message
    msg.source_id = list(pmsg.source_id_tuple_for(sender))
    msg.msg_cnt = counter
    return (msg, recv_t)


def _identify(entries):
    """Reduce entries to the (sender, counter, recv_t) triples they carry."""
    return [
        (pmsg.sender_of(msg), pmsg.sequence_of(msg), recv_t)
        for msg, recv_t in entries
    ]


def test_empty_window():
    assert _latest_per_sender([]) == ([], 0)


def test_one_message_per_sender_passes_through_untouched():
    window = [_entry(2, 10, 1.0), _entry(3, 11, 1.1), _entry(4, 12, 1.2)]
    kept, superseded = _latest_per_sender(window)
    assert superseded == 0
    assert _identify(kept) == [(2, 10, 1.0), (3, 11, 1.1), (4, 12, 1.2)]


def test_repeated_sender_keeps_the_later_message():
    window = [_entry(2, 10, 1.0), _entry(2, 11, 1.4)]
    kept, superseded = _latest_per_sender(window)
    assert superseded == 1
    assert _identify(kept) == [(2, 11, 1.4)]


def test_kept_message_carries_its_own_receive_time():
    """The surviving recv_t must be the later message's, not the dropped one's.

    recv_t feeds the kinematic freshness factor. Pairing the newer message
    with the older arrival time would make a fresh sender look stale.
    """
    window = [_entry(2, 10, 1.0), _entry(2, 11, 1.4)]
    kept, _ = _latest_per_sender(window)
    _, recv_t = kept[0]
    assert recv_t == 1.4


def test_ego_echo_is_deduped_like_any_other_sender():
    """Ego's own echo arrives on the shared topic and gets no special case."""
    window = [_entry(EGO, 5, 1.0), _entry(2, 10, 1.1), _entry(EGO, 6, 1.2)]
    kept, superseded = _latest_per_sender(window)
    assert superseded == 1
    assert _identify(kept) == [(EGO, 6, 1.2), (2, 10, 1.1)]


def test_interleaved_senders_each_keep_their_latest():
    window = [
        _entry(2, 10, 1.0), _entry(3, 20, 1.1),
        _entry(2, 11, 1.2), _entry(3, 21, 1.3),
    ]
    kept, superseded = _latest_per_sender(window)
    assert superseded == 2
    assert _identify(kept) == [(2, 11, 1.2), (3, 21, 1.3)]


def test_sender_order_follows_first_appearance():
    """Order is stable and independent of which message survived.

    Decoding groups by sender, so this ordering carries no meaning downstream
    -- it is asserted to pin the function as deterministic, not because any
    caller depends on the particular order.
    """
    window = [_entry(3, 20, 1.0), _entry(2, 10, 1.1), _entry(3, 21, 1.2)]
    kept, _ = _latest_per_sender(window)
    assert [pmsg.sender_of(msg) for msg, _ in kept] == [3, 2]


def test_many_messages_from_one_sender_collapse_to_the_last():
    window = [_entry(2, counter, 1.0 + counter) for counter in range(5)]
    kept, superseded = _latest_per_sender(window)
    assert superseded == 4
    assert _identify(kept) == [(2, 4, 5.0)]
