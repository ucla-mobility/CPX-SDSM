"""
Unit tests for the diagnostic verdict channel.

Three seams:
  - deferred.PendingVerdicts.status(): the ledger's read-only view of a track
  - TrustEngine's Stage-5b per-detection verdict labels + matched_ego
  - trust_verdicts: the codec that puts them on the wire

The channel exists so a visualization can show senders the gate REJECTED,
which the filtered SdsmTrustOutput deliberately cannot carry. The codec tests
need sdsm_interfaces (container-only); the ledger and engine tests are pure.
"""

from types import SimpleNamespace

from global_trust_perception.pipeline import trust_verdicts as tverd
from global_trust_perception.pipeline.persistent_reputation_tracker import (
    PersistentReputationTracker,
)
from global_trust_perception.pipeline.trustworthy_perception import (
    VERDICT_DEFERRED,
    VERDICT_MATCHED,
    VERDICT_UNCORROBORATED,
    TrustEngine,
)
from global_trust_perception.trust_calculations.consistency import (
    SUPPORT_THRESHOLD_THETA as THETA,
    T_DEADLINE_FLUSHES as DEADLINE,
)
from global_trust_perception.trust_calculations.deferred import PendingVerdicts

OTHER = 2
EGO_POS = [(3.0, 5.0, 0.0), (4.0, 2.0, 0.0)]   # two well-separated boxes
GHOST = (50.0, 50.0, 0.0)                      # far from anything ego sees


# --- the label vocabulary must not drift ---------------------------------------

def test_wire_mapping_covers_every_engine_label():
    """trust_verdicts duplicates the label literals on purpose, to keep the
    engine (numpy, MS-PSF, the whole pipeline) out of a visualization
    process's import graph. This is the guard that pins the two together."""
    assert set(tverd._LABEL_TO_WIRE) == {
        VERDICT_MATCHED, VERDICT_DEFERRED, VERDICT_UNCORROBORATED,
    }


def test_wire_values_are_distinct():
    assert len(set(tverd._LABEL_TO_WIRE.values())) == len(tverd._LABEL_TO_WIRE)


# --- per-ego topic naming --------------------------------------------------------

def test_topic_round_trips():
    for ego in (1, 5, 42):
        assert tverd.ego_id_from_topic(tverd.topic(ego)) == ego


def test_foreign_topics_decode_to_none():
    assert tverd.ego_id_from_topic('/perception/global_trustworthiness/sdsm') is None
    assert tverd.ego_id_from_topic(tverd.topic(1) + '/extra') is None


# --- the ledger's read-only status view -------------------------------------------

def test_status_is_none_for_an_unknown_track():
    assert PendingVerdicts().status(OTHER, 'nope') is None


def test_status_is_deferred_inside_the_grace_window():
    ledger = PendingVerdicts()
    ledger.observe(OTHER, 1, [('t', 1.0, 1.0, 0.0)])
    assert ledger.status(OTHER, 't') == 'deferred'


def test_status_is_expired_once_the_deadline_passes():
    ledger = PendingVerdicts()
    for frame in range(1, DEADLINE + 2):
        ledger.observe(OTHER, frame, [('t', 1.0, 1.0, 0.0)])
    assert ledger.status(OTHER, 't') == 'expired'


def test_status_clears_once_corroboration_settles_the_track():
    ledger = PendingVerdicts()
    ledger.observe(OTHER, 1, [('t', 1.0, 1.0, 0.0)])
    ledger.observe(OTHER, 2, [('t', 1.0, 1.0, THETA + 1.0)])
    assert ledger.status(OTHER, 't') is None


def test_status_is_a_pure_read():
    """Reading a verdict must not advance the ledger -- a display that polls
    it cannot perturb reputation."""
    ledger = PendingVerdicts()
    ledger.observe(OTHER, 1, [('t', 1.0, 1.0, 0.0)])
    for _ in range(5):
        assert ledger.status(OTHER, 't') == 'deferred'


# --- engine Stage 5b -------------------------------------------------------------

def _engine() -> TrustEngine:
    return TrustEngine(PersistentReputationTracker(':memory:'))


def test_objects_both_agents_see_read_matched():
    stats = _engine().process_frame({OTHER: list(EGO_POS)}, EGO_POS, {}, [])[0]
    assert stats.verdict == (VERDICT_MATCHED, VERDICT_MATCHED)


def test_an_object_only_the_peer_sees_is_held_as_deferred():
    """With no TrackData there are no stable track ids, so the ledger cannot
    grade the extra object -- it is held, and 'deferred' is what held means."""
    stats = _engine().process_frame(
        {OTHER: list(EGO_POS) + [GHOST]}, EGO_POS, {}, [])[0]
    assert stats.verdict == (VERDICT_MATCHED, VERDICT_MATCHED, VERDICT_DEFERRED)


def test_verdict_is_parallel_to_the_peers_detections():
    others = list(EGO_POS) + [GHOST]
    stats = _engine().process_frame({OTHER: others}, EGO_POS, {}, [])[0]
    assert len(stats.verdict) == len(others)


def test_matched_ego_reports_which_ego_boxes_were_corroborated():
    stats = _engine().process_frame({OTHER: list(EGO_POS)}, EGO_POS, {}, [])[0]
    assert stats.matched_ego == (0, 1)


def test_matched_ego_is_empty_when_the_peer_corroborates_nothing():
    stats = _engine().process_frame({OTHER: [GHOST]}, EGO_POS, {}, [])[0]
    assert stats.matched_ego == ()
    assert stats.verdict == (VERDICT_DEFERRED,)


# --- codec ------------------------------------------------------------------------

def _stats(verdict, admitted=(), matched_ego=(), trusted=True, r_new=0.7):
    """Minimal stand-in: build() reads exactly these five FrameStats fields."""
    return SimpleNamespace(trusted=trusted, r_new=r_new, verdict=verdict,
                           admitted=admitted, matched_ego=matched_ego)


def _build(stats, msg_cnt=0, ego_msg_cnt=0):
    """build() with the two agent ids fixed at ego=1, judged sender=2."""
    return tverd.build(stats, (1, 0, 0, 0), (2, 0, 0, 0), msg_cnt, ego_msg_cnt)


def test_build_encodes_identity_and_labels():
    msg = _build(_stats((VERDICT_MATCHED, VERDICT_UNCORROBORATED), admitted=(0,)),
                 msg_cnt=17)
    assert tverd.ego_of(msg) == 1
    assert tverd.sender_of(msg) == 2
    assert msg.msg_cnt == 17
    assert list(msg.verdict) == [tverd.Message.VERDICT_MATCHED,
                                 tverd.Message.VERDICT_UNCORROBORATED]


def test_admitted_indices_become_dense_per_detection_bools():
    msg = _build(_stats((VERDICT_MATCHED,) * 4, admitted=(0, 3)))
    assert list(msg.admitted) == [True, False, False, True]


def test_per_detection_arrays_stay_parallel():
    msg = _build(_stats((VERDICT_MATCHED, VERDICT_UNCORROBORATED, VERDICT_DEFERRED),
                        admitted=(1,)))
    assert len(msg.verdict) == len(msg.admitted) == 3


def test_matched_ego_indices_round_trip():
    msg = _build(_stats((VERDICT_MATCHED,), matched_ego=(0, 2, 5)))
    assert list(msg.matched_ego_index) == [0, 2, 5]


def test_ego_msg_cnt_identifies_the_frame_matched_ego_indexes():
    """matched_ego_index points into EGO's own message, which is a different
    message from the judged sender's -- so it needs its own correlation key."""
    msg = _build(_stats((VERDICT_MATCHED,), matched_ego=(3,)),
                 msg_cnt=17, ego_msg_cnt=42)
    assert msg.msg_cnt == 17
    assert msg.ego_msg_cnt == 42


def test_ego_msg_cnt_of_minus_one_marks_a_silent_ego():
    """Replay mode: ego only listens, so there is no ego message to index."""
    msg = _build(_stats((VERDICT_DEFERRED,)), ego_msg_cnt=-1)
    assert msg.ego_msg_cnt == -1


def test_a_gate_failing_sender_is_still_encoded():
    """The reason this channel exists: SdsmTrustOutput cannot carry a rejected
    sender at all, so the visualization has to learn about it here."""
    msg = _build(_stats((VERDICT_UNCORROBORATED,), admitted=(),
                        trusted=False, r_new=0.2))
    assert msg.trusted is False
    assert list(msg.admitted) == [False]
    assert round(msg.r_new, 3) == 0.2
