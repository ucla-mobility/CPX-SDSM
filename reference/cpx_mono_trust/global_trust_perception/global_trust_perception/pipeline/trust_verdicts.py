# mypy: ignore-errors
# sdsm_interfaces ships generated message classes; mypy cannot resolve them.
# A ROS-message boundary, excluded from type checking on purpose — same
# rationale as perception_message.py (see test/test_mypy.py).
"""
Trust-verdict codec: the ONE place that knows the diagnostic wire format.

Information-hiding boundary around sdsm_interfaces/TrustVerdicts, mirroring
what perception_message.py does for the perception messages. A second wire
format gets its own owner rather than diluting that module's claim to be the
one place that knows *the perception message*.

WHAT THIS CHANNEL IS. Vehicle-internal diagnostics, published for EVERY judged
sender — trusted or not — so a visualization can show what was rejected and
why. It is NOT the trust pipeline's output; SdsmTrustOutput is, and that one
exists only for gate-passing senders. Nothing in the production path may
consume this: acting on a sender found here with trusted=False would defeat
the gate it just failed.

    Message              - the message type (pub/sub + type hints)
    topic(ego_id)        - the per-ego topic it travels on
    ego_id_from_topic()  - inverse of topic()
    build(...)           - encode one agent's FrameStats
    ego_of / sender_of   - decode the judging / judged agent ids
"""

from sdsm_interfaces.msg import TrustVerdicts


# --- Public, format-agnostic handles -------------------------------------
Message = TrustVerdicts
TOPIC_PREFIX = '/perception/global_trustworthiness/verdicts'

# Engine verdict label -> wire value. The literals are deliberately NOT
# imported from trustworthy_perception: that module pulls in numpy, the MS-PSF
# fusion engine and the whole trust pipeline, none of which a visualization
# process should have to load just to colour a box. test_trust_verdicts.py
# pins this table against the engine's VERDICT_* constants so the two cannot
# drift apart unnoticed.
_LABEL_TO_WIRE = {
    'matched':        TrustVerdicts.VERDICT_MATCHED,
    'deferred':       TrustVerdicts.VERDICT_DEFERRED,
    'uncorroborated': TrustVerdicts.VERDICT_UNCORROBORATED,
}


def topic(ego_id: int) -> str:
    """
    Per-ego verdict topic: the shared prefix with the judging ego's id.

    The ego's identity rides in the topic NAME, matching how
    perception_message.trust_output_topic carries it, so a consumer can
    subscribe to one ego's judgments or discover all of them.
    """
    return f'{TOPIC_PREFIX}/agent_{ego_id}'


def ego_id_from_topic(topic_name: str) -> int | None:
    """Inverse of topic(); None if topic_name isn't a verdict topic."""
    prefix = f'{TOPIC_PREFIX}/agent_'
    if not topic_name.startswith(prefix):
        return None
    suffix = topic_name[len(prefix):]
    return int(suffix) if suffix.isdigit() else None


def build(stats, ego_source_id, source_id, msg_cnt, ego_msg_cnt) -> Message:
    """
    Encode one judged sender's FrameStats as a TrustVerdicts message.

    stats         : the FrameStats for ONE judged sender
    ego_source_id : 4-tuple, the judging ego's SDSM source_id
    source_id     : 4-tuple, the judged sender's SDSM source_id
    msg_cnt       : the judged SdsmPayload's msg_cnt — the correlation key a
                    consumer uses to line these verdicts up with the detections
                    they judge
    ego_msg_cnt   : ego's OWN msg_cnt this frame, which matched_ego_index
                    indexes into; -1 when ego broadcast nothing (replay mode)

    The per-detection arrays are emitted at len(stats.verdict), which is the
    engine's detection count for this sender, so they can never come out
    ragged against each other. A CONSUMER MUST STILL CHECK that length
    against the correlated message's num_objects before indexing: a flush
    window that happens to contain two messages from one sender accumulates
    both senders' detections into the engine while only the last message is
    kept as the correlation key, so the two can legitimately disagree.

    FrameStats.admitted is a tuple of INDICES; the wire form is a dense
    per-detection bool so a consumer reads it in parallel with `verdict`
    instead of redoing the membership test.
    """
    msg = Message()

    msg.ego_source_id = list(ego_source_id)
    msg.source_id     = list(source_id)
    msg.msg_cnt       = int(msg_cnt)
    msg.ego_msg_cnt   = int(ego_msg_cnt)

    msg.trusted = bool(stats.trusted)
    msg.r_new   = float(stats.r_new)

    admitted = set(stats.admitted)
    msg.verdict  = [_LABEL_TO_WIRE[label] for label in stats.verdict]
    msg.admitted = [i in admitted for i in range(len(stats.verdict))]

    msg.matched_ego_index = [int(i) for i in stats.matched_ego]

    return msg


def ego_of(msg: Message) -> int:
    """Decode the JUDGING ego's agent id from ego_source_id[0]."""
    return int(msg.ego_source_id[0])


def sender_of(msg: Message) -> int:
    """Decode the JUDGED sender's agent id from source_id[0]."""
    return int(msg.source_id[0])
