# mypy: ignore-errors
# sdsm_interfaces ships generated message classes; mypy cannot resolve them.
# This is the single ROS-message boundary — excluded from type checking on
# purpose (see test/test_mypy.py).
"""
Perception-message codec: the ONE place that knows the wire formats.

Information-hiding boundary around the cooperative-perception messages.
There are TWO, and the difference between them is the point:

    Message       (sdsm_interfaces/SdsmPayload)     - INPUT: what every agent
                  broadcasts. Carries per-object local certainty
                  (obj_local_scores), the sender's own view of its detections.
    OutputMessage (sdsm_interfaces/SdsmTrustOutput) - OUTPUT: what an ego
                  rebroadcasts for a sender it has JUDGED and admitted. Drops
                  the per-object array (no downstream meaning, 128 bytes on an
                  802.11p link) and adds a singular global_score.

Only gate-passing senders are ever built into an OutputMessage, so the
trusted set is expressed by presence on the topic rather than by a flag —
a consumer of the output topic cannot accidentally act on withheld
perception. Nothing outside this module names either concrete type; the
rest of the package goes through the interface below, so a format can be
swapped by editing only this file:

    Message                  - the input message type (pub/sub + type hints)
    OutputMessage            - the output message type (pub/sub + type hints)
    TOPIC                    - the topic the input message travels on
    build(...)               - encode  (publish side)
    sender_of(...)           - decode the sending agent id
    num_detections_of(...)   - decode the number of carried detections
    sequence_of(...)         - decode the wrapping message count
    ref_pos_of(...)              - decode the sender's own (x,y,z) in metres
    get_global_positions_of(...) - decode detections to (x,y,z) in metres
    get_velocities_of(...)       - decode detections to (vx,vy) in m/s
    get_dims_of(...)             - decode detections to (width,height) in m
    get_headings_of(...)         - decode detections to heading in degrees
    get_labels_of(...)           - decode per-object J3224 obj_type class ids
    class_name_of(...)           - display name for one obj_type
    get_local_scores_of(...)     - decode per-object local certainty in [0,1]
    local_score_of(...)          - decode the frame-level (singular) local certainty
    get_equipment_type_of(...)   - decode the sender's equipment type
    send_latency_of(...)         - decode send->receive latency in seconds
    capture_lag_of(...)          - decode capture->send lag in seconds
    with_global_score(...)       - build the OutputMessage rebroadcast of a sender
    global_score_of(...)         - decode an OutputMessage's global score

Callers deal only in plain Python values returned here and never touch a
raw message field.

CONVENTIONS (SdsmPayload-specific — private to this module):
- Agent ID is source_id[0].
- ref_pos_x/y/z are in metres directly — no geodetic conversion.
- offset_x/y/z, obj_width, obj_height use 0.1 m per integer unit.
- obj_speed uses 0.02 m/s per unit; obj_heading uses 0.0125 deg per unit
  (28800 = unavailable), matching J2735 conventions.
- Scenario: the ground-truth scene, motion, and visibility rules live in
  sim_world; build() only packs the Detection tuples it returns into wire units.
"""

import math
import time

from sdsm_interfaces.msg import SdsmPayload, SdsmTrustOutput
from global_trust_perception.pipeline import sim_world
from global_trust_tracker.sdsm_units import (
    HEADING_UNAVAILABLE as _HEADING_UNAVAILABLE,
    HEADING_UNIT_DEG as _HEADING_UNIT,
    OFFSET_UNIT_M as _OFFSET_UNIT_M,
    SPEED_UNIT_MS as _SPEED_UNIT,
    # Re-exported, not wrapped: the tracker node decodes the same field, and
    # one definition of a wire format is the point of sdsm_units. Callers keep
    # reading it as pmsg.capture_lag_of.
    capture_lag_of,        # noqa: F401
)


# --- Public, format-agnostic handles -------------------------------------
Message = SdsmPayload
OutputMessage = SdsmTrustOutput
TOPIC              = '/perception/global_trustworthiness/sdsm'
TRUST_OUTPUT_TOPIC = '/perception/global_trustworthiness/trust_output'

# J3224 ObjectType, named here because obj_type is a WIRE convention and this
# module owns those. A sender maps its own dataset classes onto this enum
# (local_trust_estimation's CLASS_OBJECT_TYPES), leaving anything it could not
# place on UNKNOWN rather than guessing it into a group -- so UNKNOWN is a real
# third value a receiver sees, not a can't-happen default.
OBJ_TYPE_UNKNOWN = 0
OBJ_TYPE_VEHICLE = 1
OBJ_TYPE_VRU     = 2

# --- Private constants ---------------------------------------------------
_MSG_COUNT_MODULO    = 128    # msg_cnt wraps at 127
_DAY_MS              = 86_400_000   # ms per day; sdsm_time_of_day_ms wraps here


def build(agent_id: int, counter: int) -> Message:
    """
    Build one SdsmPayload for agent_id at tick counter.

    Position and detections come from sim_world (the ground-truth scene and
    visibility model); this function only packs them into wire units. ref_pos is
    the sensor's own position this tick; offsets and sizes are 0.1 m integer units.
    """
    msg = SdsmPayload()

    msg.msg_cnt        = counter % _MSG_COUNT_MODULO
    msg.source_id      = list(source_id_tuple_for(agent_id))
    msg.equipment_type = sim_world.equipment_type_of(agent_id)
    # Real wall-clock send time (UTC), so receivers can measure freshness.
    # Epoch seconds are UTC-midnight aligned, so mod-day gives ms since
    # UTC midnight directly; sdsm_day carries the UTC day-of-month.
    now = time.time()
    msg.sdsm_day            = time.gmtime(now).tm_mday
    msg.sdsm_time_of_day_ms = int((now % 86_400) * 1000)

    (ax, ay), detections = sim_world.observe(agent_id, counter)
    msg.ref_pos_x = ax
    msg.ref_pos_y = ay
    msg.ref_pos_z = 0.0

    msg.num_objects = len(detections)
    for i, d in enumerate(detections):
        msg.obj_type[i]                = d.obj_type
        msg.object_id[i]               = d.object_id
        msg.offset_x[i]                = round((d.x - ax) / _OFFSET_UNIT_M)
        msg.offset_y[i]                = round((d.y - ay) / _OFFSET_UNIT_M)
        msg.offset_z[i]                = 0
        msg.obj_width[i]               = round(d.width  / _OFFSET_UNIT_M)
        msg.obj_length[i]              = round(d.length / _OFFSET_UNIT_M)
        msg.obj_height[i]              = round(d.height / _OFFSET_UNIT_M)
        msg.obj_speed[i]               = round(d.speed_mps / _SPEED_UNIT)
        msg.obj_heading[i]             = (
            _HEADING_UNAVAILABLE if d.heading_deg is None
            else round(d.heading_deg / _HEADING_UNIT)
        )
        # Capture lag, not an absolute time (see SdsmPayload.msg): the sim
        # observes and publishes in the same tick, so its frames are never
        # stale on send. The old value here was an absolute ms-since-start
        # that nothing read and that overflowed this uint16 field after 65 s
        # of scene.
        msg.obj_measurement_time_ms[i] = 0
        msg.obj_local_scores[i]        = d.local_score

    msg.local_score = (
        sum(d.local_score for d in detections) / len(detections)
        if detections else 0.0
    )

    return msg


def send_latency_of(msg: Message, recv_time_s: float) -> float:
    """
    Decode send->receive latency in seconds from the message's send stamp.

    recv_time_s is the receiver's wall-clock time.time() captured when the
    message arrived. Both stamps are reduced to ms-since-UTC-midnight, and
    the difference is taken as the SHORTEST wrap-aware distance, mapped to
    [-12 h, +12 h): a message sent at 23:59:59.9 and received at 00:00:00.1
    yields +0.2 s, not -86399.8 s.

    The result can be slightly negative when the sender's clock runs ahead
    of the receiver's; kinematic_freshness_factor tolerates that up to its
    CLOCK_SKEW_TOLERANCE_S and raises beyond it. Send/receive timestamps are
    trusted inputs here (a separate concern from this factor, which only
    weighs how far a sender's tracked speed could have carried it during
    this latency) -- a dishonest send stamp is not this factor's job to
    catch.
    """
    recv_ms = (recv_time_s % 86_400) * 1000.0
    delta_ms = recv_ms - float(msg.sdsm_time_of_day_ms)
    delta_ms = (delta_ms + _DAY_MS / 2) % _DAY_MS - _DAY_MS / 2
    return delta_ms / 1000.0




def sender_of(msg: Message) -> int:
    """Decode the sending agent's ID from source_id[0]."""
    return int(msg.source_id[0])


def sender_id_tuple(msg: Message) -> tuple[int, int, int, int]:
    """Decode the full 4-byte anonymized sender ID from source_id."""
    return (int(msg.source_id[0]), int(msg.source_id[1]),
            int(msg.source_id[2]), int(msg.source_id[3]))


def source_id_tuple_for(agent_id: int) -> tuple[int, int, int, int]:
    """
    The 4-byte source_id an agent broadcasts under (encode side of sender_of).

    Lets a caller name its OWN id in wire form without hard-coding the
    "agent id goes in byte 0, rest zero" convention outside this module.
    """
    return (int(agent_id), 0, 0, 0)


def num_detections_of(msg: Message) -> int:
    """Return the number of detections carried in this message."""
    return int(msg.num_objects)


def sequence_of(msg: Message) -> int:
    """Return the wrapping message count."""
    return int(msg.msg_cnt)


def get_global_positions_of(msg: Message) -> list[tuple[float, float, float]]:
    """
    Decode every detection into a global (x, y, z) position in metres.

    ref_pos_x/y/z is the sender's position, already in metres.
    offset_x/y/z is relative to that position, in 0.1 m units.
    global = ref_pos + offset * 0.1
    """
    positions = []
    for i in range(msg.num_objects):
        x = msg.ref_pos_x + msg.offset_x[i] * _OFFSET_UNIT_M
        y = msg.ref_pos_y + msg.offset_y[i] * _OFFSET_UNIT_M
        z = msg.ref_pos_z + msg.offset_z[i] * _OFFSET_UNIT_M
        positions.append((x, y, z))
    return positions


def ref_pos_of(msg: Message | OutputMessage) -> tuple[float, float, float]:
    """
    Decode the sender's OWN reference position (x, y, z) in metres.

    Already metres on the wire, unlike the per-object offsets. Accepts either
    message: ref_pos is part of the payload both carry.
    """
    return (float(msg.ref_pos_x), float(msg.ref_pos_y), float(msg.ref_pos_z))


def get_velocities_of(msg: Message) -> list[tuple[float, float]]:
    """
    Decode per-object (vx, vy) in m/s from obj_speed and obj_heading.

    Heading convention: 0 = North (+y), clockwise. 28800 = unavailable.
    """
    velocities = []
    for i in range(msg.num_objects):
        speed_ms = msg.obj_speed[i] * _SPEED_UNIT
        h = msg.obj_heading[i]
        if h == _HEADING_UNAVAILABLE:
            velocities.append((0.0, 0.0))
        else:
            heading_rad = math.radians(h * _HEADING_UNIT)
            velocities.append((
                speed_ms * math.sin(heading_rad),
                speed_ms * math.cos(heading_rad),
            ))
    return velocities


def get_object_ids_of(msg: Message) -> list[int]:
    """
    Decode each object's tracker id.

    Stable across frames for as long as the track lives, which is what a
    visualizer needs to address the same box from one frame to the next: an
    array index addresses a different object as soon as another one enters or
    leaves the message.
    """
    return [int(msg.object_id[i]) for i in range(msg.num_objects)]


def get_dims_of(msg: Message) -> list[tuple[float, float, float]]:
    """
    Decode per-object (width, length, height) in metres.

    All three fields use 0.1 m per integer unit.
    """
    return [
        (
            msg.obj_width[i]  * _OFFSET_UNIT_M,
            msg.obj_length[i] * _OFFSET_UNIT_M,
            msg.obj_height[i] * _OFFSET_UNIT_M,
        )
        for i in range(msg.num_objects)
    ]


def get_labels_of(msg: Message) -> list[int]:
    """Decode per-object class as J2735 obj_type integer (1=vehicle, 2=person, etc.)."""
    return [int(msg.obj_type[i]) for i in range(msg.num_objects)]


_CLASS_NAMES = {
    OBJ_TYPE_UNKNOWN: 'unknown',
    OBJ_TYPE_VEHICLE: 'vehicle',
    OBJ_TYPE_VRU: 'VRU',
}


def class_name_of(obj_type: int) -> str:
    """
    Display name for one J3224 obj_type; the raw integer if unrecognised.

    'VRU' rather than 'pedestrian': a sender groups cyclists and motorcyclists
    into this class alongside pedestrians, so naming it after only one of them
    is wrong on the others. UNKNOWN is named too — a class the sender could not
    place must not read as a confident 'vehicle'.

    Falls back to the integer rather than to a name: an obj_type this codec has
    no name for is a message from a sender using more of J3224 than we do, and
    showing the number says that, where any word would invent a claim.
    """
    return _CLASS_NAMES.get(int(obj_type), str(int(obj_type)))


def get_headings_of(msg: Message) -> list[float]:
    """
    Decode per-object heading in degrees (J2735: 0 = North, clockwise).

    obj_heading is in 0.0125 deg units; 28800 (= 360 deg) means unavailable and
    is passed through as 360.0 so downstream can treat it as axis-aligned.
    """
    return [float(msg.obj_heading[i]) * _HEADING_UNIT for i in range(msg.num_objects)]


def get_local_scores_of(msg: Message) -> list[float]:
    """Decode per-object local certainty in [0,1] (higher = more certain)."""
    return [float(msg.obj_local_scores[i]) for i in range(msg.num_objects)]


def local_score_of(msg: Message) -> float:
    """Decode the frame-level (singular) local certainty set by the sender."""
    return float(msg.local_score)


def get_equipment_type_of(msg: Message | OutputMessage) -> int:
    """Decode the sender's equipment type (0=unknown 1=RSU 2=OBU 3=VRU).

    Accepts either message: equipment_type is part of the object payload both
    carry, and the scene visualizer decodes it from input and output alike.
    """
    return int(msg.equipment_type)


def with_global_score(msg: Message, global_score: float) -> OutputMessage:
    """
    Build the TRUST_OUTPUT_TOPIC rebroadcast of msg for one gate-passing sender.

    Carries the whole object payload forward unchanged and swaps the trust
    annotation: the input's per-object obj_local_scores[32] is dropped (it is
    the sender's own view of its detections, which has no meaning once they
    have been judged, and costs 128 bytes on the wire) and replaced by two
    singular scores -- the sender's own frame-level local_score, passed
    through, and global_score, this ego's reputation for the sender. Both are
    kept rather than sent as one pre-multiplied product, which could not be
    decomposed again by a consumer that wants to weigh them differently.

    ONLY call this for a sender that passed the trust gate: presence on the
    output topic is what marks a sender trusted (see SdsmTrustOutput.msg).

    The object arrays are assigned field-to-field from identically declared
    fields, so no conversion or length check can go wrong; nothing mutates
    them afterwards.
    """
    out = OutputMessage()

    out.msg_cnt        = msg.msg_cnt
    out.source_id      = msg.source_id
    out.equipment_type = msg.equipment_type

    out.ref_pos_x = msg.ref_pos_x
    out.ref_pos_y = msg.ref_pos_y
    out.ref_pos_z = msg.ref_pos_z

    out.sdsm_day            = msg.sdsm_day
    out.sdsm_time_of_day_ms = msg.sdsm_time_of_day_ms

    out.num_objects              = msg.num_objects
    out.obj_type                 = msg.obj_type
    out.object_id                = msg.object_id
    out.offset_x                 = msg.offset_x
    out.offset_y                 = msg.offset_y
    out.offset_z                 = msg.offset_z
    out.obj_width                = msg.obj_width
    out.obj_length               = msg.obj_length
    out.obj_height               = msg.obj_height
    out.obj_speed                = msg.obj_speed
    out.obj_heading              = msg.obj_heading
    out.obj_measurement_time_ms  = msg.obj_measurement_time_ms

    out.local_score  = msg.local_score
    out.global_score = float(global_score)

    return out


def global_score_of(msg: OutputMessage) -> float:
    """Decode the rebroadcasting ego's reputation for this message's sender."""
    return float(msg.global_score)


def trust_output_topic(agent_id: int) -> str:
    """
    Per-ego trust_output topic: the shared prefix with the ego id as suffix.

    The rebroadcasting ego's identity is carried by the topic NAME, never the
    SDSM payload, so the on-the-wire message stays standard J3224 (no extra
    field). Consumers that care whose trust view a rebroadcast represents learn
    it from the channel, mirroring how a real receiver knows a message's origin
    from the link it arrived on. Invert with ego_id_from_trust_topic.
    """
    return f'{TRUST_OUTPUT_TOPIC}/agent_{agent_id}'


def ego_id_from_trust_topic(topic: str) -> int | None:
    """Inverse of trust_output_topic; None if topic isn't a per-ego rebroadcast."""
    prefix = f'{TRUST_OUTPUT_TOPIC}/agent_'
    if not topic.startswith(prefix):
        return None
    suffix = topic[len(prefix):]
    return int(suffix) if suffix.isdigit() else None
