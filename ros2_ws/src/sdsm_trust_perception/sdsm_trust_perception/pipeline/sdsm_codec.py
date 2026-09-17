# mypy: ignore-errors
# sdsm_msgs / sdsm_trust_interfaces ship generated message classes.
"""
SDSM wire-format codec: the ONE place that knows sdsm_msgs/SensorDataSharingMessage.

Plays the same information-hiding role CPX-Mono's
global_trust_perception/pipeline/perception_message.py plays for
sdsm_interfaces/SdsmPayload: nothing outside this module names a
SensorDataSharingMessage field directly, so trust_node and
trustworthy_perception.TrustEngine stay format-agnostic. Callers deal only in
plain Python values returned here.

WHAT THIS SIM'S WIRE FORMAT ACTUALLY CARRIES (and doesn't)
------------------------------------------------------------
RosSDSMApp.cc's buildSdsmJson() is the ground truth for what's really on the
wire (udp_bridge_node decodes its JSON straight into SensorDataSharingMessage
fields with the same names, so the two are the same shape):

  - ref_pos is GEODETIC (lat/lon, J2735 1e-7 deg units per Position3D.msg),
    produced by projecting the sender's true sim-frame (x, y) through a flat
    equirectangular approximation anchored at ORIGIN_LAT/ORIGIN_LON (see
    RosSDSMApp.h). local_xy_of() below inverts that EXACT formula to recover
    (x, y) in metres -- this is not a generic geodetic conversion, it only
    round-trips correctly against this sim's own projection and this origin.
  - every object is currently emitted as a VEHICLE (det_obj_opt_kind=1) with
    a HARDCODED size (1.8 m x 4.5 m, det_veh.size); VRU/obstacle branches are
    schema-complete but unused by the current sender. get_dims_of() still
    handles all three kinds so this codec keeps working if that changes.
  - measurement_time is always 0 (no per-object capture-time modelling).
  - class_conf / obj_type_cfd are set but do not mean per-object detection
    certainty (obj_type_cfd is hardcoded 100 in the JSON builder).
  - NO FIELD ANYWHERE ON THIS WIRE CARRIES PER-OBJECT OR FRAME-LEVEL LOCAL
    CERTAINTY (CPX-Mono's SdsmPayload.obj_local_scores / local_score have no
    counterpart here). get_local_scores_of() / local_score_of() therefore
    always return the neutral 1.0 -- this port's trust judgment rests
    entirely on the SECOND-LAYER checks (kinematic plausibility, size
    agreement, corroboration, reputation), never on a sender's own claimed
    confidence, because the sender does not send one.
  - object_id is the reported vehicle's own SUMO/OMNeT node index (per
    RosSDSMApp.cc: `p->getObject_id(i)`), i.e. a STABLE cross-sender identity
    for "the same real vehicle", not an ephemeral per-message slot index.
    trustworthy_perception.TrustEngine's matching stage relies on exactly
    this to pair a judge's own report of a vehicle with a sender's report of
    the same vehicle -- see that module's docstring for why this replaces
    CPX-Mono's spatial WBF clustering here.
"""

import math
from typing import Optional

from sdsm_msgs.msg import (
    DDateTime,
    DetectedObjectCommonData,
    DetectedObjectData,
    DetectedObstacleData,
    DetectedVehicleData,
    DetectedVRUData,
    ObstacleSize,
    Position3D,
    PositionConfidenceSet,
    PositionOffsetXYZ,
    SensorDataSharingMessage,
    VehicleSize,
)
from sdsm_trust_interfaces.msg import ReceivedSdsm

# --- Public, format-agnostic handle ------------------------------------------
Message = SensorDataSharingMessage

# --- This sim's local-tangent-plane origin (RosSDSMApp.h ORIGIN_LAT/ORIGIN_LON) -
# MUST match the simulation's constants exactly, or local_xy_of() recovers the
# wrong metres. If the sim's origin ever changes, update this to match.
ORIGIN_LAT_DEG = 34.0689
ORIGIN_LON_DEG = -118.4452
_METRES_PER_DEG_LAT = 111_320.0
_ORIGIN_COS_LAT = math.cos(math.radians(ORIGIN_LAT_DEG))

# --- Wire unit conventions (SAE J2735/J3224, as declared in the .msg files) --
LATLON_UNIT_DEG   = 1e-7     # Position3D.lat/lon -> degrees
OFFSET_UNIT_M      = 0.1      # PositionOffsetXYZ -> metres
SPEED_UNIT_MS       = 0.02     # DetectedObjectCommonData.speed -> m/s
SPEED_UNAVAILABLE   = 8191
HEADING_UNIT_DEG    = 0.0125   # DetectedObjectCommonData.heading -> degrees
HEADING_UNAVAILABLE = 28800
VEHICLE_SIZE_UNIT_M = 0.01     # VehicleSize.width/length are CENTIMETRES
VEHICLE_HEIGHT_UNIT_M = 0.05   # DetectedVehicleData.height -> 5 cm units
OBSTACLE_SIZE_UNIT_M  = 0.10   # ObstacleSize.width/length/height -> 10 cm units

OBJ_TYPE_UNKNOWN  = 0
OBJ_TYPE_VEHICLE  = 1
OBJ_TYPE_VRU      = 2
OBJ_TYPE_ANIMAL   = 3
OBJ_TYPE_OBSTACLE = 4

OPT_DATA_NONE    = 0
OPT_DATA_VEHICLE = 1
OPT_DATA_VRU     = 2
OPT_DATA_OBSTACLE = 3


def local_xy_of(ref_pos) -> tuple[float, float]:
    """
    Invert RosSDSMApp::buildSdsmJson's flat-earth projection to recover this
    sender's (x, y) in metres, sim frame.

    Forward (C++, RosSDSMApp.cc):
        latDeg = ORIGIN_LAT + y / 111320.0
        lonDeg = ORIGIN_LON + x / (111320.0 * cos(ORIGIN_LAT * pi/180))
    so:
        y = (latDeg - ORIGIN_LAT) * 111320.0
        x = (lonDeg - ORIGIN_LON) * 111320.0 * cos(ORIGIN_LAT * pi/180)

    ref_pos.lat/lon are int32 in 1e-7 deg units (Position3D.msg). elevation is
    always the J2735 "unavailable" sentinel (-4096) from this sim and is not
    decoded to a z -- every position this codec produces is 2D (z = 0.0).
    """
    lat_deg = ref_pos.lat * LATLON_UNIT_DEG
    lon_deg = ref_pos.lon * LATLON_UNIT_DEG
    y = (lat_deg - ORIGIN_LAT_DEG) * _METRES_PER_DEG_LAT
    x = (lon_deg - ORIGIN_LON_DEG) * _METRES_PER_DEG_LAT * _ORIGIN_COS_LAT
    return x, y


def sender_of(msg: Message) -> int:
    """Decode the sending node's id from source_id[0] (matches RosSDSMApp's nodeIndex_)."""
    return int(msg.source_id[0])


def sender_id_tuple(msg: Message) -> tuple[int, int, int, int]:
    """Decode the full 4-byte anonymized sender id from source_id."""
    return (int(msg.source_id[0]), int(msg.source_id[1]),
            int(msg.source_id[2]), int(msg.source_id[3]))


def source_id_tuple_for(node: int) -> tuple[int, int, int, int]:
    """The 4-byte source_id a node broadcasts under (encode side of sender_of)."""
    return (int(node), 0, 0, 0)


def num_detections_of(msg: Message) -> int:
    return len(msg.objects)


def sequence_of(msg: Message) -> int:
    return int(msg.msg_cnt)


def ref_pos_of(msg: Message) -> tuple[float, float, float]:
    """Decode the sender's own (x, y, z) in metres, sim frame. z is always 0.0
    (see local_xy_of -- elevation is never usably populated by this sim)."""
    x, y = local_xy_of(msg.ref_pos)
    return (x, y, 0.0)


def get_global_positions_of(msg: Message) -> list[tuple[float, float, float]]:
    """Decode every object into a global (x, y, z) position in metres.

    global = ref_pos + offset * OFFSET_UNIT_M; z is always 0.0 (offset_z is
    schema-present but this sim never sets has_offset_z).
    """
    rx, ry, rz = ref_pos_of(msg)
    positions = []
    for obj in msg.objects:
        pos = obj.det_obj_common.pos
        x = rx + pos.offset_x * OFFSET_UNIT_M
        y = ry + pos.offset_y * OFFSET_UNIT_M
        z = rz + (pos.offset_z * OFFSET_UNIT_M if pos.has_offset_z else 0.0)
        positions.append((x, y, z))
    return positions


def get_object_ids_of(msg: Message) -> list[int]:
    """Decode each object's reported id -- see module docstring: this is the
    reporting vehicle's own stable node index, the key TrustEngine matches on."""
    return [int(obj.det_obj_common.object_id) for obj in msg.objects]


def _dims_of_one(obj) -> tuple[float, float, float]:
    """(width, length, height) in metres for one DetectedObjectData, by its
    det_obj_opt_kind discriminator. (0.0, 0.0, 0.0) when the kind carries no
    usable size -- VRU always, obstacle/vehicle when their has_* flags are
    unset -- which attribute_checks.check_size_agreement treats as neutral
    agreement (both-near-zero), not as a disagreement."""
    kind = int(obj.det_obj_opt_kind)
    if kind == OPT_DATA_VEHICLE:
        veh = obj.det_veh
        if not veh.has_size:
            return (0.0, 0.0, 0.0)
        w = veh.size.width * VEHICLE_SIZE_UNIT_M
        l = veh.size.length * VEHICLE_SIZE_UNIT_M
        h = veh.height * VEHICLE_HEIGHT_UNIT_M if veh.has_height else 0.0
        return (w, l, h)
    if kind == OPT_DATA_OBSTACLE:
        sz = obj.det_obst.obst_size
        return (sz.width * OBSTACLE_SIZE_UNIT_M,
                sz.length * OBSTACLE_SIZE_UNIT_M,
                sz.height * OBSTACLE_SIZE_UNIT_M)
    # OPT_DATA_VRU or OPT_DATA_NONE: no size in this schema.
    return (0.0, 0.0, 0.0)


def get_dims_of(msg: Message) -> list[tuple[float, float, float]]:
    """Decode per-object (width, length, height) in metres."""
    return [_dims_of_one(obj) for obj in msg.objects]


_CLASS_NAMES = {
    OBJ_TYPE_UNKNOWN: 'unknown',
    OBJ_TYPE_VEHICLE: 'vehicle',
    OBJ_TYPE_VRU: 'VRU',
    OBJ_TYPE_ANIMAL: 'animal',
    OBJ_TYPE_OBSTACLE: 'obstacle',
}


def class_name_of(obj_type: int) -> str:
    return _CLASS_NAMES.get(int(obj_type), str(int(obj_type)))


def get_labels_of(msg: Message) -> list[int]:
    """Decode per-object class as the J3224 obj_type integer."""
    return [int(obj.det_obj_common.obj_type) for obj in msg.objects]


def get_headings_of(msg: Message) -> list[float]:
    """Decode per-object heading in degrees (0=North, clockwise). The
    HEADING_UNAVAILABLE sentinel (28800) is passed through as 360.0, same
    convention CPX-Mono's codec uses, so downstream treats it as axis-aligned."""
    return [float(obj.det_obj_common.heading) * HEADING_UNIT_DEG for obj in msg.objects]


def get_velocities_of(msg: Message) -> list[tuple[float, float]]:
    """Decode per-object (vx, vy) in m/s from speed + heading, for seeding a
    new SORT track's initial velocity. SPEED_UNAVAILABLE reads as (0, 0)."""
    velocities = []
    for obj in msg.objects:
        common = obj.det_obj_common
        speed_raw = int(common.speed)
        h = int(common.heading)
        if speed_raw == SPEED_UNAVAILABLE or h == HEADING_UNAVAILABLE:
            velocities.append((0.0, 0.0))
            continue
        speed_ms = speed_raw * SPEED_UNIT_MS
        heading_rad = math.radians(h * HEADING_UNIT_DEG)
        velocities.append((speed_ms * math.sin(heading_rad), speed_ms * math.cos(heading_rad)))
    return velocities


def get_local_scores_of(msg: Message) -> list[float]:
    """No per-object local certainty exists on this wire -- see module
    docstring. Always neutral (1.0) per object."""
    return [1.0] * len(msg.objects)


def local_score_of(msg: Message) -> float:
    """No frame-level local certainty exists on this wire -- always neutral."""
    return 1.0


def get_equipment_type_of(msg: Message) -> int:
    return int(msg.equipment_type)


def capture_lag_of(_msg: Message) -> float:
    """Capture->send lag in seconds. Always 0.0: measurement_time is
    hardcoded 0 by the current sender (see module docstring) -- unlike
    CPX-Mono's non-negative capture lag, DetectedObjectCommonData.
    measurement_time is signed (-1500..1500 ms) per J2735, so a sim that
    starts populating it would need this function to decide how to treat a
    negative value; deferred until real data exists to decide it against."""
    return 0.0


def sdsm_from_dict(d: dict) -> SensorDataSharingMessage:
    """Build a SensorDataSharingMessage from RosSDSMApp::buildSdsmJson's dict
    shape -- the wire format both veins_ros_bridge's live UDP path and any
    offline replay of a rosBridgeMode="log" .jsonl file share (RosSDSMApp.cc
    emits byte-identical JSON either way; only the transport differs). Field
    names below match that function 1:1; see it for the authoritative shape.

    Moved here (was udp_bridge_node._decode_sdsm) so both the live bridge and
    an offline log replay import one decoder instead of drifting copies."""
    msg = SensorDataSharingMessage()
    msg.msg_cnt = int(d['msg_cnt'])
    msg.source_id = [int(b) & 0xFF for b in d['source_id']]
    msg.equipment_type = int(d['equipment_type'])

    ts = d['sdsm_time_stamp']
    msg.sdsm_time_stamp = DDateTime(
        day_of_month=int(ts['day_of_month']), time_of_day=int(ts['time_of_day']),
    )

    rp = d['ref_pos']
    msg.ref_pos = Position3D(
        lat=int(rp['lat']), lon=int(rp['lon']), elevation=int(rp['elevation']),
    )

    objects = []
    for o in d.get('objects', []):
        common = o['det_obj_common']
        pos = common['pos']
        conf = common['pos_confidence']
        det_common = DetectedObjectCommonData(
            obj_type=int(common['obj_type']),
            obj_type_cfd=int(common['obj_type_cfd']),
            object_id=int(common['object_id']) & 0xFFFF,
            measurement_time=int(common['measurement_time']),
            pos=PositionOffsetXYZ(
                offset_x=int(pos['offset_x']), offset_y=int(pos['offset_y']),
                offset_z=int(pos['offset_z']), has_offset_z=bool(pos['has_offset_z']),
            ),
            pos_confidence=PositionConfidenceSet(
                pos_confidence=int(conf['pos_confidence']),
                elevation_confidence=int(conf['elevation_confidence']),
            ),
            speed=int(common['speed']) & 0xFFFF,
            speed_z=int(common['speed_z']) & 0xFFFF,
            has_speed_z=bool(common['has_speed_z']),
            heading=int(common['heading']) & 0xFFFF,
        )

        veh = o['det_veh']
        vsize = veh['size']
        det_veh = DetectedVehicleData(
            size=VehicleSize(width=int(vsize['width']) & 0xFFFF,
                             length=int(vsize['length']) & 0xFFFF),
            has_size=bool(veh['has_size']),
            height=int(veh['height']) & 0xFFFF,
            has_height=bool(veh['has_height']),
            vehicle_class=int(veh['vehicle_class']) & 0xFF,
            has_vehicle_class=bool(veh['has_vehicle_class']),
            class_conf=int(veh['class_conf']) & 0xFF,
            has_class_conf=bool(veh['has_class_conf']),
        )

        vru = o['det_vru']
        det_vru = DetectedVRUData(basic_type=int(vru['basic_type']) & 0xFF)

        obst = o['det_obst']['obst_size']
        det_obst = DetectedObstacleData(
            obst_size=ObstacleSize(width=int(obst['width']) & 0xFFFF,
                                   length=int(obst['length']) & 0xFFFF,
                                   height=int(obst['height']) & 0xFFFF),
        )

        objects.append(DetectedObjectData(
            det_obj_common=det_common,
            det_obj_opt_kind=int(o['det_obj_opt_kind']) & 0xFF,
            det_veh=det_veh, det_vru=det_vru, det_obst=det_obst,
        ))
    msg.objects = objects
    return msg


def envelope_receiver(event: ReceivedSdsm) -> int:
    """The node an event concerns: the sender for a TX event, the receiver
    for an RX event (see ReceivedSdsm.msg -- `node` means both by design)."""
    return int(event.node)
