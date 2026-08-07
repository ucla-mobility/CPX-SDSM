#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS 1 (Melodic, Python 2.7) side of the Verizon ETX bridge: the converter.

  ROS 1 topic (tx)  --ros_to_dict--> JSON --UDP--> etx_relay.py --> Verizon cloud
  Verizon cloud --> etx_relay.py --UDP--> JSON --dict_to_ros--> ROS 1 topic (rx)

Melodic's rospy runs on Python 2.7, while the Verizon ETX client stack needs
Python >= 3.9, so the two halves live in separate processes bridged over
localhost UDP (relay listens on --relay-port; this node listens on --listen-port).

The JSON wire format is the ROS 2 style used by the Windows side
(header.stamp = {"sec", "nanosec"}, no header.seq), so payloads compare equal
end to end. Conversion is generic via __slots__/_slot_types recursion.  The
dedicated --detection-mode subscribes to one structured object topic. It can
also cache one visualization_msgs/MarkerArray topic and attach the latest
complete MarkerArray to the next object-state JSON envelope. This keeps the
existing one-packet-per-tracking-frame ETX design instead of creating a second
high-rate Verizon stream. The default real-bag profile is
autosense_msgs/TrackingObjectArray on /tracking2/tracking_objects. Only IDs,
size, position, direction and velocity are converted; embedded PointCloud2[]
segments are completely omitted. The optional autoware-raw profile converts
label/score, pose, dimensions and velocity while omitting PointCloud2, Image
and trajectory fields. No camera or point-cloud topic is subscribed.

For the UCLA ROS1 sensor bag, pass --sensor-bag-mode.  The node then subscribes
to the six CameraInfo/CompressedImage/TimeReference/PointCloud2 topics directly,
samples them at ETX-safe rates, and carries compact metadata + SHA-256 evidence
inside std_msgs/String instead of trying to put multi-megabyte arrays in UDP.

Run (with roscore up, inside a Melodic-sourced shell):
    python ros1_converter_node.py _tx_topic:=/etx_tx _rx_topic:=/etx_rx
or plain argparse:
    python ros1_converter_node.py --tx-topic /etx_tx --rx-topic /etx_rx
Target ROS1 sensor bag:
    python ros1_converter_node.py --sensor-bag-mode \
      --transport-log logs/<run>/sensor_source.jsonl
Detection-only bag:
    python ros1_converter_node.py --detection-mode \
      --transport-log logs/<run>/detection_source.jsonl
"""

from __future__ import print_function

import argparse
import hashlib
import importlib
import io
import json
import math
import os
import socket
import threading
import time

import genpy
import rospy

try:
    STRING_TYPES = (basestring,)  # noqa: F821  (Python 2 / Melodic)
except NameError:
    STRING_TYPES = (str,)


SENSOR_BAG_TOPICS = {
    "/axis214/camera_info": ("sensor_msgs/CameraInfo", 0.2),
    "/axis214/image_raw/compressed": ("sensor_msgs/CompressedImage", 1.0),
    "/axis215/camera_info": ("sensor_msgs/CameraInfo", 0.2),
    "/axis215/image_raw/compressed": ("sensor_msgs/CompressedImage", 1.0),
    "/gps_nw/gps_time_300hz": ("sensor_msgs/TimeReference", 5.0),
    "/ouster_nw/points": ("sensor_msgs/PointCloud2", 1.0),
}


# --------------------------------------------------------------- conversion
def ros_to_dict(msg):
    """Generic ROS 1 message -> dict, normalized to the ROS 2 JSON style."""
    result = {}
    for slot, slot_type in zip(msg.__slots__, msg._slot_types):
        value = getattr(msg, slot)
        result[slot] = _value_to_py(value, slot_type)
    # ROS 2 headers have no 'seq'; drop it so payloads match the Windows side.
    if "seq" in result and type(msg).__name__ == "Header":
        result.pop("seq")
    return result


def _value_to_py(value, slot_type):
    if isinstance(value, genpy.Time) or isinstance(value, genpy.Duration):
        # ROS 1 secs/nsecs -> ROS 2 sec/nanosec naming
        return {"sec": int(value.secs), "nanosec": int(value.nsecs)}
    if hasattr(value, "__slots__"):
        return ros_to_dict(value)
    if isinstance(value, (list, tuple)):
        inner = slot_type.split("[")[0] if "[" in slot_type else slot_type
        return [_value_to_py(v, inner) for v in value]
    if isinstance(value, bytes) and str is not bytes:  # py3 bytes field
        return value.decode("latin-1")
    return value


def dict_to_ros(data, msg):
    """Fill a ROS 1 message instance from a ROS2-style dict (tolerant)."""
    for slot, slot_type in zip(msg.__slots__, msg._slot_types):
        key = slot
        if key not in data:
            continue  # e.g. 'seq' absent in ROS 2 dicts -> keep default
        value = data[key]
        current = getattr(msg, slot)
        if isinstance(current, (genpy.Time, genpy.Duration)):
            sec = value.get("sec", value.get("secs", 0))
            nsec = value.get("nanosec", value.get("nsecs", 0))
            cls = type(current)
            setattr(msg, slot, cls(int(sec), int(nsec)))
        elif hasattr(current, "__slots__"):
            dict_to_ros(value, current)
        elif isinstance(current, (list, tuple)):
            setattr(msg, slot, value)
        else:
            setattr(msg, slot, value)
    return msg


def import_msg_class(type_str):
    """'geometry_msgs/PoseStamped' (or ROS2-style 'geometry_msgs/msg/PoseStamped')
    -> class."""
    parts = [p for p in type_str.split("/") if p and p != "msg"]
    pkg, name = parts[0], parts[1]
    module = importlib.import_module(pkg + ".msg")
    return getattr(module, name)


def _stamp_dict(stamp):
    return {"sec": int(stamp.secs), "nanosec": int(stamp.nsecs)}


def _stamp_ns(stamp):
    return int(stamp.secs) * 1000000000 + int(stamp.nsecs)


def _binary_value(data):
    """Return a buffer accepted by hashlib on Python 2 and Python 3."""
    if isinstance(data, STRING_TYPES):
        return data
    if hasattr(data, "tobytes"):
        return data.tobytes()
    if hasattr(data, "tostring"):
        return data.tostring()
    return bytearray(data)


def _serialize_ros1(msg):
    buff = io.BytesIO()
    msg.serialize(buff)
    return buff.getvalue()


def _sensor_summary(topic, msg):
    if topic.endswith("/camera_info"):
        roi = msg.roi
        return {
            "kind": "camera_info",
            "height": int(msg.height), "width": int(msg.width),
            "distortion_model": msg.distortion_model,
            "d": list(msg.D), "k": list(msg.K),
            "r": list(msg.R), "p": list(msg.P),
            "binning_x": int(msg.binning_x),
            "binning_y": int(msg.binning_y),
            "roi": {
                "x_offset": int(roi.x_offset),
                "y_offset": int(roi.y_offset),
                "height": int(roi.height), "width": int(roi.width),
                "do_rectify": bool(roi.do_rectify),
            },
        }
    if topic.endswith("/compressed"):
        binary = _binary_value(msg.data)
        return {
            "kind": "compressed_image", "format": msg.format,
            "data_bytes": len(binary),
            "data_sha256": hashlib.sha256(binary).hexdigest(),
        }
    if topic.endswith("gps_time_300hz"):
        return {
            "kind": "time_reference",
            "time_ref": _stamp_dict(msg.time_ref), "source": msg.source,
        }
    if topic.endswith("/points"):
        binary = _binary_value(msg.data)
        return {
            "kind": "point_cloud2",
            "height": int(msg.height), "width": int(msg.width),
            "fields": [
                {"name": f.name, "offset": int(f.offset),
                 "datatype": int(f.datatype), "count": int(f.count)}
                for f in msg.fields
            ],
            "is_bigendian": bool(msg.is_bigendian),
            "point_step": int(msg.point_step),
            "row_step": int(msg.row_step),
            "is_dense": bool(msg.is_dense),
            "data_bytes": len(binary),
            "data_sha256": hashlib.sha256(binary).hexdigest(),
        }
    raise ValueError("no ROS1 sensor adapter for topic %s" % topic)


def sensor_transport_record(msg, topic, msg_type, seq, bag_id):
    raw = _serialize_ros1(msg)
    stamp = msg.header.stamp
    return {
        "schema": "ucla.sensor_transport.v1",
        "_transport": {
            "seq": "ros1bag-%06d" % seq,
            "bag": bag_id,
            # rosbag callback exposes the message header stamp, not the bag
            # record timestamp.  Preserve it explicitly and use the same value
            # as the transport time axis.
            "bag_time_ns": _stamp_ns(stamp),
            "source_stamp_ns": _stamp_ns(stamp),
            "source_topic": topic,
            "source_type": msg_type,
            "serialized_bytes": len(raw),
            "source_sha256": hashlib.sha256(raw).hexdigest(),
            "adaptation": "ros1-metadata+sha256",
            "replay_sent_unix_ns": int(time.time() * 1000000000),
        },
        "header": {"stamp": _stamp_dict(stamp),
                   "frame_id": msg.header.frame_id},
        "sensor": _sensor_summary(topic, msg),
    }


def _header_record(header):
    return {
        "seq": int(header.seq),
        "stamp": _stamp_dict(header.stamp),
        "frame_id": header.frame_id,
    }


def _vector3_record(value):
    return {"x": value.x, "y": value.y, "z": value.z}


def _quaternion_record(value):
    return {"x": value.x, "y": value.y, "z": value.z, "w": value.w}


def _pose_record(value):
    return {
        "position": _vector3_record(value.position),
        "orientation": _quaternion_record(value.orientation),
    }


def _twist_record(value):
    return {
        "linear": _vector3_record(value.linear),
        "angular": _vector3_record(value.angular),
    }


def _pointcloud_record(value):
    binary = _binary_value(value.data)
    return {
        "header": _header_record(value.header),
        "height": int(value.height),
        "width": int(value.width),
        "point_count": int(value.height) * int(value.width),
        "fields": [
            {"name": field.name, "offset": int(field.offset),
             "datatype": int(field.datatype), "count": int(field.count)}
            for field in value.fields
        ],
        "is_bigendian": bool(value.is_bigendian),
        "point_step": int(value.point_step),
        "row_step": int(value.row_step),
        "is_dense": bool(value.is_dense),
        "data_bytes": len(binary),
        "data_sha256": hashlib.sha256(binary).hexdigest(),
        "data_omitted": True,
    }


def _image_record(value):
    binary = _binary_value(value.data)
    return {
        "header": _header_record(value.header),
        "height": int(value.height),
        "width": int(value.width),
        "encoding": value.encoding,
        "is_bigendian": int(value.is_bigendian),
        "step": int(value.step),
        "data_bytes": len(binary),
        "data_sha256": hashlib.sha256(binary).hexdigest(),
        "data_omitted": True,
    }


def _detected_object_record(value):
    """Semantic raw detector fields only; no binary/point-cloud metadata."""
    return {
        "header": _header_record(value.header),
        "id": int(value.id),
        "label": value.label,
        "score": value.score,
        "color": {"r": value.color.r, "g": value.color.g,
                  "b": value.color.b, "a": value.color.a},
        "valid": bool(value.valid),
        "space_frame": value.space_frame,
        "pose": _pose_record(value.pose),
        "dimensions": _vector3_record(value.dimensions),
        "variance": _vector3_record(value.variance),
        "velocity": _twist_record(value.velocity),
        "acceleration": _twist_record(value.acceleration),
        "pose_reliable": bool(value.pose_reliable),
        "velocity_reliable": bool(value.velocity_reliable),
        "acceleration_reliable": bool(value.acceleration_reliable),
        "image_frame": value.image_frame,
        "x": int(value.x), "y": int(value.y),
        "width": int(value.width), "height": int(value.height),
        "angle": value.angle,
        "indicator_state": int(value.indicator_state),
        "behavior_state": int(value.behavior_state),
        "user_defined_info": list(value.user_defined_info),
    }


def detection_transport_record(msg, topic, msg_type, seq, bag_id):
    """Convert one raw Autoware detector array to object-state-only JSON."""
    if not hasattr(msg, "objects") or hasattr(msg, "detections"):
        raise TypeError("detection mode requires autoware_msgs/DetectedObjectArray")
    raw = _serialize_ros1(msg)
    stamp = msg.header.stamp
    objects = [_detected_object_record(value) for value in msg.objects]
    header = _header_record(msg.header)
    semantic_json = json.dumps(
        {"header": header, "objects": objects},
        sort_keys=True, separators=(",", ":"))
    return {
        "schema": "ucla.autoware_detection_transport.v3",
        "_transport": {
            "seq": "detection-%06d" % seq,
            "bag": bag_id,
            "bag_time_ns": _stamp_ns(stamp),
            "source_stamp_ns": _stamp_ns(stamp),
            "source_topic": topic,
            "source_type": msg_type,
            "source_message_md5": getattr(msg, "_md5sum", ""),
            "serialized_bytes": len(raw),
            "source_sha256": hashlib.sha256(
                semantic_json.encode("utf-8")).hexdigest(),
            "adaptation": "ros1-autoware-detection-object-state-only",
            "replay_sent_unix_ns": int(time.time() * 1000000000),
        },
        "header": header,
        "object_count": len(objects),
        "objects": objects,
        "payload_policy": {
            "pointcloud": "omitted",
            "roi_image": "omitted",
            "convex_hull": "omitted",
            "candidate_trajectories": "omitted",
        },
    }


def tracking_transport_record(msg, topic, msg_type, seq, bag_id):
    """Convert a real TrackingObjectArray, excluding all PointCloud2 data."""
    required = ("ids", "segments", "sizes", "positions",
                "directions", "velocities")
    missing = [name for name in required if not hasattr(msg, name)]
    if missing:
        raise TypeError(
            "tracking profile requires autosense TrackingObjectArray; "
            "missing: %s" % ", ".join(missing))
    lengths = [len(getattr(msg, name)) for name in required]
    if len(set(lengths)) != 1:
        raise ValueError(
            "TrackingObjectArray parallel arrays differ in length: %s" %
            dict(zip(required, lengths)))

    objects = []
    for index, track_id in enumerate(msg.ids):
        objects.append({
            "id": int(track_id),
            "size": _vector3_record(msg.sizes[index]),
            "position": _vector3_record(msg.positions[index]),
            "direction": _vector3_record(msg.directions[index]),
            "velocity": _vector3_record(msg.velocities[index]),
        })
    header = _header_record(msg.header)
    semantic_json = json.dumps(
        {"header": header, "objects": objects},
        sort_keys=True, separators=(",", ":"))
    return {
        "schema": "ucla.autosense_tracking_transport.v1",
        "_transport": {
            "seq": "tracking2-%06d" % seq,
            "bag": bag_id,
            "bag_time_ns": _stamp_ns(msg.header.stamp),
            "source_stamp_ns": _stamp_ns(msg.header.stamp),
            "source_topic": topic,
            "source_type": msg_type,
            "source_message_md5": getattr(msg, "_md5sum", ""),
            "serialized_bytes": len(_serialize_ros1(msg)),
            "source_sha256": hashlib.sha256(
                semantic_json.encode("utf-8")).hexdigest(),
            "adaptation": "ros1-autosense-tracking-object-state-only",
            "replay_sent_unix_ns": int(time.time() * 1000000000),
        },
        "header": header,
        "object_count": len(objects),
        "objects": objects,
        "payload_policy": {
            "segments": "omitted",
            "pointcloud_topics": "not_subscribed",
            "visualization_topics": "not_subscribed",
        },
    }


def marker_array_transport_attachment(msg, topic, msg_type):
    """Preserve one compact transformed-detection MarkerArray as JSON."""
    if not hasattr(msg, "markers"):
        raise TypeError(
            "marker attachment requires visualization_msgs/MarkerArray")
    message = ros_to_dict(msg)
    semantic_json = json.dumps(
        message, sort_keys=True, separators=(",", ":"))
    stamps = [
        _stamp_ns(marker.header.stamp)
        for marker in msg.markers
        if hasattr(marker, "header")
    ]
    return {
        "schema": "ucla.visualization_marker_array.v1",
        "source_topic": topic,
        "source_type": msg_type,
        "source_message_md5": getattr(msg, "_md5sum", ""),
        "source_stamp_ns": max(stamps) if stamps else 0,
        "serialized_bytes": len(_serialize_ros1(msg)),
        "source_sha256": hashlib.sha256(
            semantic_json.encode("utf-8")).hexdigest(),
        "message": message,
    }


def _fill_point(message, values):
    message.x = float(values.get("x", 0.0))
    message.y = float(values.get("y", 0.0))
    message.z = float(values.get("z", 0.0))
    return message


def marker_array_attachment_to_ros(attachment):
    """Reconstruct an attached MarkerArray, including point/color lists."""
    if (not isinstance(attachment, dict)
            or attachment.get("schema")
            != "ucla.visualization_marker_array.v1"):
        return None
    message = attachment.get("message")
    if not isinstance(message, dict):
        return None

    array_class = import_msg_class("visualization_msgs/MarkerArray")
    marker_class = import_msg_class("visualization_msgs/Marker")
    point_class = import_msg_class("geometry_msgs/Point")
    color_class = import_msg_class("std_msgs/ColorRGBA")
    array = array_class()
    for marker_data in message.get("markers", []):
        if not isinstance(marker_data, dict):
            continue
        marker = marker_class()
        scalar_data = dict(marker_data)
        point_values = scalar_data.pop("points", [])
        color_values = scalar_data.pop("colors", [])
        dict_to_ros(scalar_data, marker)
        marker.points = [
            dict_to_ros(value, point_class())
            for value in point_values
            if isinstance(value, dict)
        ]
        marker.colors = [
            dict_to_ros(value, color_class())
            for value in color_values
            if isinstance(value, dict)
        ]
        array.markers.append(marker)
    return array


def marker_array_reset_message(message):
    """Create DELETEALL before a state array to prevent stale RViz boxes."""
    array_class = import_msg_class("visualization_msgs/MarkerArray")
    marker_class = import_msg_class("visualization_msgs/Marker")
    reset_array = array_class()
    reset = marker_class()
    # visualization_msgs/Marker.DELETEALL is 3 in ROS1 Melodic.
    reset.action = int(getattr(marker_class, "DELETEALL", 3))
    if message is not None and getattr(message, "markers", None):
        reset.header = message.markers[0].header
    reset.ns = "etx_transformed_detection_reset"
    reset_array.markers.append(reset)
    return reset_array


def object_transport_record_to_ros(record, profile):
    """Reconstruct the semantic JSON as a typed ROS1 object-array message.

    Point-cloud fields intentionally stay empty.  Tracking arrays receive one
    empty PointCloud2 placeholder per object so all parallel arrays retain the
    same length expected by autosense consumers.
    """
    if profile == "autosense-tracking":
        if record.get("schema") != "ucla.autosense_tracking_transport.v1":
            return None
        array = import_msg_class("autosense_msgs/TrackingObjectArray")()
        point_class = import_msg_class("geometry_msgs/Point")
        cloud_class = import_msg_class("sensor_msgs/PointCloud2")
        dict_to_ros(record.get("header", {}), array.header)
        for value in record.get("objects", []):
            array.ids.append(int(value.get("id", 0)))
            array.segments.append(cloud_class())
            array.sizes.append(
                _fill_point(point_class(), value.get("size", {})))
            array.positions.append(
                _fill_point(point_class(), value.get("position", {})))
            array.directions.append(
                _fill_point(point_class(), value.get("direction", {})))
            array.velocities.append(
                _fill_point(point_class(), value.get("velocity", {})))
        return array

    if record.get("schema") != "ucla.autoware_detection_transport.v3":
        return None
    array = import_msg_class("autoware_msgs/DetectedObjectArray")()
    object_class = import_msg_class("autoware_msgs/DetectedObject")
    dict_to_ros(record.get("header", {}), array.header)
    for value in record.get("objects", []):
        item = object_class()
        dict_to_ros(value, item)
        array.objects.append(item)
    return array


def parse_sensor_rates(overrides):
    rates = dict((topic, values[1])
                 for topic, values in SENSOR_BAG_TOPICS.items())
    for value in overrides:
        try:
            topic, rate_text = value.rsplit("=", 1)
            rate = float(rate_text)
        except ValueError:
            raise ValueError("invalid --sensor-topic-rate %r; use /topic=Hz" % value)
        if topic not in SENSOR_BAG_TOPICS:
            raise ValueError("unknown target-bag topic: %s" % topic)
        if rate < 0:
            raise ValueError("sensor topic rate must be >= 0")
        rates[topic] = rate
    return rates


# ------------------------------------------------------------------- node
class Ros1Converter(object):
    def __init__(self, args):
        self._sensor_bag_mode = args.sensor_bag_mode
        self._detection_mode = args.detection_mode
        self._detection_profile = args.detection_profile
        if self._sensor_bag_mode and self._detection_mode:
            raise ValueError("--sensor-bag-mode and --detection-mode are mutually exclusive")
        self._envelope_mode = self._sensor_bag_mode or self._detection_mode
        selected_type = "std_msgs/String" if self._envelope_mode else args.msg_type
        self._msg_class = import_msg_class(selected_type)
        self._msg_type = selected_type
        self._max_udp_bytes = args.max_udp_bytes
        self._require_transport_envelope = (args.require_transport_envelope
                                            or self._envelope_mode)
        self._bag_id = (args.detection_bag_id if self._detection_mode
                        else args.sensor_bag_id)
        self._sensor_rates = parse_sensor_rates(args.sensor_topic_rate)
        self._sensor_next_due = {}
        self._sensor_seq = 0
        self._sensor_lock = threading.Lock()
        self._detection_rate = args.detection_rate
        self._detection_next_due = None
        self._detection_last_stamp_ns = None
        self._detection_seq = 0
        self._marker_topic = args.marker_topic
        self._marker_msg_type = args.marker_msg_type
        self._marker_attachment = None
        self._marker_source_received = 0
        self._marker_embedded = 0
        self._marker_received_published = 0
        self._marker_lock = threading.Lock()
        self._sender_role = args.sender_role
        self._sender_id = args.sender_id
        self._default_lat = float(args.default_lat)
        self._default_lon = float(args.default_lon)
        self._location_lock = threading.Lock()
        self._latest_lat = self._default_lat
        self._latest_lon = self._default_lon
        self._location_updates = 0
        self._transport_log = None
        self._received_transport_log = None
        if self._envelope_mode:
            for log_path in (args.transport_log, args.received_transport_log):
                parent = os.path.dirname(log_path)
                if parent and not os.path.isdir(parent):
                    os.makedirs(parent)
            self._transport_log = open(args.transport_log, "w")
            self._received_transport_log = open(args.received_transport_log, "w")
        self._relay_addr = (args.relay_host, args.relay_port)
        self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._recv_sock.bind(("127.0.0.1", args.listen_port))
        self._recv_sock.settimeout(0.5)

        self._pub = rospy.Publisher(args.rx_topic, self._msg_class, queue_size=50)
        self._object_pub = None
        if self._detection_mode:
            self._object_pub = rospy.Publisher(
                args.object_rx_topic,
                import_msg_class(args.detection_msg_type),
                queue_size=20)
        self._marker_pub = None
        if self._detection_mode and args.marker_rx_topic:
            self._marker_pub = rospy.Publisher(
                args.marker_rx_topic,
                import_msg_class(args.marker_msg_type),
                queue_size=20)
        self._subscribers = []
        if self._sensor_bag_mode:
            for topic, values in sorted(SENSOR_BAG_TOPICS.items()):
                if self._sensor_rates[topic] <= 0:
                    continue
                msg_type = values[0]
                sub = rospy.Subscriber(
                    topic, import_msg_class(msg_type), self._on_sensor_tx,
                    callback_args=(topic, msg_type), queue_size=1,
                    buff_size=16 * 1024 * 1024)
                self._subscribers.append(sub)
        elif self._detection_mode:
            self._subscribers.append(
                rospy.Subscriber(
                    args.detection_topic,
                    import_msg_class(args.detection_msg_type),
                    self._on_detection_tx,
                    callback_args=(args.detection_topic,
                                   args.detection_msg_type),
                    queue_size=10,
                    buff_size=4 * 1024 * 1024))
            if args.marker_topic:
                self._subscribers.append(
                    rospy.Subscriber(
                        args.marker_topic,
                        import_msg_class(args.marker_msg_type),
                        self._on_marker_tx,
                        callback_args=(args.marker_topic,
                                       args.marker_msg_type),
                        queue_size=10,
                        buff_size=4 * 1024 * 1024))
            if args.location_topic:
                self._subscribers.append(
                    rospy.Subscriber(
                        args.location_topic,
                        import_msg_class(args.location_msg_type),
                        self._on_location,
                        queue_size=5))
        else:
            self._subscribers.append(
                rospy.Subscriber(args.tx_topic, self._msg_class,
                                 self._on_ros_tx, queue_size=50))
        self._sent = 0
        self._recv = 0
        self._running = True
        thread = threading.Thread(target=self._recv_loop)
        thread.daemon = True
        thread.start()
        if self._sensor_bag_mode:
            rospy.loginfo(
                "ROS1 sensor-bag converter up: %d topics -> relay %s:%d; "
                "relay -> :%d -> %s; source log=%s; received log=%s",
                len(self._subscribers), args.relay_host, args.relay_port,
                args.listen_port, args.rx_topic, args.transport_log,
                args.received_transport_log)
        elif self._detection_mode:
            rospy.loginfo(
                "ROS1 detection-only converter up: %s (%s) at <= %.3f Hz; "
                "relay %s:%d; source log=%s; received log=%s",
                args.detection_topic, args.detection_msg_type,
                args.detection_rate, args.relay_host, args.relay_port,
                args.transport_log, args.received_transport_log)
            if args.marker_topic:
                rospy.loginfo(
                    "Marker attachment source: %s (%s); latest complete "
                    "MarkerArray is embedded in the tracking JSON",
                    args.marker_topic, args.marker_msg_type)
            if args.marker_rx_topic:
                rospy.loginfo(
                    "Received MarkerArray publish topic: %s (%s)",
                    args.marker_rx_topic, args.marker_msg_type)
            if args.location_topic:
                rospy.loginfo(
                    "Verizon routing location: waiting for valid %s on %s; "
                    "fallback=(%.9f, %.9f)",
                    args.location_msg_type, args.location_topic,
                    self._default_lat, self._default_lon)
            else:
                rospy.loginfo(
                    "Verizon routing location: fixed configured coordinate "
                    "(%.9f, %.9f)", self._default_lat, self._default_lon)
        else:
            rospy.loginfo("converter up: %s -> relay %s:%d ; relay -> :%d -> %s",
                          args.tx_topic, args.relay_host, args.relay_port,
                          args.listen_port, args.rx_topic)

    def _on_location(self, msg):
        """Track the current vehicle latitude/longitude without modifying ROS."""
        try:
            lat = float(getattr(msg, "latitude"))
            lon = float(getattr(msg, "longitude"))
            if math.isnan(lat) or math.isnan(lon):
                raise ValueError("latitude/longitude is NaN")
            if math.isinf(lat) or math.isinf(lon):
                raise ValueError("latitude/longitude is infinite")
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                raise ValueError("latitude/longitude outside valid range")
            # Many GNSS receivers emit (0, 0) before they have a valid fix.
            # Treat it as unavailable so it cannot move Verizon routing to the
            # Gulf of Guinea and silently hide the vehicle from the RSU.
            if abs(lat) < 1.0e-9 and abs(lon) < 1.0e-9:
                raise ValueError("latitude/longitude is the no-fix (0, 0) value")
        except Exception as exc:
            rospy.logwarn_throttle(10.0, "invalid GPSFix location: %s", exc)
            return
        with self._location_lock:
            first_update = self._location_updates == 0
            self._latest_lat = lat
            self._latest_lon = lon
            self._location_updates += 1
        if first_update:
            rospy.loginfo(
                "Verizon routing switched from configured fallback to live "
                "GPSFix: (%.9f, %.9f)", lat, lon)

    def _add_sender_metadata(self, record):
        transport = record.setdefault("_transport", {})
        transport["sender_id"] = self._sender_id
        transport["sender_role"] = self._sender_role
        transport["seq"] = "%s-%08d" % (
            self._sender_role, self._detection_seq - 1)
        transport["transport_sent_unix_ns"] = int(
            time.time() * 1000000000)
        with self._location_lock:
            transport["sender_location"] = {
                "lat": self._latest_lat,
                "lon": self._latest_lon,
                "source": ("ros-gpsfix" if self._location_updates
                           else "configured-fallback"),
            }

    def _on_ros_tx(self, msg):
        try:
            payload_obj = ros_to_dict(msg)
            if self._require_transport_envelope:
                self._validate_transport_string(payload_obj, "ROS->JSON")
        except Exception as exc:  # keep converting
            rospy.logwarn("ros->json failed: %s", exc)
            return
        self._send_payload_obj(payload_obj)

    def _on_sensor_tx(self, msg, callback_args):
        topic, msg_type = callback_args
        stamp_ns = _stamp_ns(msg.header.stamp)
        interval_ns = int(1000000000 / self._sensor_rates[topic])
        with self._sensor_lock:
            due = self._sensor_next_due.get(topic)
            if due is not None and stamp_ns < due:
                return
            self._sensor_next_due[topic] = stamp_ns + interval_ns
            seq = self._sensor_seq
            self._sensor_seq += 1
        try:
            record = sensor_transport_record(
                msg, topic, msg_type, seq, self._bag_id)
            canonical = json.dumps(record, sort_keys=True,
                                   separators=(",", ":"))
            payload_obj = {"data": canonical}
            self._validate_transport_string(payload_obj, "ROS1-bag->JSON")
        except Exception as exc:
            rospy.logwarn("sensor adaptation failed for %s: %s", topic, exc)
            return
        with self._sensor_lock:
            self._transport_log.write(canonical + "\n")
            self._transport_log.flush()
        self._send_payload_obj(payload_obj)

    def _on_detection_tx(self, msg, callback_args):
        topic, msg_type = callback_args
        stamp_ns = _stamp_ns(msg.header.stamp)
        interval_ns = (
            int(1000000000 / self._detection_rate)
            if self._detection_rate > 0 else None)
        with self._sensor_lock:
            # A looping rosbag jumps back to its first header timestamp.
            # Reset the source-time gate so every loop remains transmissible.
            if (self._detection_last_stamp_ns is not None
                    and stamp_ns < self._detection_last_stamp_ns):
                self._detection_next_due = None
            if (self._detection_next_due is not None
                    and interval_ns is not None
                    and stamp_ns < self._detection_next_due):
                self._detection_last_stamp_ns = stamp_ns
                return
            self._detection_next_due = (
                stamp_ns + interval_ns if interval_ns is not None else None)
            self._detection_last_stamp_ns = stamp_ns
            seq = self._detection_seq
            self._detection_seq += 1
        try:
            if self._detection_profile == "autosense-tracking":
                record = tracking_transport_record(
                    msg, topic, msg_type, seq, self._bag_id)
            else:
                record = detection_transport_record(
                    msg, topic, msg_type, seq, self._bag_id)
            if self._detection_profile == "autosense-tracking":
                with self._marker_lock:
                    marker_attachment = self._marker_attachment
                if marker_attachment is not None:
                    record["transformed_det_box_score_label"] = (
                        marker_attachment)
                    self._marker_embedded += 1
            self._add_sender_metadata(record)
            canonical = json.dumps(record, sort_keys=True,
                                   separators=(",", ":"))
            payload_obj = {"data": canonical}
            self._validate_transport_string(payload_obj,
                                            "ROS1-detection->JSON")
        except Exception as exc:
            rospy.logwarn("detection adaptation failed for %s: %s", topic, exc)
            return
        with self._sensor_lock:
            self._transport_log.write(canonical + "\n")
            self._transport_log.flush()
        self._send_payload_obj(payload_obj)

    def _on_marker_tx(self, msg, callback_args):
        topic, msg_type = callback_args
        try:
            attachment = marker_array_transport_attachment(
                msg, topic, msg_type)
        except Exception as exc:
            rospy.logwarn(
                "MarkerArray adaptation failed for %s: %s", topic, exc)
            return
        with self._marker_lock:
            # Atomic reference replacement gives the tracking callback one
            # coherent MarkerArray without creating a second ETX stream.
            self._marker_attachment = attachment
            self._marker_source_received += 1
        if self._marker_source_received == 1:
            rospy.loginfo(
                "first transformed detection MarkerArray cached from %s",
                topic)

    def _send_payload_obj(self, payload_obj):
        payload = json.dumps(payload_obj, separators=(",", ":")).encode("utf-8")
        if len(payload) > self._max_udp_bytes:
            rospy.logwarn_throttle(
                5.0,
                "dropping oversized JSON payload (%d bytes; limit=%d). "
                "Reduce the detection array size/rate; binary point-cloud and "
                "image buffers are summary-only in detection mode.",
                len(payload), self._max_udp_bytes)
            return
        self._send_sock.sendto(payload, self._relay_addr)
        self._sent += 1
        if self._sent % 50 == 0:
            rospy.loginfo("sent %d to relay", self._sent)

    def _recv_loop(self):
        while self._running and not rospy.is_shutdown():
            try:
                data, _ = self._recv_sock.recvfrom(65535)
            except socket.timeout:
                continue
            except socket.error:
                break
            try:
                payload_obj = json.loads(data.decode("utf-8"))
                if self._require_transport_envelope:
                    self._validate_transport_string(payload_obj, "JSON->ROS")
                msg = dict_to_ros(payload_obj, self._msg_class())
            except Exception as exc:
                rospy.logwarn("json->ros failed: %s", exc)
                continue
            self._pub.publish(msg)
            if self._object_pub is not None:
                try:
                    inner = json.loads(payload_obj["data"])
                    object_msg = object_transport_record_to_ros(
                        inner, self._detection_profile)
                    if object_msg is not None:
                        self._object_pub.publish(object_msg)
                except Exception as exc:
                    rospy.logwarn("object JSON->ROS1 reconstruction failed: %s",
                                  exc)
            if self._marker_pub is not None:
                try:
                    inner = json.loads(payload_obj["data"])
                    marker_msg = marker_array_attachment_to_ros(
                        inner.get("transformed_det_box_score_label"))
                    if marker_msg is not None:
                        # The source has separate DELETEALL/ADD updates. Since
                        # this bridge transports the latest state together with
                        # each tracking frame, reset first so fewer objects do
                        # not leave stale boxes in RViz.
                        self._marker_pub.publish(
                            marker_array_reset_message(marker_msg))
                        self._marker_pub.publish(marker_msg)
                        self._marker_received_published += 1
                except Exception as exc:
                    rospy.logwarn(
                        "MarkerArray JSON->ROS1 reconstruction failed: %s",
                        exc)
            self._recv += 1
            if self._received_transport_log is not None:
                try:
                    inner = json.loads(payload_obj["data"])
                    seq = inner.get("_transport", {}).get("seq")
                    evidence = {"t": time.time(), "seq": seq,
                                "payload": payload_obj}
                    with self._sensor_lock:
                        self._received_transport_log.write(
                            json.dumps(evidence, sort_keys=True,
                                       separators=(",", ":")) + "\n")
                        self._received_transport_log.flush()
                except Exception as exc:
                    rospy.logwarn("received evidence log failed: %s", exc)
            if self._recv % 50 == 0:
                rospy.loginfo("received %d from relay", self._recv)

    def _validate_transport_string(self, payload_obj, direction):
        """Validate a transport envelope carried by std_msgs/String.

        Keeping the ROS wire type as std_msgs/String lets Melodic and Jazzy use
        their native message packages while the inner JSON retains the source
        topic/type and bag timestamp.  Validation here catches accidentally
        starting this mode with PoseStamped or arbitrary text.
        """
        if "std_msgs" not in self._msg_type or not self._msg_type.endswith("String"):
            raise ValueError("--require-transport-envelope requires std_msgs/String")
        if (not isinstance(payload_obj, dict)
                or not isinstance(payload_obj.get("data"), STRING_TYPES)):
            raise ValueError("%s expected std_msgs/String JSON object" % direction)
        inner = json.loads(payload_obj["data"])
        transport = inner.get("_transport") if isinstance(inner, dict) else None
        required = ("seq", "source_topic", "source_type", "bag_time_ns",
                    "source_sha256")
        missing = [key for key in required
                   if not isinstance(transport, dict) or key not in transport]
        if missing:
            raise ValueError("%s transport envelope missing: %s" %
                             (direction, ", ".join(missing)))

    def shutdown(self):
        self._running = False
        if self._transport_log is not None:
            try:
                self._transport_log.close()
            except IOError:
                pass
        if self._received_transport_log is not None:
            try:
                self._received_transport_log.close()
            except IOError:
                pass
        for sock in (self._send_sock, self._recv_sock):
            try:
                sock.close()
            except socket.error:
                pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tx-topic", default="/etx_tx",
                        help="ROS 1 topic whose messages go to the cloud")
    parser.add_argument("--rx-topic", default="/etx_rx",
                        help="ROS 1 topic where cloud messages are published")
    parser.add_argument(
        "--object-rx-topic", default="/etx_object_rx",
        help="typed reconstructed object array published in detection mode")
    parser.add_argument("--msg-type", default="geometry_msgs/PoseStamped")
    parser.add_argument("--relay-host", default="127.0.0.1")
    parser.add_argument("--relay-port", type=int, default=51010)
    parser.add_argument("--listen-port", type=int, default=51011)
    parser.add_argument(
        "--max-udp-bytes", type=int, default=60000,
        help="drop JSON larger than this safe localhost UDP payload size")
    parser.add_argument(
        "--require-transport-envelope", action="store_true",
        help="validate a transport JSON envelope in std_msgs/String.data")
    parser.add_argument(
        "--sensor-bag-mode", action="store_true",
        help="subscribe to all six target ROS1 sensor bag topics and emit compact String envelopes")
    parser.add_argument(
        "--detection-mode", action="store_true",
        help="subscribe to one structured object topic and emit compact JSON")
    parser.add_argument(
        "--detection-profile",
        choices=("autosense-tracking", "autoware-raw"),
        default="autosense-tracking",
        help="real tracking2 output (default) or raw detector output")
    parser.add_argument(
        "--detection-topic", default="",
        help="the only ROS1 source topic subscribed in --detection-mode")
    parser.add_argument(
        "--detection-msg-type", default="")
    parser.add_argument(
        "--marker-topic", default="",
        help="optional MarkerArray source cached into each tracking envelope")
    parser.add_argument(
        "--marker-msg-type", default="visualization_msgs/MarkerArray",
        help="ROS1 type used by --marker-topic and --marker-rx-topic")
    parser.add_argument(
        "--marker-rx-topic", default="",
        help="optional ROS1 topic for reconstructed attached MarkerArray")
    parser.add_argument(
        "--detection-rate", type=float, default=10.0,
        help="maximum array transmit rate in Hz; 0 sends every source frame")
    parser.add_argument(
        "--sender-role", choices=("vehicle", "infrastructure"),
        default="infrastructure",
        help="role identifier used for loop prevention and unique sequences")
    parser.add_argument(
        "--sender-id", default="",
        help="stable physical sender ID written to _transport.sender_id")
    parser.add_argument(
        "--default-lat", type=float, default=34.06990275541846,
        help="fallback sender latitude when no live GPSFix is available")
    parser.add_argument(
        "--default-lon", type=float, default=-118.44384647757106,
        help="fallback sender longitude when no live GPSFix is available")
    parser.add_argument(
        "--location-topic", default="",
        help="optional gps_common/GPSFix topic used for dynamic routing")
    parser.add_argument(
        "--location-msg-type", default="gps_common/GPSFix",
        help="ROS1 message type for --location-topic")
    parser.add_argument(
        "--detection-bag-id", default="2026-07-22-16-14-46_tracking2",
        help="detection source identifier written into each envelope")
    parser.add_argument(
        "--sensor-bag-id",
        default="2023-03-23-15-39-28_3_ucla_infrastructure_lidar_nw",
        help="source bag identifier written into each envelope")
    parser.add_argument(
        "--sensor-topic-rate", action="append", default=[], metavar="/TOPIC=HZ",
        help="override target-bag sampling rate; repeat for multiple topics; 0 disables")
    parser.add_argument(
        "--transport-log", default="logs/sensor_source.jsonl",
        help="source-side expected JSONL written in --sensor-bag-mode")
    parser.add_argument(
        "--received-transport-log", default="logs/sensor_received.jsonl",
        help="cloud->ROS1 receive evidence JSONL written in --sensor-bag-mode")
    args, _ = parser.parse_known_args(rospy.myargv()[1:])
    if args.detection_rate < 0:
        parser.error("--detection-rate must be >= 0")
    args.sender_id = args.sender_id.strip()
    if args.detection_mode and not args.sender_id:
        parser.error("--sender-id is required in --detection-mode")
    if any(ch not in
           "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789._-"
           for ch in args.sender_id):
        parser.error(
            "--sender-id may contain only letters, digits, dot, underscore, "
            "and hyphen")
    if args.detection_profile == "autosense-tracking":
        args.detection_topic = (
            args.detection_topic or "/tracking2/tracking_objects")
        args.detection_msg_type = (
            args.detection_msg_type or
            "autosense_msgs/TrackingObjectArray")
    else:
        args.detection_topic = (
            args.detection_topic or
            "/detection/lidar_detector2/objects")
        args.detection_msg_type = (
            args.detection_msg_type or
            "autoware_msgs/DetectedObjectArray")

    rospy.init_node("etx_ros1_converter", anonymous=False)
    node = Ros1Converter(args)
    try:
        rospy.spin()
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()
