#!/usr/bin/env python3
"""Offline round-trip test for the attached transformed-detection MarkerArray."""

from __future__ import annotations

import importlib.util
import io
import sys
import types
from pathlib import Path


class Time:
    def __init__(self, secs=0, nsecs=0):
        self.secs = secs
        self.nsecs = nsecs


class Message:
    _md5sum = ""


class Header(Message):
    __slots__ = ("seq", "stamp", "frame_id")
    _slot_types = ("uint32", "time", "string")

    def __init__(self):
        self.seq = 0
        self.stamp = Time()
        self.frame_id = ""


class Point(Message):
    __slots__ = ("x", "y", "z")
    _slot_types = ("float64", "float64", "float64")

    def __init__(self):
        self.x = self.y = self.z = 0.0


class Quaternion(Message):
    __slots__ = ("x", "y", "z", "w")
    _slot_types = ("float64", "float64", "float64", "float64")

    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.w = 1.0


class Pose(Message):
    __slots__ = ("position", "orientation")
    _slot_types = ("geometry_msgs/Point", "geometry_msgs/Quaternion")

    def __init__(self):
        self.position = Point()
        self.orientation = Quaternion()


class Vector3(Point):
    pass


class ColorRGBA(Message):
    __slots__ = ("r", "g", "b", "a")
    _slot_types = ("float32", "float32", "float32", "float32")

    def __init__(self):
        self.r = self.g = self.b = self.a = 0.0


class Duration(Time):
    pass


class Marker(Message):
    DELETEALL = 3
    __slots__ = (
        "header", "ns", "id", "type", "action", "pose", "scale", "color",
        "lifetime", "frame_locked", "points", "colors", "text",
        "mesh_resource", "mesh_use_embedded_materials",
    )
    _slot_types = (
        "std_msgs/Header", "string", "int32", "int32", "int32",
        "geometry_msgs/Pose", "geometry_msgs/Vector3", "std_msgs/ColorRGBA",
        "duration", "bool", "geometry_msgs/Point[]", "std_msgs/ColorRGBA[]",
        "string", "string", "bool",
    )

    def __init__(self):
        self.header = Header()
        self.ns = ""
        self.id = self.type = self.action = 0
        self.pose = Pose()
        self.scale = Vector3()
        self.color = ColorRGBA()
        self.lifetime = Duration()
        self.frame_locked = False
        self.points = []
        self.colors = []
        self.text = ""
        self.mesh_resource = ""
        self.mesh_use_embedded_materials = False


class MarkerArray(Message):
    _md5sum = "d155b9ce5188fbaf89745847fd5882d7"
    __slots__ = ("markers",)
    _slot_types = ("visualization_msgs/Marker[]",)

    def __init__(self):
        self.markers = []

    def serialize(self, stream: io.BytesIO):
        stream.write(b"offline-marker-array")


genpy = types.ModuleType("genpy")
genpy.Time = Time
genpy.Duration = Duration
sys.modules["genpy"] = genpy

rospy = types.ModuleType("rospy")
sys.modules["rospy"] = rospy

visualization_msgs = types.ModuleType("visualization_msgs")
visualization_msgs_msg = types.ModuleType("visualization_msgs.msg")
visualization_msgs_msg.Marker = Marker
visualization_msgs_msg.MarkerArray = MarkerArray
visualization_msgs.msg = visualization_msgs_msg
sys.modules["visualization_msgs"] = visualization_msgs
sys.modules["visualization_msgs.msg"] = visualization_msgs_msg

geometry_msgs = types.ModuleType("geometry_msgs")
geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
geometry_msgs_msg.Point = Point
geometry_msgs_msg.Quaternion = Quaternion
geometry_msgs_msg.Pose = Pose
geometry_msgs_msg.Vector3 = Vector3
geometry_msgs.msg = geometry_msgs_msg
sys.modules["geometry_msgs"] = geometry_msgs
sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

std_msgs = types.ModuleType("std_msgs")
std_msgs_msg = types.ModuleType("std_msgs.msg")
std_msgs_msg.Header = Header
std_msgs_msg.ColorRGBA = ColorRGBA
std_msgs.msg = std_msgs_msg
sys.modules["std_msgs"] = std_msgs
sys.modules["std_msgs.msg"] = std_msgs_msg

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "ros1_converter_node", ROOT / "app" / "ros1_converter_node.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

source = MarkerArray()
marker = Marker()
marker.header.seq = 17
marker.header.stamp = Time(123, 456)
marker.header.frame_id = "map"
marker.ns = "os_sensor/car"
marker.id = 9
marker.type = 1
marker.action = 0
marker.pose.position.x = 42.25
marker.pose.position.y = -146.1
marker.scale.x = 4.6
marker.scale.y = 1.8
marker.scale.z = 1.7
marker.color.g = 1.0
marker.color.a = 0.5
marker.text = "0.912119"
point = Point()
point.x, point.y, point.z = 1.0, 2.0, 3.0
marker.points.append(point)
point_color = ColorRGBA()
point_color.r, point_color.a = 1.0, 1.0
marker.colors.append(point_color)
source.markers.append(marker)

attachment = module.marker_array_transport_attachment(
    source,
    "/tracking2/transformed_det_box_score_label",
    "visualization_msgs/MarkerArray",
)
assert attachment["source_message_md5"] == MarkerArray._md5sum
assert attachment["source_stamp_ns"] == 123000000456
assert attachment["message"]["markers"][0]["text"] == "0.912119"
assert attachment["message"]["markers"][0]["header"]["frame_id"] == "map"
assert "seq" not in attachment["message"]["markers"][0]["header"]

restored = module.marker_array_attachment_to_ros(attachment)
assert len(restored.markers) == 1
assert restored.markers[0].ns == "os_sensor/car"
assert restored.markers[0].pose.position.x == 42.25
assert restored.markers[0].scale.y == 1.8
assert restored.markers[0].text == "0.912119"
assert restored.markers[0].points[0].z == 3.0
assert restored.markers[0].colors[0].r == 1.0

reset = module.marker_array_reset_message(restored)
assert len(reset.markers) == 1
assert reset.markers[0].action == Marker.DELETEALL
assert reset.markers[0].header.frame_id == "map"

print("MARKER_TRANSPORT_TEST_OK")
