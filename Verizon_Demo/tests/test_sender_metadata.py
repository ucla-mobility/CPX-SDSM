#!/usr/bin/env python3
"""Offline test for stable sender metadata in every detection JSON."""

from __future__ import annotations

import importlib.util
import sys
import threading
import types
from pathlib import Path


class DummyTime:
    def __init__(self, secs=0, nsecs=0):
        self.secs = secs
        self.nsecs = nsecs


genpy = types.ModuleType("genpy")
genpy.Time = DummyTime
genpy.Duration = DummyTime
sys.modules["genpy"] = genpy

rospy = types.ModuleType("rospy")
sys.modules["rospy"] = rospy

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "ros1_converter_node", ROOT / "app" / "ros1_converter_node.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def verify(sender_id, role, lat, lon, location_updates, location_source):
    node = object.__new__(module.Ros1Converter)
    node._sender_id = sender_id
    node._sender_role = role
    node._detection_seq = 8
    node._location_lock = threading.Lock()
    node._latest_lat = lat
    node._latest_lon = lon
    node._location_updates = location_updates

    record = {"_transport": {}}
    node._add_sender_metadata(record)
    transport = record["_transport"]
    assert transport["sender_id"] == sender_id
    assert transport["sender_role"] == role
    assert transport["seq"] == "%s-00000007" % role
    assert transport["sender_location"] == {
        "lat": lat,
        "lon": lon,
        "source": location_source,
    }


verify(
    "ucla-infrastructure-nw",
    "infrastructure",
    34.067086,
    -118.445280,
    0,
    "configured-fallback",
)
verify(
    "ucla-vehicle-01",
    "vehicle",
    34.067100,
    -118.445200,
    1,
    "ros-gpsfix",
)
print("SENDER_METADATA_TEST_OK")
