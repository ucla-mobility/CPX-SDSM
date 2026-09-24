"""
Sanity check for sdsm_codec.local_xy_of against RosSDSMApp.cc's forward
projection -- pure math, no rclpy/sdsm_msgs import needed (a real
SensorDataSharingMessage.ref_pos is a rosidl-generated class we can't easily
construct without a built workspace; this test uses a plain stand-in with
the same .lat/.lon attributes local_xy_of actually reads).
"""

import math
from types import SimpleNamespace

from sdsm_trust_perception.global_trust_perception.pipeline import sdsm_codec as codec


def _forward_project(x_m: float, y_m: float) -> SimpleNamespace:
    """Mirror RosSDSMApp::buildSdsmJson's C++ projection exactly, to build a
    ref_pos that a real sender would have produced for sim-frame (x_m, y_m)."""
    lat_deg = codec.ORIGIN_LAT_DEG + y_m / 111320.0
    lon_deg = codec.ORIGIN_LON_DEG + x_m / (
        111320.0 * math.cos(math.radians(codec.ORIGIN_LAT_DEG))
    )
    return SimpleNamespace(
        lat=round(lat_deg * 1e7), lon=round(lon_deg * 1e7), elevation=-4096,
    )


def test_round_trip_origin():
    ref = _forward_project(0.0, 0.0)
    x, y = codec.local_xy_of(ref)
    assert abs(x) < 0.02
    assert abs(y) < 0.02


def test_round_trip_offset():
    for x_m, y_m in [(100.0, -250.0), (-1500.0, 3000.0), (0.0, 500.0)]:
        ref = _forward_project(x_m, y_m)
        x, y = codec.local_xy_of(ref)
        # J2735 1e-7 deg quantization is the only error source -- bounds it
        # generously (a few cm) rather than asserting exact equality.
        assert abs(x - x_m) < 0.05, f'x: got {x}, want {x_m}'
        assert abs(y - y_m) < 0.05, f'y: got {y}, want {y_m}'


def _msg_with_objects(*objs):
    """Stand-in message: each obj is (speed_raw, heading_raw)."""
    common = [SimpleNamespace(det_obj_common=SimpleNamespace(speed=s, heading=h))
              for s, h in objs]
    return SimpleNamespace(objects=common)


def test_heading_math_to_compass():
    # RosSDSMApp writes atan2(dy, dx) in degrees: 0 = east, 90 = north (math).
    east = round(0.0 / codec.HEADING_UNIT_DEG)
    north = round(90.0 / codec.HEADING_UNIT_DEG)
    msg = _msg_with_objects((50, east), (50, north), (50, codec.HEADING_UNAVAILABLE))
    east_c, north_c, unavailable = codec.get_headings_of(msg)
    assert abs(east_c - 90.0) < 0.05      # compass: east = 90
    assert abs(north_c - 0.0) < 0.05 or abs(north_c - 360.0) < 0.05
    assert unavailable == 360.0


def test_velocity_follows_math_heading():
    speed_raw = round(10.0 / codec.SPEED_UNIT_MS)          # 10 m/s
    north = round(90.0 / codec.HEADING_UNIT_DEG)
    (vx, vy), = codec.get_velocities_of(_msg_with_objects((speed_raw, north)))
    assert abs(vx) < 0.05 and abs(vy - 10.0) < 0.05
