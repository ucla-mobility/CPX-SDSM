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
