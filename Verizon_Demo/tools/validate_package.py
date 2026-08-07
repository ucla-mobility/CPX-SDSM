#!/usr/bin/env python3
"""Offline package integrity checks; never prints registration secrets."""

import json
import sys
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EXPECTED = {
    "vehicle": (
        "Vehicle/PassengerCar/UclaEvalDevice",
        ROOT / "clients/vehicle/registration.json",
    ),
    "infrastructure": (
        "Vehicle/PassengerCar/UclaEvalDevice",
        ROOT / "clients/infrastructure_vehicle/registration.json",
    ),
    "monitor": (
        "Software/Application/UclaEvalPlatform",
        ROOT / "clients/infrastructure_gateway/registration.json",
    ),
}
EXPECTED_SENDER_IDS = {
    "vehicle": "ucla-vehicle-01",
    "infrastructure": "ucla-infrastructure-nw",
}


def fail(message):
    print("FAIL: %s" % message, file=sys.stderr)
    raise SystemExit(1)


def read_env(path):
    values = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key] = value.strip().strip("\"'")
    return values


def main():
    required = [
        ROOT / "app/etx_dual_relay.py",
        ROOT / "app/etx_software_relay.py",
        ROOT / "app/ros1_converter_node.py",
        ROOT / "config/vehicle.env",
        ROOT / "config/infrastructure.env",
        ROOT / "scripts/start_node.sh",
        ROOT / "windows/run_monitor.ps1",
        ROOT / "catkin_ws/src/autosense_msgs/msg/TrackingObjectArray.msg",
    ]
    missing = [str(path.relative_to(ROOT)) for path in required
               if not path.is_file()]
    if missing:
        fail("missing files: %s" % ", ".join(missing))

    device_ids = {}
    for role, (expected, path) in EXPECTED.items():
        if not path.is_file():
            fail("missing %s registration: %s" % (
                role, path.relative_to(ROOT)))
        artifact = json.loads(path.read_text(encoding="utf-8"))
        attrs = artifact["frozen_config"]["identity"]["attributes"]
        identity = "%s/%s/%s" % (
            attrs["clientType"], attrs["clientSubType"], attrs["vendorId"])
        if identity != expected:
            fail("%s identity is %s, expected %s" % (
                role, identity, expected))
        expiration = artifact["registration"]["certificate"]["expiration_time"]
        expires = datetime.fromisoformat(expiration)
        if expires <= datetime.now(timezone.utc):
            fail("%s registration expired at %s" % (role, expiration))
        print("%s registration: identity OK; expiry=%s" % (
            role, expiration))
        device_ids[role] = artifact["registration"]["device_id"]
        if role == "monitor":
            continue
        config = read_env(ROOT / ("config/%s.env" % role))
        actual_sender = config.get("SENDER_ID", "")
        if actual_sender != EXPECTED_SENDER_IDS[role]:
            fail("%s SENDER_ID is %s, expected %s" % (
                role, actual_sender, EXPECTED_SENDER_IDS[role]))
        print("%s sender ID: %s" % (role, actual_sender))
        if config.get("CLIENT_MODE") != "vehicle_endpoint":
            fail("%s CLIENT_MODE must be vehicle_endpoint" % role)
        if config.get("PUBLISH_TYPE") != "BSM":
            fail("%s must publish BSM" % role)
        if config.get("RECEIVE_TYPE") != "TIM":
            fail("%s must receive TIM" % role)

    if device_ids["vehicle"] == device_ids["infrastructure"]:
        fail("the two Vehicle endpoints reuse the same Device ID")
    print("vehicle registrations: distinct Device IDs OK")

    message = (
        ROOT / "catkin_ws/src/autosense_msgs/msg/TrackingObjectArray.msg"
    ).read_text(encoding="utf-8")
    for field in ("ids", "segments", "sizes", "positions",
                  "directions", "velocities"):
        if field not in message:
            fail("TrackingObjectArray.msg missing field %s" % field)
    print("message package: required fields present")
    print("PACKAGE_VALIDATION_OK")


if __name__ == "__main__":
    main()
