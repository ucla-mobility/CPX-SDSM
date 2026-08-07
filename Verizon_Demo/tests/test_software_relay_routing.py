import argparse
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "app"))
sys.path.insert(0, str(ROOT / "vendor/python-etx-samples/src"))
os.environ["ETX_FORCE_CODEC_LITE"] = "1"

import etx_software_relay as relay_module


class FakeEndpoint:
    def __init__(self, *_args):
        self.session_id = "software-session"
        self.published = []

    def require_identity(self, _expected):
        return None

    def identity_summary(self):
        return "Software/Application/UclaEvalPlatform"

    @staticmethod
    def session_from_topic(topic):
        return topic.rsplit("/", 1)[-1]

    def private_targeted_topic(self, session, msg_type):
        return "private/%s/%s" % (session, msg_type)

    def publish_json(self, topic, payload, lat, lon):
        self.published.append((topic, payload, lat, lon))
        return True

    def disconnect(self):
        return None


def payload(sender, sequence, schema="ucla.autosense_tracking_transport.v1"):
    inner = {
        "schema": schema,
        "_transport": {
            "sender_id": sender,
            "sender_role": "infrastructure",
            "seq": sequence,
            "source_stamp_ns": 123,
            "sender_location": {"lat": 34.0, "lon": -118.0},
        },
    }
    return {"data": json.dumps(inner)}


class SoftwareRelayRoutingTest(unittest.TestCase):
    def test_learns_sessions_queues_then_cross_routes(self):
        original = relay_module.EtxEndpoint
        relay_module.EtxEndpoint = FakeEndpoint
        try:
            with tempfile.TemporaryDirectory() as directory:
                args = argparse.Namespace(
                    device_file="unused",
                    lat=34.0,
                    lon=-118.0,
                    expected_identity="Software/Application/UclaEvalPlatform",
                    infrastructure_sender_id="infra",
                    vehicle_sender_id="vehicle",
                    log_dir=directory,
                    dedup_limit=100,
                    dedup_ttl=30.0,
                    pending_limit=10,
                    pending_ttl=30.0,
                    flush_rate=10.0,
                    publish_type="TIM",
                )
                relay = relay_module.SoftwareRelay(args)
                relay._on_bsm("regional/x/infra-session", payload("infra", "i-1"))
                self.assertEqual(1, len(relay.pending["vehicle"]))
                relay._on_bsm(
                    "regional/x/vehicle-session",
                    payload("vehicle", "hello", "ucla.etx.control.v1"),
                )
                relay._flush_once()
                self.assertEqual(1, len(relay.endpoint.published))
                topic, forwarded, _lat, _lon = relay.endpoint.published[0]
                self.assertEqual("private/vehicle-session/TIM", topic)
                value = json.loads(forwarded["data"])["_transport"]
                self.assertEqual("vehicle", value["target_sender_id"])
                self.assertEqual("infra", value["original_sender_id"])
                self.assertEqual(1, value["relay_hop_count"])
                relay.close()
        finally:
            relay_module.EtxEndpoint = original


if __name__ == "__main__":
    unittest.main()
