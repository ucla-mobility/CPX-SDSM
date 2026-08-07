#!/usr/bin/env python3
"""Ubuntu ROS1 endpoint for the two-Vehicle-client/Software-relay topology.

Both physical roles use the same Verizon behavior:

  ROS1 JSON -> Vehicle client -> GeoRelevance BSM
  Software relay -> targeted Private TIM -> Vehicle client -> ROS1 JSON

The physical role and stable sender ID remain in the inner transport metadata.
They are used by the Windows Software relay for routing and by the receiver for
target validation. Session IDs are deliberately not configured here because
Verizon assigns a new value on every connection.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import threading
import time
from collections import OrderedDict, deque
from pathlib import Path
from typing import Any

from etx_endpoint import EtxEndpoint


def nested_record(payload: Any) -> dict[str, Any] | None:
    if not isinstance(payload, dict) or not isinstance(payload.get("data"), str):
        return None
    try:
        record = json.loads(payload["data"])
    except ValueError:
        return None
    return record if isinstance(record, dict) else None


def transport(payload: Any) -> dict[str, Any]:
    record = nested_record(payload)
    value = record.get("_transport") if record else None
    return value if isinstance(value, dict) else {}


def payload_location(payload: Any) -> tuple[float | None, float | None]:
    location = transport(payload).get("sender_location")
    if not isinstance(location, dict):
        return None, None
    try:
        lat, lon = float(location["lat"]), float(location["lon"])
    except (KeyError, TypeError, ValueError):
        return None, None
    return (lat, lon) if -90 <= lat <= 90 and -180 <= lon <= 180 else (None, None)


def message_key(payload: Any) -> str | None:
    value = transport(payload)
    sender = value.get("sender_id")
    sequence = value.get("seq")
    source_time = value.get("source_stamp_ns", value.get("bag_time_ns"))
    if sender is None or sequence is None:
        return None
    return "%s|%s|%s" % (sender, sequence, source_time)


class RateLimiter:
    def __init__(self, limit: float) -> None:
        self.limit = float(limit)
        self.stamps: deque[float] = deque()

    def allow(self) -> bool:
        if self.limit <= 0:
            return True
        now = time.monotonic()
        while self.stamps and now - self.stamps[0] >= 1.0:
            self.stamps.popleft()
        if len(self.stamps) >= self.limit:
            return False
        self.stamps.append(now)
        return True


class Deduplicator:
    def __init__(self, max_entries: int = 4000, ttl: float = 180.0) -> None:
        self.max_entries, self.ttl = max_entries, ttl
        self.seen: OrderedDict[str, float] = OrderedDict()
        self.lock = threading.Lock()

    def first(self, key: str | None) -> bool:
        if key is None:
            return True
        now = time.monotonic()
        with self.lock:
            while self.seen:
                _, stamp = next(iter(self.seen.items()))
                if now - stamp <= self.ttl and len(self.seen) <= self.max_entries:
                    break
                self.seen.popitem(last=False)
            if key in self.seen:
                return False
            self.seen[key] = now
            return True


class JsonlWriter:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = path.open("a", encoding="utf-8")
        self.lock = threading.Lock()

    def write(self, value: dict[str, Any]) -> None:
        with self.lock:
            self.stream.write(
                json.dumps(value, separators=(",", ":"), ensure_ascii=False) + "\n"
            )
            self.stream.flush()

    def close(self) -> None:
        with self.lock:
            self.stream.close()


class VehicleEndpointRelay:
    def __init__(self, args: argparse.Namespace) -> None:
        if args.client_mode != "vehicle_endpoint":
            raise ValueError("unsupported client mode: %s" % args.client_mode)
        self.args = args
        self.role = args.role
        self.endpoint = EtxEndpoint(
            args.device_file, args.default_lat, args.default_lon
        )
        self.endpoint.require_identity(args.expected_identity)
        self.rate = RateLimiter(args.max_publish_rate)
        self.dedup = Deduplicator()
        self.running = True

        self.to_ros_address = (args.ros_host, args.ros_port)
        self.to_ros_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.from_ros_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.from_ros_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.from_ros_socket.bind(("127.0.0.1", args.listen_port))
        self.from_ros_socket.settimeout(0.5)

        log_dir = Path(args.log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        self.rx_log = JsonlWriter(log_dir / "cloud_rx.jsonl")
        self.tx_log = JsonlWriter(log_dir / "cloud_tx.jsonl")
        self.status_path = log_dir / "status.json"
        self.stats = {
            "from_ros": 0,
            "bsm_tx": 0,
            "bsm_tx_failed": 0,
            "tim_rx": 0,
            "tim_rx_unique": 0,
            "tim_rx_duplicate": 0,
            "tim_rx_self": 0,
            "tim_rx_wrong_target": 0,
            "tim_rx_control": 0,
            "to_ros": 0,
            "control_bsm_tx": 0,
            "control_bsm_failed": 0,
            "rate_dropped": 0,
            "invalid_udp_json": 0,
        }

    def run(self) -> None:
        print(
            "[relay] role=%s sender=%s connecting as %s"
            % (self.role, self.args.sender_id, self.endpoint.identity_summary()),
            flush=True,
        )
        self.endpoint.connect(self._ready)
        if not self.endpoint.wait_ready(self.args.connect_timeout):
            raise RuntimeError("no Verizon session before timeout")
        threading.Thread(target=self._udp_loop, daemon=True).start()
        threading.Thread(target=self._hello_loop, daemon=True).start()
        last_status = 0.0
        try:
            while self.running:
                now = time.monotonic()
                if now - last_status >= self.args.stats_every:
                    self._write_status()
                    print("[relay] stats=%s" % self.stats, flush=True)
                    last_status = now
                time.sleep(0.25)
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def _ready(self) -> None:
        private_topic = self.endpoint.private_sub_topic(self.args.receive_type)
        private_ok = self.endpoint.subscribe_json(private_topic, self._on_cloud)
        geo_ok = self.endpoint.subscribe_json(
            self.endpoint.georelevance_sub_topic(self.args.receive_type),
            self._on_cloud,
        )
        print(
            "[relay] ENDPOINT_READY role=%s session=%s private=%s georelevance=%s"
            % (self.role, self.endpoint.session_id, private_ok, geo_ok),
            flush=True,
        )

    def _on_cloud(self, topic: str, payload: Any) -> None:
        self.stats["tim_rx"] += 1
        value = transport(payload)
        if value.get("sender_id") == self.args.sender_id:
            self.stats["tim_rx_self"] += 1
            return
        target = value.get("target_sender_id")
        if target and target != self.args.sender_id:
            self.stats["tim_rx_wrong_target"] += 1
            return
        record = nested_record(payload)
        if record and record.get("schema") == "ucla.etx.control.v1":
            self.stats["tim_rx_control"] += 1
            return
        key = message_key(payload)
        if not self.dedup.first(key):
            self.stats["tim_rx_duplicate"] += 1
            return
        self.stats["tim_rx_unique"] += 1
        self.rx_log.write(
            {
                "received_unix_ns": time.time_ns(),
                "topic": topic,
                "message_key": key,
                "payload": payload,
            }
        )
        try:
            encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.to_ros_socket.sendto(encoded, self.to_ros_address)
            self.stats["to_ros"] += 1
        except OSError as error:
            print("[relay] UDP cloud->ROS failed: %s" % error, flush=True)

    def _udp_loop(self) -> None:
        while self.running:
            try:
                data, _ = self.from_ros_socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                payload = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, ValueError):
                self.stats["invalid_udp_json"] += 1
                continue
            self.stats["from_ros"] += 1
            self._publish_bsm(payload, control=False)

    def _hello_loop(self) -> None:
        interval = self.args.control_hello_seconds
        if interval <= 0:
            return
        while self.running:
            self._publish_bsm(self._control_payload(), control=True)
            deadline = time.monotonic() + interval
            while self.running and time.monotonic() < deadline:
                time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))

    def _control_payload(self) -> dict[str, Any]:
        record = {
            "schema": "ucla.etx.control.v1",
            "_transport": {
                "sender_id": self.args.sender_id,
                "sender_role": self.role,
                "seq": "control-%s-%d" % (self.role, time.time_ns()),
                "source_stamp_ns": time.time_ns(),
                "bag_time_ns": 0,
                "source_topic": "__etx_session_hello__",
                "source_type": "control",
                "source_sha256": "control",
                "sender_location": {
                    "lat": self.args.default_lat,
                    "lon": self.args.default_lon,
                    "source": "configured-control",
                },
            },
        }
        return {"data": json.dumps(record, separators=(",", ":"))}

    def _publish_bsm(self, payload: dict[str, Any], control: bool) -> None:
        if not control and not self.rate.allow():
            self.stats["rate_dropped"] += 1
            return
        topic = self.endpoint.georelevance_pub_topic(self.args.publish_type)
        lat, lon = payload_location(payload)
        try:
            ok = self.endpoint.publish_json(topic, payload, lat, lon)
        except Exception as error:
            ok = False
            print("[relay] BSM publish exception: %s" % error, flush=True)
        if not ok:
            self.stats["control_bsm_failed" if control else "bsm_tx_failed"] += 1
            return
        self.stats["control_bsm_tx" if control else "bsm_tx"] += 1
        if not control:
            self.tx_log.write(
                {
                    "sent_unix_ns": time.time_ns(),
                    "topic": topic,
                    "message_key": message_key(payload),
                    "lat": lat,
                    "lon": lon,
                    "payload": payload,
                }
            )

    def _write_status(self) -> None:
        record = {
            "updated_unix": time.time(),
            "role": self.role,
            "sender_id": self.args.sender_id,
            "client_mode": self.args.client_mode,
            "identity": self.endpoint.identity_summary(),
            "session_id": self.endpoint.session_id,
            "stats": self.stats,
        }
        temporary = self.status_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(record, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        os.replace(str(temporary), str(self.status_path))

    def close(self) -> None:
        if not self.running:
            return
        self.running = False
        self._write_status()
        for sock in (self.from_ros_socket, self.to_ros_socket):
            try:
                sock.close()
            except OSError:
                pass
        self.rx_log.close()
        self.tx_log.close()
        self.endpoint.disconnect()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--role", choices=("vehicle", "infrastructure"), required=True
    )
    parser.add_argument("--client-mode", choices=("vehicle_endpoint",), required=True)
    parser.add_argument("--sender-id", required=True)
    parser.add_argument("--device-file", required=True)
    parser.add_argument("--expected-identity", required=True)
    parser.add_argument("--default-lat", type=float, required=True)
    parser.add_argument("--default-lon", type=float, required=True)
    parser.add_argument("--publish-type", required=True)
    parser.add_argument("--receive-type", required=True)
    parser.add_argument("--listen-port", type=int, default=51010)
    parser.add_argument("--ros-host", default="127.0.0.1")
    parser.add_argument("--ros-port", type=int, default=51011)
    parser.add_argument("--max-publish-rate", type=float, default=10.0)
    parser.add_argument("--control-hello-seconds", type=float, default=5.0)
    parser.add_argument("--connect-timeout", type=float, default=45.0)
    parser.add_argument("--stats-every", type=float, default=5.0)
    parser.add_argument("--log-dir", required=True)
    args = parser.parse_args()
    VehicleEndpointRelay(args).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
