#!/usr/bin/env python3
"""Windows Software-client relay between two Verizon Vehicle clients.

The relay subscribes to Regional BSM around the test site, learns the current
Verizon Session ID associated with each stable sender ID, and republishes each
data payload as a targeted Private TIM for the opposite endpoint.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import threading
import time
from collections import OrderedDict, deque
from pathlib import Path
from typing import Any

from etx_endpoint import EtxEndpoint
from etx_dual_relay import JsonlWriter, nested_record, payload_location


def payload_transport(payload: Any) -> dict[str, Any]:
    record = nested_record(payload)
    value = record.get("_transport") if record else None
    return value if isinstance(value, dict) else {}


def payload_key(payload: Any) -> str | None:
    value = payload_transport(payload)
    sender = value.get("sender_id")
    sequence = value.get("seq")
    source_time = value.get("source_stamp_ns", value.get("bag_time_ns"))
    if sender is None or sequence is None:
        return None
    return "%s|%s|%s" % (sender, sequence, source_time)


class Deduplicator:
    def __init__(self, max_entries: int, ttl: float) -> None:
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


class SoftwareRelay:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.endpoint = EtxEndpoint(args.device_file, args.lat, args.lon)
        self.endpoint.require_identity(args.expected_identity)
        self.routes = {
            args.infrastructure_sender_id: args.vehicle_sender_id,
            args.vehicle_sender_id: args.infrastructure_sender_id,
        }
        self.sessions: dict[str, str] = {}
        self.session_lock = threading.Lock()
        self.pending: dict[str, deque[tuple[float, dict[str, Any]]]] = {
            sender: deque() for sender in self.routes
        }
        self.dedup = Deduplicator(args.dedup_limit, args.dedup_ttl)
        self.running = True
        log_dir = Path(args.log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        self.rx_log = JsonlWriter(log_dir / "bsm_rx.jsonl")
        self.tx_log = JsonlWriter(log_dir / "tim_tx.jsonl")
        self.status_path = log_dir / "status.json"
        self.stats = {
            "bsm_rx": 0,
            "bsm_data_rx": 0,
            "bsm_control_rx": 0,
            "duplicate": 0,
            "unknown_sender": 0,
            "session_updates": 0,
            "queued": 0,
            "queue_expired": 0,
            "queue_dropped": 0,
            "tim_attempted": 0,
            "tim_published": 0,
            "tim_failed": 0,
            "infra_to_vehicle": 0,
            "vehicle_to_infra": 0,
        }

    def run(self) -> None:
        print(
            "[monitor] connecting as %s" % self.endpoint.identity_summary(),
            flush=True,
        )
        self.endpoint.connect(self._ready)
        if not self.endpoint.wait_ready(self.args.connect_timeout):
            raise RuntimeError("no Verizon session before timeout")
        threading.Thread(target=self._flush_loop, daemon=True).start()
        last_status = 0.0
        try:
            while self.running:
                now = time.monotonic()
                if now - last_status >= self.args.stats_every:
                    self._write_status()
                    print(
                        "[monitor] sessions=%d/2 pending=%d stats=%s"
                        % (
                            len(self.sessions),
                            sum(len(queue) for queue in self.pending.values()),
                            self.stats,
                        ),
                        flush=True,
                    )
                    last_status = now
                time.sleep(0.25)
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def _ready(self) -> None:
        topics = self.endpoint.regional_sub_topics(
            self.args.receive_type, self.args.geohash_precision
        )
        accepted = sum(
            1 for topic in topics if self.endpoint.subscribe_json(topic, self._on_bsm)
        )
        print(
            "[monitor] SOFTWARE_RELAY_READY session=%s regional=%d/%d"
            % (self.endpoint.session_id, accepted, len(topics)),
            flush=True,
        )

    def _on_bsm(self, topic: str, payload: Any) -> None:
        self.stats["bsm_rx"] += 1
        value = payload_transport(payload)
        sender = value.get("sender_id")
        if sender not in self.routes:
            self.stats["unknown_sender"] += 1
            return
        session = self.endpoint.session_from_topic(topic)
        if session:
            with self.session_lock:
                changed = self.sessions.get(sender) != session
                self.sessions[sender] = session
            if changed:
                self.stats["session_updates"] += 1
                print("[monitor] learned session for %s" % sender, flush=True)

        record = nested_record(payload)
        is_control = bool(record and record.get("schema") == "ucla.etx.control.v1")
        if is_control:
            self.stats["bsm_control_rx"] += 1
            return
        key = payload_key(payload)
        if not self.dedup.first(key):
            self.stats["duplicate"] += 1
            return
        self.stats["bsm_data_rx"] += 1
        self.rx_log.write(
            {
                "received_unix_ns": time.time_ns(),
                "topic": topic,
                "sender_id": sender,
                "message_key": key,
                "payload": payload,
            }
        )
        target = self.routes[sender]
        forwarded = self._forward_payload(payload, sender, target)
        with self.session_lock:
            target_session = self.sessions.get(target, "")
        if not target_session:
            self._enqueue(target, forwarded)
            return
        self._publish(target, target_session, sender, forwarded)

    def _forward_payload(
        self, payload: dict[str, Any], sender: str, target: str
    ) -> dict[str, Any]:
        forwarded = copy.deepcopy(payload)
        record = nested_record(forwarded)
        if not record:
            return forwarded
        value = record.setdefault("_transport", {})
        value["target_sender_id"] = target
        value["relay_sender_id"] = "ucla-pc-software-relay"
        value["relay_received_unix_ns"] = time.time_ns()
        value["relay_hop_count"] = int(value.get("relay_hop_count", 0)) + 1
        value["original_sender_id"] = sender
        forwarded["data"] = json.dumps(record, separators=(",", ":"))
        return forwarded

    def _enqueue(self, target: str, payload: dict[str, Any]) -> None:
        queue = self.pending[target]
        if len(queue) >= self.args.pending_limit:
            queue.popleft()
            self.stats["queue_dropped"] += 1
        queue.append((time.monotonic(), payload))
        self.stats["queued"] += 1

    def _flush_loop(self) -> None:
        while self.running:
            time.sleep(1.0 / max(self.args.flush_rate, 0.1))
            self._flush_once()

    def _flush_once(self) -> None:
        now = time.monotonic()
        for target, queue in self.pending.items():
            while queue and now - queue[0][0] > self.args.pending_ttl:
                queue.popleft()
                self.stats["queue_expired"] += 1
            if not queue:
                continue
            with self.session_lock:
                target_session = self.sessions.get(target, "")
            if target_session:
                _, payload = queue.popleft()
                source = payload_transport(payload).get("original_sender_id", "")
                self._publish(target, target_session, source, payload)

    def _publish(
        self,
        target: str,
        target_session: str,
        source: str,
        payload: dict[str, Any],
    ) -> None:
        self.stats["tim_attempted"] += 1
        topic = self.endpoint.private_targeted_topic(
            target_session, self.args.publish_type
        )
        lat, lon = payload_location(payload)
        try:
            ok = self.endpoint.publish_json(topic, payload, lat, lon)
        except Exception as error:
            ok = False
            print("[monitor] TIM publish exception: %s" % error, flush=True)
        if not ok:
            self.stats["tim_failed"] += 1
            return
        self.stats["tim_published"] += 1
        if source == self.args.infrastructure_sender_id:
            self.stats["infra_to_vehicle"] += 1
        elif source == self.args.vehicle_sender_id:
            self.stats["vehicle_to_infra"] += 1
        self.tx_log.write(
            {
                "sent_unix_ns": time.time_ns(),
                "target_sender_id": target,
                "message_key": payload_key(payload),
                "topic": topic,
                "payload": payload,
            }
        )

    def _write_status(self) -> None:
        with self.session_lock:
            session_ready = {
                sender: bool(self.sessions.get(sender)) for sender in self.routes
            }
        record = {
            "updated_unix": time.time(),
            "identity": self.endpoint.identity_summary(),
            "session_id": self.endpoint.session_id,
            "endpoint_session_ready": session_ready,
            "pending_depth": {
                sender: len(queue) for sender, queue in self.pending.items()
            },
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
        self.rx_log.close()
        self.tx_log.close()
        self.endpoint.disconnect()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device-file", required=True)
    parser.add_argument(
        "--expected-identity", default="Software/Application/UclaEvalPlatform"
    )
    parser.add_argument("--lat", type=float, default=34.067086)
    parser.add_argument("--lon", type=float, default=-118.445280)
    parser.add_argument("--infrastructure-sender-id", default="ucla-infrastructure-nw")
    parser.add_argument("--vehicle-sender-id", default="ucla-vehicle-01")
    parser.add_argument("--receive-type", default="BSM")
    parser.add_argument("--publish-type", default="TIM")
    parser.add_argument("--geohash-precision", type=int, default=6)
    parser.add_argument("--pending-limit", type=int, default=500)
    parser.add_argument("--pending-ttl", type=float, default=30.0)
    parser.add_argument("--flush-rate", type=float, default=12.0)
    parser.add_argument("--dedup-limit", type=int, default=10000)
    parser.add_argument("--dedup-ttl", type=float, default=300.0)
    parser.add_argument("--connect-timeout", type=float, default=45.0)
    parser.add_argument("--stats-every", type=float, default=5.0)
    parser.add_argument("--log-dir", required=True)
    args = parser.parse_args()
    SoftwareRelay(args).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
