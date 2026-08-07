#!/usr/bin/env python3
"""Calculate bidirectional PDR and clock-dependent transport latency.

Copy the two run directories to one machine, then provide source.jsonl from
the sender and received.jsonl from the opposite receiver for each direction.
"""

import argparse
import json
import math
import statistics
from pathlib import Path


def inner_from_payload(payload):
    if not isinstance(payload, dict):
        return None
    data = payload.get("data")
    if not isinstance(data, str):
        return None
    try:
        value = json.loads(data)
    except ValueError:
        return None
    return value if isinstance(value, dict) else None


def read_source(path):
    rows = {}
    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            row = json.loads(line)
            transport = row.get("_transport", {})
            seq = transport.get("seq")
            if seq is not None:
                rows[str(seq)] = row
    return rows


def read_received(path):
    rows = {}
    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            evidence = json.loads(line)
            inner = inner_from_payload(evidence.get("payload"))
            if not inner:
                continue
            seq = inner.get("_transport", {}).get("seq")
            if seq is not None and str(seq) not in rows:
                rows[str(seq)] = (evidence, inner)
    return rows


def percentile(values, p):
    if not values:
        return None
    values = sorted(values)
    index = (len(values) - 1) * p
    low = int(math.floor(index))
    high = int(math.ceil(index))
    if low == high:
        return values[low]
    return values[low] * (high - index) + values[high] * (index - low)


def analyze(name, source_path, received_path):
    source = read_source(source_path)
    received = read_received(received_path)
    matched = sorted(set(source).intersection(received))
    latencies = []
    for seq in matched:
        sent_ns = source[seq].get("_transport", {}).get(
            "transport_sent_unix_ns")
        recv_s = received[seq][0].get("t")
        if sent_ns is not None and recv_s is not None:
            latencies.append(float(recv_s) * 1000.0 - float(sent_ns) / 1e6)
    result = {
        "name": name,
        "sent": len(source),
        "received_unique": len(received),
        "matched": len(matched),
        "pdr": (float(len(matched)) / len(source) if source else None),
        "latency_clock_dependent_ms": {
            "count": len(latencies),
            "mean": statistics.mean(latencies) if latencies else None,
            "p50": percentile(latencies, 0.50),
            "p95": percentile(latencies, 0.95),
            "minimum": min(latencies) if latencies else None,
            "maximum": max(latencies) if latencies else None,
        },
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle-source", required=True)
    parser.add_argument("--infra-received", required=True)
    parser.add_argument("--infra-source", required=True)
    parser.add_argument("--vehicle-received", required=True)
    parser.add_argument("--json-out")
    args = parser.parse_args()
    results = [
        analyze("vehicle_to_infrastructure",
                args.vehicle_source, args.infra_received),
        analyze("infrastructure_to_vehicle",
                args.infra_source, args.vehicle_received),
    ]
    for row in results:
        print("=" * 72)
        print(row["name"])
        print("sent=%d matched=%d received_unique=%d PDR=%s" % (
            row["sent"], row["matched"], row["received_unique"],
            ("n/a" if row["pdr"] is None else "%.4f" % row["pdr"])))
        latency = row["latency_clock_dependent_ms"]
        if latency["count"]:
            print("latency_ms (requires synchronized clocks): "
                  "mean=%.1f p50=%.1f p95=%.1f min=%.1f max=%.1f" % (
                      latency["mean"], latency["p50"], latency["p95"],
                      latency["minimum"], latency["maximum"]))
        else:
            print("latency_ms=n/a")
    if args.json_out:
        Path(args.json_out).write_text(
            json.dumps(results, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
