#!/usr/bin/env python3
# mypy: ignore-errors
"""
Distance-binned PDR, split into "raw" (physically received) vs "trusted"
(received AND the sender was admitted by sdsm_trust_perception's TrustEngine
at that moment).

IMPORTANT SCOPE NOTE: the trust layer judges messages AFTER the radio has
already delivered them (it runs on rosBridgeMode="log" output, post-hoc).
It cannot change whether a packet was received -- raw PDR is identical with
or without it. What it changes is how much of that received traffic a judge
vehicle would actually act on. So "trusted PDR" is always <= raw PDR at
every distance bin; the gap between the two curves IS the trust layer's
filtering effect, visualized against distance instead of collapsed into one
aggregate rate.

Denominator (opportunities) is the same physically-grounded one
compute_pdr_true.py uses: per rounded second, pairwise sender/receiver
geometry weighted by the sender's actual tx_count_since_last that second.
Both numerators (raw received, trusted received) are counted against that
SAME denominator, so "trusted PDR" isn't just raw-PDR-times-a-constant --
it reflects where in space (near vs far) the trust layer's rejections
actually land.

JOINING an rx.csv reception to a trust-verdicts.csv row: both are bucketed
into the SAME 0.5s flush windows analysis/replay_trust_verdicts.py (and
trust_node.py) use -- floor(sim_time / flush_interval). A reception at
bucket B, from `sender` to `receiver`, is "trusted" iff
trust-verdicts.csv has a (judge_node=receiver, sender_node=sender) row
whose own bucket (floor(verdict.sim_time / flush_interval)) equals B and
trusted=True. A reception with no matching verdict row at all (e.g. the
very first bucket, before any frame closed) counts as untrusted -- nothing
to point to that would justify a judge acting on it yet.

Usage:
  python analysis/compute_trusted_pdr.py results/per_n10/seed0 --prefix per_n10-r0
  python analysis/compute_trusted_pdr.py simulations/results --prefix per_n10-r0
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

BIN_EDGES = np.arange(0, 325, 25, dtype=float)
NB = len(BIN_EDGES) - 1
LABELS = [f"{int(BIN_EDGES[i])}-{int(BIN_EDGES[i+1])}" for i in range(NB)]
FLUSH_INTERVAL_S = 0.5


def bin_index(d: np.ndarray) -> np.ndarray:
    idx = np.digitize(d, BIN_EDGES, right=False) - 1
    idx[idx >= NB] = -1
    return idx


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help="Directory containing <prefix>-rx.csv, -timeseries.csv, -trust-verdicts.csv")
    ap.add_argument("--prefix", required=True, help="e.g. per_n10-r0")
    ap.add_argument("--flush-interval", type=float, default=FLUSH_INTERVAL_S)
    ap.add_argument("--out", help="Output CSV (default: <prefix>-trusted-pdr.csv next to the input)")
    args = ap.parse_args()

    rd = Path(args.path)
    prefix = args.prefix
    rx_f = rd / f"{prefix}-rx.csv"
    ts_f = rd / f"{prefix}-timeseries.csv"
    tv_f = rd / f"{prefix}-trust-verdicts.csv"
    for f in (rx_f, ts_f, tv_f):
        if not f.exists():
            print(f"ERROR: missing {f}", file=sys.stderr)
            return 1

    rx = pd.read_csv(rx_f)
    ts = pd.read_csv(ts_f, usecols=["time", "vehicle_id", "position_x", "position_y", "tx_count_since_last"])
    tv = pd.read_csv(tv_f, usecols=["judge_node", "sender_node", "sim_time", "trusted"])

    # --- Build the trust lookup: (receiver, sender, bucket) -> trusted bool ---
    tv = tv.copy()
    tv["bucket"] = (tv["sim_time"] / args.flush_interval).astype(np.int64)
    # If a sender/receiver/bucket somehow has >1 row, trust ANY True (shouldn't
    # happen -- one verdict per (judge, sender) per flush -- but don't silently
    # drop data if it does).
    trust_lookup = (
        tv.groupby(["judge_node", "sender_node", "bucket"])["trusted"]
        .any()
        .to_dict()
    )

    # --- Numerators: raw received, and trusted-received, per distance bin ---
    rx = rx.copy()
    rx["bucket"] = (rx["time"] / args.flush_interval).astype(np.int64)
    rx["bin_idx"] = bin_index(rx["distance_to_sender"].to_numpy(float))

    def is_trusted(row) -> bool:
        return bool(trust_lookup.get((int(row["receiver"]), int(row["sender"]), int(row["bucket"])), False))

    rx["trusted"] = rx.apply(is_trusted, axis=1)

    raw_numerator = np.zeros(NB, dtype=np.int64)
    trusted_numerator = np.zeros(NB, dtype=np.int64)
    for _, row in rx.iterrows():
        b = int(row["bin_idx"])
        if b < 0:
            continue
        raw_numerator[b] += 1
        if row["trusted"]:
            trusted_numerator[b] += 1

    # --- Denominator: opportunity-weighted geometry (same method as compute_pdr_true.py) ---
    ts = ts.copy()
    ts["sec"] = ts["time"].round().astype(int)
    denom = np.zeros(NB, dtype=np.float64)
    for _, snap in ts.groupby("sec"):
        if len(snap) < 2:
            continue
        P = snap[["position_x", "position_y"]].to_numpy(float)
        w = snap["tx_count_since_last"].to_numpy(float)
        diff = P[:, None, :] - P[None, :, :]
        D = np.sqrt((diff * diff).sum(axis=2))
        np.fill_diagonal(D, np.inf)
        bidx = bin_index(D.ravel()).reshape(D.shape)
        for b in range(NB):
            neigh_in_bin = (bidx == b).sum(axis=1)
            denom[b] += float(np.dot(neigh_in_bin, w))

    rows = []
    for i in range(NB):
        d = denom[i]
        raw_pdr = round(raw_numerator[i] / d, 6) if d > 0 else -1.0
        trusted_pdr = round(trusted_numerator[i] / d, 6) if d > 0 else -1.0
        rows.append({
            "distance_bin": LABELS[i],
            "opportunities": int(round(d)),
            "received_raw": int(raw_numerator[i]),
            "received_trusted": int(trusted_numerator[i]),
            "pdr_raw": raw_pdr,
            "pdr_trusted": trusted_pdr,
        })
    out_df = pd.DataFrame(rows)
    out_path = Path(args.out) if args.out else rd / f"{prefix}-trusted-pdr.csv"
    out_df.to_csv(out_path, index=False)
    print(f"-> {out_path}")
    for r in rows:
        if r["pdr_raw"] >= 0:
            print(f"  {r['distance_bin']:>8s} m: raw={r['pdr_raw']:.4f}  trusted={r['pdr_trusted']:.4f}  "
                  f"(rx={r['received_raw']}, trusted_rx={r['received_trusted']}, opp={r['opportunities']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
