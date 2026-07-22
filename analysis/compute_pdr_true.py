#!/usr/bin/env python3
"""
TRUE distance-binned PDR (corrected denominator).

PDR(bin) = received_in_bin / TX_opportunities_in_bin, where a TX opportunity is
(one transmitted packet) x (one vehicle that was within the distance bin of the
sender at that time).

Fixes two defects in analysis/compute_pdr.py:
  1. Timeseries timestamps are per-vehicle-offset, so grouping by EXACT time
     fragments each snapshot (~27 of 225 vehicles). We round time to the nearest
     second so a snapshot holds all co-present vehicles.
  2. The denominator must be weighted by how many packets each sender actually
     transmitted in that second (tx_count_since_last), not counted once per pair.

Numerator unchanged: received packets per bin from *-rx.csv (distance_to_sender).

Output: *-pdr-true.csv  with columns distance_bin, received, opportunities, pdr.
Usage:  python analysis/compute_pdr_true.py results/Periodic-r0
        python analysis/compute_pdr_true.py results        # all prefixes
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

BIN_EDGES = np.arange(0, 325, 25, dtype=float)
NB = len(BIN_EDGES) - 1
LABELS = [f"{int(BIN_EDGES[i])}-{int(BIN_EDGES[i+1])}" for i in range(NB)]


def bin_index(d: np.ndarray) -> np.ndarray:
    idx = np.digitize(d, BIN_EDGES, right=False) - 1
    idx[idx >= NB] = -1
    return idx


def compute(prefix: str, rd: Path) -> pd.DataFrame:
    rx_f = rd / f"{prefix}-rx.csv"
    ts_f = rd / f"{prefix}-timeseries.csv"
    if not rx_f.exists() or not ts_f.exists():
        print(f"  SKIP {prefix}: missing rx/timeseries")
        return pd.DataFrame()

    rx = pd.read_csv(rx_f, usecols=["distance_to_sender"])
    ni = bin_index(rx["distance_to_sender"].to_numpy(float))
    numerator = np.bincount(ni[ni >= 0], minlength=NB).astype(np.int64)

    ts = pd.read_csv(ts_f, usecols=["time", "position_x", "position_y", "tx_count_since_last"])
    ts["sec"] = ts["time"].round().astype(int)

    denom = np.zeros(NB, dtype=np.float64)
    for _, snap in ts.groupby("sec"):
        if len(snap) < 2:
            continue
        P = snap[["position_x", "position_y"]].to_numpy(float)
        w = snap["tx_count_since_last"].to_numpy(float)        # packets sent by each sender this second
        diff = P[:, None, :] - P[None, :, :]
        D = np.sqrt((diff * diff).sum(axis=2))
        np.fill_diagonal(D, np.inf)
        bidx = bin_index(D.ravel()).reshape(D.shape)           # (sender i, receiver j) -> bin
        for b in range(NB):
            neigh_in_bin = (bidx == b).sum(axis=1)             # per-sender count of receivers in bin b
            denom[b] += float(np.dot(neigh_in_bin, w))         # weight by sender's TX count

    rows = []
    for i in range(NB):
        n = int(numerator[i])
        d = denom[i]
        pdr = round(n / d, 6) if d > 0 else -1.0
        rows.append({"distance_bin": LABELS[i], "received": n,
                     "opportunities": int(round(d)), "pdr": pdr})
    return pd.DataFrame(rows)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    args = ap.parse_args()
    p = Path(args.path)
    if p.is_dir():
        rd = p
        prefixes = sorted({f.name.replace("-rx.csv", "") for f in rd.glob("*-rx.csv")})
    else:
        rd, prefixes = p.parent, [p.name]
    if not prefixes:
        print("no rx csv found", file=sys.stderr)
        return 1
    for prefix in prefixes:
        print(f"TRUE PDR for {prefix} ...")
        df = compute(prefix, rd)
        if df.empty:
            continue
        out = rd / f"{prefix}-pdr-true.csv"
        df.to_csv(out, index=False)
        print(f"  -> {out}")
        for _, r in df.iterrows():
            if r["pdr"] >= 0:
                print(f"  {r['distance_bin']:>8s} m: PDR={r['pdr']:.4f}  (rx={r['received']}, opp={r['opportunities']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
