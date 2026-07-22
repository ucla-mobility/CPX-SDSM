#!/usr/bin/env python3
"""
TRUE distance-binned PDR for the power/MCS sweep, with bins extended to 600 m
(higher TX power pushes the reception wall well past 300 m).

Same corrected denominator as compute_pdr_true.py: snapshots rounded to the
nearest second, sender pairs weighted by tx_count_since_last.

For each sw_* prefix prints the PDR curve and three range metrics:
  max_rx_m        absolute farthest successful reception
  range_pdr10_m   farthest bin centre with PDR >= 0.10  (usable link)
  range_pdr50_m   farthest bin centre with PDR >= 0.50  (reliable link)

Writes one tidy CSV: results/sweep_pdr_long.csv (config, dist_mid, pdr, received, opportunities).
"""
import sys
from pathlib import Path
import numpy as np
import pandas as pd

RD = Path(__file__).resolve().parent.parent / "simulations" / "results"
CONFIGS = ["sw_p20_b6", "sw_p23_b6", "sw_p26_b6", "sw_p30_b6", "sw_p33_b6", "sw_p20_b3"]
BIN_EDGES = np.arange(0, 625, 25, dtype=float)
NB = len(BIN_EDGES) - 1
MID = [(BIN_EDGES[i] + BIN_EDGES[i + 1]) / 2 for i in range(NB)]


def bin_index(d):
    idx = np.digitize(d, BIN_EDGES, right=False) - 1
    idx[idx >= NB] = -1
    return idx


def compute(prefix):
    rx_f = RD / f"{prefix}-r0-rx.csv"
    ts_f = RD / f"{prefix}-r0-timeseries.csv"
    if not rx_f.exists() or not ts_f.exists():
        return None
    rx = pd.read_csv(rx_f, usecols=["distance_to_sender"])
    dd = rx["distance_to_sender"].to_numpy(float)
    ni = bin_index(dd)
    num = np.bincount(ni[ni >= 0], minlength=NB).astype(np.int64)
    ts = pd.read_csv(ts_f, usecols=["time", "position_x", "position_y", "tx_count_since_last"])
    ts["sec"] = ts["time"].round().astype(int)
    denom = np.zeros(NB)
    for _, snap in ts.groupby("sec"):
        if len(snap) < 2:
            continue
        P = snap[["position_x", "position_y"]].to_numpy(float)
        w = snap["tx_count_since_last"].to_numpy(float)
        D = np.sqrt(((P[:, None, :] - P[None, :, :]) ** 2).sum(2))
        np.fill_diagonal(D, np.inf)
        bidx = bin_index(D.ravel()).reshape(D.shape)
        for b in range(NB):
            denom[b] += float(np.dot((bidx == b).sum(axis=1), w))
    pdr = np.where(denom > 0, num / np.maximum(denom, 1), -1.0)
    return num, denom, pdr, float(dd.max())


def main():
    rows = []
    print(f"{'config':>10} {'max_rx_m':>9} {'PDR>=0.1':>9} {'PDR>=0.5':>9}")
    for cfg in CONFIGS:
        r = compute(cfg)
        if r is None:
            print(f"{cfg:>10}  (missing)")
            continue
        num, denom, pdr, maxd = r
        r10 = max([MID[i] for i in range(NB) if pdr[i] >= 0.10], default=0)
        r50 = max([MID[i] for i in range(NB) if pdr[i] >= 0.50], default=0)
        print(f"{cfg:>10} {maxd:9.1f} {r10:9.0f} {r50:9.0f}")
        for i in range(NB):
            if denom[i] > 0:
                rows.append({"config": cfg, "dist_mid": MID[i], "pdr": round(float(pdr[i]), 6),
                             "received": int(num[i]), "opportunities": int(round(denom[i]))})
    out = RD / "sweep_pdr_long.csv"
    pd.DataFrame(rows).to_csv(out, index=False)
    print("wrote", out)


if __name__ == "__main__":
    main()
