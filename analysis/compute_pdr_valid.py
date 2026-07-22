#!/usr/bin/env python3
"""True distance-binned PDR (bins 0..1000 m) for the validation experiments.
Reuses the corrected denominator (round to 1 s, weight by tx_count_since_last).
Computes for: valid_n10, valid_a200, and the reused baselines combo_per_p20
(n=400) and sw_p20_b6 (alpha=2.75, 150 veh). Writes results/valid_pdr_long.csv."""
from pathlib import Path
import numpy as np, pandas as pd

RD = Path(__file__).resolve().parent.parent / "simulations" / "results"
PREFIXES = ["combo_per_p20", "valid_n10", "sw_p20_b6", "valid_a200"]
BIN_EDGES = np.arange(0, 1025, 25, dtype=float)
NB = len(BIN_EDGES) - 1
MID = [(BIN_EDGES[i] + BIN_EDGES[i + 1]) / 2 for i in range(NB)]


def bidx(d):
    i = np.digitize(d, BIN_EDGES, right=False) - 1
    i[i >= NB] = -1
    return i


def compute(prefix):
    rx_f = RD / f"{prefix}-r0-rx.csv"
    ts_f = RD / f"{prefix}-r0-timeseries.csv"
    if not rx_f.exists() or not ts_f.exists():
        return None
    dd = pd.read_csv(rx_f, usecols=["distance_to_sender"]).distance_to_sender.to_numpy(float)
    bi = bidx(dd)
    num = np.bincount(bi[bi >= 0], minlength=NB).astype(float)
    ts = pd.read_csv(ts_f, usecols=["time", "position_x", "position_y", "tx_count_since_last"])
    ts["sec"] = ts["time"].round().astype(int)
    denom = np.zeros(NB)
    for _, s in ts.groupby("sec"):
        if len(s) < 2:
            continue
        P = s[["position_x", "position_y"]].to_numpy(float)
        w = s["tx_count_since_last"].to_numpy(float)
        D = np.sqrt(((P[:, None, :] - P[None, :, :]) ** 2).sum(2))
        np.fill_diagonal(D, np.inf)
        bb = bidx(D.ravel()).reshape(D.shape)
        for b in range(NB):
            denom[b] += float(np.dot((bb == b).sum(axis=1), w))
    pdr = np.where(denom > 0, num / np.maximum(denom, 1), np.nan)
    return num, denom, pdr, float(dd.max())


def main():
    rows = []
    print(f"{'prefix':>16} {'nearPDR':>8} {'max_rx_m':>9} {'maxbin>0':>9}")
    for p in PREFIXES:
        r = compute(p)
        if r is None:
            print(f"{p:>16}  (missing)"); continue
        num, denom, pdr, maxd = r
        near = pdr[0] if not np.isnan(pdr[0]) else 0
        lastbin = max([MID[i] for i in range(NB) if denom[i] > 0 and pdr[i] > 0.001], default=0)
        print(f"{p:>16} {near:8.3f} {maxd:9.0f} {lastbin:9.0f}")
        for i in range(NB):
            if denom[i] > 0:
                rows.append({"prefix": p, "dist_mid": MID[i],
                             "pdr": round(float(np.nan_to_num(pdr[i])), 6),
                             "received": int(num[i]), "opportunities": int(round(denom[i]))})
    pd.DataFrame(rows).to_csv(RD / "valid_pdr_long.csv", index=False)
    print("wrote valid_pdr_long.csv")


if __name__ == "__main__":
    main()
