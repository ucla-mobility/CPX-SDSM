#!/usr/bin/env python3
"""Multi-seed aggregation for the 2x2 combo. For each config and seed computes
the true distance-binned PDR (0..600 m), then reports mean +/- std across seeds
for: per-bin PDR (band chart), near-field PDR, usable range (PDR>=0.1), max range.

Writes:
  results/combo_seeds_curve.csv   (config, dist_mid, pdr_mean, pdr_std)
  results/combo_seeds_summary.csv (config, metric means/stds, n_seeds)
"""
from pathlib import Path
import numpy as np
import pandas as pd

RD = Path(__file__).resolve().parent.parent / "simulations" / "results"
CONFIGS = ["combo_per_p20", "combo_hyb_p20", "combo_per_p30", "combo_hyb_p30"]
SEEDS = [0, 1, 2, 3]
BIN_EDGES = np.arange(0, 625, 25, dtype=float)
NB = len(BIN_EDGES) - 1
MID = np.array([(BIN_EDGES[i] + BIN_EDGES[i + 1]) / 2 for i in range(NB)])


def bin_index(d):
    idx = np.digitize(d, BIN_EDGES, right=False) - 1
    idx[idx >= NB] = -1
    return idx


def pdr_for(prefix, seed):
    rx_f = RD / f"{prefix}-r{seed}-rx.csv"
    ts_f = RD / f"{prefix}-r{seed}-timeseries.csv"
    if not rx_f.exists() or not ts_f.exists():
        return None
    rx = pd.read_csv(rx_f, usecols=["distance_to_sender"])
    dd = rx["distance_to_sender"].to_numpy(float)
    bi = bin_index(dd)
    num = np.bincount(bi[bi >= 0], minlength=NB).astype(float)
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
    pdr = np.where(denom > 0, num / np.maximum(denom, 1), np.nan)
    return pdr, float(dd.max())


def main():
    curve_rows, summ_rows = [], []
    print(f"{'config':>16} {'seeds':>6} {'nearPDR':>16} {'range@0.1 m':>16} {'max_rx m':>16}")
    for cfg in CONFIGS:
        per_bin, nears, r10s, maxds = [], [], [], []
        for s in SEEDS:
            r = pdr_for(cfg, s)
            if r is None:
                continue
            pdr, maxd = r
            per_bin.append(pdr)
            nears.append(pdr[0])
            valid = MID[np.nan_to_num(pdr) >= 0.10]
            r10s.append(valid.max() if len(valid) else 0.0)
            maxds.append(maxd)
        if not per_bin:
            print(f"{cfg:>16}   (none)")
            continue
        arr = np.vstack(per_bin)
        mean = np.nanmean(arr, axis=0)
        std = np.nanstd(arr, axis=0)
        for i in range(NB):
            if not np.isnan(mean[i]):
                curve_rows.append({"config": cfg, "dist_mid": MID[i],
                                   "pdr_mean": round(float(mean[i]), 6),
                                   "pdr_std": round(float(np.nan_to_num(std[i])), 6)})
        n = len(per_bin)
        nm, ns = np.mean(nears), np.std(nears)
        rm, rs = np.mean(r10s), np.std(r10s)
        dm, ds = np.mean(maxds), np.std(maxds)
        summ_rows.append({"config": cfg, "n_seeds": n,
                          "near_mean": round(nm, 4), "near_std": round(ns, 4),
                          "range10_mean": round(rm, 1), "range10_std": round(rs, 1),
                          "maxrx_mean": round(dm, 1), "maxrx_std": round(ds, 1)})
        print(f"{cfg:>16} {n:>6} {nm:8.3f}+/-{ns:<5.3f} {rm:8.0f}+/-{rs:<5.0f} {dm:8.1f}+/-{ds:<5.1f}")
    pd.DataFrame(curve_rows).to_csv(RD / "combo_seeds_curve.csv", index=False)
    pd.DataFrame(summ_rows).to_csv(RD / "combo_seeds_summary.csv", index=False)
    print("wrote combo_seeds_curve.csv and combo_seeds_summary.csv")


if __name__ == "__main__":
    main()
