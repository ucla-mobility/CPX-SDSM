#!/usr/bin/env python3
"""Per-distance mean AND variance of PDR across the 20 seeds, for the 6 final groups.

Positions are identical across seeds (only fading/MAC RNG differs), so the per-second
neighbour geometry is computed ONCE per group; each seed only re-weights it by that
seed's tx_count_since_last (correct for Hybrid's variable rate). Numerator = receptions
per distance bin from each seed's rx.csv.

Output: results/final_dist_seedstats.csv (group, strategy, vehicles, distance,
pdr_mean, pdr_var, n_seeds).  Bins 0..1700 m (50 m).
"""
from pathlib import Path
import numpy as np, pandas as pd

RD = Path(__file__).resolve().parent.parent / "simulations" / "results"
GROUPS = {"per_n10": ("Periodic", 10), "hyb_n10": ("Hybrid", 10),
          "per_n150": ("Periodic", 150), "hyb_n150": ("Hybrid", 150),
          "per_n400": ("Periodic", 400), "hyb_n400": ("Hybrid", 400)}
SEEDS = range(20)
BIN = 50.0
EDGES = np.arange(0, 1750, BIN)
NB = len(EDGES) - 1
MID = (EDGES[:-1] + EDGES[1:]) / 2


def bidx(d):
    i = np.digitize(d, EDGES, right=False) - 1
    i[i >= NB] = -1
    return i


def geometry(prefix):
    """Per rounded-second: (vehicle_ids, nbin matrix [n_senders, NB] = neighbour count per bin)."""
    f = RD / f"{prefix}-r0-timeseries.csv"
    ts = pd.read_csv(f, usecols=["time", "vehicle_id", "position_x", "position_y"])
    ts["s"] = ts["time"].round().astype(int)
    geom = {}
    for sec, snap in ts.groupby("s"):
        vids = snap["vehicle_id"].to_numpy()
        P = snap[["position_x", "position_y"]].to_numpy(float)
        if len(P) < 2:
            continue
        D = np.sqrt(((P[:, None, :] - P[None, :, :]) ** 2).sum(2))
        np.fill_diagonal(D, np.inf)
        bi = bidx(D.ravel()).reshape(D.shape)
        nbin = np.zeros((len(vids), NB))
        for b in range(NB):
            nbin[:, b] = (bi == b).sum(axis=1)
        geom[sec] = (vids, nbin)
    return geom


def seed_pdr(prefix, seed, geom):
    ts = pd.read_csv(RD / f"{prefix}-r{seed}-timeseries.csv",
                     usecols=["time", "vehicle_id", "tx_count_since_last"])
    ts["s"] = ts["time"].round().astype(int)
    txmap = {(int(s), int(v)): float(t) for s, v, t in
             zip(ts["s"], ts["vehicle_id"], ts["tx_count_since_last"])}
    denom = np.zeros(NB)
    for sec, (vids, nbin) in geom.items():
        tx = np.array([txmap.get((sec, int(v)), 0.0) for v in vids])
        denom += tx @ nbin
    dd = pd.read_csv(RD / f"{prefix}-r{seed}-rx.csv", usecols=["distance_to_sender"]).distance_to_sender.to_numpy(float)
    bi = bidx(dd)
    num = np.bincount(bi[bi >= 0], minlength=NB).astype(float)
    return np.where(denom > 0, num / denom, np.nan)


def main():
    rows = []
    for g, (strat, n) in GROUPS.items():
        if not (RD / f"{g}-r0-timeseries.csv").exists():
            print(f"{g}: (no data)"); continue
        geom = geometry(g)
        mat = []
        for s in SEEDS:
            if (RD / f"{g}-r{s}-rx.csv").exists():
                mat.append(seed_pdr(g, s, geom))
        mat = np.vstack(mat)            # [seeds, bins]
        mean = np.nanmean(mat, axis=0)
        var = np.nanvar(mat, axis=0, ddof=1)
        std = np.nanstd(mat, axis=0, ddof=1)
        mn = np.nanmin(mat, axis=0)
        mx = np.nanmax(mat, axis=0)
        cnt = np.sum(~np.isnan(mat), axis=0)
        print(f"{g}: {mat.shape[0]} seeds, {int((cnt>0).sum())} bins")
        for i in range(NB):
            if cnt[i] >= 2 and not np.isnan(mean[i]):
                rows.append({"group": g, "strategy": strat, "vehicles": n, "distance": MID[i],
                             "pdr_mean": round(float(mean[i]), 6), "pdr_var": float(var[i]),
                             "pdr_std": round(float(std[i]), 6), "pdr_min": round(float(mn[i]), 6),
                             "pdr_max": round(float(mx[i]), 6), "n_seeds": int(cnt[i])})
    pd.DataFrame(rows).to_csv(RD / "final_dist_seedstats.csv", index=False)
    print("wrote final_dist_seedstats.csv")


if __name__ == "__main__":
    main()
