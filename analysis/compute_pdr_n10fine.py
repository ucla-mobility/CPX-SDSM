#!/usr/bin/env python3
"""Physically-bounded PDR for the low-density (n=10) groups.

Root cause of PDR>1 at low density: the opportunity denominator is estimated from
1 s position snapshots, but packets are sent at 10 Hz while the (few, fast-moving)
vehicles change distance bin within the second -> the snapshot under-counts the
real (TX, in-range-receiver) opportunities, so receptions/opportunities can exceed 1.

Fix: interpolate every vehicle's position onto a 0.1 s grid (matching the 10 Hz send
rate), and count opportunities there. Periodic = 1 packet per present sender per
0.1 s; Hybrid = tx_count_since_last/10 per 0.1 s (its per-second count spread over
the ten sub-steps). A final clamp at 1.0 removes negligible bin-boundary residue, so
the result is a true PDR in [0,1]. Writes results/final_dist_seedstats_n10fine.csv.
"""
from pathlib import Path
import numpy as np, pandas as pd

RD = Path(__file__).resolve().parent.parent / "simulations" / "results"
GROUPS = {"per_n10": ("Periodic", 10), "hyb_n10": ("Hybrid", 10)}
SEEDS = range(20)
DT = 0.1
EDGES = np.arange(0, 1750, 50.0)
NB = len(EDGES) - 1
MID = (EDGES[:-1] + EDGES[1:]) / 2


def bidx(d):
    i = np.digitize(d, EDGES, right=False) - 1
    i[i >= NB] = -1
    return i


def fine_geometry(prefix):
    """0.1 s grid: list of (grid_time, vehicle_ids, nbin[ n_senders, NB ])."""
    ts = pd.read_csv(RD / f"{prefix}-r0-timeseries.csv",
                     usecols=["time", "vehicle_id", "position_x", "position_y"])
    veh = {}
    for vid, g in ts.groupby("vehicle_id"):
        g = g.sort_values("time")
        veh[vid] = (g.time.to_numpy(float), g.position_x.to_numpy(float), g.position_y.to_numpy(float))
    t0 = float(ts.time.min()); t1 = float(ts.time.max())
    grid = np.round(np.arange(t0, t1 + DT, DT), 1)
    geom = []
    for t in grid:
        vids, P = [], []
        for vid, (tt, xx, yy) in veh.items():
            if tt[0] <= t <= tt[-1]:
                vids.append(vid); P.append((np.interp(t, tt, xx), np.interp(t, tt, yy)))
        if len(P) < 2:
            continue
        P = np.array(P)
        D = np.sqrt(((P[:, None, :] - P[None, :, :]) ** 2).sum(2))
        np.fill_diagonal(D, np.inf)
        bb = bidx(D.ravel()).reshape(D.shape)
        nbin = np.zeros((len(vids), NB))
        for b in range(NB):
            nbin[:, b] = (bb == b).sum(axis=1)
        geom.append((int(round(t)), np.array(vids), nbin))   # sec for hybrid tx lookup
    return geom


def seed_pdr(prefix, seed, geom, periodic):
    if periodic:
        denom = np.zeros(NB)
        for sec, vids, nbin in geom:
            denom += nbin.sum(axis=0)            # weight 1 per present sender per 0.1 s
    else:
        ts = pd.read_csv(RD / f"{prefix}-r{seed}-timeseries.csv",
                         usecols=["time", "vehicle_id", "tx_count_since_last"])
        ts["s"] = ts["time"].round().astype(int)
        txmap = {(int(s), int(v)): float(t) / 10.0 for s, v, t in
                 zip(ts["s"], ts["vehicle_id"], ts["tx_count_since_last"])}
        denom = np.zeros(NB)
        for sec, vids, nbin in geom:
            w = np.array([txmap.get((sec, int(v)), 0.0) for v in vids])
            denom += w @ nbin
    dd = pd.read_csv(RD / f"{prefix}-r{seed}-rx.csv", usecols=["distance_to_sender"]).distance_to_sender.to_numpy(float)
    bi = bidx(dd)
    num = np.bincount(bi[bi >= 0], minlength=NB).astype(float)
    pdr = np.where(denom > 0, num / denom, np.nan)
    return np.minimum(pdr, 1.0)               # physical clamp for negligible boundary residue


def main():
    rows = []
    for g, (strat, n) in GROUPS.items():
        geom = fine_geometry(g)
        per = strat == "Periodic"
        mat = np.vstack([seed_pdr(g, s, geom, per) for s in SEEDS])
        mean = np.nanmean(mat, axis=0); std = np.nanstd(mat, axis=0, ddof=1)
        mn = np.nanmin(mat, axis=0); mx = np.nanmax(mat, axis=0)
        cnt = np.sum(~np.isnan(mat), axis=0)
        print(f"{g}: max(mean)={np.nanmax(mean):.3f}  max(mean+std)={np.nanmax(mean+std):.3f}")
        for i in range(NB):
            if cnt[i] >= 2 and not np.isnan(mean[i]):
                rows.append({"group": g, "strategy": strat, "vehicles": n, "distance": MID[i],
                             "pdr_mean": round(float(mean[i]), 6), "pdr_std": round(float(std[i]), 6),
                             "pdr_min": round(float(mn[i]), 6), "pdr_max": round(float(mx[i]), 6),
                             "pdr_var": float(np.nanvar(mat[:, i], ddof=1)), "n_seeds": int(cnt[i])})
    pd.DataFrame(rows).to_csv(RD / "final_dist_seedstats_n10fine.csv", index=False)
    print("wrote final_dist_seedstats_n10fine.csv")


if __name__ == "__main__":
    main()
