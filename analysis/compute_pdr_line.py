#!/usr/bin/env python3
"""Exact per-pair-distance PDR for the static straight-line density experiment.

Vehicles are stationary and equally spaced, so every ordered pair (i->j) has a
fixed distance. PDR(distance) = (packets received over all pairs at that distance)
/ (packets sent by the senders of those pairs). Includes 0-reception pairs (the
denominator comes from tx counts, not from rx), so the curve is unbiased.

Writes results/line_pdr_long.csv (config, spacing, density_vehkm, distance, pdr, npairs).
"""
from pathlib import Path
import numpy as np, pandas as pd

RD = Path(__file__).resolve().parent.parent / "simulations" / "results"
CONFIGS = {"line_s500": 500, "line_s200": 200, "line_s100": 100, "line_s50": 50,
           "line_s25": 25, "line_s16": 16, "line_s10": 10, "line_s8": 8}


def compute(prefix, spacing):
    ts = pd.read_csv(RD / f"{prefix}-r0-timeseries.csv",
                     usecols=["vehicle_id", "position_x", "tx_count_since_last"])
    pos = ts.groupby("vehicle_id")["position_x"].mean()
    tx = ts.groupby("vehicle_id")["tx_count_since_last"].sum()       # total sends per node
    rx = pd.read_csv(RD / f"{prefix}-r0-rx.csv", usecols=["sender", "receiver"])
    rxc = rx.groupby(["sender", "receiver"]).size()                 # received per ordered pair
    ids = list(pos.index)
    num = {}; den = {}; npairs = {}
    for i in ids:
        for j in ids:
            if i == j:
                continue
            d = int(round(abs(pos[i] - pos[j]) / spacing)) * spacing  # snap to multiple of spacing
            if d == 0:
                continue
            num[d] = num.get(d, 0) + int(rxc.get((i, j), 0))
            den[d] = den.get(d, 0) + int(tx.get(i, 0))
            npairs[d] = npairs.get(d, 0) + 1
    rows = []
    for d in sorted(num):
        pdr = num[d] / den[d] if den[d] else 0.0
        rows.append({"config": prefix, "spacing": spacing,
                     "density_vehkm": (2000 // spacing + 1) / 2.0,
                     "distance": d, "pdr": round(pdr, 6), "npairs": npairs[d]})
    return rows


def main():
    allrows = []
    print(f"{'config':>10} {'dens(v/km)':>10} {'nearPDR':>8} {'maxdist':>8}")
    for cfg, s in CONFIGS.items():
        if not (RD / f"{cfg}-r0-rx.csv").exists():
            print(f"{cfg:>10}  (missing)"); continue
        rows = compute(cfg, s)
        allrows += rows
        near = rows[0]["pdr"] if rows else 0
        md = rows[-1]["distance"] if rows else 0
        print(f"{cfg:>10} {rows[0]['density_vehkm']:>10.1f} {near:>8.3f} {md:>8}")
    pd.DataFrame(allrows).to_csv(RD / "line_pdr_long.csv", index=False)
    print("wrote line_pdr_long.csv")


if __name__ == "__main__":
    main()
