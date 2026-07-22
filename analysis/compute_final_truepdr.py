#!/usr/bin/env python3
"""Clean, activity-weighted per-run PDR for the final 6-group x 20-seed summary.

Computed entirely from the timeseries counters (no ramp-up bias, no huge rx.csv):
  per 1 s window t:  tx(t)=sum tx_count_since_last, rx(t)=sum rx_count_since_last,
                     n(t)=distinct vehicles present
  PDR = sum_t rx(t) / sum_t [ tx(t) * (n(t)-1) ]
At alpha=2.0 the comm range (~1.6 km) covers the whole UCLA map, so (n-1) is the
potential-receiver count -> this is the true all-in-range PDR. Weighting by per-second
activity removes the ramp-up artifact that made the naive ratio exceed 1.
"""
from pathlib import Path
import numpy as np, pandas as pd

RD = Path(__file__).resolve().parent.parent / "simulations" / "results"
GROUPS = {
    "per_n10": ("Periodic", 10), "hyb_n10": ("Hybrid", 10),
    "per_n150": ("Periodic", 150), "hyb_n150": ("Hybrid", 150),
    "per_n400": ("Periodic", 400), "hyb_n400": ("Hybrid", 400),
}
SEEDS = range(20)


def pdr_run(prefix, seed):
    f = RD / f"{prefix}-r{seed}-timeseries.csv"
    if not f.exists():
        return None
    ts = pd.read_csv(f, usecols=["time", "vehicle_id", "tx_count_since_last", "rx_count_since_last"])
    ts["s"] = ts["time"].round().astype(int)
    g = ts.groupby("s")
    tx = g["tx_count_since_last"].sum()
    rx = g["rx_count_since_last"].sum()
    n = g["vehicle_id"].nunique()
    opp = (tx * (n - 1)).sum()
    if opp <= 0:
        return None
    return float(rx.sum()) / float(opp)


def main():
    summ, seedrows = [], []
    print(f"{'group':>10} {'strat':>9} {'n':>4} {'mean':>8} {'std':>8} {'var':>11} {'#seeds':>7}")
    for g, (strat, n) in GROUPS.items():
        vals = [pdr_run(g, s) for s in SEEDS]
        vals = [v for v in vals if v is not None]
        if not vals:
            print(f"{g:>10}  (no data)"); continue
        a = np.array(vals)
        summ.append({"group": g, "strategy": strat, "vehicles": n,
                     "mean": round(a.mean(), 6), "std": round(a.std(ddof=1), 6),
                     "var": round(a.var(ddof=1), 8), "n_seeds": len(a)})
        for s, v in enumerate(vals):
            seedrows.append({"group": g, "strategy": strat, "vehicles": n, "seed": s, "pdr": v})
        print(f"{g:>10} {strat:>9} {n:>4} {a.mean():>8.4f} {a.std(ddof=1):>8.4f} {a.var(ddof=1):>11.7f} {len(a):>7}")
    pd.DataFrame(summ).to_csv(RD / "final_pdr_summary.csv", index=False)
    pd.DataFrame(seedrows).to_csv(RD / "final_pdr_seeds.csv", index=False)
    print("wrote final_pdr_summary.csv and final_pdr_seeds.csv")


if __name__ == "__main__":
    main()
