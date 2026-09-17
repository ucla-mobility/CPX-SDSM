#!/usr/bin/env python3
"""Aggregate the 6-group x 20-seed final summary. Reads pdr_legacy_all_pairs from
every {group}-r{seed}-summary.csv, computes mean/std/var per group, writes
results/final_pdr_summary.csv (group, strategy, n, mean, std, var, n_seeds)
and results/final_pdr_seeds.csv (group, seed, pdr) for the box/scatter plot."""
from pathlib import Path
import numpy as np, pandas as pd

RD = Path(__file__).resolve().parent.parent / "simulations" / "results"
GROUPS = {  # prefix -> (strategy, vehicles)
    "per_n10": ("Periodic", 10), "hyb_n10": ("Hybrid", 10),
    "per_n150": ("Periodic", 150), "hyb_n150": ("Hybrid", 150),
    "per_n400": ("Periodic", 400), "hyb_n400": ("Hybrid", 400),
}
SEEDS = range(20)


def pdr_of(prefix, seed):
    f = RD / f"{prefix}-r{seed}-summary.csv"
    if not f.exists():
        return None
    df = pd.read_csv(f)
    if "pdr_legacy_all_pairs" not in df.columns or len(df) == 0:
        return None
    return float(df["pdr_legacy_all_pairs"].iloc[0])


def main():
    summ, seedrows = [], []
    print(f"{'group':>10} {'strat':>9} {'n':>4} {'mean':>8} {'std':>8} {'var':>10} {'#seeds':>7}")
    for g, (strat, n) in GROUPS.items():
        vals = [pdr_of(g, s) for s in SEEDS]
        vals = [v for v in vals if v is not None]
        if not vals:
            print(f"{g:>10}  (no data)"); continue
        a = np.array(vals)
        summ.append({"group": g, "strategy": strat, "vehicles": n,
                     "mean": round(a.mean(), 6), "std": round(a.std(ddof=1), 6),
                     "var": round(a.var(ddof=1), 8), "n_seeds": len(a)})
        for s, v in enumerate(vals):
            seedrows.append({"group": g, "strategy": strat, "vehicles": n, "seed": s, "pdr": v})
        print(f"{g:>10} {strat:>9} {n:>4} {a.mean():>8.4f} {a.std(ddof=1):>8.4f} {a.var(ddof=1):>10.6f} {len(a):>7}")
    pd.DataFrame(summ).to_csv(RD / "final_pdr_summary.csv", index=False)
    pd.DataFrame(seedrows).to_csv(RD / "final_pdr_seeds.csv", index=False)
    print("wrote final_pdr_summary.csv and final_pdr_seeds.csv")


if __name__ == "__main__":
    main()
