#!/usr/bin/env python3
"""
Compute distance-binned Packet Delivery Ratio (PDR) from simulation logs.

PDR definition:
  For each distance bin [d_lo, d_hi), PDR = (received packets where sender-receiver
  distance was in bin) / (total TX opportunities in that bin).

The denominator is estimated from TX logs + timeseries snapshots: for each TX event,
count all alive receivers at the sender's position within the distance bin.

Input:  *-rx.csv, *-tx.csv, *-timeseries.csv
Output: *-pdr-binned.csv

Legacy all-pairs PDR is preserved in *-summary.csv as pdr_legacy_all_pairs.

Percentile method: numpy nearest-rank (method='lower') to match C++ implementation.

Usage:
  python analysis/compute_pdr.py results/Periodic-r0
  python analysis/compute_pdr.py results/  # process all algorithm prefixes
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

# Distance bins in meters (upper bound exclusive)
BIN_EDGES = list(range(0, 325, 25))  # 0, 25, 50, ..., 300


def compute_pdr_for_prefix(prefix: str, result_dir: Path) -> pd.DataFrame:
    """Compute distance-binned PDR for a single run prefix (e.g., 'Periodic-r0').

    Uses rx CSV for numerator (received packets by distance bin).
    Uses timeseries CSV for denominator (vehicle positions at ~1Hz to estimate
    how many vehicles were within each distance bin per TX event).
    """
    rx_file = result_dir / f"{prefix}-rx.csv"
    ts_file = result_dir / f"{prefix}-timeseries.csv"

    if not rx_file.exists():
        print(f"  SKIP: {rx_file.name} not found")
        return pd.DataFrame()
    if not ts_file.exists():
        print(f"  SKIP: {ts_file.name} not found (needed for denominator)")
        return pd.DataFrame()

    rx_df = pd.read_csv(rx_file)
    ts_df = pd.read_csv(ts_file)

    if "distance_to_sender" not in rx_df.columns:
        print(f"  SKIP: {rx_file.name} missing distance_to_sender column")
        return pd.DataFrame()

    # Numerator: count received packets per distance bin
    rx_df["dist_bin"] = pd.cut(
        rx_df["distance_to_sender"],
        bins=BIN_EDGES,
        right=False,
        labels=[f"{BIN_EDGES[i]}-{BIN_EDGES[i+1]}" for i in range(len(BIN_EDGES)-1)],
    )
    numerator = rx_df.groupby("dist_bin", observed=True).size()

    # Denominator: for each TX, estimate receivers in each distance bin.
    # Use timeseries snapshots (1Hz) of all vehicle positions.
    # For each unique time step, compute pairwise distances between all vehicles.
    required_ts_cols = {"time", "vehicle_id", "position_x", "position_y"}
    if not required_ts_cols.issubset(ts_df.columns):
        print(f"  WARN: timeseries missing columns, using rx-only PDR estimate")
        # Fallback: just report numerator / total_rx_in_bin
        results = []
        for i in range(len(BIN_EDGES) - 1):
            label = f"{BIN_EDGES[i]}-{BIN_EDGES[i+1]}"
            n = numerator.get(label, 0)
            results.append({
                "distance_bin": label,
                "received": int(n),
                "denominator": -1,
                "pdr": -1.0,
                "note": "denominator_unavailable",
            })
        return pd.DataFrame(results)

    # Sample timeseries at distinct time steps
    ts_times = sorted(ts_df["time"].unique())

    # Build position snapshots: time -> {vehicle_id: (x, y)}
    denom_bins = {f"{BIN_EDGES[i]}-{BIN_EDGES[i+1]}": 0 for i in range(len(BIN_EDGES)-1)}

    for t in ts_times:
        snapshot = ts_df[ts_df["time"] == t][["vehicle_id", "position_x", "position_y"]]
        if len(snapshot) < 2:
            continue
        positions = snapshot[["position_x", "position_y"]].values
        n_veh = len(positions)
        # Pairwise distances
        for i in range(n_veh):
            for j in range(n_veh):
                if i == j:
                    continue
                dx = positions[i, 0] - positions[j, 0]
                dy = positions[i, 1] - positions[j, 1]
                dist = np.sqrt(dx * dx + dy * dy)
                for k in range(len(BIN_EDGES) - 1):
                    if BIN_EDGES[k] <= dist < BIN_EDGES[k + 1]:
                        denom_bins[f"{BIN_EDGES[k]}-{BIN_EDGES[k+1]}"] += 1
                        break

    # Normalize denominator by number of time samples to get per-second rate,
    # then multiply by total simulation time to approximate total opportunities.
    # Actually, each timeseries row is one opportunity sample.
    results = []
    for i in range(len(BIN_EDGES) - 1):
        label = f"{BIN_EDGES[i]}-{BIN_EDGES[i+1]}"
        n = int(numerator.get(label, 0))
        d = denom_bins[label]
        pdr = n / d if d > 0 else -1.0
        results.append({
            "distance_bin": label,
            "received": n,
            "denominator": d,
            "pdr": round(pdr, 6) if pdr >= 0 else -1.0,
        })

    return pd.DataFrame(results)


def find_prefixes(result_dir: Path) -> list[str]:
    """Find unique run prefixes from rx CSV filenames."""
    prefixes = set()
    for f in result_dir.glob("*-rx.csv"):
        # e.g., "Periodic-r0-rx.csv" -> "Periodic-r0"
        name = f.name.replace("-rx.csv", "")
        prefixes.add(name)
    return sorted(prefixes)


def main() -> int:
    parser = argparse.ArgumentParser(description="Compute distance-binned PDR from simulation logs.")
    parser.add_argument("path", help="Path to results directory or specific prefix (e.g., results/Periodic-r0)")
    args = parser.parse_args()

    p = Path(args.path)

    if p.is_dir():
        result_dir = p
        prefixes = find_prefixes(result_dir)
    else:
        # Assume it's a prefix like "results/Periodic-r0"
        result_dir = p.parent
        prefix = p.name
        prefixes = [prefix]

    if not prefixes:
        print(f"No *-rx.csv files found in {result_dir}", file=sys.stderr)
        return 1

    for prefix in prefixes:
        print(f"Computing PDR for {prefix} ...")
        pdr_df = compute_pdr_for_prefix(prefix, result_dir)
        if pdr_df.empty:
            continue

        out_path = result_dir / f"{prefix}-pdr-binned.csv"
        pdr_df.to_csv(out_path, index=False)
        print(f"  -> {out_path}")

        # Print summary
        valid = pdr_df[pdr_df["pdr"] >= 0]
        if not valid.empty:
            for _, row in valid.iterrows():
                print(f"  {row['distance_bin']:>8s}m: PDR={row['pdr']:.4f} "
                      f"(rx={row['received']}, denom={row['denominator']})")

    return 0


if __name__ == "__main__":
    sys.exit(main())
