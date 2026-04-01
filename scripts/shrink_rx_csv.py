#!/usr/bin/env python3
"""
Reduce size of *-rx.csv: keep every Nth row and round floats to fewer decimals.
Usage: python3 scripts/shrink_rx_csv.py <path-to-rx.csv> [--every N] [--decimals D]
Default: --every 2, --decimals 3. Output overwrites the file.
"""
import argparse
import csv
import sys
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_path", type=Path, help="Path to *-rx.csv")
    ap.add_argument("--every", type=int, default=2, help="Keep every Nth row (default 2)")
    ap.add_argument("--decimals", type=int, default=3, help="Decimal places for floats (default 3)")
    args = ap.parse_args()

    p = args.csv_path.resolve()
    if not p.exists():
        print(f"File not found: {p}", file=sys.stderr)
        return 1

    # Read all rows (header + data)
    rows = []
    with open(p, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        header = next(r)
        rows.append(header)
        for i, row in enumerate(r):
            if (i % args.every) == 0:
                # Round float columns: time(0), aoi(4), inter_arrival(5), snr(6), rss_dbm(7), distance(8), cbr(10), delta_state(12)
                # ints: receiver(1), sender(2), message_id(3), packet_size(9), num_objects(11), redundant(13)
                out = []
                for j, cell in enumerate(row):
                    if j in (0, 4, 5, 6, 7, 8, 10, 12):
                        try:
                            x = float(cell)
                            out.append(round(x, args.decimals))
                        except ValueError:
                            out.append(cell)
                    else:
                        out.append(cell)
                rows.append(out)

    # Write to temp then replace (so we don't corrupt if crash)
    tmp = p.with_suffix(".csv.tmp")
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        for row in rows:
            w.writerow(row)
    tmp.replace(p)
    print(f"Shrunk {p.name}: {len(rows)-1} rows (every {args.every}), {args.decimals} decimals. Size ~{p.stat().st_size / (1024*1024):.1f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
