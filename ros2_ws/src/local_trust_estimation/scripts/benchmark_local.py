#!/usr/bin/env python3
"""
Per-node latency benchmark for the local trustworthiness pipeline.

Drives the real node callbacks over a real bag -- the same wiring
``evaluate_offline`` uses, via its ``FrameReader`` and ``Pipeline`` -- and
reports how long each node spends per frame.

Two totals are reported, and the distinction is the point:

  CRITICAL PATH   max(shape, point_count, distance, temporal) + score + sdsm.
                  Live, the four criterion nodes are separate processes
                  subscribing to the same detections, so they run at the same
                  time and only the slowest one delays the score node. This is
                  the figure that belongs in an end-to-end latency budget.
                  The visualization node is deliberately absent: it hangs off
                  the score topic in parallel and nothing waits for it.

  SEQUENTIAL SUM  every node added up, which is what this harness actually
                  spends because it calls the callbacks one after another in
                  one process. Reported for reference only -- reading it as
                  the pipeline's latency overstates it by roughly the width
                  of the parallel section.

Neither total includes DDS transport: offline, the nodes are wired by direct
calls. The critical path is therefore a lower bound on live latency, short by
the inter-node hops (score waits on a TimeSynchronizer over four topics).
The harness's own deepcopy per node is excluded -- it stands in for the
per-subscriber deserialization that DDS would do, and is not node work.

Run inside the workspace container after sourcing install/setup.bash:
    python3 ros2/src/local_trust_estimation/scripts/benchmark_local.py \
        --bag simulations/V2X-Seq-SPD/rosbags/spd_0000
    python3 ros2/src/local_trust_estimation/scripts/benchmark_local.py \
        --bag <bag> --frames 100 --dump-samples /tmp/local_samples.json
"""

import argparse
import json
import os
import statistics
import sys
import time

from local_trust_estimation.evaluate_offline import FrameReader, Pipeline

import rclpy

# (label, attribute on Pipeline, callback name). Order matches Pipeline.run.
_TIMED_NODES = [
    ('shape',       'shape',    'detections_callback'),
    ('distance',    'distance', 'detections_callback'),
    ('temporal',    'temporal', 'detections_callback'),
    ('point_count', 'count',    'synced_callback'),
    ('score',       'score',    'synced_callback'),
    ('markers',     'viz',      'detections_callback'),
    ('sdsm',        'sdsm',     'synced_callback'),
]

# The four that live in separate processes off the same detections topic, so
# only the slowest delays the score node.
_PARALLEL = ('shape', 'distance', 'temporal', 'point_count')
# What the score node's output must still pass through to reach the global
# stage. 'markers' is excluded: nothing downstream waits on it.
_AFTER = ('score', 'sdsm')


def instrument(pipeline: Pipeline, times: dict) -> None:
    """
    Wrap each node callback so Pipeline.run itself stays untouched.

    Timing the callbacks rather than re-implementing the call sequence keeps
    this benchmark from drifting away from the pipeline it measures, and
    leaves Pipeline.run's per-node deepcopy outside the timed region where it
    belongs.
    """
    for label, attr, method in _TIMED_NODES:
        node = getattr(pipeline, attr)
        original = getattr(node, method)

        def wrapper(*args, _original=original, _label=label, **kwargs):
            start = time.perf_counter()
            try:
                return _original(*args, **kwargs)
            finally:
                times[_label].append(time.perf_counter() - start)

        setattr(node, method, wrapper)


def critical_path_ms(times: dict, frame: int) -> float:
    """Return the latency of one frame that a downstream consumer waits for."""
    return (max(times[label][frame] for label in _PARALLEL)
            + sum(times[label][frame] for label in _AFTER)) * 1000.0


def sequential_ms(times: dict, frame: int) -> float:
    """Every node added up: what this single-process harness spends."""
    return sum(times[label][frame] for label, _, _ in _TIMED_NODES) * 1000.0


def collect(bag: str, max_frames, warmup: int) -> tuple:
    """Score frames from the bag, returning (per-node times, frame stamps)."""
    pipeline = Pipeline()
    pipeline.load_transforms(bag)
    times = {label: [] for label, _, _ in _TIMED_NODES}
    instrument(pipeline, times)

    stamps = []
    scored = 0
    for cloud, det, velocities, frame_t in FrameReader(bag):
        pipeline.run(cloud, det, velocities)
        scored += 1
        if scored <= warmup:
            # drop the warm-up samples the wrappers just recorded
            for series in times.values():
                series.clear()
            continue
        stamps.append(frame_t)
        if max_frames is not None and len(stamps) >= max_frames:
            break

    pipeline.destroy()
    return times, stamps


def _stats(series_ms: list) -> tuple:
    """(mean, median, p95, max) of a millisecond series."""
    ordered = sorted(series_ms)
    p95 = ordered[min(len(ordered) - 1, int(round(0.95 * (len(ordered) - 1))))]
    return statistics.mean(ordered), statistics.median(ordered), p95, max(ordered)


def observed_period_ms(stamps: list):
    """Median gap between consecutive frame stamps, or None if too few."""
    if len(stamps) < 2:
        return None
    gaps = [(b - a) / 1e6 for a, b in zip(stamps, stamps[1:])]
    return statistics.median(gaps)


def print_table(times: dict, stamps: list, bag: str) -> None:
    """Print the per-node table plus both totals."""
    n = len(stamps)
    crit = [critical_path_ms(times, i) for i in range(n)]
    seq = [sequential_ms(times, i) for i in range(n)]

    header = f'{"node":<34}{"mean ms":>9}{"median ms":>11}{"p95 ms":>9}{"max ms":>9}'
    sep = '-' * len(header)
    print()
    print(f'Local trust pipeline — {os.path.basename(bag)}, {n} timed frames')
    print()
    print(header)
    print(sep)
    for label, _, _ in _TIMED_NODES:
        mean, med, p95, mx = _stats([v * 1000.0 for v in times[label]])
        mark = ' |' if label in _PARALLEL else '  '
        print(f'{mark}{label:<32}{mean:>9.3f}{med:>11.3f}{p95:>9.3f}{mx:>9.3f}')
    print(sep)
    print('| = runs in parallel with the other marked nodes')
    print()

    for name, series in (('CRITICAL PATH', crit), ('sequential sum', seq)):
        mean, med, p95, mx = _stats(series)
        print(f'{name:<34}{mean:>9.3f}{med:>11.3f}{p95:>9.3f}{mx:>9.3f}')

    period = observed_period_ms(stamps)
    print()
    if period is None:
        print('Frame period: not measurable from fewer than two frames.')
    else:
        mean_crit = statistics.mean(crit)
        print(f'Observed frame period: {period:.1f} ms '
              f'({1000.0 / period:.1f} Hz) from the bag stamps')
        print(f'Occupancy: {100.0 * mean_crit / period:.1f}% of the period '
              f'on the critical path '
              f'({"keeps up" if mean_crit < period else "CANNOT keep up"})')


def dump_samples(path: str, times: dict, stamps: list, bag: str) -> None:
    """Write the raw per-frame series consumed by tools/latency_model."""
    n = len(stamps)
    payload = {
        'kind': 'local_stage_samples',
        'version': 1,
        'bag': bag,
        'frames': n,
        'observed_period_ms': observed_period_ms(stamps),
        # S_local for the latency model is the critical path, not the sum.
        'critical_path_stages': {'parallel': list(_PARALLEL), 'then': list(_AFTER)},
        'critical_path_ms': [critical_path_ms(times, i) for i in range(n)],
        'sequential_ms': [sequential_ms(times, i) for i in range(n)],
        'stages': {label: [v * 1000.0 for v in series]
                   for label, series in times.items()},
    }
    with open(path, 'w') as fh:
        json.dump(payload, fh, indent=2)
    print(f'\nwrote {path}  ({n} frames)')


def main(argv=None):
    """Parse arguments and run the benchmark."""
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--bag', required=True, help='source bag directory')
    ap.add_argument('--frames', type=int, default=None,
                    help='stop after this many timed frames (default: all)')
    ap.add_argument('--warmup', type=int, default=3,
                    help='untimed leading frames (JIT, caches, tracker warm-up)')
    ap.add_argument('--dump-samples', metavar='PATH',
                    help='write raw per-frame samples as JSON for the latency model')
    args = ap.parse_args(argv)

    if not os.path.isdir(args.bag):
        sys.exit(f'error: bag not found: {args.bag}')
    if args.frames is not None and args.frames < 1:
        ap.error('--frames must be >= 1')
    if args.warmup < 0:
        ap.error('--warmup must be >= 0')

    rclpy.init()
    try:
        times, stamps = collect(args.bag, args.frames, args.warmup)
        if not stamps:
            sys.exit(
                f'error: no frames scored from {args.bag}. Every frame needs '
                'the cloud, detection and velocity topics on one timestamp; '
                f'--warmup {args.warmup} may also exceed the frame count.')
        print_table(times, stamps, args.bag)
        if args.dump_samples:
            dump_samples(args.dump_samples, times, stamps, args.bag)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
