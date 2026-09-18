# mypy: ignore-errors
"""
Before/after latency of the mmcooper_fuse vectorization, for the grouped-bar
figure visualizations/fusion_speedup.html draws.

Times `fusion.mspsf` -- the phase-1 fusion hot path, which internally calls
`geometry.compute_self_iou_mat` -- at four (agents x objects) scene sizes, with
and without OUR speedups, and writes visualizations/fusion_speedup_data.js.

WHAT "BEFORE" AND "AFTER" MEAN
  after  : the shipped `mspsf` (the `CHANGED (vectorized)` blocks in fusion.py
           and geometry.py -- broadcast affinity/kappa and the batched Shapely
           IoU matrix).
  before : `mspsf_old` from `mmcooper_fuse.old` -- the shipped fusion with only
           its two `CHANGED (vectorized)` blocks reverted to their loops, shared
           with benchmark_stages.py --fusion reference so there is a single
           "before". Driven with kds_scores=None below, so it is the
           pre-vectorization fusion; the reverted blocks also restore the OG
           (per-pair Shapely) IoU.

KDS IS EXCLUDED, DELIBERATELY
  The Kinematic-Dynamic orientation code (`_orient_with_kds`, commit 2afeb7db)
  is theory we received, not our speedup, so it must not colour the comparison.
  `adapter.fuse()` always passes kds_scores (defaulting to 1.0 per detection),
  which would run `_orient_with_kds` in the "after" arm and pay a cost the OG
  arm cannot -- so this script drives `mspsf` DIRECTLY with kds_scores=None
  rather than through the adapter. With KDS off, the only difference between the
  two arms is the vectorization, which is the whole point of the figure.

RUN IT IN ros2_dev (numpy/scipy/shapely; emulated x86 -- single runs swing ~2x,
so this reports best-of-N, not one shot). From the workspace root, with the
package on the path:
    PYTHONPATH=ros2/src/global_trust_perception \
        python3 ros2/src/global_trust_perception/scripts/make_fusion_speedup_viz.py

Re-run after changing fusion.py/geometry.py to refresh the figure -- the HTML
page never measures anything, it only draws whatever this last wrote.
"""

import argparse
import json
import math
import os
import subprocess
import time

import numpy as np

# HEAD fusion (the "after" arm) and its shared pre-vectorization copy (the
# "before" arm, mspsf_old), plus the one geometry helper build_inputs uses.
from global_trust_perception.mmcooper_fuse.fusion import (
    FusionConfig,
    build_class_probs,
    mspsf,
)
from global_trust_perception.mmcooper_fuse.geometry import corners_from_pose
from global_trust_perception.mmcooper_fuse.old import mspsf_old as _og_mspsf

# Equipment-type count, matching adapter.EQUIPMENT_TYPE_COUNT: sets the modality
# entropy normaliser. Timing-neutral, but kept faithful to production.
_TOTAL_MODALITIES = 4
_N_CLASSES = 4

# Scene constants, matching scripts/benchmark_stages.py so the two tools model
# the same synthetic frame: a grid of objects at 15 m spacing (wide enough that
# distinct objects never fuse), each sender observing all of them with 0.2 m
# noise, and observers occasionally disagreeing on class so kappa is non-trivial.
_GRID_SPACING_M = 15.0
_NOISE_STD_M = 0.2
_DIMS_W, _DIMS_L = 2.0, 4.5          # the sim's vehicle box (width, length)
_LABEL_DISAGREE_P = 0.15
_R_DEFAULT = 0.5                      # per-sender reliability

# The four combos the figure shows: (agents, objects).
_COMBOS = [(2, 30), (2, 256), (10, 30), (10, 256)]

_OUT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'visualizations',
    'fusion_speedup_data.js',
)


# ---------------------------------------------------------------------------
# Synthetic scene -> mspsf inputs
# ---------------------------------------------------------------------------

def build_inputs(n_agents: int, n_objects: int, rng: np.random.Generator) -> dict:
    """Build one frozen set of mspsf inputs: n_agents senders each reporting
    every object with noise, stacked into n = n_agents * n_objects detections.

    Built once and shared by both arms so the before/after bars fuse byte-for-byte
    identical inputs -- the only thing that differs between the arms is the code
    path, never the scene.
    """
    side = math.ceil(math.sqrt(n_objects))
    base = [
        (col * _GRID_SPACING_M, row * _GRID_SPACING_M)
        for row in range(side) for col in range(side)
    ][:n_objects]
    truth_labels = [int(rng.integers(0, _N_CLASSES)) for _ in range(n_objects)]

    boxes, scores, labels, agents, modalities = [], [], [], [], []
    for a in range(n_agents):
        for (bx, by), truth in zip(base, truth_labels):
            cx = bx + rng.normal(0, _NOISE_STD_M)
            cy = by + rng.normal(0, _NOISE_STD_M)
            # Axis-aligned car box; corners_from_pose lays length along the yaw.
            boxes.append(corners_from_pose(cx, cy, _DIMS_L, _DIMS_W, 0.0))
            scores.append(0.9)
            label = truth if rng.random() >= _LABEL_DISAGREE_P else int(rng.integers(0, _N_CLASSES))
            labels.append(label)
            agents.append(a)
            modalities.append(a % _TOTAL_MODALITIES)

    scores_arr = np.asarray(scores, dtype=np.float32)
    return {
        'boxes': np.asarray(boxes, dtype=np.float32),
        'scores': scores_arr,
        'class_probs': build_class_probs(np.asarray(labels, dtype=np.int64), scores_arr, _N_CLASSES),
        'agents': np.asarray(agents, dtype=np.int64),
        'modalities': np.asarray(modalities, dtype=np.int64),
        'reliabilities': np.full(n_agents, _R_DEFAULT, dtype=np.float32),
    }


# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------

def _time_after(inp: dict, cfg: FusionConfig) -> float:
    t0 = time.perf_counter()
    mspsf(inp['boxes'], inp['scores'], inp['class_probs'], inp['agents'],
          inp['modalities'], cfg, agent_reliabilities=inp['reliabilities'],
          kds_scores=None)          # KDS off: excluded from the speed assessment
    return (time.perf_counter() - t0) * 1000.0


def _time_before(inp: dict, cfg: FusionConfig) -> float:
    t0 = time.perf_counter()
    _og_mspsf(inp['boxes'], inp['scores'], inp['class_probs'], inp['agents'],
              inp['modalities'], cfg, agent_reliabilities=inp['reliabilities'])
    return (time.perf_counter() - t0) * 1000.0


def summarize(fn, inp, cfg, repeats: int, warmup: int) -> dict:
    """Distribution of `repeats` timed calls (ms) after `warmup` untimed ones.

    Reports the median (the bar), the 25th/75th percentiles (the error bar) and
    the best/min (the floor line). On emulated x86 a single run swings ~2x, so a
    spread rather than one number is the honest summary: the median is the
    central estimate, the quartiles bound the typical run, and the best shows the
    machine's jitter-free floor.
    """
    for _ in range(warmup):
        fn(inp, cfg)
    samples = sorted(fn(inp, cfg) for _ in range(repeats))
    return {
        'median': float(np.median(samples)),
        'p25': float(np.percentile(samples, 25)),
        'p75': float(np.percentile(samples, 75)),
        'best': samples[0],
    }


def _head_sha() -> str:
    try:
        return subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=os.path.dirname(os.path.abspath(__file__)),
        ).decode().strip()
    except Exception:
        return 'unknown'


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument('--repeats', type=int, default=7, help='timed calls per arm/combo')
    ap.add_argument('--warmup', type=int, default=2, help='untimed warm-up calls')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--out', default=_OUT, help='data.js path to write')
    args = ap.parse_args()

    cfg = FusionConfig(mspsf_total_modalities=_TOTAL_MODALITIES)
    combos = []
    for n_agents, n_objects in _COMBOS:
        rng = np.random.default_rng(args.seed)     # same scene seed per combo
        inp = build_inputs(n_agents, n_objects, rng)
        before = summarize(_time_before, inp, cfg, args.repeats, args.warmup)
        after = summarize(_time_after, inp, cfg, args.repeats, args.warmup)
        combos.append({
            'agents': n_agents,
            'objects': n_objects,
            'n': int(len(inp['boxes'])),
            'before_ms': round(before['median'], 4),
            'before_p25_ms': round(before['p25'], 4),
            'before_p75_ms': round(before['p75'], 4),
            'before_best_ms': round(before['best'], 4),
            'after_ms': round(after['median'], 4),
            'after_p25_ms': round(after['p25'], 4),
            'after_p75_ms': round(after['p75'], 4),
            'after_best_ms': round(after['best'], 4),
            'speedup': round(before['median'] / after['median'], 2) if after['median'] > 0 else None,
        })
        print(f'{n_agents:>3} ag x {n_objects:>4} obj (n={len(inp["boxes"]):>5}): '
              f'before {before["median"]:8.2f} ms  after {after["median"]:7.2f} ms  '
              f'({combos[-1]["speedup"]}x)')

    payload = {
        'generated_by': 'scripts/make_fusion_speedup_viz.py',
        'measures': 'fusion.mspsf phase-1 time, ms (geometry.compute_self_iou_mat + affinity/kappa)',
        'before_ref': 'mmcooper_fuse.old.mspsf_old (HEAD minus the vectorized blocks)',
        'after_ref': f'HEAD ({_head_sha()})',
        'kds': 'excluded (mspsf driven with kds_scores=None)',
        'repeats': args.repeats,
        'warmup': args.warmup,
        'seed': args.seed,
        'note': 'measured in the emulated-x86 ros2_dev container; bar = median, whisker = 25-75th pct, line = best of N',
        'combos': combos,
    }
    with open(args.out, 'w') as fh:
        fh.write('// GENERATED by scripts/make_fusion_speedup_viz.py -- do not hand-edit.\n')
        fh.write('// Re-run it after changing fusion.py/geometry.py to refresh fusion_speedup.html.\n')
        fh.write('window.FUSION_SPEEDUP = ')
        json.dump(payload, fh, indent=2)
        fh.write(';\n')
    print(f'\nwrote {args.out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
