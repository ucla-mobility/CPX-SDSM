# mypy: ignore-errors
# Builds SdsmPayload messages directly (dev tooling; mirrors the codec
# conventions private to perception_message).
"""
Per-stage latency benchmark for the trust pipeline at configurable scale.

Times the stages of TrustEngine.process_frame plus the agent-node work around
it (message decode, DB writes) on synthetic frames of --agents senders each
reporting --objects detections.

SORT tracking is timed separately and reported both included and excluded from
the end-to-end total, so you can see its cost in isolation. The MS-PSF phase-1
fusion (the shapely-based rotated-BEV clustering) is the dominant matching cost
and is timed as its own stage.

--agents-sweep runs one point per agent count and reports N_max: the largest
count whose p95 frame still fits the flush budget (1 / --flush-hz). Past that
the frame buffer grows every window and latency has no steady state, so N_max
is the boundary any end-to-end latency figure is only valid below.

--objects-sweep sweeps object count at fixed --agents, the mirror of
--agents-sweep (which sweeps agents at fixed --objects); use one or the other.

--fusion selects which MS-PSF hot path to time: 'modified' (the shipped,
vectorized code, default) or 'reference' (the pre-optimization loops from
mmcooper_fuse/old/). Running the same sweep under both and diffing the
two --dump-samples files is how tools/fusion_ab measures what the speed rewrites
buy end-to-end; the kds-orientation feature is present in both.

--machine-label (1 = this machine, 2 = NVIDIA Thor 4) plus the auto-detected
GPU/CPU are recorded in the dump so tools/fusion_ab can overlay and label the
machine each series came from. Inside the emulated container nvidia-smi sees no
GPU, so pass --machine-gpu to record the host's real one.

Run inside the workspace container after sourcing install/setup.bash:
    python3 ros2/src/global_trust_perception/scripts/benchmark_stages.py
    python3 ros2/src/global_trust_perception/scripts/benchmark_stages.py \
        --agents 20 --objects 30 --frames 50
    python3 ros2/src/global_trust_perception/scripts/benchmark_stages.py \
        --agents-sweep 2 5 10 20 40 --dump-samples /tmp/global_samples.json

Message decode timing needs sdsm_interfaces on the PYTHONPATH; without it
the decode stage is skipped and the end-to-end total notes the omission.
"""

import argparse
import json
import math
import os
import platform
import statistics
import subprocess
import tempfile
import time

import numpy as np

from global_trust_perception.trust_calculations.consistency import (
    corroboration_support,
    is_corroborated,
    weighted_ego_consistency,
)
from global_trust_perception.trust_calculations.consistency_checks.attribute_checks import check_size_agreement
from global_trust_perception.trust_calculations.consistency_checks.kinematic_checks import (
    KinematicHistory,
    TrackData,
)
from global_trust_perception.trust_calculations.deferred import PendingVerdicts
from global_trust_perception.mmcooper_fuse.adapter import StreamInput, fuse
from global_trust_perception.pipeline.persistent_reputation_tracker import PersistentReputationTracker
from global_trust_perception.trust_calculations.reputation import dynamic_threshold, reputation_update
from global_trust_tracker.SORT.modified_SORT_centroid import Sort

try:
    from global_trust_perception.pipeline import perception_message as pmsg
    _HAVE_MSG = True
except ImportError:
    _HAVE_MSG = False

# SdsmPayload's fixed array capacity. Read from the message rather than
# restated, the same way sdsm_publisher_node does it, so widening the payload
# needs no change here -- a hardcoded copy had already gone stale at 32.
_DEFAULT_MSG_OBJECTS = 256
_MAX_MSG_OBJECTS = (len(pmsg.Message().obj_type) if _HAVE_MSG
                    else _DEFAULT_MSG_OBJECTS)

_EGO_KEY = 'ego'                 # matches trustworthy_perception._EGO_KEY
_DIMS = (2.0, 4.5, 1.5)          # (w, l, h) — the sim's vehicle box
_GRID_SPACING_M = 15.0           # keeps distinct objects from accidentally fusing
_NOISE_STD_M = 0.2               # per-agent position noise
_DT = 0.5                        # seconds between frames (production 2 Hz)
_R_DEFAULT = 0.5                 # reliability used for every synthetic sender
_LABEL_DISAGREE_P = 0.15         # chance an observer mislabels a given object


# ---------------------------------------------------------------------------
# Synthetic scene
# ---------------------------------------------------------------------------

def make_scene(n_agents: int, n_objects: int, n_classes: int,
               rng: np.random.Generator):
    """One frame: every agent (and ego) sees every object with small noise.

    Returns (ego_positions, positions_by_agent, dims_by_agent, source_id_map,
    ego_labels, labels_by_agent) in the exact shapes process_frame receives.

    Each object carries a class label, and observers mostly agree on it: the
    fusion stage's kappa term is a class-probability overlap, so labelling
    everything identically (or not at all, which collapses to one class) makes
    kappa a constant and understates the stage that dominates the frame.
    Disagreement is occasional rather than uniform, since kappa's cost comes
    from the pairs that survive its gate.
    """
    side = math.ceil(math.sqrt(n_objects))
    base = [
        (col * _GRID_SPACING_M, row * _GRID_SPACING_M, 0.75)
        for row in range(side) for col in range(side)
    ][:n_objects]
    truth_labels = [int(rng.integers(0, n_classes)) for _ in range(n_objects)]

    def noisy():
        return [
            (x + rng.normal(0, _NOISE_STD_M), y + rng.normal(0, _NOISE_STD_M), z)
            for x, y, z in base
        ]

    def observed_labels():
        return [
            label if rng.random() >= _LABEL_DISAGREE_P
            else int(rng.integers(0, n_classes))
            for label in truth_labels
        ]

    ego_positions = noisy()
    positions_by_agent = {aid: noisy() for aid in range(2, n_agents + 2)}
    dims_by_agent = {aid: [_DIMS] * n_objects for aid in positions_by_agent}
    source_id_map = {aid: (aid, 0, 0, 0) for aid in positions_by_agent}
    ego_labels = observed_labels()
    labels_by_agent = {aid: observed_labels() for aid in positions_by_agent}
    return (ego_positions, positions_by_agent, dims_by_agent, source_id_map,
            ego_labels, labels_by_agent)


def make_payloads(positions_by_agent: dict, dims_by_agent: dict):
    """Encode each agent's detections into a real SdsmPayload for decode timing."""
    msgs = []
    for aid, positions in positions_by_agent.items():
        msg = pmsg.Message()
        msg.source_id = [aid, 0, 0, 0]
        msg.num_objects = min(len(positions), _MAX_MSG_OBJECTS)
        msg.ref_pos_x, msg.ref_pos_y, msg.ref_pos_z = 0.0, 0.0, 0.0
        for i in range(msg.num_objects):
            x, y, z = positions[i]
            w, l, h = dims_by_agent[aid][i]
            msg.offset_x[i] = round(x / 0.1)
            msg.offset_y[i] = round(y / 0.1)
            msg.offset_z[i] = round(z / 0.1)
            msg.obj_width[i] = round(w / 0.1)
            msg.obj_length[i] = round(l / 0.1)
            msg.obj_height[i] = round(h / 0.1)
            msg.obj_speed[i] = 50          # 1.0 m/s in 0.02 m/s units
            msg.obj_heading[i] = 7200      # 90 deg in 0.0125 deg units
            msg.obj_local_scores[i] = 0.9  # per-object certainty
        msgs.append(msg)
    return msgs


# ---------------------------------------------------------------------------
# Stage bodies (each is one timed unit per frame)
# ---------------------------------------------------------------------------

def stage_decode(msgs):
    """agent.flush_frame's per-message decode loop."""
    positions_by_agent, scores_by_agent, dims_by_agent = {}, {}, {}
    for msg in msgs:
        sender = pmsg.sender_of(msg)
        positions_by_agent.setdefault(sender, []).extend(pmsg.get_global_positions_of(msg))
        scores_by_agent.setdefault(sender, []).extend(pmsg.get_local_scores_of(msg))
        dims_by_agent.setdefault(sender, []).extend(pmsg.get_dims_of(msg))
    return positions_by_agent


def stage_sort(sort_trackers: dict, positions_by_agent: dict, dims_by_agent: dict) -> dict:
    """Run Sort.update() per agent, as TrackerNode does.

    sort_trackers is mutated in place (one Sort instance per agent, persisting
    across frames — same as the live tracker node).
    Returns track_data_by_agent: dict[agent_id -> TrackData].
    """
    track_data_by_agent = {}
    for agent_id, positions in positions_by_agent.items():
        if agent_id not in sort_trackers:
            sort_trackers[agent_id] = Sort()
        n = len(positions)
        if n:
            xy = np.array([(p[0], p[1]) for p in positions], dtype=float)
            vel = np.zeros((n, 2))
            dims = np.array([dims_by_agent[agent_id][i] for i in range(n)], dtype=float)
        else:
            xy = np.empty((0, 2))
            vel = None
            dims = None
        tracks = sort_trackers[agent_id].update(xy, vel, dims)
        track_data_by_agent[agent_id] = TrackData(
            track_ids=[int(t.id)            for t in tracks],
            kalman_x =[float(t.kf.x[0, 0]) for t in tracks],
            kalman_y =[float(t.kf.x[1, 0]) for t in tracks],
            kalman_vx=[float(t.kf.x[4, 0]) for t in tracks],
            kalman_vy=[float(t.kf.x[5, 0]) for t in tracks],
        )
    return track_data_by_agent


def stage_kinematic(kin_history: KinematicHistory,
                    track_data_by_agent: dict,
                    positions_by_agent: dict) -> dict:
    """Run KinematicHistory.score_and_update() per agent.

    kin_history persists across frames — same as TrustEngine._kin in live code.
    Returns kds_by_agent: dict[agent_id -> list[float]].
    """
    results = {}
    for agent_id, track_data in track_data_by_agent.items():
        positions = positions_by_agent.get(agent_id, [])
        other_xy = (np.array([(p[0], p[1]) for p in positions], dtype=float)
                    if positions else np.empty((0, 2)))
        results[agent_id] = kin_history.score_and_update(
            agent_id, track_data, other_xy, _DT
        )
    return results


def stage_mspsf(ego_positions, positions_by_agent, ego_dims, dims_by_agent,
                ego_labels, labels_by_agent):
    """Phase-1 MS-PSF fusion across ego + all agents (the matching stage)."""
    streams = [StreamInput(key=_EGO_KEY, positions=ego_positions, dims=ego_dims,
                           labels=ego_labels, reliability=1.0)]
    streams += [
        StreamInput(key=aid, positions=positions_by_agent[aid],
                    dims=dims_by_agent[aid], labels=labels_by_agent[aid],
                    reliability=_R_DEFAULT)
        for aid in positions_by_agent
    ]
    return fuse(streams, _EGO_KEY)


def _support_of(fusion, agent_ids):
    """Stage 2d support from the phase-1 clusters, mirroring process_frame.

    Every synthetic sender shares _R_DEFAULT and reports no per-object scores,
    so certainty defaults to 1.0 and support is plain reputation mass.
    """
    reliability = {_EGO_KEY: 1.0}
    reliability.update({aid: _R_DEFAULT for aid in agent_ids})
    return corroboration_support(fusion.clusters, reliability, {}, _EGO_KEY)


def stage_buckets(fusion, ego_positions, positions_by_agent):
    """Mirror of process_frame Stage 2 bucket derivation + Stage 2d support."""
    out = {}
    clusters = fusion.clusters
    support = _support_of(fusion, positions_by_agent)
    for agent_id, other_positions in positions_by_agent.items():
        matched = [(c[_EGO_KEY], c[agent_id]) for c in clusters
                   if _EGO_KEY in c and agent_id in c]
        matched_ego = {e for e, _ in matched}
        matched_other = {o for _, o in matched}
        ego_only = [i for i in range(len(ego_positions)) if i not in matched_ego]
        other_only = [j for j in range(len(other_positions)) if j not in matched_other]

        support_by_det = support.get(agent_id, {})
        uncorroborated = sum(
            1 for j in other_only if not is_corroborated(support_by_det.get(j, 0.0))
        )
        out[agent_id] = (matched, ego_only, other_only, uncorroborated)
    return out


def stage_attr(buckets, ego_dims, dims_by_agent):
    """process_frame Stage 2c, including its per-agent list() copies.

    check_size_agreement() is purely geometric now; the reputation gate
    (mirrored here as the 0.5 default other_rep, same as _R_DEFAULT) and the
    diagnostic 0.8 threshold both moved to the caller in the live pipeline
    (trustworthy_perception.py's _SIZE_REP_GATE / _SS_AGREE_THRESHOLD).
    """
    total = 0
    for agent_id, (matched, _, _, _) in buckets.items():
        ss_scores = check_size_agreement(
            matched,
            list(ego_dims or []),
            list(dims_by_agent.get(agent_id) or []),
        )
        if _R_DEFAULT <= 0.70:
            ss_scores = [0.0] * len(ss_scores)
        total += sum(1 for ss in ss_scores if ss >= 0.8)
    return total


def stage_consistency(buckets, fusion, ledger, frame_num):
    """process_frame Stage 3: weighted matched/ego_only + the deferred ledger."""
    results = {}
    support = _support_of(fusion, buckets)
    for agent_id, (matched, ego_only, other_only, _) in buckets.items():
        matched_certainties = [0.9] * len(matched)
        ego_only_certainties = [1.0] * len(ego_only)
        C, I, held = weighted_ego_consistency(matched_certainties, ego_only_certainties)
        support_by_det = support.get(agent_id, {})
        obs = [(j, 0.9, 0.9, support_by_det.get(j, 0.0)) for j in other_only]
        vindicated = [oi for _, oi in matched]
        delta = ledger.observe(agent_id, frame_num, obs, vindicated)
        C += delta.correct
        I += delta.incorrect
        results[agent_id] = (C, I, held + delta.pending)
    return results


def stage_reputation(consistency_results, source_id_map):
    """process_frame Stage 0 gate precheck + Stage 4 update, per agent."""
    out = {}
    for agent_id, (C, I, _) in consistency_results.items():
        sid = source_id_map[agent_id]
        _ = ' '.join(f'{b:02x}' for b in sid)       # Stage 0 sid-string build
        tau = dynamic_threshold(0.5)
        _ = 0.5 >= tau
        R_new, s = reputation_update(0.5, C, I, C + I)
        out[agent_id] = R_new
    return out


def stage_db(tracker, reputations, source_id_map, frame_num):
    """agent.flush_frame's record loop + a flush every frame (worst-case cadence)."""
    for agent_id, r_new in reputations.items():
        tracker.record(source_id_map[agent_id], r_new, frame_num)
    tracker.flush(frame_num)


# ---------------------------------------------------------------------------
# Machine identity (recorded in the dump so tools/fusion_ab can label the
# machine each series came from, and show its GPU/CPU)
# ---------------------------------------------------------------------------

_MACHINE_NAMES = {'1': 'This machine', '2': 'NVIDIA Thor 4'}


def _detect_gpu():
    """GPU name/memory/driver from nvidia-smi, or None. Works natively on the
    Thor; returns None inside the emulated-x86 container (no NVIDIA visible),
    which is why --machine-gpu exists to supply it by hand there."""
    try:
        out = subprocess.run(
            ['nvidia-smi', '--query-gpu=name,memory.total,driver_version',
             '--format=csv,noheader'],
            capture_output=True, text=True, timeout=5)
        line = out.stdout.strip().splitlines()[0] if out.returncode == 0 else ''
        if line:
            name, mem, drv = (p.strip() for p in line.split(','))
            return f'{name} ({mem}, driver {drv})'
    except Exception:
        pass
    return None


def _detect_cpu():
    """CPU model + core count, best-effort across Linux and macOS."""
    model = ''
    try:
        with open('/proc/cpuinfo') as fh:
            for ln in fh:
                if ln.lower().startswith('model name'):
                    model = ln.split(':', 1)[1].strip()
                    break
    except Exception:
        pass
    model = model or platform.processor() or platform.machine()
    return f'{model} x{os.cpu_count()}'


def machine_info(label: str, name_override, gpu_override) -> dict:
    """Identity block written into the dump. gpu is auto-detected (nvidia-smi)
    unless overridden -- the override is for the emulated container, where the
    run cannot see the host's real GPU."""
    return {
        'label': label,
        'name': name_override or _MACHINE_NAMES.get(label, f'machine {label}'),
        'gpu': gpu_override or _detect_gpu(),
        'cpu': _detect_cpu(),
        'platform': platform.platform(),
        'arch': platform.machine(),
    }


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def run(n_agents: int, n_objects: int, n_classes: int, n_frames: int,
        warmup: int, seed: int) -> dict:
    """Time every stage over n_frames and return the raw per-stage samples.

    Returns rather than prints so one run can feed both the printed table and
    the sample dump, and so a sweep can call it once per agent count without
    the reporting shape leaking in here.
    """
    rng = np.random.default_rng(seed)
    db_path = os.path.join(tempfile.mkdtemp(prefix='bench_trust_'), 'bench.db')
    tracker = PersistentReputationTracker(db_path)

    # persistent state across frames (mirrors live nodes)
    sort_trackers: dict = {}
    kin_history = KinematicHistory()
    ledger = PendingVerdicts()

    stages = [
        'decode', 'sort tracking', 'kinematic checks',
        'mspsf phase-1 fusion', 'buckets+support', 'attribute checks',
        'consistency+ledger', 'reputation+gate', 'db record+flush',
    ]
    times: dict = {s: [] for s in stages}

    for frame in range(n_frames + warmup):
        (ego_positions, positions_by_agent, dims_by_agent, source_id_map,
         ego_labels, labels_by_agent) = make_scene(
            n_agents, n_objects, n_classes, rng)
        ego_dims = [_DIMS] * len(ego_positions)
        record = frame >= warmup

        def timed(name, fn, *args):
            t0 = time.perf_counter()
            result = fn(*args)
            if record:
                times[name].append(time.perf_counter() - t0)
            return result

        if _HAVE_MSG:
            msgs = make_payloads(positions_by_agent, dims_by_agent)
            timed('decode', stage_decode, msgs)

        track_data_by_agent = timed('sort tracking', stage_sort,
                                    sort_trackers, positions_by_agent, dims_by_agent)
        timed('kinematic checks', stage_kinematic,
              kin_history, track_data_by_agent, positions_by_agent)

        fusion = timed('mspsf phase-1 fusion', stage_mspsf,
                       ego_positions, positions_by_agent, ego_dims, dims_by_agent,
                       ego_labels, labels_by_agent)
        buckets = timed('buckets+support', stage_buckets,
                        fusion, ego_positions, positions_by_agent)
        timed('attribute checks', stage_attr, buckets, ego_dims, dims_by_agent)
        consistency = timed('consistency+ledger', stage_consistency,
                            buckets, fusion, ledger, frame)
        reputations = timed('reputation+gate', stage_reputation,
                            consistency, source_id_map)
        timed('db record+flush', stage_db,
              tracker, reputations, source_id_map, frame)

    tracker.close()
    return times


# Stages that run inside agent.flush_frame. SORT is excluded: it lives in the
# separate tracker node and runs concurrently, so it is not on this budget.
_AGENT_STAGES = ['decode', 'kinematic checks', 'mspsf phase-1 fusion',
                 'buckets+support', 'attribute checks', 'consistency+ledger',
                 'reputation+gate', 'db record+flush']


def e2e_per_frame_ms(times: dict) -> list:
    """Per-frame flush_frame totals in ms, in frame order.

    Summed per frame rather than by adding per-stage statistics, so median and
    max describe frames that actually occurred. Frame order is preserved --
    the printed table sorts its own copy, and the sample dump wants the
    unsorted series.
    """
    stages = [s for s in _AGENT_STAGES if times.get(s)]
    n = len(times[stages[0]])
    return [sum(times[s][i] for s in stages) * 1000.0 for i in range(n)]


def _p95(sorted_ms: list) -> float:
    """95th percentile of an already-sorted millisecond series."""
    return sorted_ms[min(len(sorted_ms) - 1, int(round(0.95 * (len(sorted_ms) - 1))))]


def print_table(times: dict, n_agents: int, n_objects: int, n_frames: int,
                budget_ms: float):
    """Aggregate raw stage timings and print the summary table."""
    def ms(vals):
        return [v * 1000.0 for v in vals]

    def row(vals):
        v = ms(vals)
        v.sort()
        return statistics.mean(v), statistics.median(v), _p95(v), max(v)

    e2e_per_frame = e2e_per_frame_ms(times)
    e2e_per_frame.sort()
    e2e_mean = statistics.mean(e2e_per_frame)
    e2e_med  = statistics.median(e2e_per_frame)
    e2e_max  = max(e2e_per_frame)

    budget = budget_ms
    header = f'{"stage":<38}{"mean ms":>9}{"median ms":>11}{"p95 ms":>9}{"max ms":>9}'
    sep = '-' * len(header)

    def show(label, key, indent=False):
        if not times.get(key):
            return
        mean, med, p95, mx = row(times[key])
        name = ('  ' if indent else '') + label
        print(f'{name:<38}{mean:>9.3f}{med:>11.3f}{p95:>9.3f}{mx:>9.3f}')

    print()
    print(f'Trust pipeline benchmark — {n_agents} agents x '
          f'{n_objects} objects, {n_frames} timed frames')
    if not times['decode']:
        print('NOTE: sdsm_interfaces unavailable — decode stage skipped.')

    # --- Table 1: Agent process (flush_frame) ---
    print()
    print(f'Agent process  (runs in flush_frame, budget = {budget:.0f} ms)')
    print(header)
    print(sep)
    show('decode SDSM messages',              'decode')
    show('kinematic checks (total across agents)',       'kinematic checks')
    show('matching: MS-PSF phase-1 fusion',    'mspsf phase-1 fusion')
    show('bucket derivation + support',        'buckets+support')
    show('attribute checks',                   'attribute checks')
    show('consistency scoring + ledger',       'consistency+ledger')
    show('reputation update + gate',           'reputation+gate')
    show('DB record + flush (every frame)',    'db record+flush')
    print(sep)
    print(f'{"END-TO-END":<38}{e2e_mean:>9.3f}{e2e_med:>11.3f}{"":>9}{e2e_max:>9.3f}')
    print()
    status = 'OVER budget' if e2e_mean > budget else 'within budget'
    print(f'budget {budget:.0f} ms: {status}  '
          f'(mean {e2e_mean:.1f} ms, max {e2e_max:.1f} ms)')

    # --- Table 2: Tracker node (separate ROS process) ---
    print()
    print('Tracker node  (separate ROS process, publishes TrackUpdate independently)')
    print(header)
    print(sep)
    show('sort tracking (total across agents)', 'sort tracking')
    print(sep)
    print('Runs concurrently with agent node; does not add to flush_frame budget.')


def print_sweep(results: list, sweep_axis: str, n_classes: int, n_frames: int,
                budget_ms: float):
    """One row per swept point, then the largest that holds the budget.

    Works for either axis: 'agents' varies agents at fixed objects, 'objects'
    varies objects at fixed agents. p95 rather than mean decides the verdict:
    the flush timer has no queue to absorb a slow frame, so a stage that exceeds
    the window one frame in ten is already dropping the window, however good its
    average looks.
    """
    swept = 'objects' if sweep_axis == 'objects' else 'agents'
    fixed = (f'{results[0][0]} agents' if sweep_axis == 'objects'
             else f'{results[0][1]} objects')
    xs = [(no if sweep_axis == 'objects' else na) for na, no, _ in results]

    print()
    print(f'{swept.capitalize()}-count sweep — {fixed} x {n_classes} classes, '
          f'{n_frames} timed frames, budget {budget_ms:.0f} ms')
    header = (f'{swept:>8}{"mean ms":>10}{"median ms":>11}{"p95 ms":>9}'
              f'{"max ms":>9}  verdict')
    print(header)
    print('-' * len(header))

    n_max = None
    for x, (_, _, e2e) in zip(xs, results):
        series = sorted(e2e)
        mean, med = statistics.mean(series), statistics.median(series)
        p95, mx = _p95(series), max(series)
        ok = p95 < budget_ms
        if ok:
            n_max = x
        print(f'{x:>8}{mean:>10.3f}{med:>11.3f}{p95:>9.3f}{mx:>9.3f}'
              f'  {"OK" if ok else "OVER"}')

    print('-' * len(header))
    if n_max is None:
        print(f'N_max: none — even {xs[0]} {swept} exceeds the budget at p95.')
    elif n_max == xs[-1]:
        print(f'N_max: >= {n_max} {swept} — the sweep never reached the budget; '
              f'extend the sweep to find the limit.')
    else:
        print(f'N_max: {n_max} {swept} sustain {1000.0 / budget_ms:.0f} Hz at p95.')


def dump_samples(path: str, results: list, stage_times: list, n_classes: int,
                 flush_hz: float, budget_ms: float, n_frames: int, warmup: int,
                 seed: int, fusion_variant: str, sweep_axis: str, machine: dict):
    """Write the raw per-frame series consumed by tools/latency_model and
    tools/fusion_ab.

    Raw series rather than summary statistics: the model composes this with a
    batching term and a local-stage series, and percentiles of a sum cannot be
    recovered from percentiles of its parts. Each run carries both agents and
    objects, and the dump names its sweep axis and machine, so a consumer can
    join runs across dumps without being told how they were produced.
    """
    payload = {
        'kind': 'global_stage_samples',
        'version': 1,
        # Representative only; the authoritative per-run values are in 'runs'.
        'objects_per_agent': results[0][1],
        'classes': n_classes,
        'flush_hz': flush_hz,
        'budget_ms': budget_ms,
        'frames': n_frames,
        'warmup': warmup,
        'seed': seed,
        # Which MS-PSF hot path ran: 'modified' (shipped, vectorized) or
        # 'reference' (pre-optimization loops). tools/fusion_ab compares two
        # dumps that must differ only in this field.
        'fusion_variant': fusion_variant,
        # 'agents' (agents vary, objects fixed) or 'objects' (the reverse).
        'sweep_axis': sweep_axis,
        # Identity of the box this ran on (label 1/2, name, gpu, cpu, ...).
        'machine': machine,
        # Which stages e2e_ms sums. 'stages' also carries 'sort tracking',
        # which is deliberately NOT part of it: SORT runs in the tracker node,
        # concurrently, so adding it would double-count against this budget.
        'e2e_stages': _AGENT_STAGES,
        'runs': [
            {
                'agents': n_agents,
                'objects': n_objects,
                'e2e_ms': e2e,
                'stages': {
                    name: [v * 1000.0 for v in samples]
                    for name, samples in times.items() if samples
                },
            }
            for (n_agents, n_objects, e2e), times in zip(results, stage_times)
        ],
    }
    with open(path, 'w') as fh:
        json.dump(payload, fh, indent=2)
    print(f'\nwrote {path}  ({len(results)} run(s), sweep={sweep_axis}, '
          f'machine {machine["label"]})')


def main():
    """Parse CLI args and run."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument('--agents', type=int, default=20,
                    help='number of other agents (senders)')
    ap.add_argument('--agents-sweep', type=int, nargs='+', metavar='N',
                    help='sweep these agent counts (objects fixed at --objects); '
                         'prints one row each plus N_max')
    ap.add_argument('--objects-sweep', type=int, nargs='+', metavar='K',
                    help='sweep these object counts (agents fixed at --agents) '
                         'instead of --agents-sweep')
    ap.add_argument('--objects', type=int, default=_DEFAULT_MSG_OBJECTS,
                    help=f'objects each agent reports (the decode stage caps '
                         f'at the SdsmPayload capacity, {_MAX_MSG_OBJECTS})')
    ap.add_argument('--classes', type=int, default=4,
                    help='distinct object classes; drives the fusion kappa term')
    ap.add_argument('--flush-hz', type=float, default=20.0,
                    help='target flush rate; sets the per-frame budget')
    ap.add_argument('--frames', type=int, default=50, help='timed frames')
    ap.add_argument('--warmup', type=int, default=3,
                    help='untimed warm-up frames (JIT, caches)')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--fusion', choices=('modified', 'reference'), default='modified',
                    help="which MS-PSF hot path to time: 'modified' (shipped, "
                         "vectorized) or 'reference' (pre-optimization loops from "
                         "mmcooper_fuse/old/). Run both to A/B the speed "
                         'rewrites; see tools/fusion_ab.')
    ap.add_argument('--machine-label', default='1',
                    help="machine id recorded in the dump: '1' (this machine) or "
                         "'2' (NVIDIA Thor 4); tools/fusion_ab overlays by machine")
    ap.add_argument('--machine-name',
                    help='override the display name for --machine-label')
    ap.add_argument('--machine-gpu',
                    help='GPU string to record when nvidia-smi cannot see it (e.g. '
                         'inside the emulated container); else auto-detected')
    ap.add_argument('--dump-samples', metavar='PATH',
                    help='write raw per-frame samples as JSON for the latency model')
    args = ap.parse_args()

    if args.classes < 1:
        ap.error('--classes must be >= 1')
    if args.flush_hz <= 0.0:
        ap.error('--flush-hz must be > 0')
    if args.agents_sweep and args.objects_sweep:
        ap.error('use --agents-sweep or --objects-sweep, not both')
    budget_ms = 1000.0 / args.flush_hz

    # Swap the shipped mspsf for its pre-optimization copy before any frame runs.
    # Redirecting the module global is enough: fuse_detections resolves mspsf by
    # name at call time, and mspsf_old calls the old compute_self_iou_mat itself.
    if args.fusion == 'reference':
        from global_trust_perception.mmcooper_fuse.old import mspsf_old
        from global_trust_perception.mmcooper_fuse import fusion as _fusion
        _fusion.mspsf = mspsf_old
    print(f'fusion variant: {args.fusion}')

    # Build the sweep as a list of (agents, objects) points, so agents-sweep and
    # objects-sweep share one driver loop and one dump shape.
    if args.objects_sweep:
        sweep_axis = 'objects'
        swept = sorted(set(args.objects_sweep))
        if any(k < 1 for k in swept):
            ap.error('object counts must be >= 1')
        points = [(args.agents, k) for k in swept]
    else:
        sweep_axis = 'agents'
        swept = sorted(set(args.agents_sweep or [args.agents]))
        if any(n < 1 for n in swept):
            ap.error('agent counts must be >= 1')
        points = [(n, args.objects) for n in swept]
    is_sweep = bool(args.agents_sweep or args.objects_sweep)

    results, stage_times = [], []
    for n_agents, n_objects in points:
        # Same seed per point so runs differ only by the swept axis, not the scene.
        times = run(n_agents, n_objects, args.classes, args.frames,
                    args.warmup, args.seed)
        results.append((n_agents, n_objects, e2e_per_frame_ms(times)))
        stage_times.append(times)

    if is_sweep:
        print_sweep(results, sweep_axis, args.classes, args.frames, budget_ms)
    else:
        print_table(stage_times[0], points[0][0], points[0][1], args.frames,
                    budget_ms)

    mi = machine_info(args.machine_label, args.machine_name, args.machine_gpu)
    print(f'machine {mi["label"]} ({mi["name"]}): gpu={mi["gpu"]}, cpu={mi["cpu"]}')

    if args.dump_samples:
        dump_samples(args.dump_samples, results, stage_times, args.classes,
                     args.flush_hz, budget_ms, args.frames, args.warmup,
                     args.seed, args.fusion, sweep_axis, mi)


if __name__ == '__main__':
    main()
