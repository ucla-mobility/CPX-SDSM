#!/usr/bin/env python3
"""A/B the trust pipeline with vs without the MS-PSF speed rewrites, across
agent- and object-count sweeps and across machines.

Consumes any number of `global_stage_samples` dumps from benchmark_stages.py.
Each dump self-describes its fusion variant (reference|modified), its sweep axis
(agents|objects) and the machine it ran on, so this tool just groups them and
composes the comparison --- prints a per-machine, per-axis delta report and,
with --viz-data, writes the JS the committed visualizations/fusion_ab.html loads.

    # per machine, run four sweeps (2 variants x 2 axes); on the Thor pass
    # --machine-label 2 (nvidia-smi fills the GPU); in the emulated container
    # pass --machine-gpu to record the host's real one.
    python3 .../benchmark_stages.py --fusion reference --agents-sweep 2 5 10 \
        --objects 256 --frames 30 --machine-label 1 --dump-samples ref_ag.json
    python3 .../benchmark_stages.py --fusion modified  --agents-sweep 2 5 10 \
        --objects 256 --frames 30 --machine-label 1 --dump-samples mod_ag.json
    python3 .../benchmark_stages.py --fusion reference --objects-sweep 16 64 256 \
        --agents 2 --frames 30 --machine-label 1 --dump-samples ref_ob.json
    python3 .../benchmark_stages.py --fusion modified  --objects-sweep 16 64 256 \
        --agents 2 --frames 30 --machine-label 1 --dump-samples mod_ob.json

    python3 tools/fusion_ab/compare_stages.py --dumps *.json \
        --viz-data .../visualizations/fusion_ab_data.js --table-agents 2 10

Like tools/latency_model, this depends only on the dump's JSON shape --- no ROS,
no import of either package, stdlib only --- so it runs anywhere.
"""

import argparse
import json
import statistics
import sys

# The one fusion stage the speed rewrites actually touch; named so the chart and
# the "where the delta lives" summary can single it out.
_FUSION_STAGE = 'mspsf phase-1 fusion'

# Dump stage keys -> the benchmark's own descriptive labels (from
# benchmark_stages.print_table), so the page reads like the terminal table and
# the matching stage is named as such. Unknown keys fall back to the raw key.
_STAGE_LABELS = {
    'decode': 'decode SDSM messages',
    'kinematic checks': 'kinematic checks',
    'mspsf phase-1 fusion': 'matching: MS-PSF phase-1 fusion',
    'buckets+support': 'bucket derivation + support',
    'attribute checks': 'attribute checks',
    'consistency+ledger': 'consistency scoring + ledger',
    'reputation+gate': 'reputation update + gate',
    'db record+flush': 'DB record + flush',
    'sort tracking': 'SORT tracking (concurrent)',
}


# ---------------------------------------------------------------------------
# Stats
# ---------------------------------------------------------------------------

def _pct(sorted_ms, q):
    """q-th percentile (0..1), nearest-rank --- the convention benchmark_stages
    uses for p95, so a percentile quoted here matches one quoted there."""
    return sorted_ms[min(len(sorted_ms) - 1, int(round(q * (len(sorted_ms) - 1))))]


def _stats(series):
    """(mean, median, p95) for the chart lines, or None if empty."""
    if not series:
        return None
    s = sorted(series)
    return statistics.mean(s), statistics.median(s), _pct(s, 0.95)


def _full_stats(series):
    """{mean, p25, median, p75, max} ms for a table cell group, or None. median
    stays statistics.median (matching the terminal report); quartiles are
    nearest-rank via _pct, consistent with how p95 is taken elsewhere."""
    if not series:
        return None
    s = sorted(series)
    return {'mean': statistics.mean(s), 'p25': _pct(s, 0.25),
            'median': statistics.median(s), 'p75': _pct(s, 0.75), 'max': s[-1]}


def _round(x, n=4):
    return round(x, n) if x is not None else None


# ---------------------------------------------------------------------------
# Loading and grouping
# ---------------------------------------------------------------------------

def _load(path):
    """Read a dump and normalise the fields this tool groups on (variant, sweep
    axis, machine), tolerating dumps predating those fields."""
    with open(path) as fh:
        payload = json.load(fh)
    if payload.get('kind') != 'global_stage_samples':
        sys.exit(f'{path}: not a global_stage_samples dump (kind='
                 f'{payload.get("kind")!r})')
    payload.setdefault('sweep_axis', 'agents')
    if payload.get('fusion_variant') not in ('reference', 'modified'):
        sys.exit(f'{path}: fusion_variant is {payload.get("fusion_variant")!r}; '
                 're-run benchmark_stages.py with --fusion.')
    payload.setdefault('machine', {'label': '?', 'name': 'unknown machine'})
    payload['_path'] = path
    return payload


def _axis_key(payload):
    """The per-run field the sweep varies: 'objects' or 'agents'."""
    return 'objects' if payload['sweep_axis'] == 'objects' else 'agents'


def _swept(payload):
    """Sorted swept values for a dump (agent counts, or object counts)."""
    key = _axis_key(payload)
    return sorted(run[key] for run in payload['runs'])


def _runs_by_x(payload):
    """swept value -> run dict, so dumps join on the swept axis."""
    key = _axis_key(payload)
    return {run[key]: run for run in payload['runs']}


def _fixed_label(payload):
    """The non-swept dimension, constant across the sweep, for captions."""
    run0 = payload['runs'][0]
    if payload['sweep_axis'] == 'objects':
        return f'{run0["agents"]} agents'
    return f'{run0.get("objects", payload.get("objects_per_agent"))} objects'


def _stage_series(run, stage):
    """The per-frame series for a stage, or the e2e total for 'END-TO-END'."""
    return run['e2e_ms'] if stage == 'END-TO-END' else run['stages'].get(stage, [])


def _stage_order(payload):
    """END-TO-END, then the stages that sum into it, then concurrent extras."""
    e2e_stages = list(payload['e2e_stages'])
    present = set()
    for run in payload['runs']:
        present |= set(run['stages'])
    concurrent = [s for s in present if s not in e2e_stages]
    return ['END-TO-END'] + e2e_stages + concurrent, concurrent


# ---------------------------------------------------------------------------
# Terminal report (per machine, per axis)
# ---------------------------------------------------------------------------

def _fmt_delta(ref_v, mod_v):
    """'-3.21 ms (-18.4%, 1.23x)' for a reference->modified change, or '--'."""
    if ref_v is None or mod_v is None:
        return '--'
    d = mod_v - ref_v
    pct = (d / ref_v * 100.0) if ref_v else float('nan')
    speedup = (ref_v / mod_v) if mod_v else float('inf')
    return f'{d:+.3f} ms ({pct:+.1f}%, {speedup:.2f}x)'


def print_report(ref, mod, machine_name):
    """One block per swept point: END-TO-END then each stage, ref vs mod."""
    axis = ref['sweep_axis']
    unit = 'objects' if axis == 'objects' else 'agents'
    rb, mb = _runs_by_x(ref), _runs_by_x(mod)
    xs = sorted(set(rb) & set(mb))
    if not xs:
        return
    order, _ = _stage_order(ref)
    print()
    print(f'=== machine {ref["machine"]["label"]} ({machine_name}) --- {unit} sweep '
          f'({_fixed_label(ref)}, {ref["classes"]} classes, {ref["frames"]} frames, '
          f'seed {ref["seed"]}) ===')
    print('reference = pre-optimization loops, modified = shipped vectorized; '
          'delta is reference -> modified.')
    for x in xs:
        rr, mr = rb[x], mb[x]
        print(f'\n--- {x} {unit} ---')
        header = (f'{"stage":<28}{"ref med":>10}{"mod med":>10}  delta (median)')
        print(header)
        print('-' * len(header))
        for stage in order:
            rs = _stats(_stage_series(rr, stage))
            ms = _stats(_stage_series(mr, stage))
            r_med = rs[1] if rs else None
            m_med = ms[1] if ms else None
            label = ('  ' if stage != 'END-TO-END' else '') + stage
            rc = f'{r_med:>10.3f}' if r_med is not None else f'{"--":>10}'
            mc = f'{m_med:>10.3f}' if m_med is not None else f'{"--":>10}'
            print(f'{label:<28}{rc}{mc}  {_fmt_delta(r_med, m_med)}')


# ---------------------------------------------------------------------------
# Viz data
# ---------------------------------------------------------------------------

def _axis_block(groups, axis):
    """Chart block for one sweep axis: x values, and median/p95 lines per
    machine per variant, for the end-to-end and fusion panels."""
    sub = {(m, v): p for (m, ax, v), p in groups.items() if ax == axis}
    machines = sorted({m for m, _ in sub})
    xs = sorted({x for p in sub.values() for x in _swept(p)})
    panels = [('e2e', 'End-to-end (flush_frame)'),
              ('fusion', 'Matching / fusion stage (MS-PSF phase-1)')]

    data = {}
    for key, _title in panels:
        data[key] = {}
        for m in machines:
            per_variant = {}
            for v in ('reference', 'modified'):
                p = sub.get((m, v))
                if not p:
                    continue
                rb = _runs_by_x(p)
                med, p95 = [], []
                for x in xs:
                    run = rb.get(x)
                    stage = 'END-TO-END' if key == 'e2e' else _FUSION_STAGE
                    s = _stats(_stage_series(run, stage)) if run else None
                    med.append(_round(s[1]) if s else None)
                    p95.append(_round(s[2]) if s else None)
                per_variant[v] = {'median': med, 'p95': p95}
            if per_variant:
                data[key][m] = per_variant

    return {
        'axis': axis,
        'x': xs,
        'fixed_label': _fixed_label(next(iter(sub.values()))),
        'panels': [{'key': k, 'title': t} for k, t in panels],
        'machines': machines,
        'data': data,
    }


def _combo_table(pv, x, order):
    """{variant: {stage: {5 stats}}} for one swept point of one machine."""
    out = {}
    for v, p in pv.items():
        run = _runs_by_x(p).get(x)
        stages = {}
        for stage in order:
            fs = _full_stats(_stage_series(run, stage)) if run else None
            stages[stage] = {k: _round(val) for k, val in fs.items()} if fs else None
        out[v] = stages
    return out


def _machine_tables(groups, machine, order, table_agents, table_objects):
    """List of table entries for one machine: the requested agent-sweep combos
    then the requested object-sweep combos, each with both variants."""
    entries = []
    for axis, requested in (('agents', table_agents), ('objects', table_objects)):
        if not requested:
            continue
        pv = {v: p for (m, ax, v), p in groups.items()
              if m == machine and ax == axis}
        if not pv:
            continue
        available = set().union(*[set(_swept(p)) for p in pv.values()])
        fixed = _fixed_label(next(iter(pv.values())))
        for x in requested:
            if x not in available:
                continue
            title = (f'{x} agents &times; {fixed}' if axis == 'agents'
                     else f'{fixed} &times; {x} objects')
            entries.append({'axis': axis, 'x': x, 'title': title,
                            'stages': _combo_table(pv, x, order)})
    return entries


def write_viz_data(groups, machines, path, table_agents, table_objects):
    """Write the JS data file visualizations/fusion_ab.html loads. The data file
    is the whole contract: the page never reads a dump and this never draws, the
    same split reputation_history.html / make_reputation_history_viz.py use."""
    rep = next(iter(groups.values()))            # scene params are shared
    order, concurrent = _stage_order(rep)
    labels = {'END-TO-END': 'END-TO-END'}
    labels.update({s: _STAGE_LABELS.get(s, s) for s in order if s != 'END-TO-END'})
    axes_present = sorted({ax for _, ax, _ in groups})

    data = {
        'classes': rep['classes'],
        'seed': rep['seed'],
        'frames': rep['frames'],
        'machines': machines,
        'stat_columns': ['mean', 'p25', 'median', 'p75', 'max'],
        'stage_order': order,
        'concurrent_stages': concurrent,
        'stage_labels': labels,
        'axes': [_axis_block(groups, ax) for ax in axes_present],
        'tables': {m: _machine_tables(groups, m, order, table_agents, table_objects)
                   for m in sorted(machines)},
    }

    with open(path, 'w') as fh:
        fh.write('// GENERATED by tools/fusion_ab/compare_stages.py --viz-data '
                 '-- do not hand-edit.\n')
        fh.write('// Re-run it after a new sweep to refresh visualizations/'
                 'fusion_ab.html.\n')
        fh.write('window.FUSION_AB = ')
        json.dump(data, fh, indent=1)
        fh.write(';\n')
    combos = {m: len(v) for m, v in data['tables'].items()}
    print(f'wrote {path}  (axes {axes_present}, machines {sorted(machines)}, '
          f'table combos {combos})')


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--dumps', required=True, nargs='+', metavar='PATH',
                    help='global_stage_samples dumps; each self-identifies its '
                         'variant, sweep axis and machine')
    ap.add_argument('--viz-data', metavar='PATH',
                    help='write the JS data file visualizations/fusion_ab.html '
                         'loads (window.FUSION_AB)')
    ap.add_argument('--table-agents', type=int, nargs='+', metavar='N',
                    help='agent-sweep counts to build a per-stage table for '
                         '(default: all present on the agents axis)')
    ap.add_argument('--table-objects', type=int, nargs='+', metavar='K',
                    help='object-sweep counts to build a per-stage table for '
                         '(default: none)')
    args = ap.parse_args()

    groups, machines = {}, {}
    for path in args.dumps:
        p = _load(path)
        m, ax, v = p['machine']['label'], p['sweep_axis'], p['fusion_variant']
        key = (m, ax, v)
        if key in groups:
            sys.exit(f'two dumps for machine {m}, {ax} sweep, {v} variant '
                     f'({groups[key]["_path"]} and {path}); pass one each.')
        groups[key] = p
        machines[m] = p['machine']

    # Default agent-sweep tables to every agent count present; object tables opt-in.
    table_agents = args.table_agents
    if table_agents is None:
        table_agents = sorted({x for (_, ax, _), p in groups.items()
                               if ax == 'agents' for x in _swept(p)})

    # Terminal report per (machine, axis) that has both variants.
    for m in sorted(machines):
        for ax in sorted({a for (mm, a, _) in groups if mm == m}):
            ref, mod = groups.get((m, ax, 'reference')), groups.get((m, ax, 'modified'))
            if ref and mod:
                print_report(ref, mod, machines[m].get('name', m))

    if args.viz_data:
        write_viz_data(groups, machines, args.viz_data, table_agents,
                       args.table_objects)


if __name__ == '__main__':
    main()
