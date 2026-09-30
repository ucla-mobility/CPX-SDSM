# Fusion A/B — end-to-end latency with vs without the MS-PSF speed rewrites

This compares the trust pipeline `flush_frame` latency **with vs without the
MS-PSF speed rewrites**, across an **agent-count sweep** and an **object-count
sweep**, and across **machines** (this box vs the NVIDIA Thor 4). It drives
`visualizations/fusion_ab.html`.

"Speed rewrites" means exactly the two `# ==== CHANGED (vectorized) ====`
blocks — the broadcast affinity/kappa in `fusion.mspsf` and the batched GEOS
call in `geometry.compute_self_iou_mat`. The **kds-orientation feature stays on
both sides**; only the speed is toggled. The pre-optimization bodies live in
`mmcooper_fuse/old/` (shared with `scripts/make_fusion_speedup_viz.py`, which
draws the fusion-stage-only `fusion_speedup.html`), and `benchmark_stages.py
--fusion reference` swaps them in for the run.

## Regenerating `visualizations/fusion_ab.html`

For each machine: **one command** runs its four sweeps ({reference, modified} ×
{agents, objects}) into `tools/fusion_ab/dumps/`; then one compare folds every
dump into the page's data file. Dumps self-describe their variant, axis and
machine, so `compare_stages.py` just takes `--dumps *.json` and sorts it out.

**Per machine — `run_sweeps.sh <label>`** (in the `ros2_dev` container here,
natively on the Thor; it does the `source` + `PYTHONPATH` setup itself):

```bash
# this machine (label 1). The emulated container can't see the host GPU, so name
# it and pass the GPU by hand:
bash tools/fusion_ab/run_sweeps.sh 1 \
    --machine-name 'MacBook (Apple M5)' --machine-gpu 'Apple M5, 10-core GPU (Metal 4)'

# the Thor (label 2): nvidia-smi fills the GPU, name auto-defaults to "NVIDIA Thor 4"
bash tools/fusion_ab/run_sweeps.sh 2
```

Args after the label are forwarded to every run. `--frames 30` (the default the
wrapper uses) times 30 scenes per sweep point, enough for stable percentiles;
raise it for tighter tails at the cost of runtime (each reference frame at 10
agents × 256 objects is ~8–10 s, so that one point already costs ~5 min). The
container is emulated x86 (inflates numpy ~5×) — compare like-for-like machines.

### Sharing the Thor's results (git, not scp)

The Thor's dumps reach this machine through the repo, since that's the only
channel: `tools/fusion_ab/dumps/` is a tracked directory.

```
# on the Thor, after run_sweeps.sh 2:
git add tools/fusion_ab/dumps/m2_*.json && git commit -m 'Thor fusion-A/B dumps' && git push

# here:
git pull                       # brings in tools/fusion_ab/dumps/m2_*.json
```

**Compose the page** from every dump in the tracked dir (stdlib only):

```bash
python3 tools/fusion_ab/compare_stages.py --dumps tools/fusion_ab/dumps/*.json \
    --viz-data ros2/src/global_trust_perception/visualizations/fusion_ab_data.js \
    --table-agents 2 10
```

Then open `ros2/src/global_trust_perception/visualizations/fusion_ab.html` and
commit the regenerated `fusion_ab_data.js`. Compose also prints, per machine and
axis, END-TO-END and each stage for both variants with the delta (ms, %,
speedup).

### Knobs

- **`--frames N`** (on `benchmark_stages.py`) — timed scenes per point; drives
  how stable the percentiles are. 30 is the wrapper default; 50 is the benchmark
  default.
- **`--agents-sweep` / `--objects-sweep`** are the two chart x-axes; one per run
  (the wrapper does both). Every swept point is timed for both variants.
- **`--table-agents N …` / `--table-objects K …`** (on `compare_stages.py`) pick
  which swept points *also* get a full per-stage **table**, rendered once per
  machine. Charts always show the whole sweep. Default: a table for every
  agent-sweep point, none for objects. The committed page uses `--table-agents 2 10`.
- **`--machine-label` / `--machine-name` / `--machine-gpu`** stamp each dump's
  machine identity, which the page uses to color the lines and caption the
  hardware.

The data file (`fusion_ab_data.js`) carries the chart series plus, per table
combo per machine, every stage's mean / p25 / median / p75 / max for both
variants. The page draws itself and never reads a dump — same producer/page
split as `make_reputation_history_viz.py` / `reputation_history.html`.

## How the scenes are made (there is no multi-agent dataset)

`benchmark_stages.py` synthesizes each frame: objects on a 15 m grid, every
agent and ego seeing every one with 0.2 m Gaussian noise, so the correct
matching is known by construction and the 15 m/0.2 m ratio keeps it
unambiguous. Labels agree ~85% of the time (the 15% disagreement is what makes
the fusion `kappa` term do real work). SORT trackers, kinematic history, the
deferred ledger, and the reputation DB persist across frames like the live
nodes. It measures **latency**, not detection accuracy — matching *correctness*
is pinned separately in `test/test_fusion_equivalence.py`.

## Why it's split this way

Same contract as `tools/latency_model`: `benchmark_stages.py` is the only
producer and emits a JSON shape (`e2e_stages`, `stages`, `runs`,
`fusion_variant`, `sweep_axis`, `machine`); this tool only *composes* those
dumps and imports neither ROS package. Each dump self-describes its variant,
axis and machine, so adding a machine or an axis needs no change here — the tool
groups whatever it's handed. The comparison reads `e2e_stages` from the dump to
decide what sums into end-to-end, so it can't drift from what the benchmark
actually timed (SORT, for instance, is timed but deliberately outside the total).
