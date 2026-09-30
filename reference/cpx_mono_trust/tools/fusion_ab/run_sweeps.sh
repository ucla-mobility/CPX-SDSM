#!/usr/bin/env bash
# Run all four fusion-A/B sweeps for one machine in a single command, writing
# the dumps into tools/fusion_ab/dumps/ (a tracked dir, so the Thor's results
# reach the other machine by git push/pull -- see tools/fusion_ab/README.md).
#
# The 2x2 grid (reference|modified x agents|objects) is inherently four
# benchmark_stages processes: the reference swap is a process-global
# monkeypatch, and --agents-sweep / --objects-sweep are mutually exclusive. This
# just orchestrates them.
#
#   bash tools/fusion_ab/run_sweeps.sh 1 \
#       --machine-name 'MacBook (Apple M5)' --machine-gpu 'Apple M5, 10-core GPU (Metal 4)'
#   bash tools/fusion_ab/run_sweeps.sh 2        # on the Thor; nvidia-smi fills the GPU
#
# Any extra args after the label are forwarded to every benchmark run (e.g.
# --machine-name, --machine-gpu, --frames, --seed).
set -euo pipefail

LABEL="${1:?usage: run_sweeps.sh <machine-label: 1|2> [extra benchmark_stages args...]}"
shift || true

R="$(cd "$(dirname "$0")/../.." && pwd)"                       # repo root
# Prefer the workspace overlay if present, otherwise fall back to system ROS.
if [[ -r "$R/ros2/install/setup.bash" ]]; then
	source "$R/ros2/install/setup.bash"
else
	# ROS setup scripts may reference unset variables; temporarily disable nounset.
	set +u
	source /opt/ros/jazzy/setup.bash
	set -u
fi
# Both source dirs on PYTHONPATH so the source packages shadow the stale
# ros2/install copies: global_trust_tracker.SORT and mmcooper_fuse.old (the
# '--fusion reference' path) both live in source only.
export PYTHONPATH="$R/ros2/src/global_trust_perception:$R/ros2/src/global_trust_tracker:${PYTHONPATH:-}"

BS="$R/ros2/src/global_trust_perception/scripts/benchmark_stages.py"
OUT="$R/tools/fusion_ab/dumps"
mkdir -p "$OUT"
cd "$R/ros2/src/global_trust_perception/scripts"

COMMON=(--classes 4 --frames 30 --seed 0 --machine-label "$LABEL" "$@")

echo "[1/4] agents  x reference"
python3 "$BS" --fusion reference --agents-sweep 2 5 10 --objects 256 "${COMMON[@]}" --dump-samples "$OUT/m${LABEL}_ref_ag.json"
echo "[2/4] agents  x modified"
python3 "$BS" --fusion modified  --agents-sweep 2 5 10 --objects 256 "${COMMON[@]}" --dump-samples "$OUT/m${LABEL}_mod_ag.json"
echo "[3/4] objects x reference"
python3 "$BS" --fusion reference --objects-sweep 16 32 64 128 256 --agents 2 "${COMMON[@]}" --dump-samples "$OUT/m${LABEL}_ref_ob.json"
echo "[4/4] objects x modified"
python3 "$BS" --fusion modified  --objects-sweep 16 32 64 128 256 --agents 2 "${COMMON[@]}" --dump-samples "$OUT/m${LABEL}_mod_ob.json"

echo "done -> $OUT/m${LABEL}_*.json  (commit + push these to share across machines)"
