#!/usr/bin/env bash
# Delete all results and re-run Periodic, EventTriggered, Greedy (seeds 0,1,2). No ROS/Docker.
# Run in your Terminal: ./run_all_non_ros.sh   (or: bash run_all_non_ros.sh)

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# OMNeT++ (adjust if your path is different)
export OMNETPP_ROOT="${OMNETPP_ROOT:-$SCRIPT_DIR/../sim_workspace/omnetpp-6.0.3}"
export PATH="$OMNETPP_ROOT/bin:$PATH"

echo "Working directory: $(pwd)"
echo "OMNeT++: $OMNETPP_ROOT"

# 1) Delete all current results
echo "Deleting all results..."
rm -rf results simulations/results
mkdir -p results simulations/results

# 2) Create folders for each algorithm (script will fill seed0, seed1, seed2 when copying)
mkdir -p results/Periodic results/EventTriggered results/Greedy
echo "Result folders: results/Periodic, results/EventTriggered, results/Greedy"

# 3) Run each algorithm (with SEEDS=[0]: 1 run each = 3 runs total; no GreedyROS)
echo "Running Periodic (seed 0)..."
python3 run_experiments.py --algorithm Periodic 2>&1 | tee -a results/experiment_log.txt

echo "Running EventTriggered (seed 0)..."
python3 run_experiments.py --algorithm EventTriggered 2>&1 | tee -a results/experiment_log.txt

echo "Running Greedy (seed 0)..."
python3 run_experiments.py --algorithm Greedy 2>&1 | tee -a results/experiment_log.txt

echo "Done. Results in results/{Periodic,EventTriggered,Greedy}/seed0/"
