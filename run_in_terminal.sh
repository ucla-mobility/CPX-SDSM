#!/usr/bin/env bash
# Run experiments in your Terminal so the process isn't killed when Cursor closes.
# Usage: ./run_in_terminal.sh   (or: bash run_in_terminal.sh)

set -e
cd "$(dirname "$0")"
echo "Working directory: $(pwd)"
echo "Ensure Docker is running (for ROS 2 bridge). Starting in 3s..."
sleep 3
python3 run_experiments.py 2>&1 | tee results/experiment_run.log
echo "Exit code: $?"
