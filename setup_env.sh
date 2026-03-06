#!/usr/bin/env bash
# Source this to build and run the sim when OMNeT++/Veins are in sim_workspace:
#   source setup_env.sh
# Then: make -C src, ./run_experiment.sh, or ./run_greedy_ros_test.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
SIM_WORKSPACE="${SIM_WORKSPACE:-$SCRIPT_DIR/../sim_workspace}"
OMNET_ROOT="${OMNET_ROOT:-$SIM_WORKSPACE/omnetpp-6.0.3}"
VEINS_DIR="${VEINS_DIR:-$SIM_WORKSPACE/veins}"

if [ -d "$OMNET_ROOT/bin" ] && [ -x "$OMNET_ROOT/bin/opp_run" ]; then
    export PATH="$OMNET_ROOT/bin:$PATH"
    export OMNETPP_ROOT="$OMNET_ROOT"
fi
if [ -d "$VEINS_DIR/src/veins" ]; then
    export VEINS_DIR
fi

# Optional: point project's ../veins symlink at sim_workspace/veins if missing
if [ ! -d "$SCRIPT_DIR/../veins" ] && [ -d "$VEINS_DIR" ]; then
    echo "Note: Link ../veins to $VEINS_DIR for Makefile. Use: ln -snf $VEINS_DIR $SCRIPT_DIR/../veins"
fi
