# Project Status Log (SDSM + BSM-Implied Study)

## What has been completed

1. Added new dissemination controls in `RosSDSMApp`:
   - `hybridEnabled`
   - `bsmImpliedMode`
   - `useVoiObjectSelection`
   - hybrid utility/VoI/redundancy parameters
2. Implemented **HybridSDSM** decision path:
   - Utility combines self-change, object novelty, freshness, and CBR penalty.
   - Trigger reasons logged as `hybridUtility` / `hybridMaxInterval`.
3. Implemented **BSM-implied payload mode**:
   - If `bsmImpliedMode=true`, packets include no object list (`num_objects_sent=0`).
4. Added object-packing branch in sender:
   - Default distance-topK (existing behavior),
   - VoI-ranked topK (hybrid path),
   - BSM-implied zero-object mode.
5. Added new simulation configs:
   - `GreedyBSMImplied`
   - `HybridSDSM`
6. Updated `run_experiments.py` supported algorithms to include:
   - `GreedyBSMImplied`
   - `HybridSDSM`
7. Executed runs (`--sim-duration 90`) and archived outputs:
   - `results/GreedyBSMImplied/seed0/`
   - `results/HybridSDSM/seed0/`

## Current run outputs (seed0)

- `results/GreedyBSMImplied/seed0/GreedyBSMImplied-r0-summary.csv`
- `results/HybridSDSM/seed0/HybridSDSM-r0-summary.csv`
- `results/GreedyBSMImplied/seed0/GreedyBSMImplied-r0-tx.csv`
- `results/HybridSDSM/seed0/HybridSDSM-r0-tx.csv`

Quick check:
- `GreedyBSMImplied-r0-tx.csv` shows `num_objects_sent=0` as expected.
- `HybridSDSM-r0-tx.csv` shows mixed nonzero object counts with `hybridUtility` triggers.

## Commands used

```bash
export OMNETPP_ROOT="/Users/sahilpuranik/Downloads/sim_workspace/omnetpp-6.0.3"
export PATH="$OMNETPP_ROOT/bin:$PATH"

make -C src -j4
python3 run_experiments.py --algorithm GreedyBSMImplied --sim-duration 90
python3 run_experiments.py --algorithm HybridSDSM --sim-duration 90
```

## What still needs to be done

1. Run multi-seed experiments (e.g., seeds 0/1/2) for both new configs.
2. Add side-by-side baseline comparison including `Greedy` and `EventTriggered` (same duration/seed set).
3. Generate consolidated plots/tables:
   - AoI, throughput, redundancy rate, packet delivery proxy, and object count distribution.
4. If needed for paper/PI review, add strict ETSI threshold profile as a dedicated config.
5. Integrate comm-policy outputs with perception evaluation workflow (SDSM vs BSM-implied impact analysis).
