# veins_ros_v2v_ucla

V2V **Sensor Data Sharing (SDSM)** dissemination benchmark on a **UCLA-area SUMO network**, using **Veins** (OMNeT++ + SUMO) with optional ROS 2 bridge. The stack uses **full IEEE 802.11p PHY/MAC** (path loss, fading, interference, CSMA/CA) instead of toy drop/delay models.

**Canonical paper comparison (v2):** `Periodic`, `Greedy_v2`, and `HybridSDSM_v2` at **~400 vehicles**, **300 s**, **seed 0** (effective `num_vehicles` in `*-metadata.csv` may be slightly below the requested `-n` due to SUMO insertion dynamics). See `CLAUDE.md` for the latest headline numbers and `REWRITE_SUMMARY.md` for design provenance.

---

## Simulation stack

| Component | Role |
|-----------|------|
| **[SUMO](https://eclipse.dev/sumo/)** | Traffic: car-following, lanes, signals on the UCLA-area net. |
| **[OMNeT++](https://omnetpp.org/)** | Discrete-event core, modules, `.ini` / `.ned`. |
| **[Veins](https://veins.car2x.org/)** | TraCI bridge, `Mac1609_4`, `Decider80211p`, mobility. |
| **ROS 2** (optional) | UDP bridge for live TX/RX; not needed for batch CSV runs. |

Each **TraCI step (0.1 s)** SUMO advances vehicles; Veins syncs OMNeT++ `Car` modules. **`RosSDSMApp`** decides when to send and which objects to pack, then the MAC/PHY delivers or drops packets. Successful receptions are logged to CSV under `results/` (see `.gitignore`: raw `simulations/results/` is local scratch).

---

## Simulation world (current defaults)

| Aspect | Setting |
|--------|---------|
| Playground | **1300 m × 1000 m** (`simulations/omnetpp.ini`) |
| Channel | **`config.xml`:** `SimplePathlossModel` **α = 2.75**, **Nakagami** **m = 1.5** (constM); **5.89 GHz** decider |
| TX power | **100 mW (20 dBm)** |
| Noise / sensitivity | **−98 dBm** noise floor, **−110 dBm** min power (typical Veins 802.11p) |
| Interference range | **`maxInterfDist = 1500 m`** (avoids clipping far interferers) |
| Obstacle shadowing | **Off** (no building polygons); metadata records `obstacle_shadowing,false` |
| Fleet size | Set via `run_experiments.py --num-vehicles N`; **metadata `num_vehicles`** is the **effective** count for that run |

---

## Implementation parameters (reference)

**Authoritative definitions:** `simulations/apps/RosSDSMApp.ned` (all `appl.*` defaults), `simulations/omnetpp.ini` (world + PHY + per-`[Config]` overrides), `simulations/config.xml` (analogue models), `sumo/scenario.sumo.cfg` (SUMO horizon / step).

### Experiment runner (`run_experiments.py`)

| Symbol / flag | Typical value | Notes |
|---------------|---------------|--------|
| `SIM_DURATION_S` | **300** | Default wall-clock sim horizon unless overridden |
| `DEFAULT_NUM_VEHICLES` | **400** | Canonical density; routes regenerated via `sumo/gen_ucla_routes.py` → `sumo/routes.rou.xml` |
| `--seed` / `runnumber` | e.g. **0** | Drives `*.node[*].appl.runNumber = ${runnumber}` and reproducibility |
| `SEEDS` | `0..9` | Full multi-seed sweep when you run without narrowing `--seed` |

### SUMO (`sumo/scenario.sumo.cfg`)

| Parameter | Value |
|-----------|--------|
| `begin` | 0 |
| `end` | **300** (s) |
| `step-length` | **0.1** (s) |

### OMNeT++ network & TraCI (`simulations/omnetpp.ini` `[General]`)

| Parameter | Value |
|-----------|--------|
| `network` | `networks.TwoCarsScenario` |
| `sim-time-limit` | **300 s** |
| `seed-set` | `${runnumber}` |
| `repeat` | **1** |
| `*.manager.updateInterval` | **0.1 s** |
| `*.world.playgroundSize{X,Y,Z}` | **1300 m**, **1000 m**, **50 m** |
| `*.connectionManager.maxInterfDist` | **1500 m** |
| `*.node[*].nic.phy80211p.noiseFloor` | **−98 dBm** |
| `*.node[*].nic.phy80211p.minPowerLevel` | **−110 dBm** |
| `*.node[*].nic.mac1609_4.txPower` | **100 mW** |

### PHY analogue models (`simulations/config.xml`)

| Model | Parameters |
|-------|------------|
| `SimplePathlossModel` | **α = 2.75**, thresholding on |
| `NakagamiFading` | **m = 1.5**, `constM = true` |
| `Decider80211p` | **5.89 GHz** center frequency |

### `RosSDSMApp` — mode switches (NED defaults; override in `omnetpp.ini`)

| Parameter | Default | Role |
|-----------|---------|------|
| `sendInterval` | **1 s** | Base interval; **[Config Periodic]** sets **0.1 s** |
| `periodicEnabled` | true | Periodic path vs greedy-driven |
| `greedyEnabled` | false | v1 weighted-sum greedy |
| `hybridEnabled` | false | v1 / v2 hybrid |
| `hybridVariant` | `"v1"` | **`"v2"`** for HybridSDSM_v2 |
| `greedyVariant` | `"v1"` | **`"v2"`** for Greedy_v2 (shared scheduler with Hybrid v2) |
| `bsmImpliedMode` | false | Header-only / zero-object ablation |
| `useVoiObjectSelection` | false | Legacy VoI object ranking (v1 hybrid path) |
| `rosBridgeMode` | `"off"` | **`"off"`** / `"log"` / `"live"` |

### Greedy / hybrid v1 utility (NED + `[General]` in `omnetpp.ini`)

| Parameter | Default (NED) | Typical `[General]` |
|-----------|---------------|---------------------|
| `greedyTickInterval` | 0.1 s | 0.1 s |
| `greedyAlphaPos` / `greedyAlphaSpeed` / `greedyAlphaHeading` | 1.0 / 0.5 / 0.1 | same |
| `greedyW1` / `greedyW2` / `greedyW3` / `greedyW4` | 1.0 / 0.5 / 0.3 / **0.0** | W4 often **0.5** in `[Config Greedy]` |
| `greedyThreshold` | 1.0 | same |
| `greedyMinInterval` / `greedyMaxInterval` | **0.2 s** / **5.0 s** | same |
| `congestionWindow` | 1.0 s | same |
| `cbrEwmaAlpha` | **0.3** | (NED only; not duplicated in ini) |
| `redundancyEpsilon` | **0.5** | receiver-side sender-state redundancy |
| `hybridThreshold` | 1.2 | v1 hybrid |
| `hybridRedundancyWindow` | 1 s | v1 sender object resend suppression |
| `hybridWSelf` / `hybridWTime` / `hybridWCBR` / `hybridWObj` | 1.0 / 0.4 / 0.4 / 0.6 | v1 hybrid utility weights |
| `hybridVoiDistWeight` / `hybridVoiAgeWeight` / `hybridVoiRelSpeedWeight` | 0.6 / 0.3 / 0.1 | v1 VoI terms |
| `hybridMinVoi` | 0.0 | v1 VoI floor |

### Multi-object SDSM caps & sensing window

| Parameter | `[General]` | **`[Config Greedy_v2]` / `[Config HybridSDSM_v2]`** |
|-----------|-------------|-----------------------------------------------------|
| `maxObjectsPerSdsm` | **16** | **32** |
| `K_max` | **32** (NED default) | **32** (explicit in v2 configs) |
| `detectionRange` | **300 m** | (inherits) |
| `detectionMaxAge` | **2 s** | (inherits) |

### v2 parallel-threshold scheduler + Hybrid object pipeline (NED defaults)

Used when `greedyVariant="v2"` and/or `hybridVariant="v2"`. Values below are **defaults** unless you uncomment overrides in `omnetpp.ini` under the v2 configs.

| Group | Parameter | Default |
|-------|-----------|---------|
| **Normalize** | `refSelfChange` | **10.0** |
| | `refDist` | **5.0** |
| | `T_max` | **5.0 s** (backstop / time term scale) |
| | `K_max` | **32** (object-set change denominator) |
| | `alpha_p` / `alpha_v` / `alpha_h` | **1.0** / **0.5** / **2.0** |
| **Thresholds** | `tau_self` / `tau_obj` / `tau_time` / `tau_cbr` | **0.4** / **0.4** / **0.5** / **0.6** |
| **VoI (Lyu-style)** | `w_novelty` / `w_quality` | **0.5** / **0.5** |
| | `tau_decay` / `tau_quality` | **1.5 s** / **2.0 s** |
| | `d_ref` | **50 m** |
| **Confidence (future)** | `p_highConfidence` / `tau_conf` | **0.85** / **0.5** (inactive at `conf=1.0` in code today) |
| **Spatial association** | `assocCoarseGate` | **5 m** |
| | `assocChiSquaredThreshold` | **13.28** |
| | `assocSigmaPosSquared` / `assocSigmaVelSquared` | **1.0** / **0.25** |
| | `assocPruneAge` | **3 s** |
| **Redundancy (LARM-inspired)** | `redundancyWindow` | **0.5 s** |

### Logging

| Parameter | Default | Notes |
|-----------|---------|--------|
| `csvLoggingEnabled` | true | Master CSV switch |
| `txrxLogEnabled` | false | Combined TX/RX stream |
| `rxLogEveryNth` | **1** | e.g. **2** in `[Config EventTriggered]` to thin `rx.csv` |
| `logPrefix` | `"default"` | Per-`[Config]` → `Periodic`, `Greedy_v2`, etc. |
| `runNumber` | 0 | Set to **`${runnumber}`** in ini |
| `timeseriesSampleInterval` | **1.0 s** | `*-timeseries.csv` cadence |
| Object-AoI sampler interval | **0.1 s** | Hard-coded in `RosSDSMApp` (`objectAoiSampleInterval_`); not a NED parameter |

---

## Algorithms

### Canonical v2 (primary comparison)

All three share the **same PHY/MAC, SDSM schema, and (for the two adaptive policies) the same 100 ms evaluation tick**.

| Algorithm | When to send | What goes in the SDSM (objects) |
|-----------|--------------|----------------------------------|
| **`Periodic`** | Fixed **10 Hz** (`sendInterval = 0.1 s`) | **Distance top‑K**; payload capped by **`maxObjectsPerSdsm`** (**16** under default `[General]`, unless you raise it for fair payload size) |
| **`Greedy_v2`** | **`evaluateV2Schedule`:** parallel thresholds on self-change, object-set change, time; **OR** combine; **CBR suppressor**; **backstop** at **T_max**; **min inter-send** (backstop can override) | **Distance top‑K**; **`maxObjectsPerSdsm = 32`** and **`K_max = 32`** in `[Config Greedy_v2]` |
| **`HybridSDSM_v2`** | **Same scheduler code** as `Greedy_v2` (reason prefix `hybrid_*` vs `greedy_*` in `*-triggers.csv`) | **RX spatial association** → **LARM-style redundancy window** → **Lyu-style VoI** → **top‑K**; **`maxObjectsPerSdsm = 32`**; **confidence** is **`conf = 1.0`** until a perception module supplies scores |

**Factor isolation**

- **Periodic vs Greedy_v2:** isolates **scheduler** (fixed vs adaptive).
- **Greedy_v2 vs HybridSDSM_v2:** isolates **object selection** (same scheduler).

Scheduler logic is **ETSI TS 103 324 / TS 102 687–inspired**, not a conformance certification. See comments in `src/RosSDSMApp.cc` (`evaluateV2Schedule`).

### Legacy v1 (preserved, not canonical for the current study)

`Greedy`, `EventTriggered`, `GreedyBSMImplied`, `HybridSDSM` (`hybridVariant="v1"`) remain in `simulations/omnetpp.ini` for reproducibility. They use the older weighted-sum / v1 hybrid paths. Prefer v2 configs for new results.

---

## SDSM payload (J3224-aligned)

- Schema supports up to **`K_max`** objects (default **32** in NED); **`maxObjectsPerSdsm`** in `omnetpp.ini` is the **packer cap** (**16** in `[General]`, **32** in **`[Config Greedy_v2]`** / **`[Config HybridSDSM_v2]`**). **`SDSM_PER_OBJECT_BYTES = 26`**; **`obj_measurement_time_ms`** per object.
- Objects are drawn from **`neighborInfo_`** within **`detectionRange`** (default **300 m**) and **`detectionMaxAge`** (default **2 s**).

---

## Metrics and CSVs

### Summary (`*-summary.csv`)

| Column | Meaning |
|--------|---------|
| `total_tx`, `total_rx` | Global SDSM send / successful receive counts |
| `avg_one_way_latency`, `p95_*`, `p99_*` | **`simTime − sendTimestamp`** at receiver (seconds). **Not** Kaul et al. sawtooth AoI. |
| `sender_state_redundancy_rate` | Fraction of RX where sender’s position+speed delta vs **previous** message **< epsilon** |
| `avg_throughput` | **`(total_rx / sim_duration) / num_vehicles`** — **receptions per second per vehicle** (not bytes/s) |
| `pdr_legacy_all_pairs` | **`total_rx / (total_tx × (num_vehicles−1))`** broadcast-style PDR (optimistic denominator); **distance-binned** PDR: `analysis/compute_pdr.py` |
| `avg_object_aoi`, `p95_*`, `p99_*` | v2 sampler: age of last update per tracked object; **−1** if unused (e.g. Periodic). Absolute seconds can be **inflated by stale tracks** — compare **across algorithms** or condition on distance/relevance offline. |
| `assoc_*` | Spatial-association stage totals (**HybridSDSM_v2**; **0** for Greedy_v2 / Periodic) |

### Per-vehicle (`*-vehicle-summary.csv`)

Includes `avg_latency` / `p95_latency` from **`simTime − BSM envelope timestamp`**, and `avg_one_way_latency` / `p95_one_way_latency` from **payload `sendTimestamp`** (different clocks).

### Reception log (`*-rx.csv`)

`time,receiver,sender,message_id,one_way_latency,inter_arrival,snr,rss_dbm,distance_to_sender,packet_size,cbr,num_objects,delta_state,sender_state_redundant`

### v2-only logs

- **`*-triggers.csv`:** one row per vehicle per tick (`greedy_*` / `hybrid_*` reasons).
- **`*-object-aoi.csv`:** 100 ms samples `(receiver, object_id, aoi)`.

### Post-processing (repo)

| Script | Purpose |
|--------|---------|
| `analysis/compute_aoi.py` | Kaul-style AoI from full `*-rx.csv` |
| `analysis/compute_pdr.py` | Distance-binned PDR (**fixed denominator**: per-second geometry, TX-rate weighted — see "PDR metric (fixed)") |
| `analysis/compute_redundancy.py` | Object-level redundancy (v2 + object-AoI) |
| `analysis/scale_by_distance.py` | Auxiliary scaling / distance analysis helper |
| `analysis/compute_pdr_true.py` | Activity-weighted true-PDR distance curves (baseline diagnosis) |
| `analysis/compute_final_truepdr.py` | Per-run scalar true PDR (Σrx / Σtx×in-range neighbours) |
| `analysis/compute_final_summary.py` | Aggregate table for the 6-group final experiment |
| `analysis/compute_pdr_seedvar.py` | Per-distance mean/var/std/min/max across the 20 seeds |
| `analysis/compute_pdr_n10fine.py` | Low-density correction: 0.1 s position interpolation, PDR ≤ 1 |
| `analysis/compute_pdr_sweep.py` | TX power / MCS sweep post-processing |
| `analysis/compute_pdr_combo_seeds.py` | Scheduler × power combo with per-seed error bars |
| `analysis/compute_pdr_valid.py` | Density / α / protocol validation post-processing |
| `analysis/compute_pdr_line.py` | Static straight-line density experiment post-processing |

---

## Limitations (disclose in papers/talks)

1. **No urban obstacle shadowing** — LOS-style links with fading only.
2. **Hybrid confidence** not driven by a real detector (**`conf = 1.0`**).
3. **Association** uses **greedy** one-to-one matching after Mahalanobis gating, not an optimal assignment solver.
4. **ETSI-inspired** scheduler/suppressor — not a standards compliance claim.
5. **`num_vehicles`** may be **< requested N**; always use metadata for fair normalization.
6. **Multi-seed support**: `simulations/omnetpp_final.ini` provides 6 groups ×
   20 seeds (`repeat = 20`) with statistics via `analysis/compute_pdr_seedvar.py`;
   the canonical single-seed comparison above remains seed 0. Note that `repeat`
   varies the fading/MAC RNG only — SUMO routes are fixed per scenario file.

---

## Sources (design + standards)

- **SAE J3224** — SDSM structure.
- **SAE J2945/1** — periodic safety messaging context (10 Hz baseline).
- **ETSI TS 103 324** — parallel-threshold style motivation (scheduler).
- **ETSI TS 102 687** — DCC / CBR motivation.
- **S. Kaul, R. Yates, M. Gruteser** — Age of Information (for `compute_aoi.py` definition).
- **T. Thandavarayan et al., JNCA 2023** — LARM (redundancy gate inspiration).
- **X. Lyu et al., IEEE VNC 2025** — VoI-style object ranking.
- **C. Sommer et al.** — adaptive beaconing / self-change literature (v1 Greedy lineage).

ROS message alignment: [ucla-mobility/CPX-SDSM](https://github.com/ucla-mobility/CPX-SDSM).

---

## Setup

**Prerequisites:** OMNeT++ 6.0+, Veins 5.2+, SUMO 1.8+. Set `*.manager.commandLine` in `simulations/omnetpp.ini` to your `sumo` binary.

```bash
source <omnetpp-install>/setenv
cd src && make -j$(nproc)
```

### Windows / OMNeT++ 5.6.2 (MinGW64) build

The project also builds on **OMNeT++ 5.6.2 with the bundled MinGW64 toolchain**
(validated on Windows 11 with Veins 5.2 and SUMO 1.8.0). `src/makefrag` adds the
two required flags (`-std=c++17`, `-lws2_32`); it is picked up automatically by
`opp_makemake`, and is a no-op for the 6.x build:

```bash
source <omnetpp-5.6.2>/setenv
cd src && opp_makemake -f --deep -O out -KVEINS_PROJ=<veins-5.2> -DVEINS_IMPORT \
  -I<veins-5.2>/src -L<veins-5.2>/out/clang-release/src -lveins
make -j8
```

Run batches with the `run_*.ps1` PowerShell runners (see the PDR investigation
section below). They locate the toolchain via the `OMNETPP_ROOT` / `VEINS_ROOT` /
`SUMO_ROOT` environment variables, with in-file defaults you can edit instead.
Two Windows gotchas: run the simulation binary as a native process from
`simulations/` (not through an MSYS shell), and make sure no stale `libveins.dll`
sits next to the executable.

---

## Running

### Canonical v2 (example)

```bash
cd /path/to/veins_ros_v2v_ucla
python3 run_experiments.py -a Periodic      -s 0 -n 400 --skip-probe
python3 run_experiments.py -a Greedy_v2     -s 0 -n 400 --skip-probe
python3 run_experiments.py -a HybridSDSM_v2 -s 0 -n 400 --skip-probe
```

Artifacts: **`results/n400/<Algorithm>/seed0/`** (and optional symlinks under `results/n400_seed0_bundle/` if you use them).

### Legacy v1 examples

```bash
python3 run_experiments.py --algorithm Greedy --sim-duration 90
python3 run_experiments.py --algorithm HybridSDSM --sim-duration 90
```

### Shrink huge RX CSV (optional)

```bash
python3 scripts/shrink_rx_csv.py results/n400/Periodic/seed0/Periodic-r0-rx.csv --every 4 --decimals 3
```

---

## PDR investigation experiments

A set of reproducible experiments investigating **why broadcast PDR is low** in
this benchmark. Two independent loss mechanisms were identified:

1. **Channel congestion** — at high density the 10 Hz Periodic baseline saturates
   the channel (CBR ≈ 0.43 at ~400 vehicles) and MAC collisions dominate;
   adaptive scheduling (HybridSDSM_v2) recovers most of this loss.
2. **Link-budget wall at ~200 m** — with `config.xml` (α = 2.75, 20 dBm TX,
   ≈ −92 dBm decode floor) the mean RSS crosses the decode floor near 200 m, so
   PDR collapses there regardless of congestion. Raising TX power or lowering α
   (see `config_alpha2.xml`) moves this wall outward as predicted by the
   path-loss equation.

Each experiment is a self-contained `.ini` (all `extends` the base configs in
`simulations/omnetpp.ini`) plus a PowerShell batch runner and an analysis script:

| Experiment | Config (`simulations/`) | Runner | Analysis (`analysis/`) |
|---|---|---|---|
| Baseline 3 strategies (399 veh, 300 s) | `omnetpp.ini` | `run_all3.ps1` | `compute_pdr_true.py` |
| TX power / MCS sweep | `omnetpp_sweep.ini` | `run_sweep.ps1` | `compute_pdr_sweep.py` |
| Scheduler × power combo (multi-seed) | `omnetpp_combo.ini` | `run_combo_seeds.ps1` | `compute_pdr_combo_seeds.py` |
| Density / α / protocol validation | `omnetpp_valid.ini` | `run_valid.ps1` | `compute_pdr_valid.py` |
| α × density 2×2 grid | `omnetpp_grid.ini` | `run_grid.ps1` | `compute_pdr_valid.py` |
| Static straight-line density (idealized) | `omnetpp_line.ini` | `run_line.ps1`, `run_line2.ps1` | `compute_pdr_line.py` |
| **Final summary: 6 groups × 20 seeds** (10/150/400 veh × Periodic/Hybrid, α = 2.0) | `omnetpp_final.ini` | `run_final.ps1` | `compute_pdr_seedvar.py`, `compute_final_truepdr.py`, `compute_final_summary.py`, `compute_pdr_n10fine.py` |

Multi-seed runs use OMNeT++'s `repeat = 20` (`seed-set = ${runnumber}`), so a
single config covers seeds 0–19:

```bash
cd simulations
../src/veins_ros_v2v_ucla -u Cmdenv -c per_n400 -r 0..19 -f omnetpp_final.ini
```

Notes:

- The extra `.ini` files set `*.manager.commandLine` to plain `sumo`; either keep
  SUMO's `bin/` on `PATH` or edit the line, exactly as with the base
  `omnetpp.ini`.
- Fast-departure route sets `sumo/routes_d{10,150,400}.rou.xml` insert all
  vehicles within 0–20 s. The stock `routes.rou.xml` spreads departures over
  ~160 s, so short high-density runs silently plateau at ~150 vehicles — use the
  `scenario_d*.sumo.cfg` scenarios for density studies.
- `sumo/line/` holds the idealized scenario (2 km straight road, stationary
  equally-spaced vehicles, 8 density levels) used to separate sampling noise at
  low vehicle counts from real protocol effects.

### PDR metric (fixed)

`analysis/compute_pdr.py` previously undercounted the opportunity denominator
(raw-float timestamp grouping fragmented the geometry, and each neighbour pair
counted one opportunity per second regardless of the actual send rate), which
could inflate PDR past 1. It now aggregates geometry per rounded second and
weights each sender's neighbour counts by `tx_count_since_last`, giving a true
PDR in [0, 1]:

```
PDR(bin) = received(bin) / Σ_seconds Σ_senders tx_that_second × receivers_in_bin
```

`compute_pdr_seedvar.py` (per-distance mean/variance across seeds) and
`compute_pdr_n10fine.py` (0.1 s position interpolation for sparse, fast-moving
fleets) implement the same definition for the multi-seed studies.

---

## ROS 2 bridge (optional)

```bash
cd ros2_ws && colcon build && source install/setup.bash
ros2 run veins_ros_bridge udp_bridge_node --ros-args -p udp_port:=50010
```

Enable with `rosBridgeMode = "live"` in `omnetpp.ini` when needed.

---

## License

See upstream [ucla-mobility/CPX-SDSM](https://github.com/ucla-mobility/CPX-SDSM).
