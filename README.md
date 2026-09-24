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

### `trust_sdsm_v2` — imperfection injection (NED defaults; all off unless set)

Used by the `trust_*` configs in `simulations/omnetpp_trust.ini`. With every value at its default the data is the original clean ground truth.

| Parameter | Default | Role |
|-----------|---------|------|
| `positionNoiseStdDev` | **0 m** | Gaussian jitter (σ) on each reported object position; applied to the serialized report only, never the stored ground truth |
| `attackerFraction` | **0.0** | Fraction of vehicles that lie; deterministic (`nodeIndex % round(1/fraction) == 0`) |
| `attackType` | `"phantom"` | **`"phantom"`** fabricates an extra object (id `900000+nodeIndex`) at a fixed offset; **`"spoof"`** teleports one locked-on real neighbor by `spoofJumpDistance` |
| `spoofJumpDistance` | **150 m** | Teleport distance for `attackType="spoof"` |
| `attackerPureMode` | false | Attacker sends **only** its lie (no honest objects), so mixed-in honest traffic cannot inflate its reputation |
| `phantomOffsetDistance` | **0 m** | >0: fixed distance of the phantom from its sender (default: random 20-80 m); large values hide it from other vehicles |
| `hiddenObjects` | **0** | Number of real, static, **non-broadcasting** obstacles (think parked vehicles, pedestrians) only sensed by vehicles within `sensorRange`. Vehicles with `nodeIndex % hiddenHostPeriod == 0` each create one near where they first send |
| `hiddenHostPeriod` / `sensorRange` | **2** / **50 m** | Which vehicles create a hidden object / how close a vehicle must be to sense it |

### `trust_sdsm_v2` — engine constants (`sdsm_trust_perception`)

| Constant | Value | Role |
|----------|-------|------|
| `REPUTATION_DEFAULT` | **0.5** | Starting reputation for every sender; decay baseline |
| `DELTA` | **0.10** | Max reputation change per frame: `R_new = R_old + DELTA·S_frame`, `S_frame = (C−I)/N_total` |
| `DECAY_RATE` | **0.003** | Drift toward baseline per frame with nothing to score |
| `TAU_MIN` / `TAU_MAX` / `GAMMA` | **0.30** / **0.90** / **2.0** | Dynamic gate: `τ(R) = 0.90 − 0.60·R²` |
| `HIGH_TRUST` | **0.95** | At/above this, all of a sender's objects are admitted; below, only corroborated ones |
| `SUPPORT_THRESHOLD_THETA` | **1.0** | Reputation mass needed to corroborate a report |
| `T_DEADLINE_S` | **15 s** | Grace window before an uncorroborated report is back-charged |
| `flush_interval_s` | **0.5 s** | Verdict bucket width (launch parameter) |
| `OPPORTUNITY_RANGE_M` / `NEAR_REPORT_M` | **150 m** / **10 m** | Probabilistic admission: who could have witnessed an object / what counts as having reported it |
| `P_ADMIT` / `KDS_IMPLAUSIBLE` | **0.4** / **0.3** | Probabilistic admission: minimum belief to admit / kinematic score below which motion is implausible |
| `LIAR_EWMA_ALPHA` / `LIAR_FREQ` | **0.1** / **0.5** | Probabilistic admission: weight of the newest frame / false-frequency at which a sender is filtered out |
| `contradiction_persist` / `opportunity_range_m` | **1 flush** / **150 m** | Strict + unverified / probabilistic: a contradiction must persist this many flushes before it counts (`--persist`); how close a witness must be to have been able to see an object (`--opportunity-range`; set it to the sensors' real range) |

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

### Trust layer (`trust_sdsm_v2`, receiver-side)

Unlike the three policies above, this is **not** a dissemination policy: it changes nothing about *what* or *when* a vehicle sends. It is a **receiver-side judge** that decides which senders' SDSMs to believe. Each scenario extends `Periodic` (10 Hz, n10 density) so the radio side is identical, and only the data quality differs.

| Config (`omnetpp_trust.ini`) | Data quality | What it tests |
|------------------------------|--------------|---------------|
| **`trust_clean_n10`** | Clean ground truth | Control: honest senders should stay trusted |
| **`trust_noise_n10`** | `positionNoiseStdDev = 0.5 m` on every report | Honest-but-imperfect sensors should not be rejected |
| **`trust_phantom_n10`** | 20 % of vehicles fabricate a fake object | Uncorroborated fabrication should be caught |
| **`trust_spoof_n10`** | 20 % of vehicles teleport a real neighbor's position by 150 m | Kinematically impossible jumps should be caught |
| **`trust_hidden_n10`** | 5 real static obstacles only sensed within 50 m | Real objects that only some vehicles see: are they admitted? |
| **`trust_hidden_phantom_n10`** | Hidden real obstacles + 20 % phantom attackers | Can the layer separate real unique sightings from phantoms? |

`attackerPureMode` (a command-line/ini override, not a named config) additionally strips an attacker's honest traffic so its reputation reflects only the lie.

| Stage (per judging vehicle, per 0.5 s flush) | What it does |
|----------------------------------------------|--------------|
| **Gate** | `R_eff` vs the dynamic threshold `τ(R)`; a failing sender's data is withheld but it keeps being scored so it can recover |
| **Kinematic check** | Flags a tracked object whose reported position jumps implausibly from its prior track |
| **Align** | Every detection (the judge's and each sender's) is dead-reckoned to the bucket's common instant using its speed and heading, so views taken up to 0.5 s apart can be compared |
| **Cluster (MS-PSF)** | One reputation-weighted fusion (`mmcooper_fuse`) over the judge and every sender groups all reports of the same real object into a cluster; a sender's report is *matched* if the judge is in its cluster, else `other_only` |
| **Corroboration** | An `other_only` report is corroborated when the *other* senders in its cluster carry enough reputation mass (`SUPPORT_THRESHOLD_THETA`); a sender can never vouch for itself, and low-reputation colluders cannot corroborate each other |
| **Deferred ledger** | Uncorroborated reports are held for `T_DEADLINE_S`, then back-charged as incorrect (back-paid as correct if corroborated in time) |
| **Reputation** | `R_new = R_old + 0.10·(C−I)/N_total`; every sender starts at 0.5, below the gate's fixed point (~0.648), so new senders start gate-failed by design |
| **Admission** | Per object: a sender at/above `HIGH_TRUST` (0.95) has everything admitted (optimistic tier); below it only corroborated objects pass. `--high-trust 2` (engine arg `high_trust`) removes the optimistic tier |
| **Probabilistic admission** (opt-in, `--admission probabilistic`) | Replaces the tiers with a per-object belief `R · kinematic score · exp(−missed witnesses)`, where *missed witnesses* are reputation-weighted vehicles within 150 m that did not report the object (a vehicle reporting anything within 10 m does not count). An object is *contradicted* if missed weight ≥ 1 or its motion is implausible; contradicted objects are never admitted, and a sender with contradicted objects in most recent frames (EWMA ≥ 0.5) is filtered out entirely until it recovers. Objects nobody could have witnessed are not penalized. **Experimental:** it does not scale to high density (see validation) and cannot catch a phantom hidden away from other vehicles |
| **Strict + unverified** (opt-in, `--admission strict_unverified`, engine `admission='strict_unverified'`) | Strict `admitted` (matched or peer-corroborated only), plus a separate `unverified` class in `FrameStats.unverified`: objects from a gate-passing sender that nobody confirms and nobody contradicts (belief `R · kinematic score · exp(−missed witnesses)` ≥ 0.4). The consumer sees three classes: **confirmed** / **unverified** / **rejected**. A real object only one vehicle senses lands in *unverified* rather than being lost; a phantom that other vehicles could have seen but did not report is *rejected* |

**Validation (single seed, isolated reputation DBs, honest judges only).** Each cell that has two values reads *default admission → strict admission* (`--high-trust 0.95` → `--high-trust 2`, i.e. no optimistic tier: every admitted object must be corroborated). "Within 150 m" is the share of a sender's objects within 150 m of the judge that were admitted, which is what the judge can act on; objects farther from every vehicle cannot be checked by anyone yet.

*10 vehicles, 100 s*

| Scenario | Attacker reputation | Attacker rejected | Attacker's lie admitted | Honest reputation | Honest rejected | Honest objects admitted (within 150 m) |
|----------|--------------------|-------------------|-------------------------|-------------------|-----------------|----------------------------------------|
| Clean (control) | n/a | n/a | n/a | 0.99 | 7 % | 93 → 90 % |
| Noise (σ = 0.5 m) | n/a | n/a | n/a | 0.98 | 8 % | 93 → 90 % |
| Phantom (mixed) | 0.98 | 3 % | phantoms 81 → **0 %** | 0.99 | 7 % | 94 → 91 % |
| Spoof (mixed) | 0.71 | 51 % | n/a | 0.99 | 7 % | 93 → 90 % |
| Pure phantom | 0.10 | 100 % | 0 % | 0.99 | 7 % | 94 → 91 % |
| Pure spoof | 0.22 | 100 % | 0 % | 0.99 | 7 % | 94 → 90 % |

*150 vehicles, 90 s (about 30 senders per judge, 20 % attackers)*

| Scenario | Attacker reputation | Attacker rejected | Attacker's lie admitted | Honest reputation | Honest rejected | Honest objects admitted (within 150 m) |
|----------|--------------------|-------------------|-------------------------|-------------------|-----------------|----------------------------------------|
| Clean (control) | n/a | n/a | n/a | 0.95 | 7 % | 89 % (strict) |
| Phantom (mixed) | 0.94 | 7 % | phantoms 79 → **1 %** | 0.95 | 6 % | 94 → 89 % |
| Spoof (mixed) | 0.95 | 6 % | spoofed reports 0.4 % admitted (19,508 identified by comparing each report against the real sender's own broadcast position) | 0.96 | 6 % | 91 % (strict) |
| Pure phantom | 0.35 | 92 % | 2 % | 0.96 | 6 % | 91 % (strict) |
| Pure spoof | 0.43 | 100 % | 0 % | 0.95 | 6 % | 89 % (strict) |

Reading it:

- **Senders that only lie are shut out** (rejected on nearly every message). A sender that mixes a lie among honest reports stays *trusted as a sender* (its honest objects dilute the charge in `S_frame = (C−I)/N_total`), so the protection there is per object: under strict admission a mixed attacker's phantom is admitted 0-1 % of the time, versus about 80 % under default admission.
- **Cost of strict:** about 3-5 points of honest objects within 150 m. The honest objects strict still drops are almost all objects more than 150 m from every vehicle, which nothing can corroborate.
- **A phantom hidden far from every other vehicle** (10 vehicles, 400 m from its sender) is *not* stopped by `--admission probabilistic` (83 % admitted) but is by strict (0 %).
- **Real objects that only one vehicle senses are the cost of strict.** In `trust_hidden_n10` (5 real static obstacles, 10 vehicles, sensor range 50 m; 150 of 182 reports were unique sightings) strict admits **0 %** of the unique sightings and 11 % of the hidden-object reports overall, versus 68 % / 67 % under default admission and 57 % / 60 % under probabilistic. With phantom attackers added (`trust_hidden_phantom_n10`) strict still admits 0 % of phantoms but also 0 % of unique real sightings; default admits 84 % of phantoms. This scenario deliberately makes unique sightings common; this simulator's ordinary traffic (every object is a V2X-equipped vehicle) rarely produces them, so the ~90 % honest-object figures above understate the cost for real sensor-shared objects.
- **Strict + unverified recovers most of them as "unverified"** (10 vehicles, 100 s, gate-passing senders only; each cell is confirmed / unverified / rejected):

| Scenario | Real unique sightings | Phantoms | Honest vehicle objects |
|----------|-----------------------|----------|------------------------|
| `trust_hidden_n10`, strict | 0 / 0 / 100 % | n/a | 86 / 0 / 14 % |
| `trust_hidden_n10`, strict + unverified | 0 / **66** / 34 % | n/a | 86 / 10 / 4 % |
| `trust_hidden_phantom_n10`, strict | 0 / 0 / 100 % | 0 / 0 / 100 % | 80 / 0 / 20 % |
| `trust_hidden_phantom_n10`, strict + unverified | 0 / **62** / 38 % | 0 / 3 / **97 %** | 80 / 16 / 4 % |
| far-hidden phantom (400 m), strict | n/a | 0 / 0 / 100 % | 81 / 0 / 19 % |
| far-hidden phantom (400 m), strict + unverified | n/a | 0 / **87** / 13 % | 81 / 14 / 5 % |

  Confirmed sets are identical to strict (nothing new is admitted). What changes is what happens to the rest: nearly all honest objects strict dropped (13-21 %) become *unverified* (10-17 %) instead of vanishing, and about two thirds of real unique sightings do too. The price is that a phantom **hidden away from every vehicle** is also *unverified* (87 %), so the unverified class cannot separate a real unique sighting from a far-hidden phantom; that ambiguity is inherent to receiver-side checks. Phantoms placed among other vehicles are rejected (97 %). Spoofed objects (10 vehicles, ground-truth detector): 0 % admitted, 19 % unverified, 81 % rejected. The 34-38 % of real unique sightings still rejected are objects that some vehicle 60-150 m away "could have seen" by the opportunity rule (150 m) but not by this scenario's 50 m sensor range, so the opportunity range should match the sensor.
- **Reducing real objects rejected in strict + unverified mode.** Diagnosis on the all-real run (`trust_hidden_n10`): real objects were rejected because vehicles in range did not report them (100 % of rejected hidden objects, 53 % of rejected vehicle objects) or because motion looked implausible (42 % of rejected vehicle objects), and 71 % of rejected frames were blips of 3 flushes or fewer. Two options address this: `--persist N` (a contradiction must last N flushes) and `--opportunity-range M` (the distance within which a witness could have seen an object). Sweep, strict + unverified mode, 10 vehicles (this scenario's sensor range is 50 m):

| `--persist` | `--opportunity-range` | Real unique sightings rejected | Honest vehicle objects rejected | Near phantoms (conf / unv / rej) | Far phantoms (conf / unv / rej) |
|-------------|-----------------------|--------------------------------|---------------------------------|-----------------------------------|----------------------------------|
| 1 | 150 | 34 % | 3.6 % | 0 / 3 / **97 %** | 0 / 87 / 13 % |
| 3 | 150 | 26 % | 0.6 % | 0 / 7 / 93 % | 0 / 96 / 4 % |
| 3 | **100** | **4 %** | 0.7 % | 0 / 12 / **88 %** | 0 / 96 / 4 % |
| 3 | 60 | 0 % | 0.7 % | 0 / 34 / 66 % | 0 / 96 / 4 % |
| 5 | 100 | 1 % | 0.2 % | 0 / 18 / 82 % | 0 / 96 / 4 % |

  Persistence removes most honest-vehicle rejections (3.6 % → 0.6 %) at a small cost: phantoms slip from *rejected* to *unverified* (97 % → 93 %). A range closer to the sensor's real range cuts real unique sightings rejected from 34 % to 4-7 %, but a shorter range also lets more near phantoms through to *unverified* because fewer vehicles count as able to contradict them. Nothing is ever newly *confirmed*: near phantoms and spoofed objects stay 0 % confirmed in every row, though with `--persist 3` spoofed objects move from 19 % to 51 % *unverified* (0 % admitted either way). A reasonable starting point is `--persist 3 --opportunity-range` set to roughly twice the sensor range.
- **`--admission probabilistic` does not scale:** it worked at 10 vehicles but rejected 37-42 % of honest senders at 150 vehicles (many vehicles in range means some always "missed" an object), so treat it as experimental.
- Honest senders' ~6-7 % rejection is the start-up cost (every sender begins below the gate), all before the first trusted verdict.

Fixes needed to get here: (1) a sender's own reputation no longer satisfies its own corroboration requirement; (2) the deferred ledger is keyed by the reported `object_id`, because SORT track ids flapped and every flap reset the 15 s grace clock; (3) the judge's own vehicle counts as ego-known; (4) `mmcooper_fuse` clustering and peer corroboration replace `object_id` matching; (5) all detections are dead-reckoned to a common instant, and an unmatched sender object within 6 m of an unmatched judge detection is paired with it; (6) the simulator's heading (a math angle) is decoded as the compass bearing the codec assumed; (7) the phantom reports its attacker's speed and heading, and SORT uses the real flush interval.

**Factor isolation**

- **Periodic vs Greedy_v2:** isolates **scheduler** (fixed vs adaptive).
- **Greedy_v2 vs HybridSDSM_v2:** isolates **object selection** (same scheduler).

Scheduler logic is **ETSI TS 103 324 / TS 102 687–inspired**, not a conformance certification. See comments in `src/RosSDSMApp.cc` (`evaluateV2Schedule`).

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
| `analysis/compute_pdr.py` | Distance-binned PDR |
| `analysis/compute_redundancy.py` | Object-level redundancy (v2 + object-AoI) |
| `analysis/scale_by_distance.py` | Auxiliary scaling / distance analysis helper |
| `analysis/replay_trust_verdicts.py` | `trust_sdsm_v2`: offline replay of a `*-ros-events.jsonl` through the trust engine → `*-trust-verdicts.csv` |
| `analysis/compute_trusted_pdr.py` | `trust_sdsm_v2`: raw vs trusted PDR per distance bin (joins `*-rx.csv` with verdicts) |

---

## Limitations (disclose in papers/talks)

1. **No urban obstacle shadowing** — LOS-style links with fading only.
2. **Hybrid confidence** not driven by a real detector (**`conf = 1.0`**).
3. **Association** uses **greedy** one-to-one matching after Mahalanobis gating, not an optimal assignment solver.
4. **ETSI-inspired** scheduler/suppressor — not a standards compliance claim.
5. **`num_vehicles`** may be **< requested N**; always use metadata for fair normalization.
6. **Single-scenario / seed** until you publish multi-seed CIs.
7. **`trust_sdsm_v2` has no Layer 1.** Local per-object confidence is hardcoded to `1.0`; there is no simulated sensor model, so the judge rests on cross-agent checks only.
8. **`trust_sdsm_v2` clusters on aged beacon positions.** The fusion expects sub-metre agreement between views; this sim's views are dead-reckoned beacon positions (no per-object capture time on the wire), so a few percent of honest reports fail to cluster with the judge and rely on peer corroboration.
9. **Trust results are 10-vehicle, single-seed**, and not bit-reproducible across SUMO versions (1.12 vs 1.18 gave different traffic).
10. **Strict admission drops real objects that only one vehicle senses.** A real object seen by a single vehicle is indistinguishable from a fabricated one to everybody else, so strict admits neither (see `trust_hidden_n10`). Non-V2X objects such as pedestrians, cyclists, and occluded vehicles are exactly the case sensor sharing exists for. Mitigation: `--admission strict_unverified` exposes them as an "unverified" class instead of dropping them (see validation); not implemented: crediting senders whose unique sightings are later confirmed once another vehicle comes in range.

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
- **CPX-Mono `global_trust_perception`** — reputation, dynamic gate, deferred ledger (`trust_sdsm_v2`); reference copies in `reference/cpx_mono_trust/`.

ROS message alignment: [ucla-mobility/CPX-SDSM](https://github.com/ucla-mobility/CPX-SDSM).

---

## Setup

**Prerequisites:** OMNeT++ 6.0+, Veins 5.2+, SUMO 1.8+. Set `*.manager.commandLine` in `simulations/omnetpp.ini` to your `sumo` binary.

```bash
source <omnetpp-install>/setenv
cd src && make -j$(nproc)
```

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

### Trust layer scenarios (`trust_sdsm_v2`)

```bash
cd simulations
../src/veins_ros_v2v_ucla omnetpp_trust.ini -c trust_phantom_n10 -r 0 -u Cmdenv \
    -n "<your NED path, same as run_experiments.py uses>" --sim-time-limit=40s
# writes results/trust_phantom_n10-r0-ros-events.jsonl

# offline judgment (needs the built ros2_ws overlay sourced)
cd ..
python3 analysis/replay_trust_verdicts.py simulations/results/trust_phantom_n10-r0-ros-events.jsonl \
    --db-dir results/trust_dbs/phantom --out simulations/results/trust_phantom_n10-trust-verdicts.csv
```

`analysis/compute_trusted_pdr.py` then joins the `*-rx.csv` with the verdicts CSV (see its `--help`).

Use a **fresh `--db-dir` per run**: reputation persists in SQLite, so reusing a directory carries state between supposedly independent runs.

### Shrink huge RX CSV (optional)

```bash
python3 scripts/shrink_rx_csv.py results/n400/Periodic/seed0/Periodic-r0-rx.csv --every 4 --decimals 3
```

---

## ROS 2 bridge (optional)

```bash
cd ros2_ws && colcon build && source install/setup.bash
ros2 run veins_ros_bridge udp_bridge_node --ros-args -p udp_port:=50010
```

Enable with `rosBridgeMode = "live"` in `omnetpp.ini` when needed.

`udp_bridge_node` decodes each TX/RX JSON line RosSDSMApp emits into a
structured `sdsm_trust_interfaces/ReceivedSdsm` and publishes it on
`/veins/sdsm_events` (the raw text still goes to `/veins/rx_raw` too).

### SDSM trust layer (`trust_sdsm_v2`, optional second-layer check)

Algorithm description, scenarios and validation results: see **Algorithms → Trust layer (`trust_sdsm_v2`)** above. This section covers running it live over ROS 2.

`sdsm_trust_perception` is an independent per-vehicle trust judge — reputation,
kinematic-plausibility and size-agreement checks, peer corroboration, a
deferred grace ledger for objects only one sender reports — ported from
[CPX-Mono](../CPX-Mono)'s `global_trust_perception` cooperative-perception
trust pipeline and adapted to this repo's `SensorDataSharingMessage` wire
format. It runs one `TrustEngine` per simulated vehicle that shows up as an
RX receiver in the bridged event stream, so each vehicle's verdicts reflect
only what it actually received after the 802.11p channel model — not a
global oracle view.

This wire format carries no per-object local-certainty field, so unlike
CPX-Mono's two-layer design (local sensor confidence + cross-agent judgment),
this port's trust judgment rests entirely on the second layer: whether a
reported object is kinematically plausible given its prior track, whether its
reported size agrees with what the judging vehicle itself sees, and whether
peers corroborate it. See
`sdsm_trust_perception/global_trust_perception/pipeline/sdsm_codec.py`'s
module docstring for the full list of what this sim's wire format can and
cannot supply, and `global_trust_perception/pipeline/trustworthy_perception.py`'s
docstring for what was and wasn't ported from CPX-Mono. Matching and peer
corroboration use CPX-Mono's `mmcooper_fuse` spatial clustering (ported into
`global_trust_perception/mmcooper_fuse/`); the phase-2 display fusion and the
Foxglove visualizers were not ported.

```bash
cd ros2_ws && colcon build && source install/setup.bash
pip install filterpy   # no rosdep key; SORT tracking + sender-speed Kalman filters need it
ros2 launch sdsm_trust_perception sdsm_trust.launch.py
```

Per-vehicle reputation history persists to SQLite under
`ros2_ws/data/sdsm_trust_perception/historical_reputations_<node>.db`.
Verdicts publish on `/veins/trust_verdicts`
(`sdsm_trust_interfaces/TrustVerdict`) — one message per (judging vehicle,
judged sender) per flush, `trusted=false` senders included (diagnostic
channel: nothing should act on perception a judge withheld).

---

## License

See upstream [ucla-mobility/CPX-SDSM](https://github.com/ucla-mobility/CPX-SDSM).
