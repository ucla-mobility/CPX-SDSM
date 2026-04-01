# veins\_ros\_v2v\_ucla

V2V cooperative perception communication benchmark on the UCLA campus road network, using **Veins** (OMNeT++ + SUMO) with an optional ROS 2 bridge.

Compares three SDSM dissemination strategies under realistic IEEE 802.11p communication with full PHY/MAC modeling — replacing the heuristic noise/delay/drop models used in most cooperative perception benchmarks.

---

## Simulation Framework

| Component | Version | Role |
|-----------|---------|------|
| **[SUMO](https://eclipse.dev/sumo/)** | ≥ 1.8.0 | Microscopic traffic simulation — vehicles drive on a UCLA-area road network with car-following models, lane changes, and traffic signals. |
| **[OMNeT++](https://omnetpp.org/)** | 6.0+ | Discrete-event network simulation engine — provides the module system, event scheduler, signal/listener infrastructure, `.ini`/`.ned` configuration, and result recording. |
| **[Veins](https://veins.car2x.org/)** | 5.2+ | Vehicular network simulation framework bridging SUMO and OMNeT++. Implements full IEEE 802.11p PHY/MAC: signal propagation with Nakagami fading, CSMA/CA channel access (`Mac1609_4`), SNR-based packet decoding (`Decider80211p`), and TraCI-based vehicle synchronization. |
| **ROS 2** (optional) | Humble/Iron | UDP bridge for real-time forwarding of TX/RX events to ROS 2 nodes. Only required for `GreedyROS` config. |

### How it works

Each simulation timestep (0.1 s):

1. **SUMO** advances traffic — ~225 vehicles move along the road network.
2. **Veins TraCI** pulls updated positions and creates/moves/destroys OMNeT++ Car modules.
3. **`RosSDSMApp`** on each vehicle decides whether to transmit an SDSM (based on the active algorithm), builds a multi-object payload from its local perception set, and hands it to the 802.11p stack.
4. **Veins PHY/MAC** handles CSMA/CA channel access, signal propagation with distance-dependent path loss and Nakagami fading, interference from concurrent transmissions, and SNR-based decoding at each receiver.
5. Successfully decoded packets are delivered to the receiving vehicle's `RosSDSMApp`, which logs all metrics to CSV.

---

## Dissemination Algorithms

All algorithms use the same 802.11p PHY/MAC and SDSM message format. They differ only in **when** a vehicle decides to transmit.

### Periodic (Baseline)

Fixed-rate broadcast at 10 Hz (every 0.1 s), per SAE J2945/1 safety application requirements.

- No intelligence — sends regardless of state changes or channel conditions.
- Provides the baseline communication load for comparison.

> **Reference:** SAE J2945/1, *On-Board System Requirements for V2V Safety Communications* (2020).

### Event-Triggered

Sends only when the vehicle's state or perceived environment has changed significantly.

**Utility function:**

```
U = w₁ · selfChange + w₄ · objectSetChange
```

Where:
- `selfChange = α_pos · Δposition + α_spd · Δspeed + α_hdg · Δheading` — weighted kinematic change since last transmission.
- `objectSetChange` — novelty of the local perception set compared to the last sent snapshot: sum of position/speed deltas for persisting objects, plus a penalty (5 m equivalent) for each new or lost object.

A transmission is triggered when `U > threshold` and `timeSinceSend ≥ minInterval`. A starvation guard forces a send if `timeSinceSend ≥ maxInterval`.

**Default weights:** `w₁ = 1.0`, `w₄ = 1.0`, `w₂ = 0` (no AoI), `w₃ = 0` (no congestion).

> **References:**
> - C. Sommer, O. K. Tonguz, and F. Dressler, "Traffic information systems: efficient message dissemination via adaptive beaconing," *IEEE Communications Magazine*, vol. 49, no. 5, pp. 173–179, 2011.
> - T. Tielert, D. Jiang, Q. Chen, L. Delgrossi, and H. Hartenstein, "Design methodology and evaluation of rate adaptation based congestion control for Vehicle Safety Communications," *IEEE VNC*, 2011.

### Greedy (Full Utility)

Combines all four factors — self-change, object novelty, information freshness, and channel congestion — into a unified utility:

**Utility function:**

```
U = w₁ · selfChange + w₄ · objectSetChange + w₂ · timeSinceSend − w₃ · CBR
```

Where:
- `timeSinceSend` — seconds since last transmission, acting as an Age-of-Information (AoI) pressure term that grows linearly to prevent stale information.
- `CBR` — true PHY channel busy ratio ∈ [0, 1], measured by subscribing to Veins' `Mac1609_4::sigChannelBusy` signal and tracking accumulated busy time over a sliding window. High CBR suppresses transmission to reduce congestion.

**Default weights:** `w₁ = 1.0`, `w₄ = 0.5`, `w₂ = 0.5`, `w₃ = 0.3`.

> **References:**
> - S. Kaul, R. Yates, and M. Gruteser, "Real-time status: How often should one update?" *IEEE INFOCOM*, pp. 2731–2735, 2012. *(AoI freshness term)*
> - ETSI TS 102 687, "Intelligent Transport Systems (ITS); Decentralized Congestion Control Mechanisms," v1.2.1, 2018. *(CBR-based congestion adaptation)*
> - C. Sommer et al., 2011. *(Adaptive beaconing / self-change term)*

---

## SDSM Message Structure

Messages follow **SAE J3224** (Sensor Data Sharing Message), aligned with the [ucla-mobility/CPX-SDSM](https://github.com/ucla-mobility/CPX-SDSM) ROS 2 message definitions.

Each SDSM contains:

| Section | Fields | Notes |
|---------|--------|-------|
| **Header** | `msg_cnt` (0–127), 4-byte `source_id`, `equipment_type` (OBU) | Per J3224 |
| **Reference position** | `ref_pos_x`, `ref_pos_y`, `ref_pos_z` | Sender's SUMO coordinates (meters) |
| **Timestamp** | `sdsm_day`, `sdsm_time_of_day_ms` | Milliseconds since midnight |
| **Detected objects** (up to 16) | `obj_type`, `object_id`, `offset_x/y/z` (0.1 m), `obj_speed` (0.02 m/s), `obj_heading` (0.0125°) | Per-object arrays |
| **Legacy fields** | `senderNodeIndex`, `senderX/Y`, `senderSpeed`, `senderHeading`, `sendTimestamp`, `message_id` | For AoI/greedy tracking and CSV linking |

**Packet sizing:** 40 bytes base + 24 bytes per detected object. A full 16-object SDSM is 424 bytes — representative of real-world J3224 message sizes.

The detected-object set is built each TX by filtering `neighborInfo_` (vehicles heard within `detectionRange` = 300 m and `detectionMaxAge` = 2 s), sorted by distance (closest first), capped at `maxObjectsPerSdsm` = 16.

> **Reference:** SAE J3224, *Sensor Data Sharing Message (SDSM)*, 2023.

---

## Metrics Collected

### PHY / MAC Layer

| Metric | Formula | Dir | Meaning | Source |
|--------|---------|-----|---------|--------|
| **SNR** | P\_signal / P\_noise (dB) | ↑ | Signal-to-noise ratio; decodability indicator | IEEE 802.11p §17.3.10 |
| **RSS** | P\_rx (dBm) | ↑ | Received signal strength; range estimation | IEEE 802.11-2020 §11.11.9 |
| **CBR** | T\_busy / T\_window | ↓ | Channel Busy Ratio — fraction of time the PHY senses the channel busy. Measured via `Mac1609_4::sigChannelBusy` signal subscription. | ETSI TS 102 687 §4.2 |

### Network / Application Layer

| Metric | Formula | Dir | Meaning | Source |
|--------|---------|-----|---------|--------|
| **AoI** | t\_now − t\_send | ↓ | Age of Information — staleness of the most recent update from a neighbor | Kaul et al., IEEE INFOCOM 2012 |
| **Latency** | t\_now − t\_BSM | ↓ | End-to-end delay from message creation to reception | SAE J2945/1 §5.2 |
| **Throughput** | total\_rx / (T · N) | ↑ | Average successful receptions per vehicle per second | Jiang & Delgrossi, IEEE VTC 2008 |
| **PDR** | total\_rx / (total\_tx · (N−1)) | ↑ | Packet Delivery Ratio — fraction of all possible broadcast receptions received | Bai & Krishnan, IEEE ITSC 2006 |
| **IAT** | t\_now − t\_lastRxFromS | ↓ | Inter-Arrival Time — gap between consecutive receptions from the same sender | SAE J2945/1 §5.2.2 |
| **Redundancy** | R\_redundant / R\_total | ↓ | Fraction of receptions carrying no new kinematic state (wasted airtime) | ETSI TS 103 574; Tielert et al., IEEE VNC 2013 |
| **Δ state** | √((Δx)² + (Δy)²) + \|Δv\| | — | Combined position + speed change since sender's last TX | App-specific (`RosSDSMApp.cc`) |

### CSV Output Files

| File | Contents |
|------|----------|
| `*-rx.csv` | Per-receive: `time`, `receiver`, `sender`, `message_id`, `aoi`, `inter_arrival`, `snr`, `rss_dbm`, `distance_to_sender`, `packet_size`, `cbr`, `num_objects`, `delta_state`, `redundant` |
| `*-tx.csv` | Per-send: `time`, `node`, `message_id`, `neighbor_count_at_tx`, `num_objects_sent`, `trigger_reason` |
| `*-summary.csv` | Per-run: `total_tx`, `total_rx`, `avg_aoi`, `std_aoi`, `p95_aoi`, `p99_aoi`, `redundancy_rate`, `avg_throughput`, `packet_delivery_ratio` |
| `*-vehicle-summary.csv` | Per-vehicle: `avg_latency`, `p95_latency`, `avg_aoi`, `p95_aoi`, `mean_iat`, `p95_iat`, `redundancy_rate`, `pdr` |
| `*-metadata.csv` | Run metadata: `algorithm_name`, `run_id`, `num_vehicles`, `sim_duration`, `transmission_interval` |
| `*-timeseries.csv` | Sampled per vehicle at configurable interval: `avg_aoi_to_neighbors`, `num_neighbors`, `tx_count_since_last`, `rx_count_since_last`, `position_x`, `position_y`, `speed` |
| `*-txrx.csv` | Combined TX/RX event log (optional): `time`, `node`, `event`, `cumulative_count` |

---

## Setup

### Prerequisites

- **OMNeT++** 6.0+ ([download](https://omnetpp.org/download/))
- **Veins** 5.2+ ([download](https://veins.car2x.org/download/))
- **SUMO** 1.8.0+ ([download](https://eclipse.dev/sumo/))
- **ROS 2** Humble/Iron (optional, for `GreedyROS` mode only)

### Build

```bash
source <omnetpp-install>/setenv
cd src && make -j$(nproc)
```

Update the SUMO path in `simulations/omnetpp.ini` under `*.manager.commandLine` if needed.

## Running

### Single algorithm

```bash
python3 run_experiments.py --algorithm Periodic --sim-duration 90
python3 run_experiments.py --algorithm EventTriggered --sim-duration 90
python3 run_experiments.py --algorithm Greedy --sim-duration 90
```

### All non-ROS algorithms

```bash
python3 run_experiments.py --sim-duration 90
```

### Post-processing (shrink large CSVs)

```bash
python3 scripts/shrink_rx_csv.py results/Periodic/seed0/Periodic-r0-rx.csv --every 4 --decimals 3
```

### Output directories

- **`simulations/results/`** — raw output written by OMNeT++ during the run (full-size, overwritten each time you rerun a config).
- **`results/<Algorithm>/seed0/`** — organized archive created by `run_experiments.py` after each run. This is the canonical location for analysis — includes `meta.json` and can be post-processed (e.g. shrunk) without affecting the raw data.

## ROS 2 Bridge (Optional)

```bash
cd ros2_ws && colcon build && source install/setup.bash
ros2 run veins_ros_bridge udp_bridge_node --ros-args -p udp_port:=50010
```

TX/RX events are forwarded as JSON over UDP. External nodes can send commands (`PING`, `SEND_BSM`, `SET_INTERVAL`, `ENABLE_PERIODIC`) to port `50000 + nodeIndex`. Enable with `rosBridgeMode = "live"` in `omnetpp.ini`.

---

## Key Configuration Parameters

Configurable via `simulations/omnetpp.ini` or NED defaults in `simulations/apps/RosSDSMApp.ned`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sendInterval` | 1 s (0.1 s for Periodic) | Periodic broadcast interval |
| `greedyEnabled` | false | Enable utility-based dissemination |
| `greedyW1` | 1.0 | Self-change weight |
| `greedyW2` | 0.5 | AoI freshness weight |
| `greedyW3` | 0.3 | CBR congestion penalty weight |
| `greedyW4` | 0.0 | Object-set novelty weight |
| `greedyThreshold` | 1.0 | Utility threshold to trigger send |
| `greedyMinInterval` | 0.2 s | Minimum inter-send guard |
| `greedyMaxInterval` | 5.0 s | Maximum silence (starvation prevention) |
| `congestionWindow` | 1.0 s | CBR measurement window |
| `maxObjectsPerSdsm` | 16 | Max detected objects per SDSM |
| `detectionRange` | 300 m | Include neighbors within this range |
| `detectionMaxAge` | 2 s | Discard neighbors not heard within this time |
| `rxLogEveryNth` | 1 | Subsample RX logging (2 = half the rows) |
| `rosBridgeMode` | "off" | `"off"` / `"log"` / `"live"` |

---

## License

See upstream [ucla-mobility/CPX-SDSM](https://github.com/ucla-mobility/CPX-SDSM).
