# veins_ros_v2v_ucla

V2V SDSM dissemination simulation on the UCLA campus road network using Veins (OMNeT++ + SUMO) with a ROS 2 bridge.

Compares three broadcasting strategies for SAE J3224 Sensor Data Sharing Messages:
- **Periodic** — fixed 10 Hz broadcast (baseline)
- **Event-Triggered** — send only when vehicle state changes significantly
- **Greedy** — utility-based decision combining state change, age-of-information, and local congestion

## SDSM message structure

Messages follow **SAE J3224** / **CPX-SDSM** (Sensor Data Sharing Message), aligned with the [ucla-mobility/CPX-SDSM](https://github.com/ucla-mobility/CPX-SDSM) ROS 2 `sdsm_msgs` package (GREEN-priority fields). The simulation payload (`SdsmPayload`) includes:

- **Header:** `msg_cnt` (0–127, wraps), 4-byte `source_id`, `equipment_type` (OBU/RSU/VRU).
- **Reference position:** sender position in meters (`ref_pos_x/y/z`) — in sim we use SUMO coordinates instead of lat/lon.
- **Timestamp:** `sdsm_day`, `sdsm_time_of_day_ms` (ms since midnight).
- **Detected object (one per message):** `obj_type` (e.g. vehicle), position offset (0.1 m units), `speed` (0.02 m/s units, J2735), `heading` (0.0125° units), optional vehicle size (width/length in cm).

For greedy/AoI and logging we also carry **legacy fields** on the same payload: `senderNodeIndex`, `senderX`, `senderY`, `senderSpeed`, `senderHeading`, `sendTimestamp`, and `message_id` (global id linking TX and RX in the CSVs). The payload is carried inside a Veins `DemoSafetyMessage` (BSM) for 802.11p transmission.

## Metrics collected

When `csvLoggingEnabled = true`, each run produces CSVs and OMNeT++ scalars. **Lower is better** for AoI and redundancy; **higher is better** for SNR, RSS, and PDR.

| Output | Contents |
|--------|----------|
| **`*-summary.csv`** | Per-run: `total_tx`, `total_rx`, `avg_aoi`, `std_aoi`, `p95_aoi`, `p99_aoi`, `redundancy_rate`, `avg_throughput`, `packet_delivery_ratio`. |
| **`*-metadata.csv`** | Run metadata: `algorithm_name`, `run_id`, `num_vehicles`, `sim_duration`, `transmission_interval`. |
| **`*-aoi.csv`** | Per receive: `time`, `receiver`, `sender`, **aoi**, `inter_arrival`, **snr**, **rss_dbm**, `distance_to_sender`, `packet_size`, `channel_busy_ratio`. |
| **`*-tx.csv`** | Per send: `time`, `node`, `message_id`, `neighbor_count_at_tx`, `trigger_reason` (periodic / utility / maxInterval / ros_cmd). |
| **`*-rx.csv`** | Per receive: `time`, `node`, `message_id`, `sender`, `delay`, `was_redundant`. |
| **`*-redundancy.csv`** | Per receive: `time`, `receiver`, `sender`, `delta_state`, `redundant` (0/1), `time_since_last_rx_from_sender`. |
| **`*-txrx.csv`** | Combined TX/RX event log: `time`, `node`, `event` (TX/RX), `cumulative_count`. |
| **`*-timeseries.csv`** | Sampled every 0.1 s per vehicle: `time`, `vehicle_id`, `avg_aoi_to_neighbors`, `num_neighbors`, `tx_count_since_last`, `rx_count_since_last`, `position_x`, `position_y`, `speed`. |
| **`*-scalars.sca`** | OMNeT++ per-node: `sdsm_sent`, `sdsm_received`. |

See **`docs/METRICS.md`** for column definitions and interpretation (e.g. when higher vs lower is better).

## Setup

Requires OMNeT++ 6.0+, Veins 5.2+, and SUMO 1.8.0+. ROS 2 (Humble/Iron) is optional for the bridge.

```bash
source setup_env.sh
make -j$(nproc)
```

Update the SUMO path in `simulations/omnetpp.ini` under `*.manager.commandLine`.

## Running

Single run from `simulations/`:
```bash
opp_run -n ".;../src;./apps;./networks;../../veins/src" \
  -l ../src/veins_ros_v2v_ucla -l ../../veins/src/veins \
  -c Periodic omnetpp.ini
```

Full experiment suite (3 algorithms x 3 seeds):
```bash
python run_experiments.py
```

## ROS 2 Bridge

```bash
cd ros2_ws && colcon build && source install/setup.bash
ros2 run veins_ros_bridge udp_bridge_node --ros-args -p udp_port:=50010
```

TX/RX events are forwarded as JSON over UDP. External nodes can send commands (PING, SEND_BSM, SET_INTERVAL, ENABLE_PERIODIC) to port `50000 + nodeIndex`.
