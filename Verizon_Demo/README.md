# Verizon ETX ROS 1 Bridge

This project connects the original UCLA infrastructure and vehicle computers
through Verizon ETX while keeping their existing ROS 1 Melodic perception
applications unchanged.

## Architecture

```text
Infrastructure ROS 1
  -> Vehicle ETX client
  -> Verizon public GeoRelevance BSM
  -> Windows Software ETX relay
  -> Verizon targeted Private TIM
  -> Vehicle ETX client
  -> Vehicle ROS 1

Vehicle ROS 1
  -> Vehicle ETX client
  -> Verizon public GeoRelevance BSM
  -> Windows Software ETX relay
  -> Verizon targeted Private TIM
  -> Infrastructure ETX client
  -> Infrastructure ROS 1
```

The two Ubuntu endpoints use separate Verizon Vehicle registrations. The
Windows PC uses a Software registration, learns the current endpoint session
IDs, and forwards each payload to the opposite endpoint. Session IDs are
learned again automatically after reconnects.

BSM and TIM are the Verizon routes allowed by the client ACLs. Their payload is
the compact project JSON schema; it is not presented as a standard J2735 BSM or
TIM application payload.

## Transmitted data

The default profile transports structured tracking results instead of raw
sensor data:

| Role | ROS source topic | ROS type |
| --- | --- | --- |
| Infrastructure | `/tracking2/tracking_objects` | `autosense_msgs/TrackingObjectArray` |
| Vehicle | `/tracking/tracking_objects` | `autosense_msgs/TrackingObjectArray` |

The converter preserves tracking IDs, positions, dimensions, directions, and
velocities. Point clouds and images are intentionally omitted to keep messages
within the Verizon size and rate limits.

The infrastructure converter also caches
`/tracking2/transformed_det_box_score_label` and attaches the latest
`visualization_msgs/MarkerArray` to the next tracking JSON packet. The vehicle
reconstructs that attachment and publishes it on the topic configured by
`MARKER_RX_TOPIC` in `config/vehicle.env`.

Every JSON envelope includes a sender ID:

- Infrastructure: `ucla-infrastructure-nw`
- Vehicle: `ucla-vehicle-01`

## Repository contents

- `app/`: ROS conversion, ETX endpoints, and Windows relay logic
- `catkin_ws/src/`: minimal ROS message overlay used by the bridge
- `config/`: role-specific source topics, destination topics, IDs, and GPS
- `scripts/`: Ubuntu setup, validation, start, status, and stop commands
- `windows/`: Windows Software-client registration, setup, relay, and status
- `tests/`: offline converter, routing, sender metadata, and analysis tests
- `tools/`: log validation and PDR/latency analysis
- `vendor/`: the Python ETX sample runtime required by this bridge
- `bags/`: bag placement notes and checksums only; bag files are not committed

Runtime logs, rosbag data, credentials, build products, and local backups are
excluded from the repository.

## Security: provide registrations locally

Registration JSON files contain passwords, tokens, certificates, and private
keys. They are intentionally excluded from Git. Provision these three files on
authorized machines before setup:

```text
clients/infrastructure_vehicle/registration.json
clients/vehicle/registration.json
clients/infrastructure_gateway/registration.json
```

The infrastructure and vehicle files must be two different Vehicle client
registrations. Never reuse one registration on both endpoints. Keep every
registration file at permission mode `0600` on Ubuntu.

## Expected deployment paths

```text
Infrastructure: /home/mobility/ros_ws/Verizon_Demo
Vehicle:        /home/dev/ros_ws/Verizon_Demo
Windows relay:  D:\sim\Verizon_Demo
```

## Requirements

- Ubuntu 18.04 and ROS 1 Melodic on both endpoints
- Each original tracking workspace and its message dependencies
- Internet access to the Verizon MQTT/TLS endpoint on TCP 8883
- A Python 3.12 Conda environment named `etx` on Ubuntu
- Windows PowerShell and the configured Python runtime for the relay
- Three valid and unexpired Verizon registrations described above

The Ubuntu setup script installs or reuses Miniforge, prepares the Python
runtime, and builds only the small `autosense_msgs` overlay. It deliberately
uses Ubuntu's `/usr/bin/cmake` to remain compatible with ROS Melodic catkin.

## One-time setup

Infrastructure:

```bash
cd /home/mobility/ros_ws/Verizon_Demo
bash scripts/setup_ubuntu18.sh infrastructure
bash scripts/preflight.sh infrastructure
```

Vehicle:

```bash
cd /home/dev/ros_ws/Verizon_Demo
bash scripts/setup_ubuntu18.sh vehicle
bash scripts/preflight.sh vehicle
```

Windows PowerShell:

```powershell
cd "D:\sim\Verizon_Demo"
Set-ExecutionPolicy -Scope Process Bypass -Force
.\windows\setup_monitor.ps1
```

Do not begin a field test unless both Ubuntu preflight checks print
`PREFLIGHT_OK` and the TCP 443 and TCP 8883 checks pass.

## Live test

1. Start the original detection and tracking applications on both Ubuntu
   devices.
2. Verify the configured source topics:

```bash
# Infrastructure
cd /home/mobility/ros_ws/Verizon_Demo
bash scripts/verify_live_topic.sh infrastructure 20

# Vehicle
cd /home/dev/ros_ws/Verizon_Demo
bash scripts/verify_live_topic.sh vehicle 20
```

3. Start the Windows relay first and keep the terminal open:

```powershell
cd "D:\sim\Verizon_Demo"
.\windows\run_monitor.ps1
```

4. Start the infrastructure endpoint:

```bash
cd /home/mobility/ros_ws/Verizon_Demo
bash scripts/start_node.sh infrastructure
```

5. Start the vehicle endpoint:

```bash
cd /home/dev/ros_ws/Verizon_Demo
bash scripts/start_node.sh vehicle
```

6. Verify the Windows relay in a second PowerShell window:

```powershell
cd "D:\sim\Verizon_Demo"
.\windows\status_monitor.ps1
```

Both sessions must be learned, `BSM_DATA_RX` and `TIM_PUBLISHED` must increase,
and `TIM_FAILED` must remain zero.

7. Verify the reconstructed ROS messages:

```bash
# Infrastructure receives Vehicle tracking
source /home/mobility/ros_ws/Verizon_Demo/scripts/load_role.sh infrastructure
rostopic hz /etx/from_vehicle/tracking_objects
rostopic echo -n 1 /etx/from_vehicle/tracking_objects

# Vehicle receives Infrastructure tracking and MarkerArray
source /home/dev/ros_ws/Verizon_Demo/scripts/load_role.sh vehicle
rostopic hz /etx/from_infrastructure/tracking_objects
rostopic echo -n 1 /etx/from_infrastructure/tracking_objects
rostopic hz /tracking2/transformed_det_box_score_label
rostopic echo -n 1 /tracking2/transformed_det_box_score_label
```

See `Test Steps.txt` for the complete field checklist and expected counters.

## GPS routing

The infrastructure endpoint uses its confirmed static routing coordinate:

```text
34.067086, -118.445280
```

The vehicle reads dynamic `gps_common/GPSFix` data from `/veh_2/gps/raw` and
uses `34.067055556, -118.445250000` only as a startup fallback. These
coordinates control Verizon geographic routing; they do not by themselves
transform both tracking outputs into one shared map frame.

## Time synchronization and metrics

PDR is matched by transport sequence ID and does not require synchronized
clocks. One-way latency does require synchronized clocks. Enable NTP on both
Ubuntu devices before collecting latency evidence:

```bash
sudo timedatectl set-ntp true
timedatectl status
```

Each endpoint writes `source.jsonl`, `received.jsonl`, `cloud_tx.jsonl`,
`cloud_rx.jsonl`, and `status.json` under `logs/<role>_<UTC>/`. The Windows
relay writes corresponding evidence under `logs/pc_monitor_<UTC>/`. These
runtime files are ignored by Git.

## Stop

Stop the Windows relay with `Ctrl+C`, then run:

```bash
# Infrastructure
bash /home/mobility/ros_ws/Verizon_Demo/scripts/stop_node.sh infrastructure

# Vehicle
bash /home/dev/ros_ws/Verizon_Demo/scripts/stop_node.sh vehicle
```
