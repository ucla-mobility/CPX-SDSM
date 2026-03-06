# veins_ros_v2v_ucla

V2V SDSM dissemination simulation on the UCLA campus road network using Veins (OMNeT++ + SUMO) with a ROS 2 bridge.

Compares three broadcasting strategies for SAE J3224 Sensor Data Sharing Messages:
- **Periodic** — fixed 10 Hz broadcast (baseline)
- **Event-Triggered** — send only when vehicle state changes significantly
- **Greedy** — utility-based decision combining state change, age-of-information, and local congestion

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
