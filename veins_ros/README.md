# veins_ros_v2v (Veins 5.2 + SUMO 1.8.0) — Minimal V2V SDSM-like Demo

Goal of this project: in a **minimal Veins (SUMO + OMNeT++ co-simulation)** scenario, make two vehicles periodically broadcast an “SDSM-like” message, so you can validate the most basic V2V communication link end-to-end.

> Your current Veins NED package is `org.car2x.veins...` (the default in official Veins 5.x). This project has already been adjusted to match that package name.

---

## Directory Structure
- `simulations/`: OMNeT++ scenario (NED + `omnetpp.ini`)
- `src/`: application-layer code `RosSDSMApp` (inherits `DemoBaseApplLayer`, sends/receives BSM)
- `src/messages/`: custom message `SdsmPayload.msg` (encapsulated inside `DemoSafetyMessage`)
- `sumo/`: minimal SUMO road network + routes + cfg + `build_net` scripts
- `ros/ROS/`: SDSM-related ROS2 msg definitions (optional; for field mapping / future bridging)

---

## Prerequisites (Mostly Done on Your Side)
1. The Veins example `examples/veins` can run in Qtenv ✅  
2. `SUMO_HOME` is correctly set: `D:\sim\sumo-1.8.0` (and `sumo.exe`, `netconvert.exe` are available)

---

## Step 1: Place the Project
Put the entire `veins_ros_v2v` folder into your OMNeT++ workspace, for example:

- `D:\sim\ws_omnetpp\veins_ros_v2v`

Then in OMNeT++ IDE:

- `File -> Import -> Existing Projects from File System`
- Select `D:\sim\ws_omnetpp` or `...\veins_ros_v2v`

---

## Step 2: Configure Project References (Very Important)
Right-click `veins_ros_v2v` -> `Properties`:

1) **Project References**
- Check/select your `veins` project (the Veins 5.2 project in your workspace)

2) **OMNeT++ -> Makemake**
- Select the `veins_ros_v2v` project root or `src/`
- Set **Build** to **Makemake**
- Click `Apply`, then right-click the project and `Build Project`

> If you previously set Makemake to “No Makefile”, make sure to switch it back to Makemake.

---

## Step 3: Run Configuration (Recommended)
In the IDE: `Run -> Run Configurations -> OMNeT++ Simulation`

- **Executable**: select the compiled `veins_ros_v2v` executable (e.g., `.../veins_ros_v2v.exe`)
- **Working dir**: `veins_ros_v2v/simulations`
- **Ini file(s)**: `omnetpp.ini`

### Advanced
- **Dynamic libraries**: `${opp_shared_libs:/veins}`
- **NED Source Path**: `${opp_ned_path:/veins_ros_v2v};${opp_ned_path:/veins}`
- **Image Path**: `${opp_image_path:/veins}`

---

## Step 4: Run
- Just run (Qtenv or Cmdenv both work)
- If SUMO is launched correctly, you should see a successful TraCI connection in the console, and logs like:

```text
[RosSDSMApp] RX BSM latency=... payload="SensorDataSharingMessage{...}"
```

---

# Controlling Communication in Veins with ROS 2 (UDP Bridge)

This project integrates **Veins <-> ROS2 via UDP**:
- On the **Veins side**, it only relies on standard sockets (Windows: Winsock). No need to link ROS2 into the OMNeT++ project.
- On the **ROS2 side**, run a `veins_ros_bridge` node that converts ROS topics into UDP commands, and receives RX/TX/ACK feedback from Veins.

## 1) Port Convention
- Each vehicle (each `node[i]`) listens on a command port:
  - `cmd_port = rosCmdPortBase + nodeIndex`
  - Default: node[0] -> 50000, node[1] -> 50001
- The port used by vehicles to send feedback back to ROS2 (all vehicles share the same destination port):
  - `rosRemotePort` (default 50010)

These parameters are configurable in `simulations/omnetpp.ini`:
```ini
*.node[*].appl.rosCmdPortBase = 50000
*.node[*].appl.rosRemoteHost = "127.0.0.1"
*.node[*].appl.rosRemotePort = 50010
```

## 2) ROS2 Workspace (Provided)
Under the project root `ros2_ws/` you will find:
- `sdsm_msgs`: SDSM message definitions (your existing package)
- `veins_ros_bridge`: the UDP Bridge node

## 3) Recommended Launch Order
1. **Start the Veins simulation** (Run from IDE)
2. **Start the ROS2 bridge**
3. Publish control commands from ROS2 (e.g., force node[0] to send immediately)

## 4) ROS2 Bridge Example
(Works on Windows / Linux. Source your ROS2 environment first.)

```bash
cd ros2_ws
colcon build
source install/setup.bash   # On Windows: install\setup.bat

ros2 run veins_ros_bridge udp_bridge   --ros-args -p host:=127.0.0.1 -p cmd_port_base:=50000 -p ros_rx_port:=50010
```

Open another terminal to publish commands:
```bash
# Force node[0] to send immediately with payload="hello"
ros2 topic pub --once /veins/cmd_raw std_msgs/msg/String "{data: '0 SEND_BSM hello'}"

# Disable periodic sending on node[0]
ros2 topic pub --once /veins/cmd_raw std_msgs/msg/String "{data: '0 ENABLE_PERIODIC 0'}"

# Set node[1] periodic interval to 0.2s
ros2 topic pub --once /veins/cmd_raw std_msgs/msg/String "{data: '1 SET_INTERVAL 0.2'}"
```

You should see Veins feedback text like `HELLO / ACK / TX / RX` in the bridge terminal, and it will also be published to:
- `/veins/rx_raw` (`std_msgs/String`)

## 5) Non-ROS Test (Optional)
If you want to verify UDP first before using ROS:
```bash
cd ros2_ws
python3 -m veins_ros_bridge.send_cmd --id 0 --cmd "PING"
python3 -m veins_ros_bridge.send_cmd --id 1 --cmd "SEND_BSM test_payload"
```

---

## Rebuilding the SUMO Network (Optional)
If you modify nodes/edges and want to regenerate the net:

```bat
cd sumo
set SUMO_HOME=D:\sim\sumo-1.8.0
build_net.bat
```

---

## Key Fixes Already Applied (For Your Previous Errors)
1. **Unified NED package name to `org.car2x.veins...`** (otherwise you’ll hit missing base type / package mismatch errors)
2. `SdsmPayload.msg` uses correct OMNeT++ syntax: `namespace ...;` (not `package ...;`)
3. The network NED includes `ConnectionManager / World / AnnotationManager / ObstacleControl` to avoid missing components in a minimal network
4. `omnetpp.ini` updates the default SUMO path to `D:/sim/sumo-1.8.0/...` and adds `--remote-port $port`
