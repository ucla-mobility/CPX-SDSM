# veins_ros_v2v (Veins 5.2 + SUMO 1.8.0) — 最小 V2V SDSM-like Demo

这个项目目标：在 **Veins (SUMO+OMNeT++ Co-simulation)** 的最小场景中，让两辆车周期性广播一条 "SDSM-like" 消息，实现最基础的 V2V 通信链路验证。

> 你现在的 Veins NED package 是 `org.car2x.veins...`（官方 Veins 5.x 系列默认），本项目已按该包名修正。

---

## 目录结构
- `simulations/`：OMNeT++ 场景（NED + omnetpp.ini）
- `src/`：应用层代码 `RosSDSMApp`（继承 DemoBaseApplLayer，发/收 BSM）
- `src/messages/`：自定义消息 `SdsmPayload.msg`（被封装在 DemoSafetyMessage 里）
- `sumo/`：SUMO 最小路网 + routes + cfg + build_net 脚本
- `ros/ROS/`：SDSM 相关 ROS2 msg（可选，仅用于对照字段/后续桥接）

---

## 前置条件（你已基本完成）
1. Veins 示例 `examples/veins` 能在 Qtenv 跑通 ✅
2. SUMO_HOME 正确：`D:\sim\sumo-1.8.0`（并且 `sumo.exe`、`netconvert.exe` 可用）

---

## Step 1：放置项目
把整个 `veins_ros_v2v` 文件夹放到你的 OMNeT++ workspace 下，例如：

- `D:\sim\ws_omnetpp\veins_ros_v2v`

然后在 OMNeT++ IDE：

- `File -> Import -> Existing Projects from File System`
- 选到 `D:\sim\ws_omnetpp` 或 `...\veins_ros_v2v`

---

## Step 2：配置项目引用（非常重要）
右键 `veins_ros_v2v` -> `Properties`：

1) **Project References**
- 勾选你的 `veins` 项目（也就是 Veins 5.2 的工程）

2) **OMNeT++ -> Makemake**
- 选中 `veins_ros_v2v` 工程根目录或 `src/`
- `Build` 选择 **Makemake**
- 点击 `Apply`，然后右键项目 `Build Project`

> 如果你之前 `Makemake` 选成了 "No Makefile"，一定要改回 Makemake。

---

## Step 3：Run Configuration（推荐配置）
在 IDE：`Run -> Run Configurations -> OMNeT++ Simulation`

- **Executable**：选择编译出来的 `veins_ros_v2v` 可执行文件（例如 `.../veins_ros_v2v.exe`）
- **Working dir**：`veins_ros_v2v/simulations`
- **Ini file(s)**：`omnetpp.ini`

### Advanced
- **Dynamic libraries**：`${opp_shared_libs:/veins}`
- **NED Source Path**：`${opp_ned_path:/veins_ros_v2v};${opp_ned_path:/veins}`
- **Image Path**：`${opp_image_path:/veins}`

---

## Step 4：运行
- 直接 Run（Qtenv 或 Cmdenv 均可）
- 如果 SUMO 能正常拉起，你会在 Console 看到 TraCI 连接成功，并在日志里看到类似：

```
[RosSDSMApp] RX BSM latency=... payload="SensorDataSharingMessage{...}"
```

---

## SUMO 路网生成（可选）
如果你想自己改 nodes/edges 后重新生成 net：

```bat
cd sumo
set SUMO_HOME=D:\sim\sumo-1.8.0
build_net.bat
```

---

## 已做的关键修复（针对你之前遇到的报错）
1. **NED 包名统一为 `org.car2x.veins...`**（否则会出现 missing base type / package mismatch）
2. `SdsmPayload.msg` 使用 OMNeT++ 正确语法：`namespace ...;`（不是 `package ...;`）
3. 网络 NED 补齐 `ConnectionManager / World / AnnotationManager / ObstacleControl`，避免最小网络缺组件
4. `omnetpp.ini` 默认 SUMO 路径更新为 `D:/sim/sumo-1.8.0/...` 并添加 `--remote-port $port`

