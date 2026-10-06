# Episode 01：双车道超车

一辆车停在车道上，一个行人从它前方横穿马路，同时有 4 辆 CAV 要绕过这辆车。
这是 MDrive 场景 `interaction/Overtaking_on_Two-Lane_Road/3` 在 CARLA 0.9.12（Town01）里的一次闭环运行，4 辆 CAV 都由 TCP 策略驾驶。

**时长 23.85 s，共 478 帧，20 Hz。** 所有数据逐帧对齐：CSV 的第 *i* 帧对应规划第 *i* 步，也对应视频第 *i* 帧。

| Actor | 车型 | 角色 |
|---|---|---|
| Vehicle 1–4 | Tesla Model 3 | CAV。Vehicle 4 打头，是 CP-X 特征图里的观察车 |
| entity_1 | Audi A2 | 挡住车道的停放车 |
| entity_2 | 行人 | 从停放车前方横穿马路 |

**过程：** 行人在 1.9–12.6 s 之间横穿马路。Vehicle 4 在 8.4 s 停在停放车后方约 9 m，之后一直没有超车，Vehicle 3 和 1 跟在后面排队。对向的 Vehicle 2 在 19 s 跑完路线。此后所有 actor 都不再移动，所以录到 23.85 s 就结束了。

## 文件

| 想要什么 | 看哪个文件 |
|---|---|
| 路网 | `opendrive/Town01.xodr`；或者 `map/road_geometry.json`（车道、边界、人行横道、红绿灯，不需要装 CARLA） |
| 场景回放 | `openscenario/*.xosc`（OpenSCENARIO 1.0，包含所有 actor 的轨迹，已通过 schema 校验） |
| 场景 XML | `xml/source/` 是 MDrive 原始文件；`xml/recorded/*_REPLAY.xml` 是录到的轨迹，MDrive 回放格式 |
| 轨迹 | `trajectories/all_actors.csv`：每个 actor 每帧的位姿、速度、加速度、车道和控制量 |
| 每辆车看到了什么 | `perception/objects_per_ego.csv`：每辆 CAV 100 m 内的所有目标，包括相对位置、3D 框、是否可见、被谁挡住 |
| 可见性汇总 | `perception/visibility_summary.json`，`preview/visibility_timeline.png` |
| 规划输出 | `planner/tcp_<Vehicle>.csv`：TCP 预测的轨迹点和控制量 |
| 视频 | `cameras/<Vehicle>_{front,chase}.mp4` |
| 原始数据 | `raw/frames.jsonl`（全部信息，每帧一行）；`raw/lidar/`（点云，10 Hz）；`raw/carla_recorder.log`（可以在 CARLA 里重放） |

## 约定

- **坐标系：** CSV、JSON 和 XML 用的是 CARLA 世界坐标系：x 向东，**y 向南**，z 向上，单位是米和度。`.xosc` 用的是 OpenDRIVE 坐标系：`y → -y`，`yaw → -yaw`，单位是弧度。相对位置 `rel_*` 用的是做感知的那辆车的坐标系：x 朝前，**y 朝右**。
- **可见：** 这辆 CAV 车顶的 LiDAR（64 线，120 m）至少有 1 个点打到了这个目标。这是传感器层面的真值，不是检测器的输出。TCP 只用相机，本身不输出目标列表。
- **V2V 可补：** 这辆车看不到某个目标，但同一帧里有别的 CAV 能看到。
- **包围盒：**
  - 每个被感知的目标都有一个 3D 框，在做感知的那辆 CAV 的坐标系下：中心 `rel_x/y/z`，朝向 `rel_yaw_deg`，半尺寸 `extent_x/y/z`。
  - 8 个角点（自车系和世界系都有）只在 `raw/frames.jsonl` 里。
  - `front_camera_bbox_2d` 是前视相机上的 2D 框。它只做了投影，不考虑遮挡，所以判断是否可见要看 `visible`。

## 这一集的遮挡

| CAV | 行人被挡住的时长 | 期间其他 CAV 能看到的时长 | 被谁挡住 |
|---|---|---|---|
| Vehicle 1 | 1.75 s | 1.75 s | Vehicle 4、Vehicle 3、停放车 |
| Vehicle 3 | 0.75 s | 0.75 s | Vehicle 4、停放车 |
| Vehicle 4 | 0.40 s | 0.40 s | 停放车 |

这些遮挡都发生在前 5 秒内。CP-X 特征图用的是 2D 视线近似，图里行人被停放车完全挡住。实际用 3D LiDAR 时，车顶的传感器大部分时间能越过低矮的 Audi A2 看到行人，所以真正被挡住的时间短得多。

## 说明

- XML 里指定的车型是 Lincoln MKZ，CARLA 0.9.12 里没有这个车型，所以 4 辆 CAV 都换成了 Tesla Model 3。
- 天气以实际录到的为准：云量是 0，而 XML 里写的是 100。
- 用 CARLA ScenarioRunner 跑 `.xosc` 时，要把 LogicFile 改成 `<LogicFile filepath="Town01"/>`。
- 每次闭环运行的结果都不一样，这一集只是其中一次。

重新生成的方法见 [CP-X README](../../README.md)。
