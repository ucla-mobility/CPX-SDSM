# Episode 01 — Overtaking on a two-lane road

A pedestrian crosses in front of a car parked in the lane, while four CAVs try to pass it.
This is one closed-loop run in CARLA 0.9.12 (Town01) of MDrive scenario `interaction/Overtaking_on_Two-Lane_Road/3`. All four CAVs are driven by the TCP policy.

**23.85 s · 478 frames · 20 Hz.** Every data stream is frame-aligned: frame *i* in the CSVs = planner step *i* = video frame *i*.

| Actor | Blueprint | Role |
|---|---|---|
| Vehicle 1–4 | Tesla Model 3 | CAVs. Vehicle 4 leads and is the observer in the CP-X figure |
| entity_1 | Audi A2 | Parked car blocking the lane |
| entity_2 | Pedestrian | Crosses the road in front of the parked car |

**What happened:** the pedestrian crosses from 1.9 s to 12.6 s. Vehicle 4 stops about 9 m behind the parked car at 8.4 s and never overtakes, and Vehicles 3 and 1 queue behind it. Vehicle 2 (oncoming) finishes its route at 19 s. After that nothing moves, so the recording stops at 23.85 s.

## Files

| What you want | File |
|---|---|
| Road network | `opendrive/Town01.xodr`, or `map/road_geometry.json` (lanes, boundaries, crosswalks, traffic lights; no CARLA needed) |
| Scenario replay | `openscenario/*.xosc` (OpenSCENARIO 1.0, all actor trajectories, schema-validated) |
| Scenario XML | `xml/source/` (original MDrive files), `xml/recorded/*_REPLAY.xml` (recorded trajectories, MDrive replay format) |
| Trajectories | `trajectories/all_actors.csv`: pose, velocity, acceleration, lane, and controls for every actor at every frame |
| What each CAV perceives | `perception/objects_per_ego.csv`: every object within 100 m of each CAV, with relative position, 3D box, visibility, occluders |
| Visibility summary | `perception/visibility_summary.json`, `preview/visibility_timeline.png` |
| Planner outputs | `planner/tcp_<Vehicle>.csv`: TCP predicted waypoints and controls |
| Videos | `cameras/<Vehicle>_{front,chase}.mp4` |
| Raw data | `raw/frames.jsonl` (everything, one line per frame), `raw/lidar/` (point clouds, 10 Hz), `raw/carla_recorder.log` (replay in CARLA) |

## Conventions

- **Coordinates:** CSV, JSON and XML files use the CARLA world frame (x east, **y south**, z up, metres, degrees). The `.xosc` uses OpenDRIVE (`y → -y`, `yaw → -yaw`, radians). Relative positions (`rel_*`) use the perceiving vehicle's frame: x forward, **y right**.
- **Visible** means the CAV's roof LiDAR (64 channels, 120 m) hit the object with at least one point. This is ground truth from the sensor, not a detector output. TCP is camera-only and has no object list.
- **V2V-recoverable** means the CAV cannot see the object but another CAV can, in the same frame.
- **Bounding boxes:**
  - 3D box per perceived object, in the perceiving CAV's frame: center `rel_x/y/z`, yaw `rel_yaw_deg`, half-size `extent_x/y/z`.
  - 8 box corners, in both the ego and the world frame: `raw/frames.jsonl` only.
  - 2D box in the front camera: `front_camera_bbox_2d`. It is a projection that ignores occlusion, so use `visible` to check visibility.

## Occlusions in this episode

| CAV | Pedestrian hidden for | Seen by another CAV meanwhile | Blocked by |
|---|---|---|---|
| Vehicle 1 | 1.75 s | 1.75 s | Vehicles 4 and 3, parked car |
| Vehicle 3 | 0.75 s | 0.75 s | Vehicle 4, parked car |
| Vehicle 4 | 0.40 s | 0.40 s | parked car |

All of this happens in the first 5 s. The CP-X figure (2D line of sight) shows the pedestrian fully hidden by the parked car. With a 3D LiDAR the roof sensor sees over the low Audi A2 most of the time, so the real occlusion is much shorter.

## Notes

- The XML asks for a Lincoln MKZ, which CARLA 0.9.12 lacks, so all four CAVs are Tesla Model 3s.
- Weather is as recorded: cloudiness 0, although the XML says 100.
- To run the `.xosc` in CARLA ScenarioRunner, set `<LogicFile filepath="Town01"/>`.
- Closed-loop runs differ from run to run, so this episode is one sample.

To regenerate, see the [CP-X README](../../README.md).
