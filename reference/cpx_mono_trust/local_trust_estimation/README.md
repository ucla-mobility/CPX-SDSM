# Local Trust Estimation (`local_trust_estimation`)

> **Note (this repo):** unmodified reference copy from CPX-Mono — not part of
> this repo's `ros2_ws` colcon build (see
> [`reference/cpx_mono_trust/README.md`](../README.md)). This is the
> **Layer 1 (local, sender-side)** trust package and is **not currently
> ported** into
> [`ros2_ws/src/sdsm_trust_perception`](../../../ros2_ws/src/sdsm_trust_perception)
> at all — that package's per-object local certainty is hardcoded to `1.0`
> (see `sdsm_codec.py`'s `get_local_scores_of`). This package scores an
> agent's own Autoware-tracked objects *before* broadcast, combining four
> per-object criteria (shape plausibility, lidar point-count support,
> distance, temporal persistence) into that local score — but it assumes a
> real LiDAR + Autoware tracked-object pipeline this simulator doesn't have,
> so porting it would mean building a synthetic stand-in for that input, not
> a line-for-line port. Everything below this line is CPX-Mono's original
> documentation.

This ROS 2 package computes local trustworthiness for Autoware tracked
objects. Its runtime contract is:

```text
PointXYZIRC
  -> BEVFusion DetectedObjects
  -> Autoware tracker TrackedObjects
       -> local trust factors + synchronized lidar
       -> optional MarkerArray visualization
       -> SDSM publisher
```

Scope: **local** trustworthiness only — each agent (vehicle or smart
infrastructure) assesses its *own* perception, so every node instance runs
on one agent's topics (defaults: vehicle; remap the topic parameters for an
infrastructure instance). Merging the per-agent assessments — including the
cooperative detections — is the job of the later *global* trustworthiness
stage and is not part of this package.

## Build

From `CPX-Mono/ros2`:

```bash
colcon build --packages-select sdsm_interfaces local_trust_estimation
source install/setup.bash
```

The central `tools/setup.sh` installs all dependencies.

## Topics

The default vehicle-side wiring is:

| Node | Inputs | Output |
| --- | --- | --- |
| `object_shape` | `/vehicle/perception/tracked_objects` | `/local_trust_estimation/object_shape` |
| `lidar_point_count` | tracked objects + `/vehicle/lidar/points` | `/local_trust_estimation/lidar_point_count` |
| `object_distance` | tracked objects + TF | `/local_trust_estimation/object_distance` |
| `temporal_presence` | tracked objects | `/local_trust_estimation/temporal_presence` |
| `trustworthiness_score` | four factor score arrays | `/local_trust_estimation/score` |
| `trustworthiness_visualization` | tracked objects + combined scores | `/local_trust_estimation/markers` |
| `sdsm_publisher` | tracked objects + combined scores + TF | `/perception/global_trustworthiness/sdsm` |

All score topics use `sdsm_interfaces/msg/TrustScoreArray`. The marker output
uses `visualization_msgs/msg/MarkerArray`, and only the final SDSM topic is an
external CPX object interface.

## Factors

### Object shape

Scores the tracked box dimensions against class-specific V2X-Seq-SPD priors.
The most probable Autoware classification is used. Unmapped Autoware classes
receive `unknown_class_score` (default `0.5`).

### Lidar point count

Pairs `TrackedObjects` and `PointCloud2` by exact source timestamp. Tracker
boxes normally arrive in `map`; each pose is transformed into the point
cloud's actual frame at that timestamp before the oriented-box point test.
The score grows logarithmically and saturates at `full_trust_points`
(default `50`).

### Object distance

Transforms each tracked pose into `sensor_frame` (default `vehicle_lidar`) at
the tracked-object timestamp. This makes distance relative to the sensing
agent instead of relative to the `map` origin. Trust is `1.0` through
`near_range` (default `30 m`), `0.5` at `half_trust_range` (default `50 m`),
and decays exponentially beyond.

### Temporal presence

Tracks consecutive presence by the full 16-byte Autoware UUID. A UUID that is
absent for a frame starts over if it returns. With the defaults, a new track
starts at `0.25` and approaches `1.0` using:

```text
min_trust + (1 - min_trust) * age / (age + k)
```

### Combined score

Synchronizes the four UUID-keyed factor arrays and computes their normalized
weighted average. Defaults are equal `0.25` weights. A frame is rejected if
factor frames or UUID sets disagree.

## Visualization

`trustworthiness_visualization` synchronizes the unchanged `TrackedObjects`
with the combined score sidecar and emits map-frame cube and text markers:

- red below `0.5`
- yellow from `0.5` to `0.8`
- green at or above `0.8`

These markers can be displayed directly in Lichtblick.

## SDSM conversion

`sdsm_publisher` resolves the lidar origin into `global_frame` (default
`map`) and encodes each tracked map-frame position as an offset from that
sender reference position. It:

- derives `object_id` deterministically from the tracker UUID;
- maps Autoware class enums directly to vehicle, vulnerable-road-user, or
  unknown J3224 types;
- reads speed directly from
  `kinematics.twist_with_covariance.twist.linear`;
- reads orientation from the tracked pose and converts it to J2735 heading;
- copies the UUID-keyed combined trust score to `obj_local_scores`.

The old `vision_msgs/Detection3DArray` bridge and index-aligned
`nav_msgs/Trajectory` velocity topic are not part of this contract.

## Launch

Start the complete trust pipeline:

```bash
ros2 launch local_trust_estimation local_trust_estimation.launch.py
```

The main launch arguments are:

```text
tracked_objects_topic:=/vehicle/perception/tracked_objects
cloud_topic:=/vehicle/lidar/points
sensor_frame:=vehicle_lidar
global_frame:=map
score_topic:=/local_trust_estimation/score
sdsm_topic:=/perception/global_trustworthiness/sdsm
use_sim_time:=auto
```

`auto` enables ROS simulation time when this launch file also owns bag
playback through `bag_path`; otherwise it uses wall clock. Set the argument to
`true` or `false` to select the clock explicitly.

It can optionally play and/or record a bag:
ros2 run local_trust_estimation trustworthiness_visualization
```

### `sdsm_publisher` — SDSM payload for the global stage

Encodes the combined scores as `sdsm_interfaces/SdsmPayload`, the message the
cooperative (global) trustworthiness stage consumes. Runs alongside
`trustworthiness_visualization`, off the same combined-score topic.
`obj_local_scores` carries the per-detection combined score and `local_score`
their mean.

Detections arrive in the agent's lidar frame, while a receiver reconstructs
each object as `ref_pos + offset * 0.1 m`. The reference position and the
per-object offsets are therefore resolved through TF into the world frame, so
`ref_pos` is the lidar origin in world coordinates and `obj_heading` follows
J2735 (0 = north, clockwise) with the world frame read as ENU. A frame whose
transform cannot be resolved is dropped with a warning — expect one such drop
at startup, before `/tf_static` has arrived.

Classes map to J3224 object types following the DAIR-V2X evaluation grouping:
Car, Van, Truck and Bus are vehicles (1), the pedestrian and cyclist groups are
vulnerable road users (2), and any unmapped class stays unknown (0), since the
receiver uses this as a hard class gate. `agent_id` becomes `source_id[0]`, the
sender identity the global stage tracks reputation against, so each agent
instance needs its own. Detections beyond the payload's object slots are
dropped with a warning.

Timestamps come from the wall clock, which is what the global stage measures
message latency against; `evaluate_offline` stamps from the source data instead
so that baking a bag twice yields identical messages.

`obj_measurement_time_ms` carries this frame's **capture lag** — the milliseconds the local chain spent between the sensor stamp and this publish — not an absolute time. It is measured on the *node* clock against the source stamp, so under `use_sim_time` both ends sit on bag time and the lag is stated in scene seconds (a wall-clock lag would scale with playback rate while the stamps it is subtracted from would not). The global stage subtracts it from arrival to recover which instant the frame describes, which is how two agents' reports of one instant are grouped together however far apart their chains finish — see `global_trust_perception`'s frame-window notes.

`obj_speed` comes from each tracked object's own twist
(`kinematics.twist_with_covariance`). Only the ground-plane magnitude is taken,
since J2735 speed is over ground.

- Input: `autoware_perception_msgs/TrackedObjects` on `tracked_objects_topic`
  (default `/vehicle/perception/tracked_objects`) and
  `sdsm_interfaces/TrustScoreArray` on `score_topic`
  (default `/local_trust_estimation/score`), plus `/tf` and `/tf_static`
- Output: `sdsm_interfaces/SdsmPayload` on `output_topic`
  (default `/perception/global_trustworthiness/sdsm`)

```bash
ros2 run local_trust_estimation sdsm_publisher
# infrastructure instance: RSU equipment type and a distinct sender ID
ros2 run local_trust_estimation sdsm_publisher --ros-args \
    -p tracked_objects_topic:=/infrastructure/gt/tracked_objects \
    -p score_topic:=/infrastructure/local_trust_estimation/score \
    -p sensor_frame:=infrastructure_lidar -p global_frame:=map \
    -p agent_id:=1 \
    -p equipment_type:=1
```

## Launching the complete pipeline

Start all nodes and play a converted bag with one command. Playback starts
after a two-second delay so the node subscriptions can be discovered first.

```bash
ros2 launch local_trust_estimation local_trust_estimation.launch.py \
  bag_path:=/path/to/input_bag \
  output_bag_path:=/path/to/new_output_bag
```

An input bag for this launch must already contain `TrackedObjects`, or a live
Autoware tracker must be running.

## Offline evaluator

`evaluate_offline` copies every source message verbatim and appends the four
factor arrays, combined score, markers, and SDSM output. It requires
synchronized `PointCloud2` and `TrackedObjects`; velocity is taken from each
tracked object, so no `Trajectory` topic is required.

```bash
ros2 run local_trust_estimation evaluate_offline \
  --input /path/to/input_bag \
  --output /path/to/enriched_bag
```

Use `--cloud-topic` or `--tracked-objects-topic` for non-default agent
namespaces, and `--force` to replace an existing output bag. Converted CPX
dataset bags keep ground truth separate from live tracker output, so evaluate
them with:
ros2 launch local_trust_estimation local_trust_estimation.launch.py
```

The combined detections are published on `/local_trust_estimation/score`, their
traffic-light markers on `/local_trust_estimation/markers`, and their SDSM
payloads on `/perception/global_trustworthiness/sdsm`.

## End-to-end with the global stage (rosbag demo)

The SDSM topic above is the seam: the global stage consumes it and never reads
a bag itself. A full local → global run from a field recording is therefore
three steps.

**1. Convert the source recording.** For a ROS 1 autosense bag, see the
[autosense adapter](../../../tools/rosbag_conversion/adapters/AUTOSENSE_ROS1.md);
converted V2X-Seq bags can be played directly.

```bash
python3 -m tools.rosbag_conversion.cli autosense-ros1 \
    --bag simulations/7.22_ROSBAG/rosbag_veh/veh_local_det_and_trac/2026-07-22-17-25-07.bag \
    --out simulations/7.22_ROSBAG/rosbags
```

**2. Start both stages.** `sdsm_publisher`'s `agent_id` is the sender identity
the global stage tracks reputation against, so it decides what the receiving
ego makes of the traffic:

```bash
# global stage — tracker first, the agents consume its TrackUpdate stream
ros2 run global_trust_tracker tracker
ros2 run global_trust_perception agent --ros-args \
    -p agent_id:=1 -p mode:=replay -r __node:=agent_1
ros2 run global_trust_perception trust_view --ros-args \
    -p ego_source_id:=1 -p frame_id:=map

# local stage
ros2 run local_trust_estimation object_shape
ros2 run local_trust_estimation lidar_point_count
ros2 run local_trust_estimation object_distance
ros2 run local_trust_estimation temporal_presence
ros2 run local_trust_estimation trustworthiness_score
ros2 run local_trust_estimation sdsm_publisher --ros-args -p agent_id:=1
```

**3. Play the converted bag.**

```bash
ros2 bag play simulations/7.22_ROSBAG/rosbags/autosense_vehicle_2026-07-22-17-25-07
```

**Viewing it.** Start `foxglove_bridge`, connect Lichtblick to
`ws://localhost:8765`, and import the `trustworthiness_autosense` layout from
[`tools/lichtblick/layouts/`](../../../tools/lichtblick/README.md#layouts). The
V2X-Seq layout will not work on this source: it expects camera topics this data
does not have, follows an ego pose it does not publish, and parks its camera at
the V2X-Seq map coordinates kilometres from these detections.

### Two agents from two time-synchronized recordings

A pair of recordings covering the same scene at the same wall-clock time (the
7.29 `scene_01_ros2` infrastructure + vehicle pair) runs the trust engine for
real: one agent is ego, the other is a scored peer.

Two things have to hold, and both are properties of the *conversion*, not of
this package:

1. **The agents must share a world frame.** The autosense adapter takes each
   agent's world pose from its own `/tf`, so bags converted separately land
   in one frame — see
   [the adapter's world transform section](../../../tools/rosbag_conversion/adapters/AUTOSENSE_ROS1.md#the-world-transform----map-frame).
   A bag converted without that pose pins its sensor at the world origin, and
   the two agents' detections will never cluster: every peer detection becomes
   `other_only`, and the peer's reputation drains to zero. That is a
   conversion bug presenting as a trust result, so check for
   `world_transform_from_tf` in the conversion output before believing any
   reputation number.
2. **Their topic namespaces must be disjoint**, which the adapter gives for
   free by naming topics after the agent (`/infrastructure/...` versus
   `/vehicle/...`).

Only the *ego* needs a `global_trust_perception agent` node — it subscribes to
the shared SDSM topic and scores every other sender it hears. The peer
contributes a `sdsm_publisher` and nothing more. Add a second `agent` node
only if you also want the reverse judgment.

`scripts/run_two_agent_demo.sh` starts this whole graph — both local chains,
the tracker, the ego judge, both visualizers — and plays the two bags. Point
it at another converted pair with `BAGS=<dir> scripts/run_two_agent_demo.sh`;
logs land in `/tmp/scene01`. The rest of this section is what it does and why.

Every node parameter that names a topic has to be remapped for the
non-default agent, **outputs included**; `local_trust_estimation.launch.py`
takes no topic or `agent_id` arguments at all, so it covers only a
single agent on the default `/vehicle/...` namespace.

```bash
# infrastructure = ego, agent 1 (the launch file cannot express this)
ros2 run local_trust_estimation object_shape --ros-args \
    -p input_topic:=/infrastructure/gt/tracked_objects \
    -p output_topic:=/infrastructure/local_trust_estimation/object_shape
# ... the other criteria the same way, then:
ros2 run local_trust_estimation trustworthiness_score --ros-args \
    -p object_shape_topic:=/infrastructure/local_trust_estimation/object_shape \
    -p lidar_point_count_topic:=/infrastructure/local_trust_estimation/lidar_point_count \
    -p object_distance_topic:=/infrastructure/local_trust_estimation/object_distance \
    -p temporal_presence_topic:=/infrastructure/local_trust_estimation/temporal_presence \
    -p output_topic:=/infrastructure/local_trust_estimation/score
ros2 run local_trust_estimation sdsm_publisher --ros-args \
    -p tracked_objects_topic:=/infrastructure/gt/tracked_objects \
    -p score_topic:=/infrastructure/local_trust_estimation/score \
    -p sensor_frame:=infrastructure_lidar -p global_frame:=map \
    -p agent_id:=1 -p equipment_type:=1

# vehicle = scored peer, agent 2 -- the score chain defaults to /vehicle/..., but
# the source topic must be overridden off the tracker default to the GT topic
ros2 run local_trust_estimation object_shape --ros-args \
    -p input_topic:=/vehicle/gt/tracked_objects   # and the other criteria
ros2 run local_trust_estimation sdsm_publisher --ros-args \
    -p tracked_objects_topic:=/vehicle/gt/tracked_objects \
    -p global_frame:=map -p agent_id:=2

# global stage: one judge, the ego
ros2 run global_trust_tracker tracker
ros2 run global_trust_perception agent --ros-args \
    -p agent_id:=1 -p mode:=replay -r __node:=agent_1
```

Play both bags in one command; `-i` may be repeated:

```bash
ros2 bag play -i <infrastructure_bag> -i <vehicle_bag> \
    --read-ahead-queue-size 200
```

**One command is not by itself enough to keep the two recordings aligned.**
`ros2 bag play` orders messages across input bags only within the read-ahead
queue (see its `--help`), so that queue has to span the gap between the bags'
start times or the merge silently degrades into playing each bag from its own
beginning at the same instant. On the `scene_01_ros2` pair — 4.72 s apart —
a queue of 20 leaves the infrastructure stream serving frames **4.7 s older**
than the vehicle's for the whole run, measured with nothing subscribed but a
probe; 200 brings that to 0.03 s.

Nothing downstream can detect this. The SDSM carries a transmit stamp, not a
measurement time, so the trust engine cannot tell that it is corroborating
ego's view of *t* against a peer's view of *t + 4.7 s*; it simply produces
`DEFERRED` verdicts and a reputation that never rises. **If peer
corroboration looks broken, check the stream skew before touching the
engine.**

The queue is bounded on both sides. Too small breaks cross-bag ordering as
above; the default of 1000 buffers gigabytes of raw cloud before publishing
its first message, which looks like a hang at `Duration 0.00` and exhausts a
3 GB container outright. Re-check the value for any other bag pair.

`--start-offset` is safe **only while something else publishes `/tf_static`**.
Each bag stamps it at that bag's own first frame, so an offset past it drops
the transform from playback, and that agent's `sdsm_publisher` then logs
`No world -> {agent}_lidar transform - dropping the SDSM for this frame` for
the entire run. The symptom is asymmetric and easy to misread: the agent whose
recording starts *later* keeps its transform and reports normally, while the
earlier one goes silent, so the scene looks like a one-sided fabrication
rather than a playback flag. `scripts/run_two_agent_demo.sh` runs
`scripts/republish_tf_static.py`, which reads the transforms straight out of
the bags and holds them up independently of playback, so an offset is fine
there. `--playback-duration` is safe either way — it trims the tail, which
carries no `tf_static`.

The demo now also passes `--exclude-topics /tf_static` to `ros2 bag play`, so
that republisher is the **single** publisher of the static tree. Two
publishers of the same static transform race and the loser is silent, which
matters because the republisher applies a correction (below) that the bag's
own uncorrected copy would otherwise overwrite at random.

### The two agents' `world` frames do not coincide (`YAW_FIX`)

The two recordings were converted separately, and `world` is each recording's
own localization root — `geo_anchor.py` says it outright: the bags carry no
GNSS, so `world` has no tie to Earth. Measured against each other, the
**infrastructure** is about **4.3°** out in yaw:

| evidence | value |
|---|---|
| cross-validated rigid fit, disjoint halves of the clip | +4.277° / +4.167° |
| rotation pivot of that fit | (29.3, −158.8) |
| `world -> infrastructure_base` translation in the bag | (29.402, −157.891) |
| yaw sweep through the fusion, `matched`/frame | peak flat over −4.2…−4.5° |

The pivot landing on the infrastructure sensor's own position (to within a
metre) is what names *it* rather than the vehicle: a world-pose yaw error
rotates that agent's detections about exactly that point. The sign is not in
doubt — the correct sign takes `matched` from 4.80 to 7.60 per frame, the
opposite sign collapses it to 0.25.

`run_two_agent_demo.sh` therefore passes
`--yaw-correction infrastructure_base=${YAW_FIX:--4.3}`, moving
`world -> infrastructure_base` from −2.557° to −6.857° with its translation
untouched. Measured on a clean run: `matched` per frame rose from a hard
ceiling of 3–5 to a mode of 8 (mean 7.5 over frames where ego contributed).
`YAW_FIX=0` restores the recorded tree.

This is applied at republish time rather than baked into the bags on purpose:
it is an empirical constant for **this bag pair**, and the real fix is
re-converting both agents against a shared localization root — see
`convert_to_rosbag.py`'s `--map-frame`, whose help already warns that agents
converted without that shared link do not share a frame.

**Verifying it after any change to the bags or the constant:** sweep an extra
yaw onto ego's detections and confirm `matched` peaks at 0. If the peak sits
anywhere else, the correction is stale, not applied, or being overwritten by a
second `/tf_static` publisher.

### One sender is not enough to produce output

With a single recording, `sdsm_publisher`'s `agent_id` and the agent's
`agent_id` are the same, and the global stage **publishes nothing**: a message
whose sender matches ego is taken as *ego echo* — ground truth used for
scoring, never scored itself — so there are no peers to judge. Ingest,
decoding and SORT tracking all run (visible as `Stage 1 [ego] dets=N`),
and `trust_view` draws ego's boxes, but
`trust_output/agent_N` and `verdicts/agent_N` stay empty. That is correct
behaviour, not a failure, and it is enough to verify the pipeline end to end.

To exercise the trust engine itself, the sender and ego IDs must differ —
give `sdsm_publisher` `-p agent_id:=2` against an ego on `agent_id:=1`, and
that sender is scored as a peer. Real corroboration needs a second agent whose
recording overlaps the first *in time*.

## Creating an enriched bag offline (`evaluate_offline`)

`evaluate_offline` writes a new bag that is a verbatim copy of the source bag
plus the `/local_trust_estimation/*` and SDSM topics, computed with the same
node logic on the original timestamps. It is a deterministic file-to-file pass —
no playback clock, no DDS, no separate recorder — so, unlike `play | record`,
it cannot drop, reorder or retime messages.

```bash
ros2 run local_trust_estimation evaluate_offline \
  --input /path/to/input_bag \
  --output /path/to/enriched_bag \
  --tracked-objects-topic /vehicle/gt/tracked_objects
```

"Deterministic" here means the *content* is reproducible: baking the same bag twice yields the same messages, on the same topics, at the same timestamps. The serialized **bytes** are not stable — CDR alignment padding differs between processes — so comparing two bakes has to be done on deserialized values, not on hashes of the raw payloads. Source messages are copied verbatim and are byte-identical; only the generated topics are re-serialized.

## Measuring per-node latency (`benchmark_local`)

`scripts/benchmark_local.py` drives the real node callbacks over a real bag — the same `FrameReader` and `Pipeline` that `evaluate_offline` uses — and reports how long each node spends per frame:

```bash
python3 ros2/src/local_trust_estimation/scripts/benchmark_local.py \
    --bag simulations/V2X-Seq-SPD/rosbags/spd_0000

# cap the frame count and export raw samples for the latency model
python3 ros2/src/local_trust_estimation/scripts/benchmark_local.py \
    --bag <bag> --frames 100 --dump-samples /tmp/local_samples.json
```

```
node                                mean ms  median ms   p95 ms   max ms
 |shape                               0.159      0.056    0.567    1.183
 |distance                            0.046      0.045    0.073    0.117
 |temporal                            0.028      0.025    0.047    0.153
 |point_count                        13.278     13.334   20.081   23.735
  score                               0.053      0.048    0.089    0.202
  markers                             0.533      0.523    0.778    3.631
  sdsm                                0.200      0.188    0.296    0.838

CRITICAL PATH                        13.531     13.606   20.363   24.036
sequential sum                       14.297     14.360   21.490   25.497

Observed frame period: 100.0 ms (10.0 Hz) from the bag stamps
Occupancy: 13.5% of the period on the critical path (keeps up)
```

### Which total to use

**`CRITICAL PATH` = max(shape, point\_count, distance, temporal) + score + sdsm.** This is the figure that belongs in a latency budget. Live, the four criterion nodes are separate processes subscribing to the same detections topic, so they run concurrently and only the slowest delays `trustworthiness_score`. `trustworthiness_visualization` is deliberately excluded: it hangs off the score topic in parallel and nothing waits for it to reach the SDSM.

**`sequential sum`** is every node added up, which is what the harness actually spends — it calls the callbacks one after another in a single process. It is reported for reference only; reading it as the pipeline's latency overstates it by the width of the parallel section.

On this bag the two totals are close (13.5 vs 14.3 ms) only because `lidar_point_count` dominates so heavily that `max(parallel) ≈ sum(parallel)`. The distinction matters as soon as the branches are more balanced.

`lidar_point_count` is ~98% of the critical path; every other node is sub-millisecond. If local latency ever needs reducing, that is the only node worth looking at.

### What the number does and does not include

Timing wraps the node callbacks themselves. `Pipeline.run`'s per-node `deepcopy` is **excluded** — it stands in for the per-subscriber deserialization DDS would do anyway, and is not node work.

Neither total includes **DDS transport**: offline the nodes are wired by direct calls. The critical path is therefore a *lower bound* on live latency, short by the inter-node hops — `trustworthiness_score` waits on a `TimeSynchronizer` across four topics. The synchronizer is exact-stamp, not approximate, so it adds no fixed wait of its own; only the hops and scheduling.

Occupancy is measured against the frame period observed from the bag's own stamps rather than an assumed rate, so it stays honest on a bag recorded at something other than 10 Hz.

### This does not add to the global stage's number

The local and global figures **do not sum into a single pipeline latency**. The two stages are separate processes running at different rates. They compose only through the four-term decomposition in the `global_trust_perception` README, where this benchmark supplies `S_local` — and `S_local` is the critical path above, not the sequential sum. `--dump-samples` writes the raw per-frame series for exactly that purpose; the payload names its own composition in `critical_path_stages` so a consumer cannot accidentally use the wrong total.
