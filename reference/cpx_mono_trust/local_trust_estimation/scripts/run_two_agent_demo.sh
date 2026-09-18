#!/bin/bash
# Two-agent trust demo for the scene_01_ros2 pair:
#   infrastructure = agent 1 (ego), vehicle = agent 2 (scored peer).
# See ros2/src/local_trust_estimation/README.md for what each flag is for.
cd /workspaces/mobilityLab/ros2_ws
source /opt/ros/kilted/setup.bash
source install/setup.bash

BAGS=${BAGS:-src/CPX-Mono/simulations/scene_01_ros2/rosbags}
INFRA=$BAGS/autosense_infrastructure_2026-07-29-infra_recording_full2-waymo_2026-07-29-16-15-59
VEH=$BAGS/autosense_veh*2026-07-29-16-16-03_waymo
LOG=/tmp/scene01
rm -rf $LOG && mkdir -p $LOG

# --- SIM TIME, on every node in the graph.
# The pipeline measures its windows (the deferred ledger's deadline, sender
# speed) on the node clock. Under use_sim_time that clock is the BAG's, fed by
# `ros2 bag play --clock` below, so scene time advances at the same rate no
# matter what RATE playback runs at -- which is what makes RATE a pure viewing
# knob instead of something that silently changes the verdicts.
#
# It has to be ALL nodes or none: a mixed graph has publishers stamping on one
# clock and subscribers reading another, and the disagreement is silent.
#
# SIM_TIME=0 turns it off and restores the pre-sim-time behaviour, where RATE
# DOES change the verdicts (a slowed run gives more flushes per scene second,
# so the ledger's deadline expires after less scene). Keep it on unless you are
# isolating a playback problem -- it is the only reason RATE is safe to use
# while reading numbers.
if [ "${SIM_TIME:-1}" = "0" ]; then
    SIM=""
    CLOCK=""
    echo "=== use_sim_time OFF: RATE will affect the verdicts ==="
else
    SIM="-p use_sim_time:=true"
    # /clock RATE matters twice over. Bare --clock is only 40 Hz, so every
    # sim-time timer quantises to 25 ms and a node that misses a tick catches
    # up in a burst -- the view freezes, then jumps. But --clock-topics-all
    # (an update before EVERY replayed message, point clouds included) is worse
    # the other way: measured, it cost a fifth of the vehicle's SDSMs, 59
    # delivered against the 80 every other configuration gets. A fixed high
    # rate sits between the two -- 5 ms granularity, one small message per tick.
    CLOCK="--clock ${CLOCK_HZ:-200}"
fi

I=/infrastructure/local_trust_estimation

# Ground-truth TrackedObjects topics the converted bags publish (see
# tools/rosbag_conversion). GT replay reads these directly, in place of the
# /{agent}/perception/tracked_objects the node defaults target for the live
# compose pipeline's tracker.
ITRK=/infrastructure/gt/tracked_objects
VTRK=/vehicle/gt/tracked_objects

# --- reputation starts from the 0.5 default every run.
# The ego persists its reputation for each peer to SQLite and reseeds from it
# on reappearance (decayed toward 0.5 by the time away), so without this the
# peer would start each run wherever the previous one left it and the climb
# from the default would not be visible. Set KEEP_REPUTATION=1 to carry
# history across runs instead.
if [ -z "${KEEP_REPUTATION:-}" ]; then
    echo "=== clearing reputation history (KEEP_REPUTATION=1 to preserve) ==="
    python3 src/CPX-Mono/ros2/src/global_trust_perception/scripts/show_reputations.py \
        --clear-all
fi

# --- foxglove_bridge for Lichtblick, unless one is already up (it binds 8765,
# so a second instance dies with "Bind Error"). Left running afterwards.
if ! pgrep -f 'lib/foxglove_bridge/foxglove_bridge' >/dev/null; then
    echo "=== starting foxglove_bridge on ws://localhost:8765 ==="
    # disowned so the cleanup below (which kills this script's jobs) leaves
    # it up: Lichtblick stays connected between runs.
    nohup ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
        >$LOG/bridge.log 2>&1 &
    disown
    sleep 3
fi

# --- hold /tf_static up for the whole session. `ros2 bag play` publishes it
# once and then exits at the end of the clip, taking the latched message with
# it, so a viewer loses "world -> {agent}_lidar" the moment playback finishes.
# Playback is told to skip /tf_static (see --exclude-topics below) so this is
# the ONLY publisher: two publishers of the same static transform race, and
# with a correction applied here the race would decide whether it is applied
# at all.
#
# YAW_FIX corrects the infrastructure's world pose. The two agents were
# converted separately and their `world` frames do not coincide -- the bags
# record world->infrastructure_base at yaw -2.557 deg and
# world->vehicle_base at -97.976 deg, and measured against each other the
# infrastructure is about 4.3 deg out. Two independent estimates agree:
# a cross-validated rigid fit over matched detections gives 4.277 / 4.167 deg
# on disjoint halves of the clip, and its rotation PIVOT lands on
# (29.3, -158.8) -- the infrastructure sensor's own position, (29.402,
# -157.891), to within a metre -- which is what names the infrastructure as
# the misregistered agent rather than the vehicle. Sweeping the correction
# through the fusion peaks flat over -4.2..-4.5 deg: matched detections per
# frame go 4.80 -> 7.60, while the opposite sign collapses them to 0.25,
# so the sign is not in doubt.
#
# It is applied at REPUBLISH time, not baked into the recording: it is an
# empirical constant for THIS bag pair, and the honest fix is re-converting
# both agents against a shared localization root (see convert_to_rosbag's
# --map-frame). YAW_FIX=0 restores the recorded tree.
YAW_FIX=${YAW_FIX:--4.3}
python3 src/CPX-Mono/ros2/src/local_trust_estimation/scripts/republish_tf_static.py \
    $INFRA $VEH --yaw-correction infrastructure_base=$YAW_FIX \
                                                                   >$LOG/tf.log 2>&1 &

# --- global stage: tracker first, then the single ego judge (infra = agent 1)
ros2 run global_trust_tracker tracker --ros-args $SIM                              >$LOG/tracker.log 2>&1 &
sleep 2
# FLUSH is how wide an INSTANT is; BUDGET is how long ego waits for the slow
# sender describing it. They used to be one knob and it could not serve both.
# Both chains process the SAME instant -- the streams are aligned to 0.03 s --
# but finish ~59 ms apart (33 ms for the infrastructure chain, 92 ms for the
# vehicle's, which carries a denser cloud). Grouped by ARRIVAL, ego's own frame
# and the peer's landed in one 50 ms bucket only ~41% of the time; in the other
# 59% ego contributed nothing, every peer detection was other_only by
# construction, and the frame produced zero matches no matter how well the
# boxes overlapped. The fix was to widen FLUSH to 0.15 -- which also widened
# the span of real time a frame CLAIMED was simultaneous, and that is a debt
# that comes due with speed: at 30 m/s a 150 ms frame pairs boxes up to 4.5 m
# apart, which is the whole IoU association budget for a car.
# Messages are now grouped by CAPTURE time instead (each sender reports its own
# chain lag, see sdsm_publisher's capture_lag_ms), so BUDGET absorbs the spread
# and the window FLUSH states is a span of capture instants rather than of
# arrivals -- the 150 ms below now really does mean "within 150 ms of each
# other", where before it meant 150 ms of inbox plus up to 59 ms of chain.
#
# FLUSH CANNOT BE SHRUNK TO NOTHING, and this was measured the hard way.
# Capture grouping removes the PROCESSING spread; it does NOT remove the
# SENSOR offset -- the two lidars are not synchronized, and the residual
# stream alignment is the same 0.03 s the read-ahead note below is about.
# Bucketing is a fixed grid, so a pair captured d apart is split whenever a
# boundary falls between them, with probability ~d/FLUSH: at FLUSH=0.05 that
# is ~60% split, which measured out as 15 of the vehicle's 80 messages lost
# and reputation oscillating between its bounds. At 0.15 it is ~20%.
# So FLUSH is bounded BELOW by the sensor offset and above by how much real
# motion a frame may pretend is simultaneous. Tune it, do not minimize it.
#
# BUDGET is sized from the slowest chain, also measured: at 0.15 the vehicle
# lost 19% of its messages to 'dropped N message(s) from bucket M, already
# judged' -- its chain sometimes runs far past the nominal 92 ms under load in
# this container. 0.5 covers it. The cost is decision latency, not accuracy.
# Raise BUDGET, not FLUSH, if that warning appears.
ros2 run global_trust_perception agent --ros-args $SIM \
    -p agent_id:=1 -p mode:=replay -p flush_interval_s:=${FLUSH:-0.15} \
    -p close_budget_s:=${BUDGET:-0.5} \
    -r __node:=agent_1                                             >$LOG/agent1.log 2>&1 &

# --- read-only visualizers for the Lichtblick layout. Both publish in the
# world frame the recovered poses put the two agents in, which is now both
# nodes' default; frame_id stays spelled out here because getting it wrong is
# invisible rather than loud -- an entity in a frame the TF tree does not
# contain is silently dropped, so the point cloud renders and the boxes do
# not, which reads as "the global stage isn't publishing".
ros2 run global_trust_perception scene --ros-args $SIM \
    -p ego_source_id:=1 -p frame_id:=map                           >$LOG/scene.log 2>&1 &
ros2 run global_trust_perception trust_view --ros-args $SIM \
    -p ego_source_id:=1 -p frame_id:=map                           >$LOG/trust_view.log 2>&1 &
# The aerial backdrop, as a mesh_resource marker (Lichtblick's URDF layer draws
# the glb but not its texture -- see ground_backdrop_node's docstring). Static,
# so frame_id is all it needs; alignment is baked into the node's defaults.
# OFF BY DEFAULT now (set BACKDROP=1 to draw the aerial). When hand-tuning it,
# keep it off and start a standalone node instead: two publishers on the same
# latched topic overwrite each other every 2 s, so the view flickers between
# poses. Tune the standalone one live from Lichtblick's Parameters panel (see
# the node's docstring), then fold the result back into the defaults here.
if [ "${BACKDROP:-0}" != "0" ]; then
    ros2 run global_trust_perception ground_backdrop --ros-args $SIM \
        -p frame_id:=map                                          >$LOG/ground.log 2>&1 &
fi

# The lanelet-map overlay: lane boundaries drawn on the aerial (see
# lanelet_overlay_node's docstring). Needs the clipped map produced by
# tools/lichtblick/clip_lanelet_osm.py; if it is absent the node logs and draws
# nothing, so this is safe to leave on. Alignment defaults match the backdrop's;
# LANELET=0 skips it.
if [ "${LANELET:-1}" != "0" ]; then
    ros2 run global_trust_perception lanelet_overlay --ros-args $SIM \
        -p frame_id:=map                                          >$LOG/lanelet.log 2>&1 &
fi

# --- local stage, INFRASTRUCTURE (agent 1, ego): every topic remapped
ros2 run local_trust_estimation object_shape --ros-args $SIM \
    -p input_topic:=$ITRK \
    -p output_topic:=$I/object_shape                               >$LOG/i_shape.log 2>&1 &
ros2 run local_trust_estimation lidar_point_count --ros-args $SIM \
    -p cloud_topic:=/infrastructure/lidar/points \
    -p tracked_objects_topic:=$ITRK \
    -p output_topic:=$I/lidar_point_count                          >$LOG/i_pc.log 2>&1 &
ros2 run local_trust_estimation object_distance --ros-args $SIM \
    -p input_topic:=$ITRK \
    -p sensor_frame:=infrastructure_lidar \
    -p output_topic:=$I/object_distance                            >$LOG/i_dist.log 2>&1 &
ros2 run local_trust_estimation temporal_presence --ros-args $SIM \
    -p input_topic:=$ITRK \
    -p output_topic:=$I/temporal_presence                          >$LOG/i_temp.log 2>&1 &
ros2 run local_trust_estimation trustworthiness_score --ros-args $SIM \
    -p object_shape_topic:=$I/object_shape \
    -p lidar_point_count_topic:=$I/lidar_point_count \
    -p object_distance_topic:=$I/object_distance \
    -p temporal_presence_topic:=$I/temporal_presence \
    -p output_topic:=$I/score                                      >$LOG/i_score.log 2>&1 &
ros2 run local_trust_estimation trustworthiness_visualization --ros-args $SIM \
    -p tracked_objects_topic:=$ITRK -p score_topic:=$I/score \
    -p output_topic:=$I/markers                                    >$LOG/i_viz.log 2>&1 &
ros2 run local_trust_estimation sdsm_publisher --ros-args $SIM \
    -p tracked_objects_topic:=$ITRK -p score_topic:=$I/score \
    -p sensor_frame:=infrastructure_lidar -p global_frame:=map \
    -p agent_id:=1 -p equipment_type:=1                            >$LOG/i_sdsm.log 2>&1 &

# --- local stage, VEHICLE (agent 2): the /local_trust_estimation/* score chain
# stays on defaults, but the SOURCE topic must be overridden -- the node
# defaults now target the tracker's /vehicle/perception/tracked_objects, while
# GT replay publishes /vehicle/gt/tracked_objects.
ros2 run local_trust_estimation object_shape --ros-args $SIM \
    -p input_topic:=$VTRK                                          >$LOG/v_shape.log 2>&1 &
ros2 run local_trust_estimation lidar_point_count --ros-args $SIM \
    -p tracked_objects_topic:=$VTRK                                >$LOG/v_pc.log 2>&1 &
ros2 run local_trust_estimation object_distance --ros-args $SIM \
    -p input_topic:=$VTRK                                          >$LOG/v_dist.log 2>&1 &
ros2 run local_trust_estimation temporal_presence --ros-args $SIM \
    -p input_topic:=$VTRK                                          >$LOG/v_temp.log 2>&1 &
ros2 run local_trust_estimation trustworthiness_score --ros-args $SIM              >$LOG/v_score.log 2>&1 &
ros2 run local_trust_estimation trustworthiness_visualization --ros-args $SIM \
    -p tracked_objects_topic:=$VTRK                                >$LOG/v_viz.log 2>&1 &
ros2 run local_trust_estimation sdsm_publisher --ros-args $SIM \
    -p tracked_objects_topic:=$VTRK -p global_frame:=map \
    -p agent_id:=2                                                 >$LOG/v_sdsm.log 2>&1 &

sleep 6

# --- RECORD=1 captures exactly what the Lichtblick layout draws into one
# MCAP, so the 17 s run can be scrubbed afterwards instead of watched once.
# Open it with Lichtblick's "Open local file" -- the browser reads it off the
# host, so nothing has to be mounted or forwarded.
if [ -n "${RECORD:-}" ]; then
    OUT=${OUT:-src/CPX-Mono/simulations/scene_01_ros2/two_agent_viz}
    rm -rf "$OUT"
    echo "=== recording -> $OUT ==="
    ros2 bag record --storage mcap --output "$OUT" \
        --disable-keyboard-controls \
        /tf /tf_static \
        /infrastructure/lidar/points /vehicle/lidar/points \
        /infrastructure/local_trust_estimation/markers \
        /local_trust_estimation/markers \
        /perception/global_trustworthiness/viz/scene_raw \
        /perception/global_trustworthiness/viz/scene_trust \
        /perception/global_trustworthiness/viz/scene_ground \
        /perception/global_trustworthiness/viz/ground_backdrop \
        /perception/global_trustworthiness/viz/ego_boxes \
        /perception/global_trustworthiness/viz/peer_boxes \
        /perception/global_trustworthiness/viz/fused_boxes_markers \
        /perception/global_trustworthiness/verdicts/agent_1 \
        /perception/global_trustworthiness/sdsm                     >$LOG/record.log 2>&1 &
    RECORDER=$!
    sleep 3
fi

echo "=== playing both bags ==="
# --read-ahead-queue-size has to sit between two failures, and the window is
# narrower than it looks. TOO LARGE: the default (1000) buffers ~2 GB of raw
# clouds before the first message is published, which looks like a hang at
# 0.00, and exhausts a 3 GB container outright. TOO SMALL: `ros2 bag play`
# orders messages ACROSS INPUT BAGS only within this queue (see its --help),
# and these two recordings start 4.72 s apart, so a queue holding less than
# that much content cannot interleave them. It then plays each bag from its
# own beginning at the same instant, and the infrastructure stream serves
# frames 4.72 s older than the vehicle's for the whole run -- measured, with
# nothing subscribed but a probe: 20 gives a 4.03 s stream skew, 200 gives
# 0.03 s. Every downstream trust number is meaningless when this is wrong,
# because ego then corroborates its own view of t against the peer's view of
# t+4.7 s. Re-check it against the stream skew for any other bag pair.
# --start-offset defaults to 4.72 s (the vehicle's start, see below): each bag's
# /tf_static is stamped at its own first frame, so skipping past it would leave
# that agent's sdsm_publisher unable to resolve world -> {agent}_lidar and drop
# every frame -- safe here ONLY because republish_tf_static.py (above) holds
# /tf_static independently. --playback-duration is safe -- it trims the tail,
# which carries no tf_static.
# RATE slows playback so the labels and verdict changes are actually readable
# (RATE=0.5 is half speed, RATE=0.25 quarter). It is a PURE VIEWING aid: with
# --clock above and use_sim_time on every node, the trust pipeline measures
# scene time, so the same clip judged at 0.25 and at 1.0 reaches the same
# verdicts. It did NOT used to be: the flush timer ran on the wall, so half
# speed gave twice as many flushes per scene second and the deferred ledger's
# deadline expired after half as much scene -- an honest peer that survived at
# RATE=1.0 was drained to zero at RATE=0.5.
# Send latency stays on the wall deliberately (agent.py's recv_wall): a
# message really did take that long to arrive, and playback speed does not
# change that.
# --playback-duration is in BAG time, so it covers the same clip either way.
# DUR / OFFSET trim the clip, both in BAG seconds. The VEHICLE recording is
# only 10.3 s long and starts 4.72 s after the infrastructure one, so the
# default OFFSET=4.72 DUR=10.3 starts the clip where the vehicle starts and runs
# its full length, keeping both agents present throughout. Widen the window
# (OFFSET=0, larger DUR) to see the infrastructure's solo lead-in, where agent 2
# is simply not in the data yet.
# Safe here only because republish_tf_static.py (above) holds /tf_static up
# independently of playback; without it, skipping past each bag's own
# tf_static would leave sdsm_publisher unable to resolve world -> {agent}_lidar
# and it would drop every frame.
play_once() {
    # --exclude-topics /tf_static: republish_tf_static.py above is the single
    # publisher of the static tree, so the YAW_FIX correction cannot be
    # overwritten by the bag's own uncorrected copy landing after it.
    ros2 bag play -i $INFRA -i $VEH --disable-keyboard-controls $CLOCK \
        --exclude-topics /tf_static \
        --read-ahead-queue-size ${QUEUE:-200} --playback-duration ${DUR:-10.3} \
        --start-offset ${OFFSET:-4.72} \
        --rate ${RATE:-0.5}
}

# LOOP=1 replays the clip instead of playing it once. Off by default: a single
# pass is ~20 s of real time (10.3 s of bag at RATE=0.5), which is enough to
# watch once and is the more predictable default for a single screenshot/take.
# It cannot combine with EXIT_AFTER (a scripted caller wants one pass and a
# clean exit) or RECORD (whose whole point is one bounded clip to scrub
# afterward, see below) -- looping either would run with no natural stop.
if [ "${LOOP:-0}" != "0" ] && [ -z "${EXIT_AFTER:-}" ] && [ -z "${RECORD:-}" ]; then
    echo "=== playing both bags on loop (Ctrl+C to stop) ==="
    while :; do play_once; done                                    >$LOG/play.log 2>&1
else
    echo "=== playing both bags ==="
    play_once                                                      >$LOG/play.log 2>&1
    echo "=== playback done ==="
    sleep 3
fi

# The recorder needs SIGINT, not SIGTERM, and needs to be waited on: rosbag2
# writes the MCAP summary/footer during its shutdown, and a file killed before
# that is unreadable by Lichtblick. SIGINT the process group, since `ros2 bag
# record` is a wrapper whose child does the writing.
if [ -n "${RECORDER:-}" ]; then
    echo "=== closing recording ==="
    pkill -INT -f 'bag record' 2>/dev/null
    for _ in $(seq 30); do
        pgrep -f 'bag record' >/dev/null || break
        sleep 1
    done
fi

# Stay up after playback unless told otherwise. The clip is ~17 s; tearing the
# graph down at the end leaves nothing publishing /tf_static or the scene
# topics, so a viewer that looks a moment later sees an empty 3D panel and
# "Missing transform from <agent>_lidar to <world>". Ctrl+C when done.
if [ -z "${EXIT_AFTER:-}" ]; then
    echo
    echo "=== nodes still running: TF, scene and trust views stay published."
    echo "=== Ctrl+C to stop.  (EXIT_AFTER=1 to tear down automatically.)"
    wait
fi

kill $(jobs -p) 2>/dev/null
wait 2>/dev/null
echo "=== stopped ==="
