"""
Combined V2X agent node: publisher AND subscriber on the perception topic.

Each agent:
- in 'sim' mode (default), publishes its own perception message at 2 Hz; in
  'replay' mode, publishes nothing and only consumes whatever already
  publishes prerecorded SdsmPayload messages on the perception topic (e.g.
  `ros2 bag play`)
- subscribes to the perception topic and feeds all agents' detections
  (including its own echo, in sim mode) into its embedded TrustEngine, in
  frame buckets of flush_interval_s -- 0.5 s in sim, aligned to the 2 Hz
  publish rate, and 50 ms in replay; ego detections serve as ground truth
  and are never scored or written to the DB
- subscribes to the tracker node's TrackUpdate topic and forwards TrackData
  into TrustEngine.process_frame
- after each flush, rebroadcasts the SDSM of every OTHER agent that passed
  this frame's Stage-0 reputation gate on TRUST_OUTPUT_TOPIC, with
  obj_local_scores zeroed and global_score set to this ego's R_new for
  that sender
- serves GetTrustScore at /agent_<id>/get_trust_score

Run two agents (tracker node must already be running):
    ros2 run global_trust_tracker tracker
    ros2 run global_trust_perception agent --ros-args -p agent_id:=1 -r __node:=agent_1
    ros2 run global_trust_perception agent --ros-args -p agent_id:=2 -r __node:=agent_2

Replay mode (bag already publishing prerecorded messages on the perception
topic):
    ros2 run global_trust_perception agent --ros-args -p agent_id:=1 -p mode:=replay -r __node:=agent_1

The on-the-wire message format lives entirely in perception_message; this
node never names a concrete message type.
"""

import logging
import os
import time
import traceback
from typing import Optional

from global_trust_perception.pipeline import fused_objects as fobj
from global_trust_perception.pipeline import perception_message as pmsg
from global_trust_perception.pipeline import sim_world
from global_trust_perception.pipeline import trust_verdicts as tverd
from global_trust_perception.trust_calculations.consistency import T_DEADLINE_S
from global_trust_perception.trust_calculations.consistency_checks.kinematic_checks import TrackData
from global_trust_perception.pipeline.persistent_reputation_tracker import (
    BATCH_SIZE,
    PersistentReputationTracker,
)
from global_trust_perception.pipeline.trustworthy_perception import (
    REPUTATION_DEFAULT,
    TrustEngine,
)

# Default frame window per mode. Sim publishes its own synthetic traffic on
# sim_world's clock, so its window must equal that clock or windows would sit
# empty; replay consumes whatever a bag emits and defaults to the 20 Hz
# production target. Either can be overridden with the flush_interval_s param.
_REPLAY_FLUSH_INTERVAL_S = 0.05

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sdsm_interfaces.msg import TrackUpdate
from sdsm_interfaces.srv import GetTrustScore

_TRACKS_TOPIC = '/perception/global_trustworthiness/tracks'

# Most TrackUpdates retained across flushes while their SDSMs are still in
# flight. See where it is applied in flush_frame for why the buffer outlives a
# single frame at all.
_TRACK_BUFFER_MAX = 512

_MODES = ('sim', 'replay')


def _default_db_path(agent_id: int) -> str:
    """
    Absolute path for this agent's reputation DB, inside the CPX-Mono repo.

    Derived from COLCON_PREFIX_PATH (set by `source install/setup.bash`) so the
    DB always lands in <ros2_ws>/src/CPX-Mono/ros2/data/ regardless of the
    directory the node is launched from. Falls back to the working directory
    if the workspace isn't sourced.
    """
    prefix = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)[0]
    if prefix:
        ws_root = os.path.dirname(prefix)  # <ros2_ws>/install -> <ros2_ws>
        data_dir = os.path.join(ws_root, 'src', 'CPX-Mono', 'ros2', 'data')
    else:
        data_dir = os.getcwd()
    os.makedirs(data_dir, exist_ok=True)
    return os.path.join(data_dir, f'historical_reputations_{agent_id}.db')


def _concat_track_data(updates: list[TrackUpdate]) -> TrackData:
    """Concatenate TrackData from multiple TrackUpdates (one per buffered SDSM)."""
    return TrackData(
        track_ids=sum((list(u.track_id)  for u in updates), []),
        kalman_x= sum((list(u.kalman_x)  for u in updates), []),
        kalman_y= sum((list(u.kalman_y)  for u in updates), []),
        kalman_vx=sum((list(u.kalman_vx) for u in updates), []),
        kalman_vy=sum((list(u.kalman_vy) for u in updates), []),
    )


class _RunningStat:
    """Count, mean and max of a diagnostic series, without keeping the series.

    The probe runs every flush for the life of the node, so retaining samples
    would grow without bound for a number only ever read as a summary. Mean
    and max are what a latency budget is argued from; the full distribution
    comes from the offline benchmark, which can afford to store it.
    """

    def __init__(self):
        self.count = 0
        self._total = 0.0
        self.max = 0.0

    def add(self, value: float) -> None:
        self.count += 1
        self._total += value
        if value > self.max:
            self.max = value

    @property
    def mean(self) -> float:
        """Mean so far, or 0.0 before anything has been added."""
        return self._total / self.count if self.count else 0.0


def _capture_time_of(entry: tuple) -> float:
    """
    Scene time this buffered message's sensor frame was CAPTURED.

    (msg, recv_wall, recv_scene) -> recv_scene - capture_lag.

    Transmission latency is deliberately NOT subtracted as well, even though
    it is measured a few lines away in flush_frame. The two legs live on
    different clocks on purpose (see pmsg.capture_lag_of): the capture lag is
    scene time and reproducible, while the transmission latency is wall time
    across two hosts. Mixing them would make which bucket a message lands in
    depend on the wall clock, and so on playback rate -- reintroducing exactly
    what use_sim_time was adopted to remove. The neglected term is the time on
    the wire, which is sub-millisecond on the local transport this runs over
    and is already the freshness gate's concern, not grouping's. A real link
    wants a sender-stamped capture time on a synchronized clock, not this
    term added here.
    """
    return entry[2] - pmsg.capture_lag_of(entry[0])


def _latest_per_sender(
    frame_msgs: list[tuple[pmsg.Message, float, float]],
) -> tuple[list[tuple[pmsg.Message, float, float]], int]:
    """
    Reduce a frame window to one message per sender: the last one to arrive.

    A sender publishing faster than the flush timer, or one whose message
    jitters across a window boundary, lands twice in the same window. Decoding
    both concatenates their detections, so one physical object enters fusion
    twice at two different timestamps and reads as the sender contradicting
    itself. The scalar fields (raw_msg_by_agent, equipment_by_agent,
    source_id_map) already kept only the last message, so the published
    verdict cited one message's counter while the scoring had used both.

    Keeping the latest resolves both halves: a superseded message is stale
    state the newer one replaces, and every field now describes the same
    message. The superseded message's TrackUpdate is simply never looked up,
    since lookup is keyed by the message counter.

    Selection is by arrival order rather than by ``pmsg.sequence_of``, whose
    counter wraps; reordering inside a single window is not a realistic
    concern. Senders appear in the order they first sent this window, which
    downstream does not depend on -- decoding groups by sender.

    Returns (deduped messages, count of messages dropped as superseded).
    """
    latest: dict[int, tuple[pmsg.Message, float, float]] = {}
    for entry in frame_msgs:
        latest[pmsg.sender_of(entry[0])] = entry
    return list(latest.values()), len(frame_msgs) - len(latest)


class AgentNode(Node):

    def __init__(self):
        super().__init__('agent')

        self.agent_id = int(self.declare_parameter('agent_id', 1).value)

        self.mode = str(self.declare_parameter('mode', 'sim').value)
        if self.mode not in _MODES:
            raise ValueError(f"mode={self.mode!r} must be one of {_MODES}")
        self.flush_interval_s = float(self.declare_parameter(
            'flush_interval_s',
            sim_world.TICK_DT_S if self.mode == 'sim' else _REPLAY_FLUSH_INTERVAL_S,
        ).value)
        if self.flush_interval_s <= 0.0:
            raise ValueError(
                f'flush_interval_s={self.flush_interval_s} must be > 0')

        # How long a capture bucket stays open past its own end, waiting for
        # slower senders. THIS is what absorbs the spread in how long each
        # agent's perception chain takes -- not flush_interval_s, which now
        # only states how wide an instant is.
        #
        # Splitting the two is the point of bucketing by capture time. Before
        # it, one knob did both jobs and they pull opposite ways: widening it
        # to catch a slow sender also widened the span of real time a frame
        # claimed was simultaneous, so at speed the window that fixed pairing
        # was the same window that made honest reports fail to associate.
        # Now flush_interval_s can be tightened for accuracy while this is
        # sized independently, from the slowest chain in the graph.
        self.close_budget_s = float(self.declare_parameter(
            'close_budget_s', 0.15).value)
        if self.close_budget_s < 0.0:
            raise ValueError(
                f'close_budget_s={self.close_budget_s} must be >= 0')
        # How far behind the last judged bucket an arrival can be and still be
        # explained by lateness. Beyond it, the capture times are not late --
        # they are from another timeline (see _take_closed_bucket). Derived
        # from the budget rather than tuned, plus one bucket of slack for the
        # boundary case, so it tracks any retuning of close_budget_s.
        self._max_late_buckets = int(
            self.close_budget_s / self.flush_interval_s) + 2
        # Steady state holds roughly one bucket per (interval + budget) worth
        # of senders; an order of magnitude above that is a backlog, not
        # jitter. Warned about once it is exceeded (see _take_closed_bucket).
        self._backlog_warn = 10 * (self._max_late_buckets + 1)

        db_path = str(self.declare_parameter('db_path', _default_db_path(self.agent_id)).value)
        self.reputationDB = PersistentReputationTracker(db_path)
        self._frame_count = 0 # in Python 3, standard integers have no maximum limit and cannot overflow --> ok to accumulate indefinitely

        self.visualize = bool(self.declare_parameter('visualize', False).value)
        if self.visualize:
            import matplotlib.pyplot as plt
            plt.ion()

        # OWN view of every other agent's trust. The engine decays its
        # persistence penalty once per flush, so it is told the real rate.
        # How long an uncorroborated other_only track is held before
        # the ledger charges it. Exposed because the right value depends on the
        # scene: agents with overlapping views corroborate each other in a
        # second, while two sensors covering mostly-disjoint ground may never,
        # and charging those punishes an honest unique vantage.
        self.deferred_deadline_s = float(self.declare_parameter(
            'deferred_deadline_s', T_DEADLINE_S).value)
        self.engine = TrustEngine(self.reputationDB,
                                  flush_hz=1.0 / self.flush_interval_s,
                                  deadline_s=self.deferred_deadline_s,
                                  now=self._now)

        # incoming (message, recv_wall, recv_scene) triples buffered until the
        # capture bucket they belong to closes -- see on_message for why
        # arrival is stamped on two clocks and which quantity each one feeds.
        self.frame_buffer: list[tuple[pmsg.Message, float, float]] = []
        # Highest capture-bucket index already judged. A message for this
        # bucket or earlier arrived after its frame was decided and cannot be
        # folded in retroactively: reopening a closed judgement would let
        # arrival order change a verdict. It is dropped and counted instead.
        self._last_bucket: Optional[int] = None
        # DIAGNOSTIC: messages dropped for arriving after their bucket closed.
        # A nonzero rate means close_budget_s is under the real chain spread.
        self._diag_late = 0

        # TrackUpdate messages buffered since last flush, keyed by (source_id_tuple, msg_cnt)
        self._track_buffer: dict[tuple, TrackUpdate] = {}

        # DIAGNOSTIC: tick→flush gap and TrackUpdate miss tracking
        self._tick_end_t: Optional[float] = None
        self._diag_track_hits = 0
        self._diag_track_misses = 0
        # DIAGNOSTIC: messages dropped by _latest_per_sender. A nonzero rate
        # means senders are drifting across window boundaries relative to the
        # flush timer, which is worth knowing when reading frame latencies.
        self._diag_superseded = 0
        # DIAGNOSTIC: frames the engine failed on and flush_frame swallowed to
        # keep the node alive. Expected to stay 0; anything else is a bug that
        # the last-resort guard is hiding from the executor, not a tuning knob.
        self._diag_frame_errors = 0
        # DIAGNOSTIC: the two latency terms this node can observe directly.
        # D_tx is send -> receive; W_batch is receive -> the flush that
        # consumes the message. Together with the measured flush duration they
        # account for everything between a sender's stamp and its verdict, so
        # the offline latency model can be checked against a live run rather
        # than trusted on its own.
        self._diag_tx = _RunningStat()
        self._diag_wait = _RunningStat()

        # --- ROS interfaces ---
        # The topic is absolute on purpose: all agents share one topic.
        self.pub = self.create_publisher(pmsg.Message, pmsg.TOPIC, 10)
        self.sub = self.create_subscription(
            pmsg.Message, pmsg.TOPIC, self.on_message, 10
        )
        # rebroadcast, per gate-passing sender, after this ego's flush_frame.
        # Published on a per-ego topic so a consumer knows whose trust view a
        # rebroadcast represents from the channel, without adding a field to the
        # payload (see pmsg.trust_output_topic). The OUTPUT message type differs
        # from the input: no per-object local scores, one global score instead,
        # and only senders that passed the gate are ever published here.
        self.trust_output_pub = self.create_publisher(
            pmsg.OutputMessage, pmsg.trust_output_topic(self.agent_id), 10
        )
        # Vehicle-internal diagnostics for the visualizer: EVERY judged sender,
        # trusted or NOT, plus per-detection verdicts. Deliberately a separate
        # topic and message type from trust_output above, which must never
        # carry a sender that failed the gate (see trust_verdicts.py).
        self.verdicts_pub = self.create_publisher(
            tverd.Message, tverd.topic(self.agent_id), 10
        )
        # The fused scene, for the visualizer only (see fused_objects.py). The
        # geometry already exists every frame; publishing it is gated on there
        # being a subscriber, so an unwatched run pays one integer compare.
        self.fused_pub = self.create_publisher(
            fobj.Message, fobj.topic(self.agent_id), 10
        )
        self.track_sub = self.create_subscription(
            TrackUpdate, _TRACKS_TOPIC, self._on_track_update, 100
        )

        # service name is derived from agent_id so two agents never collide.
        self.srv = self.create_service(
            GetTrustScore,
            f'/agent_{self.agent_id}/get_trust_score',
            self.handle_get_trust_score,
        )

        # replay mode: no self-published synthetic traffic -- this agent is a
        # pure subscriber to whatever already publishes prerecorded SdsmPayload
        # messages on pmsg.TOPIC (e.g. `ros2 bag play`).
        if self.mode == 'sim':
            # sim_world advances its scene one step per tick and reports speeds
            # as metres per TICK_DT_S, so publishing on any other period would
            # make every reported speed wrong by that ratio.
            self.publish_timer = self.create_timer(sim_world.TICK_DT_S, self.tick)
        self.flush_timer = self.create_timer(self.flush_interval_s, self.flush_frame)
        self.i = 0

        self.get_logger().info(
            f'Agent {self.agent_id} up ({self.mode} mode): pub+sub on {pmsg.TOPIC}, '
            f'rebroadcasting gate-passers on {pmsg.trust_output_topic(self.agent_id)}, '
            f'trust service /agent_{self.agent_id}/get_trust_score, '
            f'frame window={int(self.flush_interval_s * 1000)}ms, R_default={REPUTATION_DEFAULT}'
        )

    def _now(self) -> float:
        """Seconds on the NODE clock -- the one every trust decision is timed on.

        Under `use_sim_time:=true` (with `ros2 bag play --clock`) this is the
        bag's clock, so scene time advances at the same rate no matter what
        `--rate` playback runs at. That is what keeps a judgement identical
        between a real-time run and a slowed-down one you are watching: every
        window the pipeline derives -- the deferred ledger's deadline, sender
        speed, send latency -- is measured against this, not the wall.

        Without use_sim_time it is ordinary wall time, i.e. today's behaviour.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    # publishing
    def tick(self):
        # DIAGNOSTIC clocks stay on the WALL deliberately (here and in
        # flush_frame): these measure how long our own code took to run, which
        # is a property of the machine, not of the scene. Slowing playback must
        # not make the pipeline look faster than it is.
        self._tick_end_t = time.monotonic()  # DIAGNOSTIC: record when timer fires
        msg = pmsg.build(self.agent_id, self.i)
        self.pub.publish(msg)
        self.get_logger().info(
            f'Published cnt={pmsg.sequence_of(msg)} '
            f'as agent_id={self.agent_id} '
            f'with {pmsg.num_detections_of(msg)} object(s)'
            '\n'
        )
        self.i += 1

    # subscribing
    def on_message(self, msg: pmsg.Message):
        """Buffer all perception messages (with receive time) for this flush window."""
        # TWO clocks on purpose, both captured at arrival (not at flush, so
        # flush lag never counts against the sender's freshness):
        #   recv_wall  -- real UTC-aligned time, the ONLY thing comparable
        #                 against the sender's J2735 send stamp
        #                 (sdsm_time_of_day_ms), which another node writes from
        #                 its own wall clock. Transmission delay is a physical
        #                 fact and does not slow down when playback does.
        #   recv_scene -- node clock, i.e. bag time under use_sim_time. Used
        #                 for the dt between a sender's consecutive messages,
        #                 which is a SCENE interval: how far the sender could
        #                 have travelled. Reading that on the wall would halve
        #                 every sender's apparent speed at --rate 0.5.
        recv_wall = time.time()
        recv_scene = self._now()
        sender = pmsg.sender_of(msg)
        self.get_logger().info(
            f'Got cnt={pmsg.sequence_of(msg)} from agent_id={sender} '
            f'with {pmsg.num_detections_of(msg)} object(s)'
        )
        self.frame_buffer.append((msg, recv_wall, recv_scene))

    def _on_track_update(self, msg: TrackUpdate):
        """Buffer TrackUpdate messages keyed by (source_id_tuple, msg_cnt)."""
        key = (tuple(int(x) for x in msg.source_id), int(msg.msg_cnt))
        self._track_buffer[key] = msg

    def _take_closed_bucket(self) -> Optional[list]:
        """
        Remove and return the oldest capture bucket that has closed, or None.

        A bucket is the set of messages whose sensor frames were captured
        inside one flush_interval_s of SCENE time -- so what it groups is one
        instant of the world, not one instant of this node's inbox. It closes
        close_budget_s after its own end, which is the slack that lets a
        sender whose perception chain is slower still land in the frame it
        belongs to.

        ONE bucket per call, oldest first, so the engine keeps seeing one
        frame per call at roughly the timer's rate -- its persistence penalty
        decays per flush and its deferred ledger counts deadlines in flushes,
        so draining a backlog in a single call would age both by several
        frames' worth at one instant.

        Messages for an already-judged bucket are dropped here rather than
        placed in the next one, which would file a report about the past
        under a present frame.

        Messages are partitioned by their integer bucket key and never by
        message equality: a ROS message compares field by field across its
        256-slot arrays, and two senders reporting the same scene can compare
        equal, so identity of CONTENT is not identity of MESSAGE here.
        """
        while self.frame_buffer:
            keyed = [(int(_capture_time_of(e) // self.flush_interval_s), e)
                     for e in self.frame_buffer]
            oldest = min(key for key, _ in keyed)

            if self._last_bucket is not None and oldest <= self._last_bucket:
                # A backward jump far larger than lateness can explain is a
                # different TIMELINE, not a late message -- the node clock
                # switching to sim time mid-run, or a bag looping. Dropping
                # against a _last_bucket from the old timeline would discard
                # EVERY message for the rest of the run, silently and for
                # ever. Re-anchor instead: losing the ordering guarantee for
                # one frame is recoverable, deadlocking the node is not.
                if oldest < self._last_bucket - self._max_late_buckets:
                    self.get_logger().warning(
                        f'capture time jumped backwards {self._last_bucket} '
                        f'-> {oldest} buckets, far past close_budget_s; '
                        f're-anchoring (clock discontinuity?)'
                    )
                    self._last_bucket = None
                    continue

                late = sum(1 for key, _ in keyed if key == oldest)
                self._diag_late += late
                # WARNING, not debug: this is silent data loss. A sender whose
                # chain is slower than close_budget_s vanishes from the trust
                # view entirely while every other agent keeps updating, which
                # reads as "that agent stopped publishing".
                self.get_logger().warning(
                    f'dropped {late} message(s) from bucket {oldest}, already '
                    f'judged (total={self._diag_late}) -- raise close_budget_s '
                    f'(now {self.close_budget_s * 1000:.0f}ms) to cover the '
                    f'slowest perception chain in the graph'
                )
                self.frame_buffer = [e for key, e in keyed if key != oldest]
                continue   # a later bucket may still be ready this tick

            # Scene time at which this bucket stops accepting arrivals.
            closes_at = (oldest + 1) * self.flush_interval_s + self.close_budget_s
            if self._now() < closes_at:
                # Nothing is ready. That is normal for a tick or two, but a
                # buffer that keeps growing means buckets are arriving faster
                # than one per flush and the node is falling behind -- which
                # looks from the outside like the view freezing, so it is said
                # out loud rather than left to a debug channel.
                if len(self.frame_buffer) > self._backlog_warn:
                    self.get_logger().warning(
                        f'{len(self.frame_buffer)} messages buffered across '
                        f'unclosed capture buckets (oldest {oldest} closes in '
                        f'{(closes_at - self._now()) * 1000:.0f}ms) -- the '
                        f'node is not keeping up with its own flush rate'
                    )
                return None

            self._last_bucket = oldest
            self.frame_buffer = [e for key, e in keyed if key != oldest]
            # Arrival order within the bucket is preserved, which is what
            # _latest_per_sender's "keep the last one to arrive" relies on.
            return [e for key, e in keyed if key == oldest]
        return None

    def flush_frame(self):
        """Every flush_interval_s: run the trust pipeline on one closed bucket."""
        # DIAGNOSTIC: measure gap between tick() publishing and flush firing
        if self._tick_end_t is not None:
            gap_ms = (time.monotonic() - self._tick_end_t) * 1000
            self.get_logger().debug(f'[DIAG] tick→flush gap: {gap_ms:.3f}ms')
        _flush_start = time.monotonic()   # DIAGNOSTIC duration; see tick()
        # W_batch spans arrival to flush and is a real waiting cost, so it is
        # read on the same wall clock as recv_wall; _flush_start stays
        # monotonic, which is what a compute duration needs.
        _flush_wall = time.time()

        bucket_msgs = self._take_closed_bucket()
        if bucket_msgs is None:
            return   # no capture bucket has closed yet; nothing to judge

        frame_msgs, superseded = _latest_per_sender(bucket_msgs)
        if superseded:
            self._diag_superseded += superseded
            self.get_logger().debug(
                f'[DIAG] superseded {superseded} message(s) this window '
                f'(total={self._diag_superseded})'
            )
        # The track buffer is NOT cleared here. It was, back when a flush drained
        # every buffered message: whatever went unclaimed had no later frame to
        # belong to. A flush now takes ONE capture bucket while messages of
        # later buckets are still waiting, and those messages' TrackUpdates are
        # in this same dict -- clearing it would strip them before their own
        # frame ran, and a message without track ids is held ungraded rather
        # than judged, silently disabling the deferred ledger.
        # Entries are popped by the decode loop below as they are consumed;
        # what survives is trimmed after it, keyed on nothing but recency.
        track_snapshot = self._track_buffer

        # decode each sender's detections into global (x,y,z) positions.
        positions_by_agent: dict[int, list] = {}
        dims_by_agent: dict[int, list] = {}
        scores_by_agent: dict[int, list] = {}
        raw_msg_by_agent: dict[int, pmsg.Message] = {}
        headings_by_agent: dict[int, list] = {}
        equipment_by_agent: dict[int, int] = {}
        labels_by_agent: dict[int, list] = {}
        source_id_map: dict[int, tuple] = {}
        track_updates_by_sender: dict[int, list[TrackUpdate]] = {}
        ref_pos_by_agent: dict[int, tuple[float, float, float, float]] = {}

        ego_positions: list = []
        # ego's own last message this window, so the verdict channel can tell a
        # consumer WHICH message matched_ego_index indexes. None in replay
        # mode, where ego broadcasts nothing of its own.
        ego_raw_msg: Optional[pmsg.Message] = None
        ego_dims: list = []
        ego_scores: list = []
        ego_headings: list = []
        ego_equipment: int = 0
        ego_labels: list = []

        for msg, recv_wall, recv_scene in frame_msgs:
            sender = pmsg.sender_of(msg)
            # look up the corresponding TrackUpdate for this SDSM
            key = (pmsg.sender_id_tuple(msg), pmsg.sequence_of(msg))
            tu = track_snapshot.pop(key, None)
            if tu is not None:
                self._diag_track_hits += 1
            else:
                self._diag_track_misses += 1
                self.get_logger().warn(
                    f'[DIAG] TrackUpdate miss: sender={sender} msg_cnt={pmsg.sequence_of(msg)} '
                    f'(hits={self._diag_track_hits} misses={self._diag_track_misses})'
                )

            if sender == self.agent_id:
                # ego echo: ground truth for scoring, never scored or written to DB
                ego_positions.extend(pmsg.get_global_positions_of(msg))
                ego_dims.extend(pmsg.get_dims_of(msg))
                ego_scores.extend(pmsg.get_local_scores_of(msg))
                ego_headings.extend(pmsg.get_headings_of(msg))
                ego_equipment = pmsg.get_equipment_type_of(msg)
                ego_labels.extend(pmsg.get_labels_of(msg))
                ego_raw_msg = msg
                # tu is looked up (and popped) for ego too, so the buffer does
                # not accumulate ego's TrackUpdates and the hit/miss diagnostic
                # covers every SDSM -- but ego's track state has no consumer in
                # the engine: ego is never self-checked or ledger-graded.
            else:
                positions_by_agent.setdefault(sender, []).extend(
                    pmsg.get_global_positions_of(msg)
                )
                dims_by_agent.setdefault(sender, []).extend(
                    pmsg.get_dims_of(msg)
                )
                scores_by_agent.setdefault(sender, []).extend(
                    pmsg.get_local_scores_of(msg)
                )
                headings_by_agent.setdefault(sender, []).extend(
                    pmsg.get_headings_of(msg)
                )
                equipment_by_agent[sender] = pmsg.get_equipment_type_of(msg)
                labels_by_agent.setdefault(sender, []).extend(
                    pmsg.get_labels_of(msg)
                )
                source_id_map[sender] = pmsg.sender_id_tuple(msg)
                # exactly one message per sender reaches here; _latest_per_sender
                # dropped any earlier ones this window
                raw_msg_by_agent[sender] = msg
                # Latency is measured on the WALL against the sender's own
                # wall-written stamp; the sender-motion dt is measured on the
                # scene clock. See on_message for why the two cannot share one.
                #
                # Taken absolutely, in replay as in sim. An earlier version
                # anchored each sender's FIRST message to latency 0 in replay
                # mode, on the premise that a replayed sender's stamp is frozen
                # at original-recording time and so incomparable with live
                # wall-clock. That premise does not hold for this pipeline:
                # sdsm_publisher_node.timestamp() only freezes the stamp under
                # offline=True, which only evaluate_offline sets, and that path
                # never reaches this node. Every sender arriving here therefore
                # stamps live wall-clock already, so send_latency_of is correct
                # as it stands -- while the anchor SUBTRACTED a real latency
                # from every later message, and because the first message of a
                # session pays DDS discovery it was reliably the slowest one.
                # Measured: a 19.4 ms first message against a 2 ms steady state
                # put 71 of 76 later messages below kinematic_freshness's
                # 10 ms clock-skew guard, which raises rather than clamps.
                latency_s = pmsg.send_latency_of(msg, recv_wall)
                ref_pos_by_agent[sender] = (
                    msg.ref_pos_x, msg.ref_pos_y, recv_scene, latency_s,
                )
                # Ego's own echo is excluded by construction: this branch only
                # runs for other senders, and a self-published message has no
                # transmission to measure.
                self._diag_tx.add(latency_s)
                self._diag_wait.add(max(0.0, _flush_wall - recv_wall))
                if tu is not None:
                    track_updates_by_sender.setdefault(sender, []).append(tu)

        # Bound what survived. Unconsumed entries are either early (their SDSM
        # has not arrived yet) or orphaned (it never will), and the two are not
        # distinguishable from here -- so the buffer is capped by recency
        # rather than reasoned about. dicts keep insertion order, so this drops
        # the oldest. The cap only has to exceed the in-flight count, which is
        # the senders reporting across one bucket plus its close budget: a
        # handful, against a limit two orders of magnitude above it.
        if len(self._track_buffer) > _TRACK_BUFFER_MAX:
            self._track_buffer = dict(
                list(self._track_buffer.items())[-_TRACK_BUFFER_MAX:])

        # Assemble per-sender TrackData (concatenates across multiple SSDMs per sender)
        tracks_by_agent: dict[int, TrackData] = {
            sender: _concat_track_data(updates)
            for sender, updates in track_updates_by_sender.items()
        }
        self._frame_count += 1

        # Read once, before the frame: it decides both whether the engine
        # bothers to fuse a peerless frame (fuse_solo_frames) and whether the
        # result is published below. One read, so the two cannot disagree
        # mid-frame and leave a fusion computed but dropped.
        want_fused = self.fused_pub.get_subscription_count() > 0
        self.engine.fuse_solo_frames = want_fused

        # LAST RESORT, not error handling: every failure mode worth naming is
        # handled where it happens (see the freshness fallback at Stage 0).
        # This is here because flush_frame is a TIMER CALLBACK -- rclpy
        # re-raises out of the executor, so any exception the engine lets
        # escape takes the node down for the rest of the run, and with it
        # /tf-independent scene topics, the fused layer and every verdict a
        # viewer is watching. One dropped frame at 0.15 s is recoverable; a
        # dead judge is not. The bucket is already consumed and _frame_count
        # already advanced, so the frame is skipped whole rather than half
        # applied: no fused publish, no DB record, no reputation move.
        # Traceback in full, at ERROR, so this can never quietly become the
        # normal path -- a recurring entry here means a real bug upstairs.
        try:
            stats = self.engine.process_frame(
                positions_by_agent, ego_positions,
                dims_by_agent, ego_dims,
                source_id_map,
                tracks_by_agent=tracks_by_agent if tracks_by_agent else None,
                ref_pos_by_agent=ref_pos_by_agent if ref_pos_by_agent else None,
                scores_by_agent=scores_by_agent if scores_by_agent else None,
                ego_scores=ego_scores if ego_scores else None,
                headings_by_agent=headings_by_agent if headings_by_agent else None,
                ego_headings=ego_headings if ego_headings else None,
                equipment_by_agent=equipment_by_agent if equipment_by_agent else None,
                ego_equipment=ego_equipment,
                classes_by_agent=labels_by_agent if labels_by_agent else None,
                ego_classes=ego_labels if ego_labels else None,
            )
        except Exception:
            self._diag_frame_errors += 1
            self.get_logger().error(
                f'frame {self._frame_count} dropped, node kept alive '
                f'(total={self._diag_frame_errors}):\n{traceback.format_exc()}'
            )
            return
        ego_source_id = pmsg.source_id_tuple_for(self.agent_id)
        ego_msg_cnt = (
            pmsg.sequence_of(ego_raw_msg) if ego_raw_msg is not None else -1
        )
        # Read immediately after process_frame: last_fusion is valid only for
        # the frame just processed (it is cleared at the top of the next one).
        # Skipped entirely with nothing subscribed, so the trust path carries
        # no visualization cost in a headless run.
        if want_fused:
            self.fused_pub.publish(
                fobj.build(self.engine.last_fusion, ego_source_id))
        for s in stats:
            source_id = source_id_map[s.agent_id] # this will crash if the agent_id is not in the map, but that should never happen
            self.reputationDB.record(source_id, s.r_new, self._frame_count, s.risk_persist)

            if s.trusted:
                verdict = f'{s.tier.upper()} (R_eff={s.r_eff:.3f} >= tau={s.tau:.3f})'
            else:
                verdict = f'GATE FAIL - objects withheld (R_eff={s.r_eff:.3f} < tau={s.tau:.3f})'
            self.get_logger().info(
                f'agent={s.agent_id} N={s.n_total:.2f} C={s.correct:.2f} '
                f'I={s.incorrect:.2f} held={s.held} '
                f'kine={s.kine_flagged} attr={s.attr_correct} unc={s.uncorroborated} '
                f'ledger(+C={s.ledger.backpaid:.2f} +I={s.ledger.backcharged:.2f} '
                f'pend={s.ledger.pending}) '
                f'S_frame={s.s_frame:+.3f} F={s.f_factor:.3f} V={s.v_factor:.3f} '
                f'R: {s.r_old:.3f} -> {s.r_new:.3f} [{verdict}]'
                '\n'
            )

            raw = raw_msg_by_agent.get(s.agent_id)

            # Diagnostics for the visualizer go out for EVERY sender, gate-pass
            # or not -- showing what was rejected is the whole point of that
            # channel. Correlated back to the judged detections by msg_cnt.
            if raw is not None:
                self.verdicts_pub.publish(tverd.build(
                    s, ego_source_id, source_id, pmsg.sequence_of(raw), ego_msg_cnt
                ))

            # The OUTPUT, by contrast, stays filtered: a sender that failed the
            # gate is simply absent from it, so nothing downstream can act on
            # perception this ego withheld.
            published = False
            if s.trusted and raw is not None:
                self.trust_output_pub.publish(pmsg.with_global_score(raw, s.r_new))
                published = True
            status = 'PUBLISHED FINAL' if published else 'NOT PUBLISHED FINAL'
            self.get_logger().info(
                f'{status}: agent_number={s.agent_id}, '
                f'old_rep={s.r_old:.3f}, new_rep={s.r_new:.3f}'
            )

        if self._frame_count % BATCH_SIZE == 0:
            self.reputationDB.flush(self._frame_count)

        # DIAGNOSTIC: total flush_frame wall-clock duration
        flush_ms = (time.monotonic() - _flush_start) * 1000
        self.get_logger().debug(
            f'[DIAG] flush duration: {flush_ms:.3f}ms '
            f'(budget={self.flush_interval_s*1000:.0f}ms, '
            f'{"OK" if flush_ms < self.flush_interval_s * 1000 else "OVERRUN"})'
        )

        # DIAGNOSTIC: the end-to-end latency terms, cumulative since startup.
        # W_batch is bounded above by flush_interval_s and should average near
        # half of it for arrivals spread across the window; a mean that has
        # drifted toward the full interval means senders are landing just after
        # a flush and waiting nearly a whole window for the next one.
        if self._diag_tx.count:
            self.get_logger().debug(
                f'[DIAG] latency terms over {self._diag_tx.count} message(s): '
                f'D_tx mean={self._diag_tx.mean * 1000:.3f}ms '
                f'max={self._diag_tx.max * 1000:.3f}ms, '
                f'W_batch mean={self._diag_wait.mean * 1000:.3f}ms '
                f'max={self._diag_wait.max * 1000:.3f}ms '
                f'(window={self.flush_interval_s * 1000:.0f}ms)'
            )

        if self.visualize and positions_by_agent:
            from global_trust_perception.mmcooper_fuse.adapter import StreamInput, fuse
            from global_trust_perception.pipeline.visualization import visualise
            # get_dims_of returns (width, length, height); visualise + the fusion
            # adapter here expect (length, width, height).
            def _to_lwh(d):
                return [(l, w, h) for (w, l, h) in d] if d else []

            stats_by_id = {s.agent_id: s for s in stats}
            # agents that failed the Stage-0 gate: bright red on the raw panel,
            # excluded from the fused output entirely
            rejected_ids = {s.agent_id for s in stats if not s.trusted}

            # Phase-2 OUTPUT fusion over the admitted set (ego always included,
            # keyed by its own id so it leads contributor lists). Display-only and
            # deliberately after the flush timing above — production forwards the
            # admitted detections rather than re-fusing them here.
            phase2 = [StreamInput(
                key=self.agent_id, positions=ego_positions,
                dims=_to_lwh(ego_dims), headings=ego_headings,
                scores=ego_scores, labels=ego_labels or None,
                modality=ego_equipment, reliability=1.0,
            )]
            muted: list = []
            for aid, positions in positions_by_agent.items():
                s = stats_by_id[aid]
                if not s.trusted:
                    continue
                admitted = set(s.admitted)
                dims = dims_by_agent.get(aid) or []
                scores = scores_by_agent.get(aid) or []
                headings = headings_by_agent.get(aid) or []
                labels = labels_by_agent.get(aid) or []
                adm = [j for j in range(len(positions)) if j in admitted]
                if adm:
                    phase2.append(StreamInput(
                        key=aid,
                        positions=[positions[j] for j in adm],
                        dims=_to_lwh([dims[j] for j in adm]) if dims else None,
                        headings=[headings[j] for j in adm] if headings else None,
                        scores=[scores[j] for j in adm] if scores else None,
                        labels=[labels[j] for j in adm] if labels else None,
                        modality=equipment_by_agent.get(aid, 0),
                        reliability=s.r_new,
                    ))
                # muted: gate-passing but not admitted (mid-tier uncorroborated)
                for j in range(len(positions)):
                    if j in admitted:
                        continue
                    cx, cy, cz = positions[j]
                    w, l, h = dims[j] if j < len(dims) else (0.0, 0.0, 0.0)
                    muted.append((cx, cy, cz, l, w, h, aid))

            fusion_result = fuse(phase2, ego_key=self.agent_id)

            all_positions = {self.agent_id: ego_positions, **positions_by_agent}
            all_dims = {aid: _to_lwh(v)
                        for aid, v in ({self.agent_id: ego_dims} | dims_by_agent).items()}
            all_labels = {self.agent_id: ego_labels, **labels_by_agent}
            all_headings = {self.agent_id: ego_headings, **headings_by_agent}
            visualise(
                all_positions, all_dims, all_labels,
                fusion_result, muted=muted,
                ego_agent_id=self.agent_id,
                frame_n=self._frame_count,
                rejected_agent_ids=rejected_ids,
                headings_by_agent=all_headings,
            )

    def destroy_node(self):
        self.reputationDB.close()
        super().destroy_node()

    # service handler
    def handle_get_trust_score(self, request, response):
        """weighted_score = R(agent_id) * local_score, from THIS agent's view."""
        agent_id = int(request.agent_id)
        local_score = float(request.local_score)

        weighted, R, tau = self.engine.weighted_score(agent_id, local_score)

        self.get_logger().info(
            f'[svc] agent={agent_id} R={R:.3f} local={local_score:.3f} '
            f'-> weighted={weighted:.3f}  (tau(R) = {tau:.3f})'
        )

        response.weighted_score = weighted
        return response


def main():
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(name)s | %(message)s',
    )
    logging.getLogger('numba').setLevel(logging.WARNING)
    logging.getLogger('matplotlib').setLevel(logging.WARNING)
    logging.getLogger('PIL').setLevel(logging.WARNING)
    rclpy.init()
    node = AgentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
