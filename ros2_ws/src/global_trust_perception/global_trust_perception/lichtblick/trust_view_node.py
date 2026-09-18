"""
Per-object trust visualization, in ONE agent's frame (read-only).

Three layers, one per topic, meant to be stacked in one Lichtblick 3D panel.
Layer = topic on purpose: the panel's per-topic visibility is then the only
enable/disable control needed, and hiding a layer also stops it on the wire.

  ego_boxes     what ego detected itself. GREYSCALE, never hued -- ego holds
                no verdict on itself. Lightness carries its local score, and
                is INVERTED (confident = dark, doubtful = bright) so a weak
                detection is what draws the eye. See _ego_color.
  peer_boxes    what peers reported, one box per judged detection, hued by
                the verdict ego reached about that object:
                    green   accepted        (matched: corroborated)
                    yellow  pending         (deferred: inside the grace window)
                    red     rejected        (uncorroborated: charged as a ghost)
                Categorical, with fixed lightness -- the hue is a decision,
                not a measurement, and must not read as a gradient.
  fused_boxes   the consensus estimate MS-PSF produced from both of the above,
                at its own coordinates rather than any input's, hued
                red/yellow/green by the fused score itself (< 0.6 / 0.6-0.8 /
                >= 0.8 -- see _fused_color). "What ego will actually use." Also
                carries each peer's floating global-score label (see
                _agent_label): those annotate an AGENT rather than a detection,
                and riding this layer keeps them on screen when peer_boxes is
                hidden to clear the view of per-object boxes.

GEOMETRY IS THE CLASS. A vehicle draws as the car or bus mesh (by length), a
VRU as the pedestrian mesh, and anything the sender could not classify as a
cylinder --
deliberately neither, because an UNKNOWN drawn as a car would read as a
confident claim the sender never made. The class is not decoration: the
engine gates its kinematic scoring on obj_type == vehicle (_vehicle_probs in
trustworthy_perception.py), so a VRU and an UNKNOWN are judged under a
different motion model than a car, and a viewer reading a verdict needs to
know which. See _mesh_for_class.

LABELS carry the class, and the layer where naming it disambiguates one --
'EGO | Car', 'AGENT 2 | Pedestrian' -- with ONE score anywhere in the view:
the fused layer's 'Car | score: 0.87', which is the number ego acts on. The
fused label names no layer because 'score:' appears on no other layer's
label, leaving the room to the class and the score. A local score per box was
a number on every object in two layers at once, and it buried the one that
matters. What a peer's boxes are worth is on the floating agent label
instead (see _agent_label), published on the fused layer, where it annotates
the AGENT it actually describes; the verdict each box got is its colour.

Which input fed which fused box is left to the eye: a fused box sits near its
contributors. Carrying that link explicitly would mean publishing contributor
identities, which FusedObjects deliberately does not (see fused_objects.py).

peer_boxes.verdict_filter narrows the peer layer to `pending_only` or
`rejected_only`; it is read every render, so it can be changed live from
Lichtblick's Parameters panel.

FRAME. Markers are published in the world frame the SDSM positions already
live in; set Lichtblick's *Display frame* to an agent's sensor frame (e.g.
`vehicle_lidar`) and TF does the rest, so this node needs no tf2 and no
transform of its own. Converted bags ship the full `world -> {agent}_base ->
{agent}_lidar` tree; the synthetic sim publishes no TF, so there display
frame and `frame_id` are the same thing.

CORRELATION, not synchronization. The engine judges on a ~500 ms flush
window, so a TrustVerdicts message arrives AFTER the SDSM it judges. Raw
messages are buffered by (sender, msg_cnt) and a sender is drawn only once
its verdict arrives and the two agree on detection count — a flush window
that happened to contain two messages from one sender legitimately produces
more verdicts than the correlated message has objects, and drawing that would
silently mis-colour boxes. Skipping a frame is the honest failure.

That correlation is also why the two input layers are NOT contemporaneous on
screen: ego_boxes draws ego's LATEST frame (see _ego_markers on why tying it
to a verdict was reverted), while peer_boxes draws the older frame the verdict
names. That skew is real and is NOT drawn -- the agent label used to report it
as `dt`, and it was dropped because it is a property of this node's rendering
rather than of the trust decision, so it sat next to the global score reading
like something to act on. The pipeline's own pairing of the two frames is a
separate question again, decided by agent.flush_frame's window.

This node never touches the trust pipeline: it subscribes and republishes.
It reads the vehicle-internal verdict channel, NOT the trust output, because
showing rejected senders is the entire point and the output topic
deliberately cannot represent them.

    ros2 run global_trust_perception trust_view --ros-args -p ego_source_id:=1
"""

import math
import os
from collections import namedtuple

import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from builtin_interfaces.msg import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray

from global_trust_perception.pipeline import fused_objects as fobj
from global_trust_perception.pipeline import perception_message as pmsg
from global_trust_perception.pipeline import trust_verdicts as tverd

EGO_BOXES_TOPIC = '/perception/global_trustworthiness/viz/ego_boxes'
PEER_BOXES_TOPIC = '/perception/global_trustworthiness/viz/peer_boxes'
FUSED_BOXES_TOPIC = '/perception/global_trustworthiness/viz/fused_boxes_markers'

_V = tverd.Message

# peer_boxes.verdict_filter values. 'all' draws every judged detection; the
# other two narrow to one question ("what is being held?", "what was thrown
# away?") without needing a second panel or a second node.
_FILTER_ALL = 'all'
_FILTER_PENDING = 'pending_only'
_FILTER_REJECTED = 'rejected_only'
_FILTERS = {
    _FILTER_ALL: None,
    _FILTER_PENDING: (_V.VERDICT_DEFERRED,),
    _FILTER_REJECTED: (_V.VERDICT_UNCORROBORATED,),
}

_VERDICT_COLOR = {
    _V.VERDICT_MATCHED:        (0.05, 0.85, 0.20),   # green:  accepted
    _V.VERDICT_DEFERRED:       (1.00, 0.80, 0.00),   # yellow: pending
    _V.VERDICT_UNCORROBORATED: (0.95, 0.10, 0.10),   # red:    rejected
}
_UNKNOWN_COLOR = (1.0, 0.0, 1.0)   # magenta: an unmapped verdict code

# Ego's own detections carry no verdict -- ego does not judge itself -- so they
# are drawn in greyscale, out of the peer palette entirely, with LIGHTNESS
# carrying the one continuous quantity ego does have: its local score.
# The mapping is deliberately inverted (confident = dark, doubtful = bright) so
# a weak detection is what catches the eye. The floor keeps a confident box off
# pure black, which is invisible against the 3D panel's dark background.
_EGO_LIGHTNESS_MAX = 0.90    # local_score = 0
_EGO_LIGHTNESS_SPAN = 0.55   # subtracted at local_score = 1
_EGO_LIGHTNESS_FLOOR = 0.35

# The fused estimate: red/yellow/green by the fused score, same traffic-light
# read as a confidence gauge. Thresholds are fixed, not verdict-derived --
# fused confidence is a continuous MS-PSF score, not one of the discrete
# peer-verdict codes _VERDICT_COLOR is keyed on.
_FUSED_LOW_COLOR = (0.95, 0.10, 0.10)    # red:    score < 0.6
_FUSED_MID_COLOR = (1.00, 0.80, 0.00)    # yellow: 0.6 <= score < 0.8
_FUSED_HIGH_COLOR = (0.05, 0.85, 0.20)   # green:  score >= 0.8


def _fused_color(score: float) -> tuple[float, float, float]:
    """Red/yellow/green for a fused box, by its fused score."""
    if score < 0.6:
        return _FUSED_LOW_COLOR
    if score < 0.8:
        return _FUSED_MID_COLOR
    return _FUSED_HIGH_COLOR


def _ego_color(local_score: float) -> tuple[float, float, float]:
    """Greyscale for one of ego's own detections, by its local score."""
    score = max(0.0, min(1.0, float(local_score)))
    lightness = max(_EGO_LIGHTNESS_FLOOR,
                    _EGO_LIGHTNESS_MAX - _EGO_LIGHTNESS_SPAN * score)
    return (lightness, lightness, lightness)

# get_headings_of passes J2735's "unavailable" (28800 units) through as 360.0
_HEADING_UNAVAILABLE_DEG = 360.0

_MIN_BOX_M = 0.1   # never emit a zero-scale marker; Foxglove drops those

# Per-object labels sit this far behind the object (along its own heading,
# not the world frame) so the billboard doesn't float directly over -- and
# from a shallow angle, over -- the shape it names.
_LABEL_BEHIND_OFFSET_M = 0.6

# What each J3224 class draws as, and what its label calls it.
#
# The meshes are referenced by package:// URI rather than embedded: the viewer
# fetches each one ONCE over foxglove_bridge's asset channel (its default
# asset_uri_allowlist already permits package://*.glb), where a
# foxglove_msgs/SceneUpdate ModelPrimitive would inline ~165 KB in every
# entity of every frame -- at this node's 20 Hz that is the bandwidth wall
# scene_node documents, and it is why these layers stay MarkerArray.
# THE COST: package:// resolves only through a live bridge, so meshes do not
# render when a recorded MCAP is opened as a file. Embedding is the only fix
# for that, at the rate above.
#
# 'Car'/'Pedestrian'/'Bus' are DISPLAY names, deliberately not
# perception_message.class_name_of's 'vehicle'/'VRU' -- the wire vocabulary
# stays as the standard names it, and a class absent from _mesh_for_class falls
# back to it, so an UNKNOWN never renders as a confident 'Car'.
_MESH_PACKAGE = 'package://global_trust_perception/assets'

# dims, throughout this node, are (width, length, height).
_DIM_WIDTH, _DIM_LENGTH, _DIM_HEIGHT = 0, 1, 2

# name, asset, and how the mesh is fitted to the detection: `extent` is the
# mesh's OWN size along `dim` as authored (read off the glTF POSITION bounds,
# in model units -- the pedestrian is modelled ~21.7 units tall, not metres),
# so scaling by reported/extent draws every object at the size its sender
# actually claimed, which is what the cubes used to do.
# A vehicle is fitted on its length and a pedestrian on its height: a
# pedestrian's footprint is near-square and small, so its reported length is
# mostly tracker noise, while its height is the stable dimension.
_Mesh = namedtuple('_Mesh', 'name asset extent dim')
_CAR = _Mesh('Car', 'car.glb', 4.221, _DIM_LENGTH)
_PEDESTRIAN = _Mesh('Pedestrian', 'pedestrian.glb', 21.670, _DIM_HEIGHT)
# bus.glb is normalised to the same convention as the others (length on +Z,
# base on y=0); it measures 690.227 units long as authored.
_BUS = _Mesh('Bus', 'bus.glb', 690.227, _DIM_LENGTH)
_ALL_MESHES = (_CAR, _PEDESTRIAN, _BUS)

# A VEHICLE this long or longer draws as a bus, not a car. A coach and a car
# share obj_class VEHICLE on the wire, so length -- the vehicle's fitted, most
# separated dimension -- is what tells them apart. The gap between a ~4.5 m car
# and a ~12 m coach is wide, so this threshold clears both classes with room to
# spare for tracker length-noise.
_BUS_MIN_LEN_M = 8.0


def _mesh_for_class(obj_class, dims):
    """The mesh a detection draws as, or None to fall back to a cylinder.

    Pedestrians go by class; vehicles split on length (see _BUS_MIN_LEN_M).
    A class with no entry here -- UNKNOWN included -- returns None so it never
    borrows another class's silhouette. dims is (width, length, height).
    """
    if obj_class == pmsg.OBJ_TYPE_VEHICLE:
        return _BUS if dims[_DIM_LENGTH] >= _BUS_MIN_LEN_M else _CAR
    if obj_class == pmsg.OBJ_TYPE_VRU:
        return _PEDESTRIAN
    return None

# TEMP: force the pedestrian mesh to a car's height instead of the reported
# one -- size-derived Pedestrian detections can report a near-zero height
# (an implausible-shape box), which scales the mesh down to an invisible
# sliver. This is only to confirm the mesh itself shows up at all; delete
# this constant and its one use in _box to go back to the reported height.
_VRU_DEBUG_HEIGHT_M = 1.5

# TEMP: a translucent box around the pedestrian mesh, fixed-size rather than
# the reported (often noisy/thin) dims -- the mesh itself reads as too skinny
# to spot in the panel, and this pads it out without changing the mesh. Color
# matches whatever the mesh's own layer color is; only alpha and size are
# fixed. Delete _VRU_BOX_ALPHA/_VRU_BOX_WLH_M and their one use in _box to
# drop it.
_VRU_BOX_ALPHA = 0.5
_VRU_BOX_WLH_M = (1.4, 1.4, _VRU_DEBUG_HEIGHT_M * 2.0)

# Both meshes are authored facing +Z, which Foxglove's glTF Y-up -> Z-up
# mapping lands on scene +Y, so heading needs a quarter turn to point them
# along +X. Same convention as scene_node's model_yaw_deg default.
_MESH_YAW_OFFSET = math.pi / 2.0

# How far above an agent's reported ref_pos its global-score label floats.
# ref_pos is the agent's sensor origin, already above its roof, so this only
# has to clear the label of any box drawn at the same spot.
_AGENT_LABEL_HEIGHT_M = 1.5


def _yaw_from_j2735_heading(heading_deg: float) -> float:
    """J2735 heading (deg, 0 = North, clockwise) -> math yaw (rad, 0 = +x)."""
    return math.radians(90.0 - heading_deg)


class TrustViewNode(Node):
    """Subscribes to raw SDSM + the verdict channel; publishes two marker views."""

    def __init__(self):
        """Declare params and wire up the subscriptions, publishers and timers."""
        super().__init__('trust_view_node')

        self.declare_parameter('ego_source_id', 1)
        self.frame_id = str(self.declare_parameter('frame_id', 'world').value)
        self.stale_sec = float(self.declare_parameter('stale_sec', 2.0).value)
        # Matched to the engine's 20 Hz flush, NOT to scene_node's 4 Hz. The
        # two look like the same knob and are not: scene_node inlines a 165 KB
        # glTF mesh in every entity, so its rate is a bandwidth decision, while
        # this node emits cubes and text (~14 KB a frame) and can afford to
        # track the data. At 4 Hz it sampled 15-20 Hz of SDSM and verdicts four
        # times a second, which reads as a handful of stills rather than motion.
        self.render_hz = float(self.declare_parameter('render_hz', 20.0).value)
        self.marker_alpha = float(self.declare_parameter('marker_alpha', 0.45).value)
        # Which verdicts the peer layer draws. Live-settable, so a viewer can
        # ask "what is being held?" from Lichtblick's Parameters panel without
        # restarting the node or losing the rest of the scene.
        self.declare_parameter('peer_boxes.verdict_filter', _FILTER_ALL)

        if not 0.0 <= self.marker_alpha <= 1.0:
            raise ValueError('marker_alpha must be between 0 and 1')

        self._installed_meshes = self._available_meshes()

        # (sender, msg_cnt) -> (msg, recv_t): the correlation buffer. Keyed by
        # msg_cnt because a verdict names the exact message it judged.
        self._raw: dict[tuple[int, int], tuple] = {}
        # ego -> {sender -> (verdict_msg, recv_t)}
        self._verdicts: dict[int, dict[int, tuple]] = {}
        # sender -> (x, y, z, recv_t): where each broadcaster says it is.
        # z is kept so an agent-level label can sit above the position the
        # agent itself broadcasts from, rather than on the ground.
        self._sender_pos: dict[int, tuple[float, float, float, float]] = {}
        self._verdict_subs: dict[str, object] = {}
        # ego -> (FusedObjects, recv_t): the fused scene, per ego.
        self._fused: dict[int, tuple] = {}
        self._fused_subs: dict[str, object] = {}
        # topic -> {(ns, marker_id)} published last render, so a marker that
        # goes away can be retired individually instead of clearing the layer.
        self._published: dict[str, set] = {}
        # sender -> the msg_cnt of its most recent SDSM, so ego's own boxes can
        # be drawn from ego's latest frame instead of waiting for a verdict to
        # point at one.
        self._latest_seq: dict[int, int] = {}

        self.ego_pub = self.create_publisher(MarkerArray, EGO_BOXES_TOPIC, 10)
        self.peer_pub = self.create_publisher(MarkerArray, PEER_BOXES_TOPIC, 10)
        self.fused_pub = self.create_publisher(MarkerArray, FUSED_BOXES_TOPIC, 10)
        self.create_subscription(pmsg.Message, pmsg.TOPIC, self._on_raw, 50)

        self.create_timer(1.0, self._discover_verdict_topics)
        self.create_timer(1.0 / max(self.render_hz, 0.1), self._render)

        self.get_logger().info(
            f'trust_view up: ego_source_id='
            f'{int(self.get_parameter("ego_source_id").value)}, '
            f'frame_id={self.frame_id}, publishing {EGO_BOXES_TOPIC}, '
            f'{PEER_BOXES_TOPIC}, {FUSED_BOXES_TOPIC}')

    # --- assets ---------------------------------------------------------------

    def _available_meshes(self) -> frozenset:
        """Return the _ALL_MESHES entries whose asset is actually installed.

        Checked once at startup because the viewer fetches by package:// URI
        and cannot tell us it got a 404: a missing asset is simply an object
        that never draws. A mesh whose asset is absent is dropped here, so
        _mesh_for_class's pick falls back to the cylinder every unclassifiable
        object already uses, and the log says which one is missing.
        """
        try:
            assets = os.path.join(
                get_package_share_directory('global_trust_perception'),
                'assets')
        except PackageNotFoundError:
            self.get_logger().warning(
                'package share directory not found; every object draws as a '
                'cylinder')
            return frozenset()

        installed = set()
        for mesh in _ALL_MESHES:
            if os.path.exists(os.path.join(assets, mesh.asset)):
                installed.add(mesh)
            else:
                self.get_logger().warning(
                    f'{mesh.asset} not installed; {mesh.name} objects draw as '
                    'a cylinder (rebuild after adding it to assets/)')
        return frozenset(installed)

    # --- ingest --------------------------------------------------------------

    def _now(self) -> float:
        """Node clock, seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_raw(self, msg):
        """Buffer one sender's SDSM until its verdict arrives."""
        sender = pmsg.sender_of(msg)
        now = self._now()
        self._raw[(sender, pmsg.sequence_of(msg))] = (msg, now)
        x, y, z = pmsg.ref_pos_of(msg)
        self._sender_pos[sender] = (x, y, z, now)
        self._latest_seq[sender] = pmsg.sequence_of(msg)

    def _discover_verdict_topics(self):
        """Subscribe to any new per-ego verdict or fused-scene topic."""
        for name, _types in self.get_topic_names_and_types():
            ego = tverd.ego_id_from_topic(name)
            if ego is not None and name not in self._verdict_subs:
                self._verdict_subs[name] = self.create_subscription(
                    tverd.Message, name,
                    lambda m, e=ego: self._on_verdict(m, e), 20)
                self.get_logger().info(
                    f'discovered verdicts of ego {ego} on {name}')
            # The agent only packs the fused scene when someone subscribes, so
            # this subscription is what turns that publisher on.
            fego = fobj.ego_id_from_topic(name)
            if fego is not None and name not in self._fused_subs:
                self._fused_subs[name] = self.create_subscription(
                    fobj.Message, name,
                    lambda m, e=fego: self._on_fused(m, e), 10)
                self.get_logger().info(
                    f'discovered fused scene of ego {fego} on {name}')

    def _on_verdict(self, msg, ego: int):
        """Cache one ego's judgment of one sender."""
        self._verdicts.setdefault(ego, {})[tverd.sender_of(msg)] = (msg, self._now())

    def _on_fused(self, msg, ego: int):
        """Cache one ego's fused scene for this frame."""
        self._fused[ego] = (msg, self._now())

    def _prune(self):
        """Drop everything older than stale_sec so dead senders stop rendering."""
        cutoff = self._now() - self.stale_sec
        self._raw = {k: v for k, v in self._raw.items() if v[1] >= cutoff}
        self._fused = {k: v for k, v in self._fused.items() if v[1] >= cutoff}
        self._sender_pos = {
            k: v for k, v in self._sender_pos.items() if v[3] >= cutoff}
        for per_ego in self._verdicts.values():
            for sender in [s for s, (_m, t) in per_ego.items() if t < cutoff]:
                del per_ego[sender]

    # --- render --------------------------------------------------------------

    def _render(self):
        """Rebuild and publish the three layers from the current caches."""
        self._prune()
        ego = int(self.get_parameter('ego_source_id').value)
        verdicts = self._verdicts.get(ego, {})

        # One layer per topic, so the viewer's per-topic visibility IS the
        # enable/disable control -- no bespoke toggles, and hiding a layer
        # costs nothing on the wire because Foxglove unsubscribes.
        #
        # Each builder returns None for "no opinion this frame" (a correlation
        # gap) and a list -- possibly empty -- for "rendered". The distinction
        # decides whether stale markers are deleted or left to ride out their
        # lifetime; see _publish_layer.
        ego_markers = self._ego_markers(ego)
        self._publish_layer(
            self.ego_pub, EGO_BOXES_TOPIC, ego_markers or [],
            self._namespaces(f'ego_{ego}') if ego_markers is not None else set())

        peer_markers, peer_touched = [], set()
        label_markers, label_touched = [], set()
        for sender in sorted(verdicts):
            verdict_msg, _t = verdicts[sender]
            raw = self._raw.get((sender, int(verdict_msg.msg_cnt)))
            # The agent's own global score, above the position it broadcasts
            # from. Drawn independently of the correlation check and the
            # verdict filter below: both of those select DETECTIONS, while
            # this annotates the AGENT, which is there whether or not this
            # frame's objects could be lined up. It goes out on the FUSED
            # layer, not this one; see below.
            label = self._agent_label(sender, verdict_msg)
            if label is not None:
                label_markers.append(label)
                label_touched.add(f'agent_label_{sender}')

            if raw is None:
                continue     # verdict arrived without (or before) its SDSM
            drawn = self._peer_markers(sender, verdict_msg, raw[0])
            if drawn is None:
                continue     # judged a different object count; see _peer_markers
            peer_markers += drawn
            peer_touched |= self._namespaces(f'agent_{sender}')
        self._publish_layer(
            self.peer_pub, PEER_BOXES_TOPIC, peer_markers, peer_touched)

        # The agent labels ride this layer so that hiding peer_boxes -- the
        # usual way to clear per-object clutter -- does not also take away what
        # each peer is worth. The two remain independent opinions within the
        # one topic: they keep their own namespaces, so a frame with a fused
        # scene and no labels (or the reverse) retires only what it rendered.
        fused_markers = self._fused_markers(ego)
        self._publish_layer(
            self.fused_pub, FUSED_BOXES_TOPIC,
            (fused_markers or []) + label_markers,
            (self._namespaces(f'fused_{ego}')
             if fused_markers is not None else set()) | label_touched)

    @staticmethod
    def _namespaces(base: str) -> set:
        """All namespaces _box writes under, for one logical group."""
        return {base, f'{base}_text', f'{base}_box'}

    def _ego_entry(self, ego):
        """
        (msg, recv_t) of the ego frame the ego layer draws, or None.

        Named rather than inlined because "the ego frame currently on screen"
        is a distinct idea from the raw buffer it comes out of: the layer
        draws ego's LATEST message, not whichever one a peer's verdict
        happened to reference (see _ego_markers).
        """
        seq = self._latest_seq.get(ego)
        return None if seq is None else self._raw.get((ego, seq))

    def _publish_layer(self, publisher, topic, markers, touched):
        """
        Publish one layer, deleting only the markers that actually went away.

        NO DELETEALL. Clearing the array every render destroys and recreates
        every box at render_hz, which a viewer draws as a constant blink --
        the same failure trustworthiness_visualization_node was fixed for.
        Instead each marker keeps a stable id (the tracker's object id) so a
        re-ADD updates it in place, and only ids that vanished are deleted.

        `touched` is the set of namespaces this frame rendered. A namespace
        NOT in it produced no opinion -- ego's frame was missing, or a
        sender's verdict didn't line up with its message -- so its markers are
        left alone and expire on their own lifetime instead of blinking off
        for one frame and back on for the next.
        """
        current = {(m.ns, m.id) for m in markers}
        previous = self._published.get(topic, set())
        departed = [key for key in previous - current if key[0] in touched]
        kept = {key for key in previous if key[0] not in touched}

        publisher.publish(MarkerArray(
            markers=list(markers) + [self._delete(ns, i) for ns, i in departed]))
        self._published[topic] = kept | current

    def _fused_markers(self, ego):
        """The fused estimate for this ego: red/yellow/green by fused score."""
        entry = self._fused.get(ego)
        if entry is None:
            return None
        markers = []
        for i, (position, dims, yaw, score, obj_class) in enumerate(
                fobj.boxes_of(entry[0])):
            # _box takes J2735 heading degrees; the fused yaw is already math
            # radians, so it is converted here rather than widening _box.
            heading_deg = 90.0 - math.degrees(yaw)
            markers += self._box(
                ns=f'fused_{ego}', marker_id=i, position=position,
                # _box takes (width, length, height); the message states
                # size_x as length along the heading.
                dims=(dims[1], dims[0], dims[2]),
                heading_deg=heading_deg, color=_fused_color(score),
                obj_class=obj_class,
                # The one score in the view, named by the class MS-PSF settled
                # on -- which can differ from what any single input claimed,
                # and is the shape drawn here.
                label=f'{self._class_name(obj_class, (dims[1], dims[0], dims[2]))}'
                      f' | score: {score:.2f}')
        return markers

    def _peer_markers(self, sender, verdict_msg, raw):
        """Markers for one peer's detections; None when it cannot be drawn."""
        count = pmsg.num_detections_of(raw)
        if len(verdict_msg.verdict) != count:
            # The engine judged a different number of detections than this
            # message carries (two messages from one sender inside one flush
            # window). Indexing across that would mis-colour boxes. None, not
            # [], so this sender's existing markers survive the gap rather
            # than being deleted and immediately re-added.
            return None

        # Read per render, not cached at startup, so the filter can be changed
        # live from a Parameters panel.
        keep = _FILTERS.get(
            str(self.get_parameter('peer_boxes.verdict_filter').value),
            _FILTERS[_FILTER_ALL])

        positions = pmsg.get_global_positions_of(raw)
        dims = pmsg.get_dims_of(raw)
        headings = pmsg.get_headings_of(raw)
        object_ids = pmsg.get_object_ids_of(raw)
        classes = pmsg.get_labels_of(raw)

        markers = []
        for i in range(count):
            code = int(verdict_msg.verdict[i])
            if keep is not None and code not in keep:
                continue
            # The verdict is the COLOUR, not a word: naming it as well put a
            # second reading of one fact on every box. What the peer itself
            # is worth is on its agent label (see _agent_label).
            color = _VERDICT_COLOR.get(code, _UNKNOWN_COLOR)
            # Keyed on the tracker's object id, NOT the loop index: an index
            # addresses a different object as soon as another enters or leaves
            # the message, so an index-keyed marker jumps between cars. The
            # verdict filter makes that worse by skipping indices outright.
            markers += self._box(
                ns=f'agent_{sender}', marker_id=object_ids[i],
                position=positions[i], dims=dims[i], heading_deg=headings[i],
                color=color, obj_class=classes[i],
                label=f'AGENT {sender} | {self._class_name(classes[i], dims[i])}')
        return markers

    def _ego_markers(self, ego):
        """Ego's own detections, greyscale by local score; None if unavailable.

        Every `return None` below is a frame this node cannot speak for, not a
        frame with nothing in it -- the caller leaves the existing markers
        alone rather than blinking them off (see _publish_layer).

        The local score survives as the GREY LEVEL (see _ego_color) though it
        is no longer written out: a number on every one of ego's boxes was the
        bulk of the text in the view, and the fused layer is where a score is
        meant to be read.
        """
        # Ego's LATEST own frame, not whichever one a peer's verdict happened
        # to reference. What ego detected is a fact ego already has; making it
        # wait on a verdict tied it to the odds of ego's message and a peer's
        # landing in the same 50 ms flush window, which for two ~7 Hz streams
        # is about 2 a second -- so ego's boxes updated at a fraction of the
        # rate ego was actually reporting them.
        raw_entry = self._ego_entry(ego)
        if raw_entry is None:
            return None
        raw, _recv_t = raw_entry

        count = pmsg.num_detections_of(raw)
        positions = pmsg.get_global_positions_of(raw)
        dims = pmsg.get_dims_of(raw)
        headings = pmsg.get_headings_of(raw)
        scores = pmsg.get_local_scores_of(raw)
        object_ids = pmsg.get_object_ids_of(raw)
        classes = pmsg.get_labels_of(raw)

        markers = []
        for i in range(count):
            # Corroboration is neither a colour nor a word here: ego stays
            # greyscale so the green/yellow/red palette means "a peer's box,
            # judged" and nothing else. Whether ego's detection survived into
            # the acted-on scene is what the fused layer answers.
            markers += self._box(
                ns=f'ego_{ego}', marker_id=object_ids[i], position=positions[i],
                dims=dims[i], heading_deg=headings[i],
                color=_ego_color(scores[i]), obj_class=classes[i],
                label=f'EGO | {self._class_name(classes[i], dims[i])}')
        return markers

    @staticmethod
    def _class_name(obj_class, dims) -> str:
        """Display name for one detection (see _mesh_for_class).

        Falls back to the wire vocabulary for a class with no mesh of its
        own, so an UNKNOWN reads as 'unknown' rather than being rounded into
        one of the named ones. Takes dims because a vehicle's name, like its
        silhouette, depends on length -- car vs bus. dims is (width, length,
        height), so the label always matches the mesh drawn.
        """
        mesh = _mesh_for_class(int(obj_class), dims)
        return mesh.name if mesh else pmsg.class_name_of(obj_class)

    # --- primitives ----------------------------------------------------------

    def _header(self, marker: Marker) -> None:
        """Stamp a marker with this node's frame and clock."""
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

    def _agent_label(self, sender: int, verdict_msg) -> Marker | None:
        """
        One peer's global score, floating above where that peer says it is.

        The position comes from the agent's own broadcast ref_pos, not from
        any detection of it, so the label is there even when nobody has
        detected the agent -- which is exactly when "how much do I trust it"
        is hardest to find on screen. None when the agent has not broadcast
        recently enough to have a position.

        Carries the score and the gate, and nothing else: the display skew
        between this agent's drawn frame and ego's is real (see the module
        docstring) but is a property of the PIPELINE's timing rather than of
        the trust decision, and reporting it here put a number no viewer acts
        on next to the one they do.
        """
        entry = self._sender_pos.get(sender)
        if entry is None:
            return None
        x, y, z, _t = entry

        marker = Marker()
        self._header(marker)
        marker.ns = f'agent_label_{sender}'
        marker.id = sender
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z) + _AGENT_LABEL_HEIGHT_M
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.95
        # Coloured by the gate, not by the score: whether ego is ACTING on this
        # agent is a harder fact than the number, and the number is right there
        # to be read.
        red, green, blue = (
            _VERDICT_COLOR[_V.VERDICT_MATCHED] if verdict_msg.trusted
            else _VERDICT_COLOR[_V.VERDICT_UNCORROBORATED])
        marker.color.r, marker.color.g, marker.color.b = red, green, blue
        marker.color.a = 1.0
        gate = '' if verdict_msg.trusted else '  (GATED)'
        marker.text = (f'agent {sender} | global score: '
                       f'{verdict_msg.r_new:.2f}{gate}')
        marker.lifetime = Duration(sec=int(self.stale_sec),
                                   nanosec=int((self.stale_sec % 1.0) * 1e9))
        return marker

    def _delete(self, ns: str, marker_id: int) -> Marker:
        """Retire one marker whose object is gone from this layer."""
        marker = Marker()
        self._header(marker)
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def _box(self, ns, marker_id, position, dims, heading_deg, color, label,
             obj_class=pmsg.OBJ_TYPE_UNKNOWN):
        """One detection as its class's shape plus a billboard label."""
        width, length, height = (max(float(d), _MIN_BOX_M) for d in dims)
        yaw = (0.0 if heading_deg >= _HEADING_UNAVAILABLE_DEG
               else _yaw_from_j2735_heading(heading_deg))
        # Captured before the mesh branch below adds _MESH_YAW_OFFSET to yaw --
        # the label goes behind the object's actual heading, not behind the
        # mesh's authoring-axis quirk.
        obj_yaw = yaw
        red, green, blue = color
        lifetime = Duration(sec=int(self.stale_sec),
                            nanosec=int((self.stale_sec % 1.0) * 1e9))

        mesh = _mesh_for_class(int(obj_class), (width, length, height))
        if mesh is not None and mesh not in self._installed_meshes:
            mesh = None
        shape = Marker()
        self._header(shape)
        shape.ns = ns
        shape.id = marker_id
        shape.action = Marker.ADD
        shape.pose.position.x = float(position[0])
        shape.pose.position.y = float(position[1])
        shape.pose.position.z = float(position[2])
        if mesh is not None:
            reported = (width, length, height)[mesh.dim]
            if int(obj_class) == pmsg.OBJ_TYPE_VRU:
                reported = _VRU_DEBUG_HEIGHT_M   # TEMP: see note above
            scale = reported / mesh.extent
            shape.type = Marker.MESH_RESOURCE
            shape.mesh_resource = f'{_MESH_PACKAGE}/{mesh.asset}'
            # The mesh's own materials would ignore the layer's colour, which
            # is the one thing every shape in this view has to carry.
            shape.mesh_use_embedded_materials = False
            shape.scale.x = shape.scale.y = shape.scale.z = scale
            # Authored standing on y=0 (wheels, feet) while the detection
            # states its geometric centre, so the mesh is dropped half an
            # object to stand on the ground instead of floating over it.
            shape.pose.position.z -= height / 2.0
            yaw += _MESH_YAW_OFFSET
        else:
            # No mesh for this class -- including UNKNOWN, which is the point:
            # it must not borrow another class's silhouette.
            shape.type = Marker.CYLINDER
            shape.scale.x = shape.scale.y = max(width, length)
            shape.scale.z = height
        shape.pose.orientation.z = math.sin(yaw / 2.0)
        shape.pose.orientation.w = math.cos(yaw / 2.0)
        shape.color.r, shape.color.g, shape.color.b = red, green, blue
        # TEMP: pedestrians fully opaque instead of marker_alpha, so they
        # aren't lost behind the ground backdrop's depth-sorting quirk at
        # alpha < 1; see the VRU height override above for the revert note.
        shape.color.a = (1.0 if int(obj_class) == pmsg.OBJ_TYPE_VRU
                         else self.marker_alpha)
        shape.lifetime = lifetime

        text = Marker()
        self._header(text)
        text.ns = f'{ns}_text'
        text.id = marker_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = (float(position[0])
                                - _LABEL_BEHIND_OFFSET_M * math.cos(obj_yaw))
        text.pose.position.y = (float(position[1])
                                - _LABEL_BEHIND_OFFSET_M * math.sin(obj_yaw))
        text.pose.position.z = float(position[2]) + height / 2.0 + 0.5
        text.pose.orientation.w = 1.0
        text.scale.z = 0.75
        text.color.r, text.color.g, text.color.b = red, green, blue
        text.color.a = 0.95
        text.text = label
        text.lifetime = lifetime

        markers = [shape, text]
        if int(obj_class) == pmsg.OBJ_TYPE_VRU:
            box = Marker()
            self._header(box)
            box.ns = f'{ns}_box'
            box.id = marker_id
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(position[0])
            box.pose.position.y = float(position[1])
            box.pose.position.z = float(position[2])
            box.pose.orientation.z = math.sin(obj_yaw / 2.0)
            box.pose.orientation.w = math.cos(obj_yaw / 2.0)
            box.scale.x, box.scale.y, box.scale.z = _VRU_BOX_WLH_M
            box.color.r, box.color.g, box.color.b = red, green, blue
            box.color.a = _VRU_BOX_ALPHA
            box.lifetime = lifetime
            markers.append(box)

        return markers


def main(args=None):
    """Spin the trust view node."""
    rclpy.init(args=args)
    try:
        rclpy.spin(TrustViewNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
