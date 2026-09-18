"""
Lichtblick/Foxglove scene visualizer for the trust pipeline (read-only).

Turns the flat SDSM parallel-arrays into foxglove_msgs/SceneUpdate so the
Foxglove 3D panel can draw them as glTF cars (box fallback for unknown types).
It NEVER touches the trust pipeline; it only subscribes and republishes.

Two scenes, meant to be toggled in one 3D panel (before vs after fusion):

- scene_raw   (from TOPIC): every sender's raw broadcast, objects tinted by the
               sender's own per-object local score. Includes low-trust senders.
- scene_trust (from the per-ego trust_output topics): only gate-passing senders
               from the SELECTED ego's perspective, tinted by global_score.
               Held (gate-failed) senders are simply absent -> the payoff.

Ego selection: the `ego_source_id` parameter picks which broadcaster is drawn as
the sports car (in both scenes) and, for scene_trust, whose trust view to show.
It is live-settable (e.g. from Lichtblick's Parameters panel) so you can flip
between agents without restarting.

Only EGO gets a marker drawn at its own ref_pos. A peer is shown by ego's
detection of it, which already carries ego's reputation for that peer (see
_peer_gs); drawing the peer's ref_pos as well put a second car a metre above
that box, since ref_pos is the peer's lidar origin rather than its body.

Multi-agent: per-ego trust_output topics are discovered dynamically, so running
    ros2 run global_trust_tracker tracker
    ros2 run global_trust_perception agent --ros-args -p agent_id:=1 -r __node:=agent_1
    ros2 run global_trust_perception agent --ros-args -p agent_id:=2 -r __node:=agent_2
    ros2 run global_trust_perception scene
just works, and ego_source_id toggles between agent 1's and agent 2's views.
"""

import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from foxglove_msgs.msg import (
    Color,
    CubePrimitive,
    KeyValuePair,
    LinePrimitive,
    ModelPrimitive,
    SceneEntity,
    SceneUpdate,
    TextPrimitive,
)

from global_trust_perception.pipeline import perception_message as pmsg
from global_trust_perception.pipeline import trust_verdicts as tv

SCENE_RAW_TOPIC = '/perception/global_trustworthiness/viz/scene_raw'
SCENE_TRUST_TOPIC = '/perception/global_trustworthiness/viz/scene_trust'
SCENE_GROUND_TOPIC = '/perception/global_trustworthiness/viz/scene_ground'

_ASPHALT_COLOR = Color(r=0.13, g=0.13, b=0.15, a=1.0)
_GRID_COLOR = Color(r=0.45, g=0.45, b=0.5, a=0.8)

# SDSM scaled-integer units (see SdsmPayload.msg)
_OFFSET_M = 0.1
_SPEED_MPS = 0.02
_HEADING_DEG = 0.0125
_HEADING_UNAVAIL = 28800

# ref_pos carries no heading of its own (SdsmPayload.msg has none for the
# sender), so a sender's facing is derived from consecutive ref_pos deltas.
# Below this movement a message is treated as "didn't move" and the previous
# heading is kept, rather than collapsing to atan2(0, 0) == 0 -- this is what
# keeps a car facing its lane while it's held at a stop line, not just noise
# suppression.
_MOVE_EPS_M = 0.01

_VEHICLE = 1  # obj_type / detObjCommon.objType.vehicle
_EQUIP_RSU = 1  # equipment_type: roadside unit (drawn as a small box, not a car)

# Reference car length (m): meshes scale relative to this so a ~4.5 m object
# renders at the calibrated car_scale and bigger/smaller objects scale with it.
_REF_CAR_LEN_M = 4.5

# Fallback dimensions (metres) for a broadcaster, whose own size isn't in SDSM.
_SENDER_SIZE = (4.5, 2.0, 1.5)
# An RSU is infrastructure, not a vehicle: draw it as a box 1/30 the car size.
_RSU_SIZE = (4.5 / 30.0, 2.0 / 30.0, 1.5 / 30.0)
_EGO_COLOR = Color(r=0.0, g=0.65, b=1.0, a=0.95)      # ego sports car: cyan


def _yaw_from_j2735_heading(heading_deg: float) -> float:
    """J2735 heading (deg, 0=North CW) -> math yaw (rad, 0=+x East CCW)."""
    return math.radians(90.0 - heading_deg)


def _quat_z(yaw: float) -> Quaternion:
    """Quaternion for a yaw rotation about +Z."""
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _quat_x(angle: float) -> Quaternion:
    """Quaternion for a rotation about +X."""
    return Quaternion(x=math.sin(angle / 2.0), y=0.0, z=0.0, w=math.cos(angle / 2.0))


def _quat_y(angle: float) -> Quaternion:
    """Quaternion for a rotation about +Y."""
    return Quaternion(x=0.0, y=math.sin(angle / 2.0), z=0.0, w=math.cos(angle / 2.0))


def _quat_mul(a: Quaternion, b: Quaternion) -> Quaternion:
    """Hamilton product a * b."""
    return Quaternion(
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    )


def _score_color(score: float) -> Color:
    """Red (low) -> green (high) tint for a score in [0, 1]."""
    s = max(0.0, min(1.0, score))
    return Color(r=1.0 - s, g=s, b=0.0, a=0.85)


class SceneNode(Node):
    """Subscribes to SDSM + per-ego trust_output; publishes two scene topics."""

    def __init__(self):
        """Declare params, load meshes, wire up pubs/subs and timers."""
        super().__init__('scene_node')

        self.ego_source_id = int(self.declare_parameter('ego_source_id', 3).value)
        self.center_x = float(self.declare_parameter('center_x', 0.0).value)
        self.center_y = float(self.declare_parameter('center_y', 0.0).value)
        self.stale_sec = float(self.declare_parameter('stale_sec', 2.0).value)
        # 'world' is the frame converted bags put both agents in, and the frame
        # trust_view_node already defaults to -- the two viz nodes have to agree
        # or one of them silently vanishes. An entity in a frame the TF tree
        # does not contain is DROPPED by the 3D panel without an error on the
        # entity itself, so a wrong value here looks like "the global stage
        # isn't publishing", not like a frame problem.
        self.frame_id = str(self.declare_parameter('frame_id', 'world').value)
        self.render_hz = float(self.declare_parameter('render_hz', 4.0).value)
        # glTF base-orientation knobs, degrees (tune live if a mesh imports
        # tilted). Foxglove already maps glTF Y-up to the scene's Z-up, so roll
        # and pitch stay 0; the shipped car/sports meshes are authored nose-along
        # +Y, so model_yaw_deg defaults to 90 to point the nose along heading
        # (roll/pitch only if a mesh needs standing up).
        self.model_roll_deg = float(self.declare_parameter('model_roll_deg', 0.0).value)
        self.model_pitch_deg = float(self.declare_parameter('model_pitch_deg', 0.0).value)
        self.model_yaw_deg = float(self.declare_parameter('model_yaw_deg', 90.0).value)
        # car_scale calibrates a ~4.5 m car; objects scale relative to their SDSM
        # length (see _REF_CAR_LEN_M). sports_car_scale calibrates the ego mesh.
        self.car_scale = float(self.declare_parameter('car_scale', 1.0).value)
        self.sports_scale = float(self.declare_parameter('sports_car_scale', 1.0).value)
        # Ground plane (dark asphalt + grid lines) for a street-like reference.
        self.show_ground = bool(self.declare_parameter('show_ground', True).value)
        # One billboard per entity means a busy scene turns into a wall of
        # labels; off leaves the geometry and its colour to speak for itself.
        self.show_labels = bool(self.declare_parameter('show_labels', True).value)
        # See _object_entity: meshing every detected object is the dominant
        # cost on the wire, so detected objects are boxes unless asked.
        self.object_meshes = bool(
            self.declare_parameter('object_meshes', False).value)
        # How close a detected object must sit to a broadcaster's own reported
        # ref_pos to be treated as BEING that broadcaster (see _peer_gs).
        # Matches trust_view_node's parameter of the same name.
        self.sender_match_radius_m = float(
            self.declare_parameter('sender_match_radius_m', 3.0).value)
        self.ground_extent_m = float(
            self.declare_parameter('ground_extent_m', 400.0).value)
        self.ground_step_m = float(self.declare_parameter('ground_step_m', 5.0).value)

        assets = os.path.join(
            get_package_share_directory('global_trust_perception'), 'assets')
        self._car_glb = self._load_mesh(os.path.join(assets, 'car.glb'))
        self._sports_glb = self._load_mesh(os.path.join(assets, 'sports_car.glb'))

        # latest message per sender, and per (ego, sender), with receive time.
        self._raw: dict[int, tuple] = {}
        self._trust: dict[int, dict[int, tuple]] = {}
        # {ego: {sender: (r_new, trusted)}} from the verdict topics.
        self._reputation: dict[int, dict[int, tuple[float, bool]]] = {}
        self._verdict_subs: dict[str, object] = {}
        self._trust_subs: dict[str, object] = {}
        # last known (x, y, yaw) per sender, for drawing the sender's own facing.
        self._sender_pose: dict[int, tuple[float, float, float]] = {}

        self.raw_pub = self.create_publisher(SceneUpdate, SCENE_RAW_TOPIC, 10)
        self.trust_pub = self.create_publisher(SceneUpdate, SCENE_TRUST_TOPIC, 10)
        self.ground_pub = self.create_publisher(SceneUpdate, SCENE_GROUND_TOPIC, 10)
        self.create_subscription(pmsg.Message, pmsg.TOPIC, self._on_raw, 10)

        self.create_timer(1.0, self._discover_trust_topics)
        self.create_timer(1.0 / max(self.render_hz, 0.1), self._render)

        self.get_logger().info(
            f'scene_node up: ego_source_id={self.ego_source_id}, '
            f'car={"ok" if self._car_glb else "MISSING->box"}, '
            f'sports={"ok" if self._sports_glb else "MISSING->box"}, '
            f'publishing {SCENE_RAW_TOPIC} + {SCENE_TRUST_TOPIC}')

    # --- asset + discovery -------------------------------------------------

    def _load_mesh(self, path: str) -> bytes | None:
        """Return glTF bytes for embedding, or None to fall back to a box."""
        try:
            with open(path, 'rb') as fh:
                return fh.read()
        except OSError:
            self.get_logger().warning(f'mesh not found: {path} (box fallback)')
            return None

    def _discover_trust_topics(self):
        """Subscribe to any new trust_output and verdict topics."""
        for name, _types in self.get_topic_names_and_types():
            ego = pmsg.ego_id_from_trust_topic(name)
            if ego is not None and name not in self._trust_subs:
                self._trust_subs[name] = self.create_subscription(
                    pmsg.OutputMessage, name,
                    lambda m, e=ego: self._on_trust(m, e), 10)
                self.get_logger().info(
                    f'discovered trust view of ego {ego} on {name}')
            # Verdicts carry r_new for EVERY judged sender, gate-passing or
            # not, while trust_output carries only the ones that passed. The
            # reputation label reads from here so it still shows a number for
            # a sender being withheld -- which is exactly when a viewer most
            # wants to see it.
            vego = tv.ego_id_from_topic(name)
            if vego is not None and name not in self._verdict_subs:
                self._verdict_subs[name] = self.create_subscription(
                    tv.Message, name,
                    lambda m, e=vego: self._on_verdict(m, e), 10)
                self.get_logger().info(
                    f'discovered verdicts of ego {vego} on {name}')

    def _on_verdict(self, msg, ego: int):
        """Cache one ego's reputation for one judged sender."""
        self._reputation.setdefault(ego, {})[tv.sender_of(msg)] = (
            float(msg.r_new), bool(msg.trusted))

    def _gs_suffix(self, sender: int, ego: int) -> str:
        """' gs: X' for a judged sender, '' when this ego has no verdict yet."""
        entry = self._reputation.get(ego, {}).get(sender)
        if entry is None:
            return ''
        score, trusted = entry
        return f' gs: {score:.2f}' + ('' if trusted else ' GATED')

    # --- ingest ------------------------------------------------------------

    def _now(self) -> float:
        """Node clock, seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_raw(self, msg):
        """Cache one sender's raw SDSM, latching its heading from ref_pos deltas."""
        sid = int(msg.source_id[0])
        x, y = msg.ref_pos_x, msg.ref_pos_y
        prev = self._sender_pose.get(sid)
        if prev is None:
            yaw = 0.0
        else:
            px, py, pyaw = prev
            yaw = math.atan2(y - py, x - px) if math.hypot(x - px, y - py) > _MOVE_EPS_M else pyaw
        self._sender_pose[sid] = (x, y, yaw)
        self._raw[sid] = (msg, self._now())

    def _on_trust(self, msg, ego: int):
        """Cache one ego's OutputMessage rebroadcast of one gate-passing sender."""
        self._trust.setdefault(ego, {})[int(msg.source_id[0])] = (msg, self._now())

    def _fresh(self, cache: dict) -> dict:
        """Drop entries older than stale_sec, returning the survivors."""
        cutoff = self._now() - self.stale_sec
        for key in [k for k, (_m, t) in cache.items() if t < cutoff]:
            del cache[key]
        return cache

    # --- render ------------------------------------------------------------

    def _render(self):
        """Rebuild and publish both scenes from the current caches."""
        ego = int(self.get_parameter('ego_source_id').value)
        # Floored at stale_sec rather than derived from render_hz alone. A few
        # render periods is enough to ride out jitter in principle, but only if
        # the render loop actually holds its rate -- under load (an emulated
        # container, a big scene) it does not, and a sub-second lifetime then
        # expires every entity before the next publish, emptying the view.
        # Erring long costs a stale box for a moment; erring short costs the
        # whole scene.
        lifetime_s = max(self.stale_sec, 2.5 / max(self.render_hz, 0.1))
        lifetime = Duration(sec=int(lifetime_s),
                            nanosec=int((lifetime_s % 1.0) * 1e9))

        if self.show_ground:
            self.ground_pub.publish(
                SceneUpdate(deletions=[], entities=[self._ground_entity()]))

        raw_entities = []
        for sid, (msg, _t) in self._fresh(self._raw).items():
            raw_entities += self._sender_entities(
                'raw', msg, sid, ego, lifetime, trust=False)
        self.raw_pub.publish(SceneUpdate(deletions=[], entities=raw_entities))

        trust_entities = []
        ego_raw = self._raw.get(ego)
        if ego_raw is not None:  # draw "you" for context, from the raw cache
            ego_yaw = self._sender_pose.get(ego, (0.0, 0.0, 0.0))[2]
            trust_entities.append(self._vehicle_entity(
                f'trust/ego/{ego}', lifetime, self._world(ego_raw[0], 0, 0, 0),
                ego_yaw, _EGO_COLOR, f'EGO {ego}', [], self._sports_glb,
                self.sports_scale, _SENDER_SIZE))
        for sid, (msg, _t) in self._fresh(self._trust.get(ego, {})).items():
            trust_entities += self._sender_entities(
                'trust', msg, sid, ego, lifetime, trust=True)
        self.trust_pub.publish(SceneUpdate(deletions=[], entities=trust_entities))

    def _sender_entities(self, tag, msg, sid, ego, lifetime, trust):
        """Entities for one broadcaster: the sender vehicle + its objects."""
        entities = []
        is_ego = (sid == ego)

        # The broadcaster's own marker, at ref_pos -- EGO ONLY. A peer's
        # ref_pos is its LIDAR origin, so its marker floats a metre above
        # ego's detection of the same vehicle, and the two carry the same
        # reputation label (see _peer_gs): one car hovering over one box,
        # both saying gs. The detection is the better of the pair -- it sits
        # on the car and it is the thing a viewer points at.
        # A peer ego never detects therefore has no marker at all, which
        # reads correctly (ego cannot see it); its broadcast objects are
        # still drawn, so the sender is not lost from the scene.
        if is_ego:
            # An RSU is infrastructure: a small box, not a car.
            if pmsg.get_equipment_type_of(msg) == _EQUIP_RSU:
                glb, scale, box_size = None, self.car_scale, _RSU_SIZE
            else:
                glb, scale, box_size = (
                    self._sports_glb, self.sports_scale, _SENDER_SIZE)
            sender_yaw = self._sender_pose.get(sid, (0.0, 0.0, 0.0))[2]
            entities.append(self._vehicle_entity(
                f'{tag}/sender/{sid}', lifetime, self._world(msg, 0, 0, 0),
                sender_yaw, _EGO_COLOR, f'EGO {sid}',
                [('agent_id', str(sid))], glb, scale, box_size))

        # Its detected objects, at ref_pos + offset.
        for k in range(int(msg.num_objects)):
            entities.append(
                self._object_entity(tag, msg, sid, k, lifetime, trust, ego))
        return entities

    def _peer_gs(self, pos, exclude: int, ego: int) -> str:
        """' gs: X (agent N)' if this object sits where a peer reports itself.

        A detected object at a broadcaster's own ref_pos IS that broadcaster,
        so ego's reputation for it belongs on that box: it is the only place
        a viewer sees a peer's global score attached to the thing they can
        actually point at on screen. Ego is skipped along with the reporting
        sender -- neither has a global score from ego's own perspective.
        """
        best, best_distance = None, self.sender_match_radius_m
        for sender, (sx, sy, _yaw) in self._sender_pose.items():
            if sender == exclude or sender == ego:
                continue
            distance = math.hypot(pos[0] - (sx - self.center_x),
                                  pos[1] - (sy - self.center_y))
            if distance < best_distance:
                best, best_distance = sender, distance
        if best is None:
            return ''
        suffix = self._gs_suffix(best, ego)
        return f'{suffix} (agent {best})' if suffix else ''

    def _object_entity(self, tag, msg, sid, k, lifetime, trust, ego):
        """One detected object as a car (vehicle) or box (unknown type)."""
        pos = self._world(msg, msg.offset_x[k], msg.offset_y[k], msg.offset_z[k])
        raw_h = int(msg.obj_heading[k])
        yaw = 0.0 if raw_h == _HEADING_UNAVAIL else _yaw_from_j2735_heading(raw_h * _HEADING_DEG)
        size = (max(msg.obj_length[k] * _OFFSET_M, 0.1),
                max(msg.obj_width[k] * _OFFSET_M, 0.1),
                max(msg.obj_height[k] * _OFFSET_M, 0.1))
        speed = msg.obj_speed[k] * _SPEED_MPS
        score = pmsg.global_score_of(msg) if trust else float(msg.obj_local_scores[k])
        is_vehicle = int(msg.obj_type[k]) == _VEHICLE
        # Detected objects are boxes by default: a ModelPrimitive carries the
        # whole .glb inline in EVERY entity of EVERY frame, so meshing N
        # objects at render_hz costs N x 165 KB x render_hz on the wire and is
        # what makes a busy scene unusable. Senders keep their meshes -- there
        # are only a handful of them. Set object_meshes:=true to get cars back.
        glb = self._car_glb if (is_vehicle and self.object_meshes) else None
        # Only a judged peer has a global score; ego never scores itself, and
        # the raw scene predates judgement entirely.
        # Only a non-vehicle is named: "car" on every box was noise, since
        # nearly all of them are, and the shape already says so. That now
        # covers UNKNOWN as well as VRU -- a class the sender could not place
        # renders identically to a car, and silence there read as a confident
        # "it's a vehicle" when the sender claimed no such thing.
        obj_type = int(msg.obj_type[k])
        prefix = '' if obj_type == _VEHICLE else f'{pmsg.class_name_of(obj_type)} '
        lscore = (pmsg.local_score_of(msg) if trust
                  else float(msg.obj_local_scores[k]))
        label = f'{prefix}ls: {lscore:.2f}'
        # gs only where one actually exists. Two ways it can: the object was
        # reported BY a judged peer (trust scene), or the object IS a judged
        # peer -- ego's own detection of the other vehicle, which is what a
        # viewer actually looks at when asking "how much do we trust that
        # car?". Ego never scores itself, so its own objects carry no gs.
        if trust:
            label += f' gs: {pmsg.global_score_of(msg):.2f}'
        else:
            label += self._peer_gs(pos, sid, ego)
        meta = [('object_id', str(int(msg.object_id[k]))),
                ('obj_type', str(int(msg.obj_type[k]))),
                ('speed_mps', f'{speed:.2f}'),
                ('score', f'{score:.3f}'),
                ('size_LWH_m', f'{size[0]:.1f}x{size[1]:.1f}x{size[2]:.1f}'),
                ('from_agent', str(sid))]
        # Scale the mesh relative to the object's SDSM length.
        scale = self.car_scale * (size[0] / _REF_CAR_LEN_M)
        # Keyed on the tracker's object_id, NOT the array index: an object's
        # index shifts as others enter and leave the message, so an
        # index-keyed entity id addresses a different car frame to frame,
        # which reads as boxes jumping and flickering between objects.
        return self._vehicle_entity(
            f'{tag}/sender/{sid}/obj/{int(msg.object_id[k])}', lifetime, pos, yaw,
            _score_color(score), label, meta, glb, scale, size)

    def _ground_entity(self) -> SceneEntity:
        """Static asphalt plane + grid lines under the scene (street reference)."""
        half = self.ground_extent_m / 2.0
        cx, cy = -self.center_x, -self.center_y
        points = []
        n = int(self.ground_extent_m / max(self.ground_step_m, 0.1))
        for i in range(n + 1):
            g = -half + i * self.ground_step_m
            points += [Point(x=cx + g, y=cy - half, z=0.02),
                       Point(x=cx + g, y=cy + half, z=0.02),
                       Point(x=cx - half, y=cy + g, z=0.02),
                       Point(x=cx + half, y=cy + g, z=0.02)]
        return SceneEntity(
            timestamp=self.get_clock().now().to_msg(),
            frame_id=self.frame_id, id='ground',
            lifetime=Duration(sec=0, nanosec=0), frame_locked=False, metadata=[],
            cubes=[CubePrimitive(
                pose=Pose(position=Point(x=cx, y=cy, z=-0.05),
                          orientation=Quaternion(w=1.0)),
                size=Vector3(x=self.ground_extent_m, y=self.ground_extent_m, z=0.1),
                color=_ASPHALT_COLOR)],
            lines=[LinePrimitive(
                type=LinePrimitive.LINE_LIST,
                pose=Pose(orientation=Quaternion(w=1.0)),
                # pixel-width so the grid stays visible at any zoom
                thickness=1.5, scale_invariant=True, points=points,
                color=_GRID_COLOR, colors=[])])

    # --- primitives --------------------------------------------------------

    def _world(self, msg, off_x, off_y, off_z) -> tuple:
        """World position of ref_pos + offset (0.1 m units), minus the centre."""
        return (msg.ref_pos_x + off_x * _OFFSET_M - self.center_x,
                msg.ref_pos_y + off_y * _OFFSET_M - self.center_y,
                msg.ref_pos_z + off_z * _OFFSET_M)

    def _model_orientation(self, yaw: float) -> Quaternion:
        """Object heading composed with the fixed glTF base orientation."""
        d2r = math.pi / 180.0
        base = _quat_mul(
            _quat_z(self.model_yaw_deg * d2r),
            _quat_mul(_quat_y(self.model_pitch_deg * d2r),
                      _quat_x(self.model_roll_deg * d2r)))
        return _quat_mul(_quat_z(yaw), base)

    def _vehicle_entity(self, eid, lifetime, pos, yaw, color, label, meta,
                        glb, scale, box_size) -> SceneEntity:
        """A vehicle as an embedded glTF model, or a box when glb is None."""
        x, y, z = pos
        metadata = [KeyValuePair(key=k, value=v) for k, v in meta]
        entity = SceneEntity(
            timestamp=self.get_clock().now().to_msg(),
            frame_id=self.frame_id, id=eid, lifetime=lifetime,
            frame_locked=False, metadata=metadata)
        if glb is not None:
            entity.models = [ModelPrimitive(
                pose=Pose(position=Point(x=x, y=y, z=z),
                          orientation=self._model_orientation(yaw)),
                scale=Vector3(x=scale, y=scale, z=scale),
                color=color, override_color=True,
                media_type='model/gltf-binary', data=glb)]
        else:
            bx, by, bz = box_size
            entity.cubes = [CubePrimitive(
                pose=Pose(position=Point(x=x, y=y, z=z), orientation=_quat_z(yaw)),
                size=Vector3(x=bx, y=by, z=bz), color=color)]
        if self.show_labels:
            entity.texts = [TextPrimitive(
                pose=Pose(position=Point(x=x, y=y, z=z + box_size[2] / 2.0 + 0.5),
                          orientation=Quaternion(w=1.0)),
                billboard=True, font_size=0.4, scale_invariant=False,
                color=Color(r=1.0, g=1.0, b=1.0, a=1.0), text=label)]
        return entity


def main(args=None):
    """Spin the scene node."""
    rclpy.init(args=args)
    try:
        rclpy.spin(SceneNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
