"""
The lanelet-map overlay for the Lichtblick 3D panel, as SceneUpdate lines.

WHY A NODE, NOT THE AERIAL'S ROUTE. The aerial is a raster: one textured quad
(ground.glb) placed by ground_backdrop_node. A lanelet map is NOT a picture --
it is a set of lane-boundary polylines in lat/lon -- so there is no texture to
place; every vertex must be projected into `world` and drawn as line segments.
This node does exactly that: load the clipped .osm, project each way through
geo_anchor, and publish the boundaries as foxglove LinePrimitives on one topic,
toggle-able in the panel next to scene_raw / scene_trust.

ALIGNMENT MIRRORS THE BACKDROP. Nothing here is georeferenced (see geo_anchor),
so the lanes are placed by hand. The map is anchored on its OWN centroid, so
center_x/center_y place the map centre, yaw_deg rotates it IN PLACE about that
centre, and scale sizes it -- the same three-knob drag the aerial uses, and it
needs no correct guess of any real-world point. They default to the tuned
alignment for this intersection and are LIVE -- setting any of them (Lichtblick's
Parameters panel or `ros2 param set`) re-projects and redraws at once, so
alignment is done by nudging values and watching the lanes move. z sits just
above the aerial (which is at -1.0) so the lines are not buried in it.

THE MAP IS LOCAL, NOT COMMITTED. westwood_intersection.osm is produced by
tools/lichtblick/clip_lanelet_osm.py from the 121 MB campus source, and both are
gitignored; colcon still installs it to the package share, so it resolves like
the meshes. If it is absent the node logs and draws nothing -- the rest of the
graph is unaffected. Regenerate it with the clip script.

Only the ways loaded once at startup hold lat/lon; the projection is redone on
every publish, which is what makes re-tuning live and cheap (a few hundred
points). The overlay is latched (transient-local) so a client that connects
later still gets it, and is re-sent on a slow timer so it survives a bridge
restart.
"""

import os
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from foxglove_msgs.msg import Color, LinePrimitive, SceneEntity, SceneUpdate

from global_trust_perception.lichtblick import geo_anchor

OVERLAY_TOPIC = '/perception/global_trustworthiness/viz/lanelet_overlay'
_DEFAULT_MAP = 'westwood_intersection.osm'
_REPUBLISH_SEC = 2.0

# Lane boundaries are the only geometry the clip keeps (Lanelet2 tags them
# type=line_thin), so that is what this draws. Colour by subtype so the road
# reads: a solid boundary crisp, a dashed one dimmer. A way of neither known
# subtype falls back to a neutral line rather than being dropped.
_LANE_TYPE = 'line_thin'
_COLORS = {
    'solid': Color(r=1.0, g=0.85, b=0.1, a=0.9),
    'dashed': Color(r=1.0, g=0.85, b=0.1, a=0.45),
}
_DEFAULT_COLOR = Color(r=0.8, g=0.8, b=0.85, a=0.8)


class LaneletOverlayNode(Node):
    """Publishes one latched SceneUpdate: the lane boundaries as LinePrimitives."""

    def __init__(self):
        """Declare the alignment params, load the map, and start publishing."""
        super().__init__('lanelet_overlay_node')

        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('map_file', _DEFAULT_MAP)
        # Tuned to seat the map on the point-cloud intersection (Westwood Plaza x
        # Charles E Young Dr S); live-settable to re-align (see module docstring).
        self.declare_parameter('center_x', 39.5)
        self.declare_parameter('center_y', -165.5)
        self.declare_parameter('z', -0.9)
        self.declare_parameter('yaw_deg', 4.0)
        self.declare_parameter('scale', 1.17)
        # Pixel thickness (scale_invariant) so the lines stay visible at any zoom.
        self.declare_parameter('thickness', 1.0)
        self._read_params()

        # The map's ways as (subtype, [(lat, lon), ...]) -- parsed once. Only the
        # projection is redone when an alignment param changes, not the parse.
        self._ways = self._load_ways()
        # Anchor the projection on the map's own centroid, so center_x/center_y
        # place the map centre and yaw/scale pivot about it (see _map_centroid).
        self._anchor_lat, self._anchor_lon = self._map_centroid()

        # Transient-local so a Lichtblick client that connects after this node
        # started still receives the one overlay.
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(SceneUpdate, OVERLAY_TOPIC, qos)

        # Redraw the moment any param is set -- this is what makes the Parameters
        # panel a live alignment tool. post_set so get_parameter already returns
        # the new value inside the callback.
        self.add_post_set_parameters_callback(self._on_params_set)

        # Re-send on a slow timer as well: it restores the overlay after a bridge
        # restart, which the transient-local latch alone does not survive.
        self.create_timer(_REPUBLISH_SEC, self._publish)
        self._publish()

        self.get_logger().info(
            f'lanelet_overlay up: {len(self._ways)} ways from {self.map_file}, '
            f'frame_id={self.frame_id}, centre=({self.center_x}, {self.center_y}), '
            f'yaw={self.yaw_deg} deg, publishing {OVERLAY_TOPIC}')

    def _read_params(self) -> None:
        """Load the params into instance state (the single param -> state place)."""
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.map_file = str(self.get_parameter('map_file').value)
        self.center_x = float(self.get_parameter('center_x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.z = float(self.get_parameter('z').value)
        self.yaw_deg = float(self.get_parameter('yaw_deg').value)
        self.scale = float(self.get_parameter('scale').value)
        self.thickness = float(self.get_parameter('thickness').value)

    def _load_ways(self) -> list:
        """Parse the clipped .osm into (subtype, [(lat, lon), ...]) per lane way.

        Returns [] (and logs) if the map is absent, so a missing LOCAL file
        degrades to no overlay rather than a crash -- the same fail-soft the
        mesh nodes use when an asset is missing.
        """
        try:
            path = os.path.join(
                get_package_share_directory('global_trust_perception'),
                'assets', self.map_file)
        except PackageNotFoundError:
            self.get_logger().warning(
                'package share directory not found; overlay empty')
            return []
        if not os.path.exists(path):
            self.get_logger().warning(
                f'{path} not found; overlay empty (regenerate with '
                'tools/lichtblick/clip_lanelet_osm.py)')
            return []

        root = ET.parse(path).getroot()
        nodes = {el.get('id'): (float(el.get('lat')), float(el.get('lon')))
                 for el in root if el.tag == 'node'}
        ways = []
        for el in root:
            if el.tag != 'way':
                continue
            tags = {t.get('k'): t.get('v') for t in el.findall('tag')}
            if tags.get('type') != _LANE_TYPE:
                continue
            coords = [nodes[nd.get('ref')] for nd in el.findall('nd')
                      if nd.get('ref') in nodes]
            if len(coords) >= 2:
                ways.append((tags.get('subtype'), coords))
        self.get_logger().info(f'loaded {len(ways)} lane ways from {path}')
        return ways

    def _map_centroid(self) -> tuple:
        """Mean (lat, lon) of every loaded vertex -- the projection anchor.

        Anchoring on the map's own centre means center_x/center_y place that
        centre and yaw/scale pivot about it, so alignment needs no correct guess
        of any single real-world point. (0, 0) for an empty map, which never
        draws anyway.
        """
        pts = [c for _subtype, coords in self._ways for c in coords]
        if not pts:
            return (0.0, 0.0)
        return (sum(p[0] for p in pts) / len(pts),
                sum(p[1] for p in pts) / len(pts))

    def _line_primitives(self) -> list:
        """One LINE_STRIP per lane way, projected through geo_anchor."""
        lines = []
        for subtype, coords in self._ways:
            points = []
            for lat, lon in coords:
                x, y = geo_anchor.lonlat_to_world(
                    lat, lon, self._anchor_lat, self._anchor_lon,
                    self.center_x, self.center_y, self.yaw_deg, self.scale)
                points.append(Point(x=x, y=y, z=self.z))
            lines.append(LinePrimitive(
                type=LinePrimitive.LINE_STRIP,
                pose=Pose(orientation=Quaternion(w=1.0)),
                thickness=self.thickness, scale_invariant=True,
                points=points, color=_COLORS.get(subtype, _DEFAULT_COLOR),
                colors=[]))
        return lines

    def _publish(self) -> None:
        """Build and send the single overlay entity."""
        entity = SceneEntity(
            timestamp=self.get_clock().now().to_msg(),
            frame_id=self.frame_id, id='lanelet_overlay',
            lifetime=Duration(sec=0, nanosec=0), frame_locked=False,
            metadata=[], lines=self._line_primitives())
        self.pub.publish(SceneUpdate(deletions=[], entities=[entity]))

    def _on_params_set(self, params) -> None:
        """Re-read and redraw on any param set; reload the map only if it changed."""
        previous_map = self.map_file
        self._read_params()
        if self.map_file != previous_map:
            self._ways = self._load_ways()
            self._anchor_lat, self._anchor_lon = self._map_centroid()
        self._publish()
        self.get_logger().info(
            f're-tuned: centre=({self.center_x}, {self.center_y}), '
            f'yaw={self.yaw_deg} deg, scale={self.scale}, z={self.z}')


def main(args=None):
    """Spin the lanelet overlay node."""
    rclpy.init(args=args)
    try:
        rclpy.spin(LaneletOverlayNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
