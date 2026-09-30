"""
The aerial backdrop for the 3D panel, as a mesh_resource Marker.

WHY A MARKER, NOT THE URDF LAYER. Lichtblick's 3D panel can only add Grid and
URDF layers, so the backdrop first went in as a textured glb behind a URDF
layer. In Lichtblick 1.27.1 that path draws the quad but NOT its texture: the
URDF renderer substitutes the layer's fallbackColor for
the mesh's own material, so the aerial comes up a flat colour and changing the
fallbackColor changes it -- the image never reaches the scene. A
`visualization_msgs/Marker` of type MESH_RESOURCE goes through a different
renderer, the same one the car and pedestrian meshes already use, which honours
the glb's embedded materials. So this node publishes exactly that: one marker
pointing at the same generated ground.glb, with mesh_use_embedded_materials TRUE
(the object meshes set it FALSE on purpose, to tint themselves the verdict
colour; the ground wants the opposite -- its own texture, untinted).

It shares the meshes' caveat: `package://` resolves over foxglove_bridge's asset
channel, so the backdrop needs a LIVE bridge and does not render against a
recorded MCAP opened as a file.

ALIGNMENT IS THE FOUR PARAMS BELOW, and they are a guess -- nothing here is
georeferenced (`world` is the recording's own localization root and a PNG
carries no scale), so they are set by eye against the point cloud. They mirror
what that URDF layer used to hold, so the seeded values are the same:

    frame_id   the frame the aerial is pinned in. `world` for the two-agent
               layout; a sensor frame (`vehicle_lidar`) for a single-agent one.
    x, y, z    where the image CENTRE sits in that frame. z is set by eye to
               clear the LiDAR's own ground returns without z-fighting them.
    yaw_deg    rotation about the vertical axis, to line the image's north up
               with the frame. The mesh renderer applies the same glTF Y-up ->
               scene Z-up conversion the car meshes rely on, so the quad lands
               flat with no roll; if it ever comes up standing like a wall, that
               conversion did not happen and the glb, not this node, is wrong.
    scale_m    the image WIDTH in metres. The generated quad is one unit wide
               and aspect-correct in height, so this one number is the scale.

SCALE FIRST, THEN POSITION. A useful check: the two agents are 9.06 m apart --
infrastructure at (29.40, -157.89), vehicle at (32.41, -149.34) -- which at the
seeded scale is about 63 px on the 900 px image. If they do not land that far apart,
the scale is wrong and no amount of nudging x/y will fix it.

ground.glb is generated from background.png by
tools/lichtblick/make_ground_glb.py -- do not hand-edit it. Mirroring is the one
error yaw cannot undo: if the imagery reads backwards, swap the u column in _UV
in that script and regenerate.

    ros2 run global_trust_perception ground_backdrop --ros-args \
        -p frame_id:=world -p x:=45.7 -p y:=-166.0 -p scale_m:=130.0

The marker is latched (transient-local) so a client that connects later still
gets it, and is also re-sent on a slow timer so it survives a bridge restart --
cheap, because a MESH_RESOURCE marker only references the glb by URI, it does
not inline it.

The five alignment params are LIVE: setting any of them -- from Lichtblick's
Parameters panel or `ros2 param set` -- re-reads them and redraws the marker at
once, so alignment is done by nudging values and watching the image move, not by
restarting the node.
"""

import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from visualization_msgs.msg import Marker

GROUND_TOPIC = '/perception/global_trustworthiness/viz/ground_backdrop'
_GROUND_MESH = 'package://global_trust_perception/assets/ground.glb'
_REPUBLISH_SEC = 2.0


class GroundBackdropNode(Node):
    """Publishes one latched mesh_resource Marker: the aerial ground image."""

    def __init__(self):
        """Declare the alignment params and start publishing the backdrop."""
        super().__init__('ground_backdrop_node')

        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('mesh_resource', _GROUND_MESH)
        self.declare_parameter('x', 45.7)
        self.declare_parameter('y', -166.0)
        self.declare_parameter('z', -1.0)
        self.declare_parameter('yaw_deg', 4.0)
        self.declare_parameter('scale_m', 130.0)
        self._read_params()

        # Transient-local so a Lichtblick client that connects after this node
        # started still receives the one marker.
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Marker, GROUND_TOPIC, qos)

        # Redraw the moment any alignment param is set -- this is what makes the
        # Parameters panel a live alignment tool instead of a restart-per-nudge
        # one. post_set (not on_set) so get_parameter already returns the new
        # value inside the callback.
        self.add_post_set_parameters_callback(self._on_params_set)

        # Re-send on a slow timer as well: it costs nothing (the mesh is
        # referenced, not inlined) and it restores the backdrop after a bridge
        # restart, which the transient-local latch alone does not survive.
        self.create_timer(_REPUBLISH_SEC, self._publish)
        self._publish()

        self.get_logger().info(
            f'ground_backdrop up: frame_id={self.frame_id}, '
            f'centre=({self.x}, {self.y}, {self.z}), yaw={self.yaw_deg} deg, '
            f'scale={self.scale_m} m, publishing {GROUND_TOPIC}')

    def _read_params(self) -> None:
        """Load the alignment params into instance state.

        The single place param -> state happens, so startup and live re-tuning
        read the same way and cannot drift apart.
        """
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.mesh_resource = str(self.get_parameter('mesh_resource').value)
        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.z = float(self.get_parameter('z').value)
        self.yaw_deg = float(self.get_parameter('yaw_deg').value)
        self.scale_m = float(self.get_parameter('scale_m').value)

    def _on_params_set(self, params) -> None:
        """Re-read and redraw whenever an alignment param is set (live tuning)."""
        self._read_params()
        self._publish()
        self.get_logger().info(
            f're-tuned: centre=({self.x}, {self.y}, {self.z}), '
            f'yaw={self.yaw_deg} deg, scale={self.scale_m} m')

    def _publish(self) -> None:
        """Build and send the single ground marker."""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'ground_backdrop'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = self.mesh_resource
        # Draw the glb's own texture rather than tinting it -- the whole reason
        # this exists instead of the URDF layer.
        marker.mesh_use_embedded_materials = True

        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = self.z
        yaw = math.radians(self.yaw_deg)
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)

        # The quad is one unit wide and aspect-correct, so a uniform scale makes
        # scale_m the image width in metres.
        marker.scale.x = marker.scale.y = marker.scale.z = self.scale_m

        # White and opaque: with embedded materials on, this is a tint, and
        # white leaves the texture as authored.
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0

        # Static: never expire between re-sends.
        marker.lifetime = Duration(sec=0, nanosec=0)

        self.pub.publish(marker)


def main(args=None):
    """Spin the ground backdrop node."""
    rclpy.init(args=args)
    try:
        rclpy.spin(GroundBackdropNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
