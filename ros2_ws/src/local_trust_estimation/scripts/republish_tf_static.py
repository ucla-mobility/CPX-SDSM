#!/usr/bin/env python3
"""Hold the converted bags' /tf_static up for the whole session.

`ros2 bag play` is the only publisher of /tf_static, it publishes it once at
the start, and it EXITS when playback ends -- taking the latched message with
it. A viewer that connects late, or simply keeps looking after a short clip
finishes, then reports:

    Missing transform from frame <infrastructure_lidar> to frame <world>

and every frame-attached topic (point clouds, markers) silently stops
rendering. This node reads the transforms straight out of the bags once and
republishes them on a timer, so the tree is available before, during and after
playback regardless of when anything subscribes.

    python3 republish_tf_static.py <bag_dir> [<bag_dir> ...]

CORRECTING A MISREGISTERED AGENT. --yaw-correction <child>=<deg> rotates one
recorded transform about its own origin before republishing, which is where a
world-pose yaw error has to be fixed: an agent's detections are placed at
ref_pos + R_tf * local, so composing Rot_z(deg) onto R_tf swings the whole
agent -- its boxes, its point cloud and its SDSM alike -- as one rigid body,
with ref_pos left where it is.

Applied HERE, at the one place the tree is published, rather than by rewriting
the recording: the correction is an empirical per-bag-pair constant, and
baking a fitted number into a bag makes it unfalsifiable later. The demo
excludes /tf_static from `ros2 bag play` so this stays the single publisher --
two publishers of the same static transform race, and the loser is silent.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from tf2_msgs.msg import TFMessage


REPUBLISH_PERIOD_S = 1.0


def read_static_transforms(bag_dirs):
    """Collect every /tf_static transform from the given bags, de-duplicated.

    rosbags returns its OWN dataclasses, not rclpy message objects, and the
    two are not interchangeable -- handing one to a real publisher aborts the
    process inside the C extension. Each transform is therefore copied field
    by field into a genuine geometry_msgs/TransformStamped.
    """
    from pathlib import Path

    from geometry_msgs.msg import TransformStamped
    from rosbags.highlevel import AnyReader

    seen, out = set(), []
    for bag in bag_dirs:
        reader = AnyReader([Path(bag)])
        reader.open()
        conns = [c for c in reader.connections if c.topic == '/tf_static']
        for connection, _t, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, connection.msgtype)
            for src in msg.transforms:
                key = (src.header.frame_id, src.child_frame_id)
                if key in seen:
                    continue
                seen.add(key)
                dst = TransformStamped()
                dst.header.frame_id = src.header.frame_id
                dst.child_frame_id = src.child_frame_id
                dst.transform.translation.x = float(src.transform.translation.x)
                dst.transform.translation.y = float(src.transform.translation.y)
                dst.transform.translation.z = float(src.transform.translation.z)
                dst.transform.rotation.x = float(src.transform.rotation.x)
                dst.transform.rotation.y = float(src.transform.rotation.y)
                dst.transform.rotation.z = float(src.transform.rotation.z)
                dst.transform.rotation.w = float(src.transform.rotation.w)
                out.append(dst)
        reader.close()
    return out


def apply_yaw_correction(transforms, corrections):
    """Compose a yaw (degrees, about world +Z) onto the named child frames.

    corrections is {child_frame_id: degrees}. Pre-multiplying -- q_new =
    q_delta * q_old -- rotates the frame in its PARENT's axes, which is what
    turns the agent in `world`; post-multiplying would instead spin it about
    its own already-rotated axes and is not the same thing for a frame that
    is not axis-aligned (infrastructure_base sits at -2.557 deg, vehicle_base
    at -97.976 deg, so the two differ here).

    Returns the number of transforms actually touched, so the caller can say
    so out loud rather than silently doing nothing when a frame name is
    misspelled.
    """
    touched = 0
    for transform in transforms:
        degrees = corrections.get(transform.child_frame_id)
        if degrees is None:
            continue
        half = math.radians(degrees) / 2.0
        dz, dw = math.sin(half), math.cos(half)
        q = transform.rotation if hasattr(transform, 'rotation') \
            else transform.transform.rotation
        # Hamilton product q_delta * q_old for a delta about +Z only.
        x, y, z, w = q.x, q.y, q.z, q.w
        q.x, q.y = dw * x - dz * y, dw * y + dz * x
        q.z, q.w = dw * z + dz * w, dw * w - dz * z
        touched += 1
    return touched


def parse_corrections(pairs):
    """['infrastructure_base=-4.3'] -> {'infrastructure_base': -4.3}."""
    out = {}
    for item in pairs:
        frame, _, value = item.partition('=')
        if not frame or not value:
            raise ValueError(
                f'--yaw-correction expects <child_frame>=<degrees>, got {item!r}')
        out[frame] = float(value)
    return out


class StaticTfHolder(Node):
    """Republish a fixed set of transforms on /tf_static."""

    def __init__(self, transforms):
        super().__init__('tf_static_holder')
        # Transient-local so a subscriber that arrives between ticks still
        # gets the tree immediately rather than waiting for the next one.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(TFMessage, '/tf_static', qos)
        self._msg = TFMessage(transforms=transforms)
        self._publish()
        self.create_timer(REPUBLISH_PERIOD_S, self._publish)
        pairs = ', '.join(
            f'{t.header.frame_id}->{t.child_frame_id}' for t in transforms)
        self.get_logger().info(f'holding {len(transforms)} transform(s): {pairs}')

    def _publish(self):
        # Restamp each tick: a consumer that treats /tf_static as time-bounded
        # should see it as current, not as a message from the recording's epoch.
        now = self.get_clock().now().to_msg()
        for transform in self._msg.transforms:
            transform.header.stamp = now
        self._pub.publish(self._msg)


def main():
    """Read the bags, then hold their static tree up until shut down."""
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('bags', nargs='+', help='converted bag directories')
    parser.add_argument(
        '--yaw-correction', action='append', default=[], metavar='CHILD=DEG',
        help='rotate one child frame by DEG degrees about world +Z before '
             'republishing; repeatable (see the module docstring)',
    )
    args = parser.parse_args()

    try:
        corrections = parse_corrections(args.yaw_correction)
    except ValueError as error:
        parser.error(str(error))

    transforms = read_static_transforms(args.bags)
    if not transforms:
        print('no /tf_static found in the given bag(s)')
        return 1

    if corrections:
        touched = apply_yaw_correction(transforms, corrections)
        print(f'applied yaw correction {corrections} to {touched} transform(s)')
        if touched != len(corrections):
            # Names come from the bag, so a typo here fails silently otherwise:
            # the tree publishes uncorrected and everything downstream looks
            # merely mediocre rather than broken.
            present = sorted(t.child_frame_id for t in transforms)
            print(f'WARNING: asked to correct {sorted(corrections)} but the '
                  f'bags only carry {present}', file=sys.stderr)

    rclpy.init()
    node = StaticTfHolder(transforms)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
