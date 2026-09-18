#!/usr/bin/env python3
"""Score lidar support for TF-resolved Autoware tracked boxes."""

import math

from autoware_perception_msgs.msg import TrackedObjects
from local_trust_estimation.tracked_object_utils import (
    class_name,
    make_score_array,
    normalized_rotation_matrix,
    transform_pose,
    uuid_text,
)
from message_filters import Subscriber, TimeSynchronizer
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sdsm_interfaces.msg import TrustScoreArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


LIVE_TF_CACHE_SECONDS = 10.0
OFFLINE_TF_CACHE_SECONDS = 86400.0


class LidarPointCountNode(Node):
    """Publish UUID-keyed lidar point-support scores."""

    def __init__(self, offline=False):
        """Initialize parameters, TF access, synchronization, and output."""
        super().__init__('lidar_point_count_node')

        self.declare_parameter('cloud_topic', '/vehicle/lidar/points')
        self.declare_parameter(
            'tracked_objects_topic',
            '/vehicle/perception/tracked_objects',
        )
        self.declare_parameter(
            'output_topic',
            '/local_trust_estimation/lidar_point_count',
        )
        self.declare_parameter('full_trust_points', 50)
        self.declare_parameter('tf_timeout', 0.1)

        cloud_topic = self.get_parameter('cloud_topic').value
        tracked_objects_topic = self.get_parameter(
            'tracked_objects_topic'
        ).value
        output_topic = self.get_parameter('output_topic').value
        self.full_trust_points = self.get_parameter(
            'full_trust_points'
        ).value
        tf_timeout = self.get_parameter('tf_timeout').value
        if self.full_trust_points <= 0:
            raise ValueError('full_trust_points must be greater than 0')
        if tf_timeout < 0.0:
            raise ValueError('tf_timeout must be nonnegative')

        self.tf_buffer = Buffer(cache_time=Duration(
            seconds=(
                OFFLINE_TF_CACHE_SECONDS if offline
                else LIVE_TF_CACHE_SECONDS
            )
        ))
        self.tf_timeout = (
            Duration() if offline else Duration(seconds=tf_timeout)
        )
        # The listener gets its own node rather than this one: passing this
        # node would hand it to rclpy.spin()'s executor instead, leaving the
        # transform callbacks serialized behind synced_callback, which then
        # could never see a transform arrive while it waits. spin_thread=True
        # then spins that dedicated node on its own thread.
        self.tf_node = (
            None if offline
            else rclpy.create_node(f'{self.get_name()}_tf_listener')
        )
        self.tf_listener = (
            None if offline
            else TransformListener(self.tf_buffer, self.tf_node,
                                    spin_thread=True)
        )

        self.sync = TimeSynchronizer(
            [
                Subscriber(self, PointCloud2, cloud_topic),
                Subscriber(self, TrackedObjects, tracked_objects_topic),
            ],
            queue_size=10,
        )
        self.sync.registerCallback(self.synced_callback)
        self.publisher_ = self.create_publisher(
            TrustScoreArray, output_topic, 10
        )

        self.get_logger().info(
            f'Scoring {tracked_objects_topic} against {cloud_topic} '
            f'-> {output_topic}'
        )

    def destroy_node(self):
        """Tear down the listener's dedicated node alongside this one."""
        if self.tf_node is not None:
            # Stop the listener's own executor and join its thread first:
            # left running, it keeps spinning into rclpy's shutdown and dies
            # with ExternalShutdownException instead of ending quietly.
            executor = getattr(self.tf_listener, 'executor', None)
            thread = getattr(
                self.tf_listener, 'dedicated_listener_thread', None
            )
            if executor is not None:
                executor.shutdown()
            if thread is not None:
                thread.join(timeout=5.0)
            self.tf_listener = None
            self.tf_node.destroy_node()
            self.tf_node = None
        return super().destroy_node()

    def count_score(self, number_of_points):
        """Map a point count to a logarithmic score in [0, 1]."""
        if number_of_points <= 0:
            return 0.0
        return min(
            1.0,
            math.log1p(number_of_points)
            / math.log1p(self.full_trust_points),
        )

    @staticmethod
    def points_in_box(points, pose, dimensions):
        """Count points inside an arbitrarily oriented 3D box."""
        half_size = np.array([
            dimensions.x / 2.0,
            dimensions.y / 2.0,
            dimensions.z / 2.0,
        ])
        center = pose.position
        relative = points - np.array([center.x, center.y, center.z])

        radius_squared = float(np.dot(half_size, half_size))
        relative = relative[
            np.einsum('ij,ij->i', relative, relative) <= radius_squared
        ]
        if len(relative) == 0:
            return 0

        # The pose rotation maps box-local vectors into the cloud frame.
        # Row-vector points therefore use R (equivalent to R^T for columns)
        # to map the relative vectors back into the box frame.
        rotation = np.asarray(
            normalized_rotation_matrix(pose.orientation)
        )
        local = relative @ rotation
        inside = np.all(np.abs(local) <= half_size, axis=1)
        return int(np.count_nonzero(inside))

    def synced_callback(self, cloud_message, tracked_objects):
        """Transform tracked boxes into the cloud frame and score support."""
        transform = None
        if cloud_message.header.frame_id != tracked_objects.header.frame_id:
            try:
                transform = self.tf_buffer.lookup_transform(
                    cloud_message.header.frame_id,
                    tracked_objects.header.frame_id,
                    tracked_objects.header.stamp,
                    self.tf_timeout,
                )
            except TransformException as error:
                self.get_logger().warning(
                    f'No {cloud_message.header.frame_id} <- '
                    f'{tracked_objects.header.frame_id} transform ({error}) '
                    '- dropping lidar-support scores'
                )
                return

        points = point_cloud2.read_points_numpy(
            cloud_message,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        )

        scored = []
        flagged = 0
        for tracked_object in tracked_objects.objects:
            pose = tracked_object.kinematics.pose_with_covariance.pose
            if transform is not None:
                pose = transform_pose(pose, transform.transform)
            number_of_points = self.points_in_box(
                points, pose, tracked_object.shape.dimensions
            )
            score = self.count_score(number_of_points)
            scored.append((tracked_object, score))
            if score < 0.5:
                flagged += 1
                self.get_logger().warning(
                    f'Sparse support for {class_name(tracked_object)} '
                    f'(track {uuid_text(tracked_object.object_id)}): '
                    f'{number_of_points} lidar points -> score {score:.2f}'
                )

        self.publisher_.publish(make_score_array(tracked_objects, scored))
        self.get_logger().info(
            f'{len(scored)} tracked objects scored, {flagged} flagged'
        )


def main(args=None):
    """Run the lidar-point-count node."""
    rclpy.init(args=args)
    node = LidarPointCountNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
