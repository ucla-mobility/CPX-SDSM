#!/usr/bin/env python3
"""Score tracked objects by their TF-resolved distance from the lidar."""

import math

from autoware_perception_msgs.msg import TrackedObjects
from local_trust_estimation.tracked_object_utils import (
    class_name,
    make_score_array,
    transform_pose,
    uuid_text,
)
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sdsm_interfaces.msg import TrustScoreArray
from tf2_ros import Buffer, TransformException, TransformListener


LIVE_TF_CACHE_SECONDS = 10.0
OFFLINE_TF_CACHE_SECONDS = 86400.0


class ObjectDistanceNode(Node):
    """Publish UUID-keyed sensor-distance scores."""

    def __init__(self, offline=False):
        """Initialize parameters, TF access, and ROS interfaces."""
        super().__init__('object_distance_node')

        self.declare_parameter(
            'input_topic', '/vehicle/perception/tracked_objects'
        )
        self.declare_parameter(
            'output_topic', '/local_trust_estimation/object_distance'
        )
        self.declare_parameter('sensor_frame', 'vehicle_lidar')
        self.declare_parameter('near_range', 30.0)
        self.declare_parameter('half_trust_range', 50.0)
        self.declare_parameter('tf_timeout', 0.1)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.near_range = self.get_parameter('near_range').value
        self.half_trust_range = self.get_parameter(
            'half_trust_range'
        ).value
        tf_timeout = self.get_parameter('tf_timeout').value

        if not self.sensor_frame:
            raise ValueError('sensor_frame must be nonempty')
        if self.near_range < 0.0:
            raise ValueError('near_range must be greater than or equal to 0')
        if self.half_trust_range <= self.near_range:
            raise ValueError(
                'half_trust_range must be greater than near_range'
            )
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
        # transform callbacks serialized behind tracked_objects_callback,
        # which then could never see a transform arrive while it waits.
        # spin_thread=True then spins that dedicated node on its own thread.
        self.tf_node = (
            None if offline
            else rclpy.create_node(f'{self.get_name()}_tf_listener')
        )
        self.tf_listener = (
            None if offline
            else TransformListener(self.tf_buffer, self.tf_node,
                                    spin_thread=True)
        )

        self.subscription = self.create_subscription(
            TrackedObjects,
            input_topic,
            self.tracked_objects_callback,
            10,
        )
        self.publisher_ = self.create_publisher(
            TrustScoreArray, output_topic, 10
        )

        self.get_logger().info(
            f'Scoring {input_topic} relative to {self.sensor_frame} '
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

    def distance_score(self, distance):
        """Return the configured exponential range score."""
        excess = max(0.0, distance - self.near_range)
        denominator = self.half_trust_range - self.near_range
        return math.exp(-math.log(2.0) * excess / denominator)

    def tracked_objects_callback(self, message):
        """Transform object centres to the sensor and publish range scores."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.sensor_frame,
                message.header.frame_id,
                message.header.stamp,
                self.tf_timeout,
            )
        except TransformException as error:
            self.get_logger().warning(
                f'No {self.sensor_frame} <- {message.header.frame_id} '
                f'transform ({error}) - dropping distance scores'
            )
            return

        scored = []
        low_trust = 0
        for tracked_object in message.objects:
            sensor_pose = transform_pose(
                tracked_object.kinematics.pose_with_covariance.pose,
                transform.transform,
            )
            distance = math.hypot(
                sensor_pose.position.x, sensor_pose.position.y
            )
            score = self.distance_score(distance)
            scored.append((tracked_object, score))
            if score < 0.5:
                low_trust += 1
                self.get_logger().debug(
                    f'{class_name(tracked_object)} '
                    f'(track {uuid_text(tracked_object.object_id)}) at '
                    f'{distance:.0f} m -> score {score:.2f}'
                )

        self.publisher_.publish(make_score_array(message, scored))
        self.get_logger().info(
            f'{len(scored)} tracked objects scored, '
            f'{low_trust} beyond half-trust range'
        )


def main(args=None):
    """Run the object-distance node."""
    rclpy.init(args=args)
    node = ObjectDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
