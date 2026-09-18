#!/usr/bin/env python3
"""Score track persistence from Autoware tracked-object UUIDs."""

from autoware_perception_msgs.msg import TrackedObjects
from local_trust_estimation.tracked_object_utils import (
    make_score_array,
    uuid_key,
    uuid_text,
)
import rclpy
from rclpy.node import Node
from sdsm_interfaces.msg import TrustScoreArray


class TemporalPresenceNode(Node):
    """Score tracked objects according to consecutive UUID presence."""

    def __init__(self):
        """Initialize parameters, track state, and ROS interfaces."""
        super().__init__('temporal_presence_node')

        self.declare_parameter(
            'input_topic', '/vehicle/perception/tracked_objects'
        )
        self.declare_parameter(
            'output_topic',
            '/local_trust_estimation/temporal_presence',
        )
        self.declare_parameter('k', 15.0)
        self.declare_parameter('min_trust', 0.20)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.k = self.get_parameter('k').value
        self.min_trust = self.get_parameter('min_trust').value

        if self.k <= 0.0:
            raise ValueError('k must be greater than 0')
        if not 0.0 <= self.min_trust <= 1.0:
            raise ValueError('min_trust must be between 0 and 1')

        self.track_ages = {}
        self.subscription = self.create_subscription(
            TrackedObjects,
            input_topic,
            self.tracked_objects_callback,
            10,
        )
        self.publisher_ = self.create_publisher(
            TrustScoreArray,
            output_topic,
            10,
        )

        self.get_logger().info(
            f'Scoring track persistence {input_topic} -> {output_topic} '
            f'with k={self.k} and min_trust={self.min_trust}'
        )

    def presence_score(self, age):
        """Map a positive tracking age to the rational persistence score."""
        growth_factor = age / (age + self.k)
        return self.min_trust + (1.0 - self.min_trust) * growth_factor

    def tracked_objects_callback(self, message):
        """Update consecutive UUID ages and publish per-object scores."""
        previous_ages = self.track_ages
        current_ages = {}
        duplicate_ids = set()
        scored = []

        for tracked_object in message.objects:
            track_id = uuid_key(tracked_object.object_id)
            if track_id not in current_ages:
                current_ages[track_id] = previous_ages.get(track_id, 0) + 1
            else:
                duplicate_ids.add(uuid_text(tracked_object.object_id))
            scored.append((
                tracked_object,
                self.presence_score(current_ages[track_id]),
            ))

        self.track_ages = current_ages
        if duplicate_ids:
            self.get_logger().warning(
                'Duplicate track UUIDs in tracked-object frame: '
                f'{sorted(duplicate_ids)}'
            )
        self.publisher_.publish(make_score_array(message, scored))
        self.get_logger().info(
            f'{len(scored)} tracked objects scored across '
            f'{len(current_ages)} UUIDs'
        )


def main(args=None):
    """Run the temporal-presence node."""
    rclpy.init(args=args)
    node = TemporalPresenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
