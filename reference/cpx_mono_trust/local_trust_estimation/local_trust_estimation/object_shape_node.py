#!/usr/bin/env python3
"""Score the shape plausibility of Autoware tracked objects."""

import math

from autoware_perception_msgs.msg import TrackedObjects
from local_trust_estimation.tracked_object_utils import (
    class_name,
    make_score_array,
    uuid_text,
)
import rclpy
from rclpy.node import Node
from sdsm_interfaces.msg import TrustScoreArray


# Per-class ((mean, std) for length, width, height) in metres, derived from
# the V2X-Seq-SPD lidar labels (vehicle + infrastructure side, every 10th
# frame).
CLASS_SHAPE_PRIORS = {
    'Car': ((4.34, 0.28), (1.92, 0.14), (1.58, 0.15)),
    'Motorcyclist': ((1.82, 0.25), (0.74, 0.14), (1.51, 0.17)),
    'Cyclist': ((1.68, 0.24), (0.66, 0.14), (1.51, 0.24)),
    'Pedestrian': ((0.59, 0.31), (0.59, 0.15), (1.64, 0.16)),
    'Truck': ((8.43, 3.78), (2.73, 0.56), (3.12, 0.70)),
    'Bus': ((10.69, 1.92), (2.88, 0.36), (3.19, 0.40)),
}
# Avoid division by near-zero standard deviations from small samples.
MIN_STD = 0.15


class ObjectShapeNode(Node):
    """Publish UUID-keyed shape-plausibility scores."""

    def __init__(self):
        """Initialize parameters and ROS interfaces."""
        super().__init__('object_shape_node')

        self.declare_parameter(
            'input_topic', '/vehicle/perception/tracked_objects'
        )
        self.declare_parameter(
            'output_topic', '/local_trust_estimation/object_shape'
        )
        self.declare_parameter('tolerance_sigmas', 2.0)
        self.declare_parameter('unknown_class_score', 0.5)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.tolerance = self.get_parameter('tolerance_sigmas').value
        self.unknown_class_score = self.get_parameter(
            'unknown_class_score'
        ).value

        if self.tolerance < 0.0:
            raise ValueError('tolerance_sigmas must be nonnegative')
        if not 0.0 <= self.unknown_class_score <= 1.0:
            raise ValueError('unknown_class_score must be between 0 and 1')

        self.subscription = self.create_subscription(
            TrackedObjects, input_topic, self.tracked_objects_callback, 10
        )
        self.publisher_ = self.create_publisher(
            TrustScoreArray, output_topic, 10
        )

        self.get_logger().info(f'Scoring {input_topic} -> {output_topic}')

    def shape_score(
        self, object_class, length, width, height
    ):
        """Return the shape-plausibility score for one tracked object."""
        prior = CLASS_SHAPE_PRIORS.get(object_class)
        if prior is None:
            return self.unknown_class_score
        score = 1.0
        for value, (mean, standard_deviation) in zip(
            (length, width, height), prior
        ):
            z_score = abs(value - mean) / max(standard_deviation, MIN_STD)
            excess = max(0.0, z_score - self.tolerance)
            score *= math.exp(-0.5 * excess * excess)
        return score

    def tracked_objects_callback(self, message):
        """Score every tracked object without modifying tracker fields."""
        scored = []
        flagged = 0
        for tracked_object in message.objects:
            dimensions = tracked_object.shape.dimensions
            object_class = class_name(tracked_object)
            score = self.shape_score(
                object_class,
                dimensions.x,
                dimensions.y,
                dimensions.z,
            )
            scored.append((tracked_object, score))
            if score < 0.5:
                flagged += 1
                self.get_logger().warning(
                    f'Implausible shape for {object_class} '
                    f'(track {uuid_text(tracked_object.object_id)}): '
                    f'{dimensions.x:.2f} x {dimensions.y:.2f} x '
                    f'{dimensions.z:.2f} m -> score {score:.2f}'
                )

        self.publisher_.publish(make_score_array(message, scored))
        self.get_logger().info(
            f'{len(scored)} tracked objects scored, {flagged} flagged'
        )


def main(args=None):
    """Run the object-shape node."""
    rclpy.init(args=args)
    node = ObjectShapeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
