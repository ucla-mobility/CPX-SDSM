#!/usr/bin/env python3
"""Combine UUID-keyed local trustworthiness criteria."""

from local_trust_estimation.tracked_object_utils import score_map, uuid_key
from message_filters import Subscriber, TimeSynchronizer
import rclpy
from rclpy.node import Node
from sdsm_interfaces.msg import TrustScore, TrustScoreArray


class TrustworthinessScoreNode(Node):
    """Publish the weighted average of the criterion scores."""

    def __init__(self):
        """Initialize synchronized criterion inputs and score output."""
        super().__init__('trustworthiness_score_node')

        self.declare_parameter(
            'object_shape_topic', '/local_trust_estimation/object_shape'
        )
        self.declare_parameter(
            'lidar_point_count_topic',
            '/local_trust_estimation/lidar_point_count',
        )
        self.declare_parameter(
            'object_distance_topic', '/local_trust_estimation/object_distance'
        )
        self.declare_parameter(
            'temporal_presence_topic',
            '/local_trust_estimation/temporal_presence',
        )
        self.declare_parameter('output_topic', '/local_trust_estimation/score')
        self.declare_parameter('object_shape_weight', 0.25)
        self.declare_parameter('lidar_point_count_weight', 0.25)
        self.declare_parameter('object_distance_weight', 0.25)
        self.declare_parameter('temporal_presence_weight', 0.25)

        shape_topic = self.get_parameter('object_shape_topic').value
        point_count_topic = self.get_parameter(
            'lidar_point_count_topic'
        ).value
        distance_topic = self.get_parameter('object_distance_topic').value
        temporal_topic = self.get_parameter(
            'temporal_presence_topic'
        ).value
        output_topic = self.get_parameter('output_topic').value
        self.weights = (
            self.get_parameter('object_shape_weight').value,
            self.get_parameter('lidar_point_count_weight').value,
            self.get_parameter('object_distance_weight').value,
            self.get_parameter('temporal_presence_weight').value,
        )
        if any(weight < 0.0 for weight in self.weights):
            raise ValueError('trustworthiness weights must be nonnegative')
        self.weight_sum = sum(self.weights)
        if self.weight_sum <= 0.0:
            raise ValueError(
                'at least one trustworthiness weight must be greater than 0'
            )

        self.sync = TimeSynchronizer(
            [
                Subscriber(self, TrustScoreArray, shape_topic),
                Subscriber(self, TrustScoreArray, point_count_topic),
                Subscriber(self, TrustScoreArray, distance_topic),
                Subscriber(self, TrustScoreArray, temporal_topic),
            ],
            queue_size=10,
        )
        self.sync.registerCallback(self.synced_callback)
        self.publisher_ = self.create_publisher(
            TrustScoreArray, output_topic, 10
        )

        self.get_logger().info(
            f'Combining {shape_topic}, {point_count_topic}, '
            f'{distance_topic}, and {temporal_topic} -> {output_topic} '
            f'with weights {self.weights}'
        )

    def weighted_average(self, scores):
        """Return the normalized weighted average of the criterion scores."""
        return sum(
            weight * score for weight, score in zip(self.weights, scores)
        ) / self.weight_sum

    def synced_callback(
        self,
        shape_message,
        point_count_message,
        distance_message,
        temporal_message,
    ):
        """Combine one synchronized criterion set by object UUID."""
        messages = (
            shape_message,
            point_count_message,
            distance_message,
            temporal_message,
        )
        frame_ids = {message.header.frame_id for message in messages}
        if len(frame_ids) != 1:
            self.get_logger().error(
                f'Criterion frame mismatch ({sorted(frame_ids)}) - '
                'dropping score batch'
            )
            return

        try:
            mappings = [score_map(message) for message in messages]
        except ValueError as error:
            self.get_logger().error(
                f'Invalid criterion score array ({error}) - '
                'dropping score batch'
            )
            return

        expected_ids = set(mappings[0])
        if any(set(mapping) != expected_ids for mapping in mappings[1:]):
            self.get_logger().error(
                'Criterion object UUIDs do not match - dropping score batch'
            )
            return

        result = TrustScoreArray()
        result.header = shape_message.header
        for shape_score in shape_message.scores:
            key = uuid_key(shape_score.object_id)
            combined = TrustScore()
            combined.object_id = shape_score.object_id
            combined.score = self.weighted_average([
                mapping[key] for mapping in mappings
            ])
            result.scores.append(combined)

        self.publisher_.publish(result)
        self.get_logger().info(
            f'Published combined scores for {len(result.scores)} '
            'tracked objects'
        )


def main(args=None):
    """Run the trustworthiness-score node."""
    rclpy.init(args=args)
    node = TrustworthinessScoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
