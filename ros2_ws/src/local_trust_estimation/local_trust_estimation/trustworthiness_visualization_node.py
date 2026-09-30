#!/usr/bin/env python3
"""Visualize tracked objects using UUID-keyed local trust scores."""

import zlib

from autoware_perception_msgs.msg import TrackedObjects
from builtin_interfaces.msg import Duration
from local_trust_estimation.tracked_object_utils import (
    class_name,
    score_map,
    uuid_key,
    uuid_text,
)
from message_filters import Subscriber, TimeSynchronizer
import rclpy
from rclpy.node import Node
from sdsm_interfaces.msg import TrustScoreArray
from visualization_msgs.msg import Marker, MarkerArray


RED = (1.0, 0.0, 0.0)
YELLOW = (1.0, 1.0, 0.0)
GREEN = (0.0, 1.0, 0.0)


class TrustworthinessVisualizationNode(Node):
    """Convert tracked boxes and trust scores to cube and text markers."""

    def __init__(self):
        """Initialize visualization parameters, inputs, and publisher."""
        super().__init__('trustworthiness_visualization_node')

        self.declare_parameter(
            'tracked_objects_topic',
            '/vehicle/perception/tracked_objects',
        )
        self.declare_parameter(
            'score_topic', '/local_trust_estimation/score'
        )
        self.declare_parameter(
            'output_topic', '/local_trust_estimation/markers'
        )
        self.declare_parameter('yellow_threshold', 0.5)
        self.declare_parameter('green_threshold', 0.8)
        self.declare_parameter('marker_alpha', 0.45)

        tracked_objects_topic = self.get_parameter(
            'tracked_objects_topic'
        ).value
        score_topic = self.get_parameter('score_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.yellow_threshold = self.get_parameter(
            'yellow_threshold'
        ).value
        self.green_threshold = self.get_parameter(
            'green_threshold'
        ).value
        self.marker_alpha = self.get_parameter('marker_alpha').value

        if not (
            0.0 <= self.yellow_threshold < self.green_threshold <= 1.0
        ):
            raise ValueError(
                'thresholds must satisfy 0 <= yellow < green <= 1'
            )
        if not 0.0 <= self.marker_alpha <= 1.0:
            raise ValueError('marker_alpha must be between 0 and 1')

        self.sync = TimeSynchronizer(
            [
                Subscriber(self, TrackedObjects, tracked_objects_topic),
                Subscriber(self, TrustScoreArray, score_topic),
            ],
            queue_size=10,
        )
        self.sync.registerCallback(self.synced_callback)
        self.publisher_ = self.create_publisher(
            MarkerArray, output_topic, 10
        )

        self.get_logger().info(
            f'Visualizing {tracked_objects_topic} + {score_topic} '
            f'-> {output_topic}; red < {self.yellow_threshold:.2f}, '
            f'yellow < {self.green_threshold:.2f}, green otherwise'
        )

    @staticmethod
    def marker_id(object_id):
        """Return a repeatable positive marker ID for an object UUID."""
        return zlib.crc32(uuid_key(object_id)) & 0x7FFFFFFF

    def score_color(self, score):
        """Return red, yellow, or green for a trustworthiness score."""
        if score < self.yellow_threshold:
            return RED
        if score < self.green_threshold:
            return YELLOW
        return GREEN

    def synced_callback(self, tracked_objects, scores):
        """Publish colored boxes and labels for one tracked-object frame."""
        try:
            scores_by_id = score_map(scores)
        except ValueError as error:
            self.get_logger().error(
                f'Invalid score array ({error}) - dropping markers'
            )
            return

        object_ids = {
            uuid_key(tracked_object.object_id)
            for tracked_object in tracked_objects.objects
        }
        if set(scores_by_id) != object_ids:
            self.get_logger().error(
                'Tracked-object and trust-score UUIDs do not match - '
                'dropping markers'
            )
            return

        markers = MarkerArray()
        delete_all = Marker()
        delete_all.header = tracked_objects.header
        delete_all.ns = 'local_trust_estimation'
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        for tracked_object in tracked_objects.objects:
            key = uuid_key(tracked_object.object_id)
            score = scores_by_id[key]
            marker_id = self.marker_id(tracked_object.object_id)
            red, green, blue = self.score_color(score)
            pose = tracked_object.kinematics.pose_with_covariance.pose
            dimensions = tracked_object.shape.dimensions

            cube = Marker()
            cube.header = tracked_objects.header
            cube.ns = 'local_trust_estimation'
            cube.id = marker_id
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose = pose
            cube.scale = dimensions
            cube.color.r = red
            cube.color.g = green
            cube.color.b = blue
            cube.color.a = self.marker_alpha
            cube.lifetime = Duration(nanosec=300_000_000)
            markers.markers.append(cube)

            text = Marker()
            text.header = tracked_objects.header
            text.ns = 'local_trust_estimation_text'
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pose.position.x
            text.pose.position.y = pose.position.y
            text.pose.position.z = (
                pose.position.z + dimensions.z / 2.0 + 0.5
            )
            text.pose.orientation.w = 1.0
            text.scale.z = 0.7
            text.color.r = red
            text.color.g = green
            text.color.b = blue
            text.color.a = 0.9
            short_id = uuid_text(tracked_object.object_id).split('-', 1)[0]
            text.text = (
                f'{class_name(tracked_object)} {short_id} | '
                f'trust {score:.2f}'
            )
            text.lifetime = cube.lifetime
            markers.markers.append(text)

        self.publisher_.publish(markers)


def main(args=None):
    """Run the trustworthiness-visualization node."""
    rclpy.init(args=args)
    node = TrustworthinessVisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
