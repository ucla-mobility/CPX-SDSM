#!/usr/bin/env python3
"""Bake local trust topics from tracked objects into a lossless bag copy."""

import argparse
from copy import deepcopy
from pathlib import Path
import shutil
import sys

from autoware_perception_msgs.msg import TrackedObjects
from local_trust_estimation.lidar_point_count_node import LidarPointCountNode
from local_trust_estimation.object_distance_node import ObjectDistanceNode
from local_trust_estimation.object_shape_node import ObjectShapeNode
from local_trust_estimation.sdsm_publisher_node import SdsmPublisherNode
from local_trust_estimation.temporal_presence_node import (
    TemporalPresenceNode,
)
from local_trust_estimation.trustworthiness_score_node import (
    TrustworthinessScoreNode,
)
from local_trust_estimation.trustworthiness_visualization_node import (
    TrustworthinessVisualizationNode,
)
import rclpy
import rclpy.logging
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


DEFAULT_CLOUD_TOPIC = '/vehicle/lidar/points'
DEFAULT_TRACKED_OBJECTS_TOPIC = '/vehicle/perception/tracked_objects'
TF_TOPIC = '/tf'
TF_STATIC_TOPIC = '/tf_static'

SCORE_TYPE = 'sdsm_interfaces/msg/TrustScoreArray'
MARKER_TYPE = 'visualization_msgs/msg/MarkerArray'
SDSM_TYPE = 'sdsm_interfaces/msg/SdsmPayload'
GENERATED_TOPICS = [
    ('/local_trust_estimation/object_shape', SCORE_TYPE),
    ('/local_trust_estimation/lidar_point_count', SCORE_TYPE),
    ('/local_trust_estimation/object_distance', SCORE_TYPE),
    ('/local_trust_estimation/temporal_presence', SCORE_TYPE),
    ('/local_trust_estimation/score', SCORE_TYPE),
    ('/local_trust_estimation/markers', MARKER_TYPE),
    ('/perception/global_trustworthiness/sdsm', SDSM_TYPE),
]


class Pipeline:
    """Run the live node logic through direct callbacks for offline baking."""

    def __init__(self):
        """Instantiate nodes and redirect every output to local capture."""
        self.shape = ObjectShapeNode()
        self.count = LidarPointCountNode(offline=True)
        self.distance = ObjectDistanceNode(offline=True)
        self.temporal = TemporalPresenceNode()
        self.score = TrustworthinessScoreNode()
        self.visualization = TrustworthinessVisualizationNode()
        self.sdsm = SdsmPublisherNode(offline=True)
        self.nodes = [
            self.shape,
            self.count,
            self.distance,
            self.temporal,
            self.score,
            self.visualization,
            self.sdsm,
        ]
        self.tf_nodes = (self.count, self.distance, self.sdsm)

        self.captured = {}
        for key, node in (
            ('shape', self.shape),
            ('point_count', self.count),
            ('distance', self.distance),
            ('temporal', self.temporal),
            ('score', self.score),
            ('markers', self.visualization),
            ('sdsm', self.sdsm),
        ):
            node.publisher_.publish = self._capture(key)
            rclpy.logging.set_logger_level(
                node.get_logger().name,
                rclpy.logging.LoggingSeverity.WARN,
            )

    def _capture(self, key):
        """Return a publish replacement that stores one message by key."""
        return lambda message: self.captured.__setitem__(key, message)

    def load_transforms(self, bag_directory):
        """Preload bag transforms into every TF-dependent factor."""
        reader = _open_reader(bag_directory)
        reader.set_filter(rosbag2_py.StorageFilter(
            topics=[TF_TOPIC, TF_STATIC_TOPIC]
        ))
        while reader.has_next():
            topic, data, _ = reader.read_next()
            message = deserialize_message(data, TFMessage)
            for transform in message.transforms:
                for node in self.tf_nodes:
                    if topic == TF_STATIC_TOPIC:
                        node.tf_buffer.set_transform_static(
                            transform, 'evaluate_offline'
                        )
                    else:
                        node.tf_buffer.set_transform(
                            transform, 'evaluate_offline'
                        )

    def run(self, cloud, tracked_objects):
        """Score one synchronized cloud and tracked-object frame."""
        for key in (
            'shape',
            'point_count',
            'distance',
            'temporal',
            'score',
            'markers',
            'sdsm',
        ):
            self.captured.pop(key, None)

        self.shape.tracked_objects_callback(deepcopy(tracked_objects))
        self.distance.tracked_objects_callback(deepcopy(tracked_objects))
        self.temporal.tracked_objects_callback(deepcopy(tracked_objects))
        self.count.synced_callback(cloud, deepcopy(tracked_objects))

        factor_keys = ('shape', 'point_count', 'distance', 'temporal')
        if any(key not in self.captured for key in factor_keys):
            return None
        shape = self.captured['shape']
        point_count = self.captured['point_count']
        distance = self.captured['distance']
        temporal = self.captured['temporal']
        self.score.synced_callback(
            shape,
            point_count,
            distance,
            temporal,
        )

        combined = self.captured['score']
        self.visualization.synced_callback(tracked_objects, combined)
        self.sdsm.synced_callback(tracked_objects, combined)
        return (
            shape,
            point_count,
            distance,
            temporal,
            combined,
            self.captured['markers'],
            self.captured.get('sdsm'),
        )

    def destroy(self):
        """Destroy the underlying ROS nodes."""
        for node in self.nodes:
            node.destroy_node()


def _open_reader(bag_directory):
    """Open an MCAP bag for sequential reading."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=bag_directory, storage_id='mcap'
        ),
        rosbag2_py.ConverterOptions('', ''),
    )
    return reader


def _open_writer(bag_directory, source_topics):
    """Open an MCAP writer and create source plus generated topics."""
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=bag_directory,
            storage_id='mcap',
            storage_preset_profile='zstd_fast',
        ),
        rosbag2_py.ConverterOptions('', ''),
    )
    for topic in source_topics:
        writer.create_topic(topic)
    next_id = max((topic.id for topic in source_topics), default=-1) + 1
    for offset, (name, message_type) in enumerate(GENERATED_TOPICS):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=next_id + offset,
            name=name,
            type=message_type,
            serialization_format='cdr',
            offered_qos_profiles=[],
        ))
    return writer


def _stamp_key(header):
    """Return a hashable message-header timestamp."""
    return header.stamp.sec, header.stamp.nanosec


def bake(
    input_bag,
    output_bag,
    cloud_topic=DEFAULT_CLOUD_TOPIC,
    tracked_objects_topic=DEFAULT_TRACKED_OBJECTS_TOPIC,
):
    """Copy a bag and append trust outputs for every synchronized frame."""
    reader = _open_reader(input_bag)
    writer = _open_writer(output_bag, reader.get_all_topics_and_types())

    pipeline = Pipeline()
    pipeline.load_transforms(input_bag)
    pending_clouds = {}
    pending_objects = {}
    frames = 0

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        writer.write(topic, data, timestamp_ns)

        key = None
        if topic == cloud_topic:
            cloud = deserialize_message(data, PointCloud2)
            key = _stamp_key(cloud.header)
            pending_clouds[key] = (cloud, timestamp_ns)
        elif topic == tracked_objects_topic:
            tracked_objects = deserialize_message(data, TrackedObjects)
            key = _stamp_key(tracked_objects.header)
            pending_objects[key] = tracked_objects

        if (
            key is not None
            and key in pending_clouds
            and key in pending_objects
        ):
            cloud, frame_timestamp = pending_clouds.pop(key)
            tracked_objects = pending_objects.pop(key)
            outputs = pipeline.run(cloud, tracked_objects)
            if outputs is None:
                continue
            for (name, _), message in zip(GENERATED_TOPICS, outputs):
                if message is not None:
                    writer.write(
                        name,
                        serialize_message(message),
                        frame_timestamp,
                    )
            frames += 1
            if frames % 50 == 0:
                print(f'  {frames} frames scored', flush=True)

    pipeline.destroy()
    del writer
    unpaired = len(pending_clouds) + len(pending_objects)
    print(
        f'wrote {frames} scored frames -> {output_bag}'
        + (
            f' ({unpaired} unpaired source messages skipped)'
            if unpaired else ''
        )
    )
    if frames == 0:
        sys.exit(
            f'error: nothing was scored in {input_bag}. Every frame needs '
            f'{cloud_topic} and {tracked_objects_topic} with one timestamp.'
        )


def main(argv=None):
    """Parse arguments and run the offline bake."""
    parser = argparse.ArgumentParser(
        description=__doc__.strip().splitlines()[0]
    )
    parser.add_argument(
        '--input', type=Path, required=True, help='source bag directory'
    )
    parser.add_argument(
        '--output',
        type=Path,
        required=True,
        help='new bag directory (must not already exist)',
    )
    parser.add_argument(
        '--cloud-topic',
        default=DEFAULT_CLOUD_TOPIC,
        help='PointCloud2 topic to score against',
    )
    parser.add_argument(
        '--tracked-objects-topic',
        default=DEFAULT_TRACKED_OBJECTS_TOPIC,
        help='Autoware TrackedObjects input topic',
    )
    parser.add_argument(
        '--force',
        action='store_true',
        help='overwrite the output bag if it exists',
    )
    arguments = parser.parse_args(argv)

    if not arguments.input.exists():
        sys.exit(f'error: input bag not found: {arguments.input}')
    if arguments.output.exists():
        if not arguments.force:
            sys.exit(
                f'error: {arguments.output} already exists (use --force)'
            )
        shutil.rmtree(arguments.output)
    if not arguments.output.parent.is_dir():
        sys.exit(
            f'error: output parent does not exist: '
            f'{arguments.output.parent}'
        )

    rclpy.init()
    try:
        bake(
            str(arguments.input),
            str(arguments.output),
            arguments.cloud_topic,
            arguments.tracked_objects_topic,
        )
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
