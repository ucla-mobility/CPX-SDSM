"""Behavioral tests for the TrackedObjects local-trust contract."""

from autoware_perception_msgs.msg import (
    ObjectClassification,
    TrackedObject,
    TrackedObjects,
)
from geometry_msgs.msg import TransformStamped
from local_trust_estimation.evaluate_offline import bake
from local_trust_estimation.lidar_point_count_node import LidarPointCountNode
from local_trust_estimation.object_distance_node import ObjectDistanceNode
from local_trust_estimation.object_shape_node import ObjectShapeNode
from local_trust_estimation.sdsm_publisher_node import SdsmPublisherNode
from local_trust_estimation.temporal_presence_node import (
    TemporalPresenceNode,
)
from local_trust_estimation.tracked_object_utils import make_score_array
from local_trust_estimation.trustworthiness_score_node import (
    TrustworthinessScoreNode,
)
import pytest
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from sdsm_interfaces.msg import TrustScoreArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Initialize one ROS context for all node-level tests."""
    rclpy.init()
    yield
    rclpy.shutdown()


def tracked_frame(x=0.0, y=0.0, label=ObjectClassification.CAR):
    """Return a one-object map-frame tracker message."""
    message = TrackedObjects()
    message.header.frame_id = 'map'
    message.header.stamp.sec = 10
    tracked_object = TrackedObject()
    tracked_object.object_id.uuid = list(range(16))
    tracked_object.existence_probability = 0.8
    classification = ObjectClassification()
    classification.label = label
    classification.probability = 0.7
    tracked_object.classification.append(classification)
    pose = tracked_object.kinematics.pose_with_covariance.pose
    pose.position.x = x
    pose.position.y = y
    pose.orientation.w = 1.0
    tracked_object.shape.dimensions.x = 4.34
    tracked_object.shape.dimensions.y = 1.92
    tracked_object.shape.dimensions.z = 1.58
    message.objects.append(tracked_object)
    return message


def capture(node):
    """Replace a node publisher with a list-backed capture callback."""
    messages = []
    node.publisher_.publish = messages.append
    return messages


def install_sensor_transform(node, x=10.0, y=20.0):
    """Install a static map <- vehicle_lidar transform."""
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'vehicle_lidar'
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.rotation.w = 1.0
    node.tf_buffer.set_transform_static(transform, 'test')


def test_shape_uses_autoware_class_without_mutating_tracker_confidence():
    """Shape output is UUID-keyed and leaves Autoware confidence untouched."""
    node = ObjectShapeNode()
    messages = capture(node)
    tracked_objects = tracked_frame()

    node.tracked_objects_callback(tracked_objects)

    assert messages[0].scores[0].score == pytest.approx(1.0)
    assert tracked_objects.objects[0].existence_probability == pytest.approx(
        0.8
    )
    assert tracked_objects.objects[0].classification[0].probability == (
        pytest.approx(0.7)
    )
    node.destroy_node()


def test_temporal_presence_follows_the_full_tracker_uuid():
    """Temporal trust increases for a UUID present in consecutive frames."""
    node = TemporalPresenceNode()
    messages = capture(node)
    first = tracked_frame()
    second = tracked_frame(x=1.0)

    node.tracked_objects_callback(first)
    node.tracked_objects_callback(second)

    assert messages[1].scores[0].object_id == second.objects[0].object_id
    assert messages[1].scores[0].score > messages[0].scores[0].score
    node.destroy_node()


def test_point_count_transforms_map_boxes_into_the_cloud_frame():
    """Point support uses map-to-lidar TF rather than frame coincidence."""
    node = LidarPointCountNode(offline=True)
    install_sensor_transform(node)
    messages = capture(node)
    tracked_objects = tracked_frame(x=10.0, y=20.0)
    cloud_header = Header()
    cloud_header.frame_id = 'vehicle_lidar'
    cloud_header.stamp = tracked_objects.header.stamp
    cloud = create_cloud_xyz32(
        cloud_header,
        [(0.0, 0.0, 0.0), (0.5, 0.5, 0.5), (10.0, 0.0, 0.0)],
    )

    node.synced_callback(cloud, tracked_objects)

    assert isinstance(cloud, PointCloud2)
    assert messages[0].scores[0].score == pytest.approx(
        node.count_score(2)
    )
    node.destroy_node()


def test_distance_is_relative_to_the_tf_sensor_pose():
    """Map coordinates are scored relative to the lidar, not map origin."""
    node = ObjectDistanceNode(offline=True)
    install_sensor_transform(node)
    messages = capture(node)
    tracked_objects = tracked_frame(x=70.0, y=20.0)

    node.tracked_objects_callback(tracked_objects)

    assert messages[0].scores[0].score == pytest.approx(
        node.distance_score(60.0)
    )
    node.destroy_node()


def test_combiner_joins_reordered_factor_arrays_by_uuid():
    """Combined scores follow UUIDs even if one factor changes array order."""
    node = TrustworthinessScoreNode()
    messages = capture(node)
    tracked_objects = tracked_frame()
    second_object = TrackedObject()
    second_object.object_id.uuid = list(reversed(range(16)))
    tracked_objects.objects.append(second_object)

    shape = make_score_array(
        tracked_objects,
        [(tracked_objects.objects[0], 0.2), (second_object, 0.8)],
    )
    reordered = make_score_array(
        tracked_objects,
        [(second_object, 0.4), (tracked_objects.objects[0], 1.0)],
    )
    node.synced_callback(shape, reordered, reordered, reordered)

    assert isinstance(messages[0], TrustScoreArray)
    assert messages[0].scores[0].score == pytest.approx(0.8)
    assert messages[0].scores[1].score == pytest.approx(0.5)
    node.destroy_node()


def test_sdsm_uses_uuid_classification_pose_and_tracked_twist():
    """SDSM fields come directly from the canonical tracked object."""
    node = SdsmPublisherNode(offline=True)
    install_sensor_transform(node)
    messages = capture(node)
    tracked_objects = tracked_frame(x=15.0, y=18.0)
    linear = (
        tracked_objects.objects[0]
        .kinematics.twist_with_covariance.twist.linear
    )
    linear.x = 3.0
    linear.y = 4.0
    scores = make_score_array(
        tracked_objects, [(tracked_objects.objects[0], 0.75)]
    )

    node.synced_callback(tracked_objects, scores)

    payload = messages[0]
    assert payload.ref_pos_x == pytest.approx(10.0)
    assert payload.ref_pos_y == pytest.approx(20.0)
    assert payload.offset_x[0] == 50
    assert payload.offset_y[0] == -20
    assert payload.obj_type[0] == 1
    assert payload.obj_speed[0] == 250
    assert payload.obj_local_scores[0] == pytest.approx(0.75)
    node.destroy_node()


def create_input_bag(path):
    """Create a one-frame PointCloud2 + TrackedObjects + TF MCAP bag."""
    tracked_objects = tracked_frame(x=10.0, y=20.0)
    cloud_header = Header()
    cloud_header.frame_id = 'vehicle_lidar'
    cloud_header.stamp = tracked_objects.header.stamp
    cloud = create_cloud_xyz32(cloud_header, [(0.0, 0.0, 0.0)])

    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'vehicle_lidar'
    transform.transform.translation.x = 10.0
    transform.transform.translation.y = 20.0
    transform.transform.rotation.w = 1.0
    transforms = TFMessage(transforms=[transform])

    topics = (
        (
            '/vehicle/lidar/points',
            'sensor_msgs/msg/PointCloud2',
            cloud,
        ),
        (
            '/vehicle/perception/tracked_objects',
            'autoware_perception_msgs/msg/TrackedObjects',
            tracked_objects,
        ),
        ('/tf_static', 'tf2_msgs/msg/TFMessage', transforms),
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    for identifier, (name, message_type, message) in enumerate(topics):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=identifier,
            name=name,
            type=message_type,
            serialization_format='cdr',
            offered_qos_profiles=[],
        ))
        writer.write(name, serialize_message(message), 10_000_000_000)
    del writer


def test_offline_evaluator_consumes_tracked_objects_without_trajectory(
    tmp_path,
):
    """Offline baking needs only canonical tracks, lidar, and TF."""
    input_bag = tmp_path / 'input'
    output_bag = tmp_path / 'output'
    create_input_bag(input_bag)

    bake(str(input_bag), str(output_bag))

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(output_bag), storage_id='mcap'
        ),
        rosbag2_py.ConverterOptions('', ''),
    )
    generated_scores = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/local_trust_estimation/score':
            generated_scores.append(
                deserialize_message(data, TrustScoreArray)
            )
    assert len(generated_scores) == 1
    assert len(generated_scores[0].scores) == 1
