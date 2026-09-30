"""Shared helpers for Autoware tracked objects and UUID-keyed trust scores."""

import math

from autoware_perception_msgs.msg import ObjectClassification
from geometry_msgs.msg import Pose
from sdsm_interfaces.msg import TrustScore, TrustScoreArray


CLASS_NAMES = {
    ObjectClassification.UNKNOWN: 'Unknown',
    ObjectClassification.CAR: 'Car',
    ObjectClassification.TRUCK: 'Truck',
    ObjectClassification.BUS: 'Bus',
    ObjectClassification.TRAILER: 'Trailer',
    ObjectClassification.MOTORCYCLE: 'Motorcyclist',
    ObjectClassification.BICYCLE: 'Cyclist',
    ObjectClassification.PEDESTRIAN: 'Pedestrian',
    ObjectClassification.ANIMAL: 'Animal',
    ObjectClassification.HAZARD: 'Hazard',
    ObjectClassification.OVER_DRIVABLE: 'Over-drivable',
    ObjectClassification.UNDER_DRIVABLE: 'Under-drivable',
}


def uuid_key(object_id):
    """Return an immutable key for a unique_identifier_msgs/UUID."""
    return bytes(object_id.uuid)


def uuid_text(object_id):
    """Return the canonical hexadecimal rendering of a UUID."""
    raw = uuid_key(object_id)
    hexadecimal = raw.hex()
    return (
        f'{hexadecimal[:8]}-{hexadecimal[8:12]}-'
        f'{hexadecimal[12:16]}-{hexadecimal[16:20]}-'
        f'{hexadecimal[20:]}'
    )


def primary_classification(tracked_object):
    """Return the most probable classification, or None when unavailable."""
    if not tracked_object.classification:
        return None
    return max(
        tracked_object.classification,
        key=lambda classification: classification.probability,
    )


def class_name(tracked_object):
    """Return the human-readable name of an object's primary class."""
    classification = primary_classification(tracked_object)
    if classification is None:
        return CLASS_NAMES[ObjectClassification.UNKNOWN]
    return CLASS_NAMES.get(classification.label, CLASS_NAMES[0])


def make_score_array(tracked_objects, values):
    """Build a TrustScoreArray from (tracked object, score) pairs."""
    message = TrustScoreArray()
    message.header = tracked_objects.header
    for tracked_object, value in values:
        score = TrustScore()
        score.object_id = tracked_object.object_id
        score.score = min(max(float(value), 0.0), 1.0)
        message.scores.append(score)
    return message


def score_map(message):
    """Return a UUID-keyed score map, rejecting duplicate UUID entries."""
    result = {}
    for entry in message.scores:
        key = uuid_key(entry.object_id)
        if key in result:
            raise ValueError(f'duplicate object UUID {uuid_text(entry.object_id)}')
        result[key] = entry.score
    return result


def quaternion_multiply(left, right):
    """Multiply geometry_msgs-style quaternions and return x, y, z, w."""
    return (
        left.w * right.x + left.x * right.w
        + left.y * right.z - left.z * right.y,
        left.w * right.y - left.x * right.z
        + left.y * right.w + left.z * right.x,
        left.w * right.z + left.x * right.y
        - left.y * right.x + left.z * right.w,
        left.w * right.w - left.x * right.x
        - left.y * right.y - left.z * right.z,
    )


def rotate_vector(rotation, vector):
    """Rotate a three-dimensional vector by a quaternion."""
    x, y, z = vector
    cross_x = 2.0 * (rotation.y * z - rotation.z * y)
    cross_y = 2.0 * (rotation.z * x - rotation.x * z)
    cross_z = 2.0 * (rotation.x * y - rotation.y * x)
    return (
        x + rotation.w * cross_x
        + rotation.y * cross_z - rotation.z * cross_y,
        y + rotation.w * cross_y
        + rotation.z * cross_x - rotation.x * cross_z,
        z + rotation.w * cross_z
        + rotation.x * cross_y - rotation.y * cross_x,
    )


def transform_pose(pose, transform):
    """Apply a geometry_msgs Transform to a Pose."""
    result = Pose()
    rotated = rotate_vector(
        transform.rotation,
        (pose.position.x, pose.position.y, pose.position.z),
    )
    result.position.x = rotated[0] + transform.translation.x
    result.position.y = rotated[1] + transform.translation.y
    result.position.z = rotated[2] + transform.translation.z
    orientation = quaternion_multiply(transform.rotation, pose.orientation)
    (
        result.orientation.x,
        result.orientation.y,
        result.orientation.z,
        result.orientation.w,
    ) = orientation
    return result


def normalized_rotation_matrix(rotation):
    """Return a normalized 3x3 rotation matrix as a tuple of rows."""
    norm = math.sqrt(
        rotation.x * rotation.x
        + rotation.y * rotation.y
        + rotation.z * rotation.z
        + rotation.w * rotation.w
    )
    if norm < 1e-12:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    x = rotation.x / norm
    y = rotation.y / norm
    z = rotation.z / norm
    w = rotation.w / norm
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )
