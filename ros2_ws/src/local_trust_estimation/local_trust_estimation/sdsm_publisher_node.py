#!/usr/bin/env python3
"""Publish tracked objects and local trust scores as an SDSM payload."""

import math
import time
import zlib

from autoware_perception_msgs.msg import (
    ObjectClassification,
    TrackedObjects,
)
from local_trust_estimation.tracked_object_utils import (
    primary_classification,
    rotate_vector,
    score_map,
    transform_pose,
    uuid_key,
)
from message_filters import Subscriber, TimeSynchronizer
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sdsm_interfaces.msg import SdsmPayload, TrustScoreArray
from tf2_ros import Buffer, TransformException, TransformListener


# Fixed J2735/J3224 wire units.
OFFSET_UNIT_M = 0.1
SPEED_UNIT_MS = 0.02
SPEED_MAX = 8190
SPEED_UNAVAILABLE = 8191
HEADING_UNIT_DEG = 0.0125
HEADING_UNAVAILABLE = 28800

MSG_COUNT_MODULO = 128
SECONDS_PER_DAY = 86400

OBJECT_TYPE_UNKNOWN = 0
OBJECT_TYPE_VEHICLE = 1
OBJECT_TYPE_VRU = 2
CLASS_OBJECT_TYPES = {
    ObjectClassification.CAR: OBJECT_TYPE_VEHICLE,
    ObjectClassification.TRUCK: OBJECT_TYPE_VEHICLE,
    ObjectClassification.BUS: OBJECT_TYPE_VEHICLE,
    ObjectClassification.TRAILER: OBJECT_TYPE_VEHICLE,
    ObjectClassification.MOTORCYCLE: OBJECT_TYPE_VRU,
    ObjectClassification.BICYCLE: OBJECT_TYPE_VRU,
    ObjectClassification.PEDESTRIAN: OBJECT_TYPE_VRU,
}

LIVE_TF_CACHE_SECONDS = 10.0
OFFLINE_TF_CACHE_SECONDS = float(SECONDS_PER_DAY)


class SdsmPublisherNode(Node):
    """Encode tracked objects and UUID-keyed scores for the global stage."""

    def __init__(self, offline=False):
        """Initialize parameters, TF access, synchronized inputs, and output."""
        super().__init__('sdsm_publisher_node')

        self.declare_parameter(
            'tracked_objects_topic',
            '/vehicle/perception/tracked_objects',
        )
        self.declare_parameter(
            'score_topic', '/local_trust_estimation/score'
        )
        self.declare_parameter(
            'output_topic', '/perception/global_trustworthiness/sdsm'
        )
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('sensor_frame', 'vehicle_lidar')
        self.declare_parameter('agent_id', 0)
        self.declare_parameter('equipment_type', 2)
        self.declare_parameter('tf_timeout', 0.1)

        tracked_objects_topic = self.get_parameter(
            'tracked_objects_topic'
        ).value
        score_topic = self.get_parameter('score_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.global_frame = self.get_parameter('global_frame').value
        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.agent_id = self.get_parameter('agent_id').value
        self.equipment_type = self.get_parameter('equipment_type').value
        tf_timeout = self.get_parameter('tf_timeout').value

        if not self.global_frame:
            raise ValueError('global_frame must be nonempty')
        if not self.sensor_frame:
            raise ValueError('sensor_frame must be nonempty')
        if not 0 <= self.equipment_type <= 3:
            raise ValueError('equipment_type must be between 0 and 3')
        if self.agent_id < 0:
            raise ValueError('agent_id must be nonnegative')
        if tf_timeout < 0.0:
            raise ValueError('tf_timeout must be nonnegative')

        self.offline = offline
        self.capacity = len(SdsmPayload().obj_type)
        self.msg_cnt = 0

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
                Subscriber(self, TrackedObjects, tracked_objects_topic),
                Subscriber(self, TrustScoreArray, score_topic),
            ],
            queue_size=10,
        )
        self.sync.registerCallback(self.synced_callback)
        self.publisher_ = self.create_publisher(
            SdsmPayload, output_topic, 10
        )

        self.get_logger().info(
            f'Publishing SDSM {tracked_objects_topic} + {score_topic} '
            f'-> {output_topic} as agent {self.agent_id} '
            f'(equipment type {self.equipment_type})'
        )

    @staticmethod
    def object_id(object_uuid):
        """Return a deterministic positive SDSM object ID for a UUID."""
        return zlib.crc32(uuid_key(object_uuid)) & 0x7FFFFFFF

    def heading_units(self, box_rotation):
        """Return a box's J2735 heading, clockwise from ENU north."""
        forward = rotate_vector(box_rotation, (1.0, 0.0, 0.0))
        if math.hypot(forward[0], forward[1]) < 1e-9:
            return HEADING_UNAVAILABLE

        yaw = math.degrees(math.atan2(forward[1], forward[0]))
        units = round(((90.0 - yaw) % 360.0) / HEADING_UNIT_DEG)
        return 0 if units >= HEADING_UNAVAILABLE else units

    @staticmethod
    def speed_units(tracked_object):
        """Return J2735 speed from the tracked object's direct twist."""
        linear = (
            tracked_object.kinematics.twist_with_covariance.twist.linear
        )
        speed = math.hypot(linear.x, linear.y)
        if not math.isfinite(speed):
            return SPEED_UNAVAILABLE
        return min(round(speed / SPEED_UNIT_MS), SPEED_MAX)

    @staticmethod
    def object_type(tracked_object):
        """Map the primary Autoware class enum directly to J3224 type."""
        classification = primary_classification(tracked_object)
        if classification is None:
            return OBJECT_TYPE_UNKNOWN
        return CLASS_OBJECT_TYPES.get(
            classification.label, OBJECT_TYPE_UNKNOWN
        )

    def capture_lag_ms(self, header) -> int:
        """Milliseconds between this frame's capture and now, for the receiver
        to recover which instant it describes. Measured on the node clock
        against the source stamp, so under use_sim_time it stays in scene time
        (a wall-clock lag would scale with playback rate). Clamped to the
        uint16 wire field."""
        stamp_s = header.stamp.sec + header.stamp.nanosec / 1e9
        lag_s = self.get_clock().now().nanoseconds * 1e-9 - stamp_s
        return max(0, min(round(lag_s * 1000.0), 65535))

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

    def timestamp(self, header):
        """Return year, month, day, and milliseconds since UTC midnight."""
        seconds = (
            header.stamp.sec + header.stamp.nanosec / 1e9
            if self.offline
            else time.time()
        )
        utc = time.gmtime(seconds)
        return (
            utc.tm_year,
            utc.tm_mon,
            utc.tm_mday,
            int((seconds % SECONDS_PER_DAY) * 1000.0),
        )

    def _lookup_transforms(self, tracked_objects):
        sensor_transform = self.tf_buffer.lookup_transform(
            self.global_frame,
            self.sensor_frame,
            tracked_objects.header.stamp,
            self.tf_timeout,
        )
        object_transform = None
        if tracked_objects.header.frame_id != self.global_frame:
            object_transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                tracked_objects.header.frame_id,
                tracked_objects.header.stamp,
                self.tf_timeout,
            )
        return sensor_transform, object_transform

    def synced_callback(self, tracked_objects, scores):
        """Encode and publish one tracked-object frame as an SDSM."""
        try:
            scores_by_id = score_map(scores)
        except ValueError as error:
            self.get_logger().error(
                f'Invalid score array ({error}) - dropping SDSM'
            )
            return

        object_ids = [
            uuid_key(tracked_object.object_id)
            for tracked_object in tracked_objects.objects
        ]
        if len(set(object_ids)) != len(object_ids):
            self.get_logger().error(
                'Duplicate tracked-object UUIDs - dropping SDSM'
            )
            return
        if set(scores_by_id) != set(object_ids):
            self.get_logger().error(
                'Tracked-object and trust-score UUIDs do not match - '
                'dropping SDSM'
            )
            return

        try:
            sensor_transform, object_transform = self._lookup_transforms(
                tracked_objects
            )
        except TransformException as error:
            self.get_logger().warning(
                f'Cannot resolve {self.sensor_frame} and '
                f'{tracked_objects.header.frame_id} in {self.global_frame} '
                f'({error}) - dropping SDSM'
            )
            return

        payload = SdsmPayload()
        payload.msg_cnt = self.msg_cnt
        payload.source_id = [self.agent_id, 0, 0, 0]
        payload.equipment_type = self.equipment_type
        (
            payload.sdsm_year,
            payload.sdsm_month,
            payload.sdsm_day,
            payload.sdsm_time_of_day_ms,
        ) = self.timestamp(tracked_objects.header)

        # One lag for the whole message: all objects share the synchronized
        # sensor frame, so the per-object J3224 field repeats the same value.
        lag_ms = self.capture_lag_ms(tracked_objects.header)

        sensor_origin = sensor_transform.transform.translation
        payload.ref_pos_x = sensor_origin.x
        payload.ref_pos_y = sensor_origin.y
        payload.ref_pos_z = sensor_origin.z

        local_scores = []
        for tracked_object in tracked_objects.objects:
            if len(local_scores) == self.capacity:
                self.get_logger().warning(
                    f'{len(tracked_objects.objects)} tracked objects exceed '
                    f'the SDSM capacity of {self.capacity} - dropping surplus'
                )
                break

            slot = len(local_scores)
            object_pose = (
                tracked_object.kinematics.pose_with_covariance.pose
            )
            if object_transform is not None:
                object_pose = transform_pose(
                    object_pose, object_transform.transform
                )
            dimensions = tracked_object.shape.dimensions
            offset = (
                object_pose.position.x - sensor_origin.x,
                object_pose.position.y - sensor_origin.y,
                object_pose.position.z - sensor_origin.z,
            )

            payload.obj_type[slot] = self.object_type(tracked_object)
            payload.object_id[slot] = self.object_id(
                tracked_object.object_id
            )
            payload.offset_x[slot] = round(offset[0] / OFFSET_UNIT_M)
            payload.offset_y[slot] = round(offset[1] / OFFSET_UNIT_M)
            payload.offset_z[slot] = round(offset[2] / OFFSET_UNIT_M)
            payload.obj_length[slot] = round(
                dimensions.x / OFFSET_UNIT_M
            )
            payload.obj_width[slot] = round(
                dimensions.y / OFFSET_UNIT_M
            )
            payload.obj_height[slot] = round(
                dimensions.z / OFFSET_UNIT_M
            )
            payload.obj_heading[slot] = self.heading_units(
                object_pose.orientation
            )
            payload.obj_speed[slot] = self.speed_units(tracked_object)
            payload.obj_measurement_time_ms[slot] = lag_ms

            score = scores_by_id[uuid_key(tracked_object.object_id)]
            score = min(max(score, 0.0), 1.0) if math.isfinite(score) else 0.0
            payload.obj_local_scores[slot] = score
            local_scores.append(score)

        payload.num_objects = len(local_scores)
        payload.local_score = (
            sum(local_scores) / len(local_scores) if local_scores else 0.0
        )

        self.publisher_.publish(payload)
        self.msg_cnt = (self.msg_cnt + 1) % MSG_COUNT_MODULO
        self.get_logger().info(
            f'Published SDSM {payload.msg_cnt} with {payload.num_objects} '
            f'objects, frame score {payload.local_score:.2f}'
        )


def main(args=None):
    """Run the SDSM publisher node."""
    rclpy.init(args=args)
    node = SdsmPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
