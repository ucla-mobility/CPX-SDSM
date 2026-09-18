# mypy: ignore-errors
# sdsm_interfaces ships generated classes; mypy cannot resolve them.
"""
SORT tracker node: subscribes to SDSM, publishes TrackUpdate per message.

One Sort() tracker is maintained per observed sender (keyed by source_id tuple).
Each incoming SdsmPayload triggers Sort.update() for that sender; the resulting
per-detection track states are published immediately as a TrackUpdate whose
arrays are parallel to the SDSM detections (length = num_objects).

Launch before agent nodes:
    ros2 run global_trust_tracker tracker
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sdsm_interfaces.msg import SdsmPayload, TrackUpdate
from global_trust_tracker.sdsm_units import (
    HEADING_UNAVAILABLE,
    HEADING_UNIT_DEG,
    OFFSET_UNIT_M,
    SPEED_UNIT_MS,
)
from global_trust_tracker.SORT.modified_SORT_centroid import Sort

_SDSM_TOPIC   = '/perception/global_trustworthiness/sdsm'
_TRACKS_TOPIC = '/perception/global_trustworthiness/tracks'


class TrackerNode(Node):

    def __init__(self):
        super().__init__('tracker')
        self._trackers: dict[tuple, Sort] = {}
        self.sub = self.create_subscription(
            SdsmPayload, _SDSM_TOPIC, self._on_sdsm, 100
        )
        self.pub = self.create_publisher(TrackUpdate, _TRACKS_TOPIC, 100)
        self.get_logger().info(
            f'TrackerNode: {_SDSM_TOPIC} -> {_TRACKS_TOPIC}'
        )

    def _sort_for(self, source_id: tuple) -> Sort:
        if source_id not in self._trackers:
            self._trackers[source_id] = Sort()
        return self._trackers[source_id]

    def _on_sdsm(self, msg: SdsmPayload):
        n = int(msg.num_objects)
        source_id = tuple(int(x) for x in msg.source_id)

        if n > 0:
            ax, ay = msg.ref_pos_x, msg.ref_pos_y
            xy = np.array(
                [[ax + msg.offset_x[i] * OFFSET_UNIT_M,
                  ay + msg.offset_y[i] * OFFSET_UNIT_M]
                 for i in range(n)],
                dtype=float,
            )
            vels = []
            for i in range(n):
                spd = msg.obj_speed[i] * SPEED_UNIT_MS
                h = msg.obj_heading[i]
                if h == HEADING_UNAVAILABLE:
                    vels.append((0.0, 0.0))
                else:
                    rad = math.radians(h * HEADING_UNIT_DEG)
                    vels.append((spd * math.sin(rad), spd * math.cos(rad)))
            vel = np.array(vels, dtype=float)
            dims = np.array(
                [[msg.obj_width[i]  * OFFSET_UNIT_M,
                  msg.obj_length[i] * OFFSET_UNIT_M,
                  msg.obj_height[i] * OFFSET_UNIT_M]
                 for i in range(n)],
                dtype=float,
            )
        else:
            xy   = np.empty((0, 2))
            vel  = None
            dims = None

        tracks = self._sort_for(source_id).update(xy, vel, dims)

        out = TrackUpdate()
        out.source_id         = list(source_id)
        out.msg_cnt           = int(msg.msg_cnt)
        out.track_id          = [int(t.id)                for t in tracks]
        out.age               = [int(t.age)               for t in tracks]
        out.confirmed         = [bool(t.confirmed)        for t in tracks]
        out.time_since_update = [int(t.time_since_update) for t in tracks]
        out.kalman_x  = [float(t.kf.x[0, 0]) for t in tracks]
        out.kalman_y  = [float(t.kf.x[1, 0]) for t in tracks]
        out.kalman_vx = [float(t.kf.x[4, 0]) for t in tracks]
        out.kalman_vy = [float(t.kf.x[5, 0]) for t in tracks]
        if tracks:
            w_vals, l_vals, h_vals = zip(*[t.get_dims() for t in tracks])
        else:
            w_vals, l_vals, h_vals = [], [], []
        out.kalman_w = [float(v) for v in w_vals]
        out.kalman_l = [float(v) for v in l_vals]
        out.kalman_h = [float(v) for v in h_vals]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
