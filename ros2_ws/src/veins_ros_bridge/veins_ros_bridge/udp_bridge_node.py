#!/usr/bin/env python3
"""
UDP bridge: Veins sim <-> ROS 2. Uses sdsm_msgs (CPX-SDSM) as the message foundation.
- Listens on UDP (default 50010) for TX/RX JSON lines from the sim (RosSDSMApp,
  rosBridgeMode="live"); decodes each into a sdsm_trust_interfaces/ReceivedSdsm
  and publishes it on /veins/sdsm_events, so ROS 2 nodes (e.g.
  sdsm_trust_perception) see structured messages instead of raw text.
  Also republishes the original raw line on /veins/rx_raw for anything that
  still wants the text form.
- Subscribes to /veins/sdsm_tx (SensorDataSharingMessage) so ROS 2 nodes can
  inject SDSM in the same format as the sim and real world.

DECODE FAILURE POLICY: a line that isn't valid JSON, or whose "event"/"sdsm"
shape doesn't match what RosSDSMApp::buildSdsmJson emits, is logged at WARN
and dropped -- never raised into the executor (see _on_line), so one
malformed line from the sim cannot take the bridge down mid-run. rx_raw still
gets the line either way, so nothing is lost for a text-only subscriber.
"""

import json
import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sdsm_msgs.msg import (
    DDateTime,
    DetectedObjectCommonData,
    DetectedObjectData,
    DetectedObstacleData,
    DetectedVehicleData,
    DetectedVRUData,
    ObstacleSize,
    Position3D,
    PositionConfidenceSet,
    PositionOffsetXYZ,
    SensorDataSharingMessage,
    VehicleSize,
)
from sdsm_trust_interfaces.msg import ReceivedSdsm


def _decode_sdsm(d: dict) -> SensorDataSharingMessage:
    """Build a SensorDataSharingMessage from buildSdsmJson's dict shape.
    Field names below match that function 1:1 (RosSDSMApp.cc) -- see it for
    the authoritative wire shape."""
    msg = SensorDataSharingMessage()
    msg.msg_cnt = int(d['msg_cnt'])
    msg.source_id = [int(b) & 0xFF for b in d['source_id']]
    msg.equipment_type = int(d['equipment_type'])

    ts = d['sdsm_time_stamp']
    msg.sdsm_time_stamp = DDateTime(
        day_of_month=int(ts['day_of_month']), time_of_day=int(ts['time_of_day']),
    )

    rp = d['ref_pos']
    msg.ref_pos = Position3D(
        lat=int(rp['lat']), lon=int(rp['lon']), elevation=int(rp['elevation']),
    )

    objects = []
    for o in d.get('objects', []):
        common = o['det_obj_common']
        pos = common['pos']
        conf = common['pos_confidence']
        det_common = DetectedObjectCommonData(
            obj_type=int(common['obj_type']),
            obj_type_cfd=int(common['obj_type_cfd']),
            object_id=int(common['object_id']) & 0xFFFF,
            measurement_time=int(common['measurement_time']),
            pos=PositionOffsetXYZ(
                offset_x=int(pos['offset_x']), offset_y=int(pos['offset_y']),
                offset_z=int(pos['offset_z']), has_offset_z=bool(pos['has_offset_z']),
            ),
            pos_confidence=PositionConfidenceSet(
                pos_confidence=int(conf['pos_confidence']),
                elevation_confidence=int(conf['elevation_confidence']),
            ),
            speed=int(common['speed']) & 0xFFFF,
            speed_z=int(common['speed_z']) & 0xFFFF,
            has_speed_z=bool(common['has_speed_z']),
            heading=int(common['heading']) & 0xFFFF,
        )

        veh = o['det_veh']
        vsize = veh['size']
        det_veh = DetectedVehicleData(
            size=VehicleSize(width=int(vsize['width']) & 0xFFFF,
                             length=int(vsize['length']) & 0xFFFF),
            has_size=bool(veh['has_size']),
            height=int(veh['height']) & 0xFFFF,
            has_height=bool(veh['has_height']),
            vehicle_class=int(veh['vehicle_class']) & 0xFF,
            has_vehicle_class=bool(veh['has_vehicle_class']),
            class_conf=int(veh['class_conf']) & 0xFF,
            has_class_conf=bool(veh['has_class_conf']),
        )

        vru = o['det_vru']
        det_vru = DetectedVRUData(basic_type=int(vru['basic_type']) & 0xFF)

        obst = o['det_obst']['obst_size']
        det_obst = DetectedObstacleData(
            obst_size=ObstacleSize(width=int(obst['width']) & 0xFFFF,
                                   length=int(obst['length']) & 0xFFFF,
                                   height=int(obst['height']) & 0xFFFF),
        )

        objects.append(DetectedObjectData(
            det_obj_common=det_common,
            det_obj_opt_kind=int(o['det_obj_opt_kind']) & 0xFF,
            det_veh=det_veh, det_vru=det_vru, det_obst=det_obst,
        ))
    msg.objects = objects
    return msg


class UdpBridgeNode(Node):
    def __init__(self):
        super().__init__("udp_bridge_node")
        self.declare_parameter("udp_port", 50010)
        self.declare_parameter("udp_host", "0.0.0.0")
        self.declare_parameter("events_topic", "/veins/sdsm_events")
        port = self.get_parameter("udp_port").value
        host = self.get_parameter("udp_host").value
        events_topic = self.get_parameter("events_topic").value

        self.pub_rx_raw = self.create_publisher(String, "/veins/rx_raw", 10)
        self.pub_events = self.create_publisher(ReceivedSdsm, events_topic, 200)
        self.sub_sdsm_tx = self.create_subscription(
            SensorDataSharingMessage,
            "/veins/sdsm_tx",
            self.on_sdsm_tx,
            10,
        )
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._udp_sock.bind((host, port))
        self._udp_sock.settimeout(0.5)
        self._running = True
        self._decode_errors = 0
        self._thread = threading.Thread(target=self._udp_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"UDP bridge listening on {host}:{port}; decoding TX/RX JSON -> "
            f"{events_topic} (sdsm_msgs types)."
        )

    def on_sdsm_tx(self, msg: SensorDataSharingMessage):
        """ROS 2 SDSM received — same message type as sim and real world."""
        self.get_logger().debug(
            f"sdsm_tx: msg_cnt={msg.msg_cnt} equipment_type={msg.equipment_type} "
            f"ref_pos lat={msg.ref_pos.lat} lon={msg.ref_pos.lon} objects={len(msg.objects)}"
        )

    def _on_line(self, line: str) -> None:
        self.pub_rx_raw.publish(String(data=line))

        # HELLO/PONG/ACK control lines (RosSDSMApp::sendToRos) aren't JSON
        # events; only "{" lines are TX/RX payloads worth decoding.
        if not line.startswith('{'):
            return
        try:
            d = json.loads(line)
            event = d['event']
            if event not in ('TX', 'RX'):
                return
            out = ReceivedSdsm()
            out.is_rx = (event == 'RX')
            out.node = int(d['node'])
            out.sim_time = float(d['time'])
            out.sdsm = _decode_sdsm(d['sdsm'])
            if out.is_rx:
                out.latency = float(d.get('latency', 0.0))
                out.sender_node = int(d.get('sender', out.sdsm.source_id[0]))
            else:
                out.latency = 0.0
                out.sender_node = out.node
            self.pub_events.publish(out)
        except Exception as e:
            self._decode_errors += 1
            if self._decode_errors <= 5 or self._decode_errors % 100 == 0:
                self.get_logger().warning(
                    f"dropped malformed event line (total={self._decode_errors}): "
                    f"{e!r} line={line[:200]!r}"
                )

    def _udp_loop(self):
        buf = b""
        while self._running and rclpy.ok():
            try:
                data, _ = self._udp_sock.recvfrom(65535)
                if not data:
                    continue
                buf += data
                while b"\n" in buf or b"\r" in buf:
                    line, buf = (buf.split(b"\n", 1) if b"\n" in buf else buf.split(b"\r", 1))
                    line = line.strip().decode("utf-8", errors="replace")
                    if line:
                        self._on_line(line)
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    self.get_logger().warn(f"UDP recv: {e}")

    def shutdown(self):
        self._running = False
        if self._udp_sock:
            self._udp_sock.close()


def main(args=None):
    rclpy.init(args=args)
    node = UdpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
