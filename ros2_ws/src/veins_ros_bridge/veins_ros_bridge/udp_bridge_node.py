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
from sdsm_msgs.msg import SensorDataSharingMessage
from sdsm_trust_interfaces.msg import ReceivedSdsm
from sdsm_trust_perception.pipeline import sdsm_codec as codec

# Decoding a buildSdsmJson() dict into a SensorDataSharingMessage lives in
# sdsm_trust_perception.pipeline.sdsm_codec (codec.sdsm_from_dict) -- the same
# function an offline replay of a rosBridgeMode="log" .jsonl file uses, since
# RosSDSMApp.cc emits byte-identical JSON on both paths. Kept in that package
# (not here) because that's where the rest of this wire format's field-level
# knowledge already lives; this module only owns UDP transport.


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
            out.sdsm = codec.sdsm_from_dict(d['sdsm'])
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
