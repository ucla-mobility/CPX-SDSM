#!/usr/bin/env python3
"""
UDP bridge: Veins sim <-> ROS 2. Uses sdsm_msgs (CPX-SDSM) as the message foundation.
- Listens on UDP (default 50010) for TX/RX lines from the sim; publishes to /veins/rx_raw.
- Subscribes to /veins/sdsm_tx (SensorDataSharingMessage) so ROS 2 nodes can inject SDSM
  in the same format as the sim and real world.
"""

import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sdsm_msgs.msg import SensorDataSharingMessage


class UdpBridgeNode(Node):
    def __init__(self):
        super().__init__("udp_bridge_node")
        self.declare_parameter("udp_port", 50010)
        self.declare_parameter("udp_host", "0.0.0.0")
        port = self.get_parameter("udp_port").value
        host = self.get_parameter("udp_host").value

        self.pub_rx_raw = self.create_publisher(String, "/veins/rx_raw", 10)
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
        self._thread = threading.Thread(target=self._udp_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f"UDP bridge listening on {host}:{port}; sdsm_msgs (CPX-SDSM) in use.")

    def on_sdsm_tx(self, msg: SensorDataSharingMessage):
        """ROS 2 SDSM received — same message type as sim and real world."""
        self.get_logger().debug(
            f"sdsm_tx: msg_cnt={msg.msg_cnt} equipment_type={msg.equipment_type} "
            f"ref_pos lat={msg.ref_pos.lat} lon={msg.ref_pos.lon} objects={len(msg.objects)}"
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
                        self.pub_rx_raw.publish(String(data=line))
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
