"""Launch the UDP<->ROS2 bridge and the trust perception node together.

    ros2 launch sdsm_trust_perception sdsm_trust.launch.py

Requires the running (or about-to-run) simulation configured with
rosBridgeMode = "live" in omnetpp.ini so RosSDSMApp actually emits TX/RX
JSON over UDP -- see CPX-SDSM/README.md "ROS 2 bridge".
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    udp_port = DeclareLaunchArgument('udp_port', default_value='50010')
    flush_interval_s = DeclareLaunchArgument('flush_interval_s', default_value='0.5')

    bridge = Node(
        package='veins_ros_bridge',
        executable='udp_bridge_node',
        name='udp_bridge_node',
        parameters=[{'udp_port': LaunchConfiguration('udp_port')}],
        output='screen',
    )

    trust = Node(
        package='sdsm_trust_perception',
        executable='trust_node',
        name='trust_perception_node',
        parameters=[{'flush_interval_s': LaunchConfiguration('flush_interval_s')}],
        output='screen',
    )

    return LaunchDescription([udp_port, flush_interval_s, bridge, trust])
