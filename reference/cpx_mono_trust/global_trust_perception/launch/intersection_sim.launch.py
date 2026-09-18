"""
One-command launch for the intersection sim.

Starts the SORT tracker, the five sensor agents (Car 1-4 + the RSU), and the
read-only scene visualizer under a single `ros2 launch`, replacing the manual
one-terminal-per-node sequence. Order mirrors the README: the tracker must be up
before the agents, since they consume its TrackUpdate stream.

    ros2 launch global_trust_perception intersection_sim.launch.py

The VRU is a detected pedestrian, not a broadcasting sensor, so it has no node;
it appears only in the sensors' detections (see sim_world).
"""

from launch import LaunchDescription
from launch_ros.actions import Node

# Sensor agent id -> node name. 1-4 are the cars, 5 is the RSU.
_SENSORS = {1: 'agent_1', 2: 'agent_2', 3: 'agent_3', 4: 'agent_4', 5: 'agent_5'}


def generate_launch_description() -> LaunchDescription:
    """Tracker first, then the five agents, then the scene visualizer."""
    tracker = Node(
        package='global_trust_tracker', executable='tracker',
        name='tracker', output='screen',
    )
    agents = [
        Node(
            package='global_trust_perception', executable='agent',
            name=name, output='screen',
            parameters=[{'agent_id': agent_id}],
        )
        for agent_id, name in _SENSORS.items()
    ]
    scene = Node(
        package='global_trust_perception', executable='scene',
        name='scene_node', output='screen',
    )
    return LaunchDescription([tracker, *agents, scene])
