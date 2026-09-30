"""Launch the local trustworthiness pipeline and optional rosbag playback."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _launch_setup(context):
    """Resolve arguments, validate paths, and create pipeline actions."""
    bag_argument = LaunchConfiguration('bag_path').perform(context).strip()
    output_argument = LaunchConfiguration(
        'output_bag_path'
    ).perform(context).strip()
    tracked_objects_topic = LaunchConfiguration(
        'tracked_objects_topic'
    ).perform(context)
    cloud_topic = LaunchConfiguration('cloud_topic').perform(context)
    sensor_frame = LaunchConfiguration('sensor_frame').perform(context)
    global_frame = LaunchConfiguration('global_frame').perform(context)
    score_topic = LaunchConfiguration('score_topic').perform(context)
    sdsm_topic = LaunchConfiguration('sdsm_topic').perform(context)
    use_sim_time_argument = LaunchConfiguration(
        'use_sim_time'
    ).perform(context).strip().lower()

    bag_path = Path(bag_argument).expanduser().resolve() \
        if bag_argument else None
    output_bag_path = Path(output_argument).expanduser().resolve() \
        if output_argument else None
    if use_sim_time_argument == 'auto':
        use_sim_time = bag_path is not None
    elif use_sim_time_argument in ('true', 'false'):
        use_sim_time = use_sim_time_argument == 'true'
    else:
        raise RuntimeError(
            'use_sim_time must be auto, true, or false, got '
            f'{use_sim_time_argument!r}'
        )

    if bag_path is not None and not bag_path.exists():
        raise RuntimeError(f'bag_path does not exist: {bag_path}')
    if output_bag_path is not None:
        if output_bag_path.exists():
            raise RuntimeError(
                f'output_bag_path already exists: {output_bag_path}'
            )
        if not output_bag_path.parent.is_dir():
            raise RuntimeError(
                'output_bag_path parent does not exist: '
                f'{output_bag_path.parent}'
            )

    actions = [
        Node(
            package='local_trust_estimation',
            executable='object_shape',
            name='object_shape_node',
            parameters=[{
                'input_topic': tracked_objects_topic,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
        Node(
            package='local_trust_estimation',
            executable='lidar_point_count',
            name='lidar_point_count_node',
            parameters=[{
                'cloud_topic': cloud_topic,
                'tracked_objects_topic': tracked_objects_topic,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
        Node(
            package='local_trust_estimation',
            executable='object_distance',
            name='object_distance_node',
            parameters=[{
                'input_topic': tracked_objects_topic,
                'sensor_frame': sensor_frame,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
        Node(
            package='local_trust_estimation',
            executable='temporal_presence',
            name='temporal_presence_node',
            parameters=[{
                'input_topic': tracked_objects_topic,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
        Node(
            package='local_trust_estimation',
            executable='trustworthiness_score',
            name='trustworthiness_score_node',
            parameters=[{
                'output_topic': score_topic,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
        Node(
            package='local_trust_estimation',
            executable='trustworthiness_visualization',
            name='trustworthiness_visualization_node',
            parameters=[{
                'tracked_objects_topic': tracked_objects_topic,
                'score_topic': score_topic,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
        Node(
            package='local_trust_estimation',
            executable='sdsm_publisher',
            name='sdsm_publisher_node',
            parameters=[{
                'tracked_objects_topic': tracked_objects_topic,
                'score_topic': score_topic,
                'output_topic': sdsm_topic,
                'sensor_frame': sensor_frame,
                'global_frame': global_frame,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
    ]

    if output_bag_path is not None:
        actions.append(ExecuteProcess(
            cmd=[
                'ros2',
                'bag',
                'record',
                '--all-topics',
                '--exclude-regex',
                '^/(rosout|parameter_events|events/.*)$',
                '--storage',
                'mcap',
                '--storage-preset-profile',
                'zstd_fast',
                '--disable-keyboard-controls',
                '--output',
                str(output_bag_path),
            ],
            output='screen',
        ))

    if bag_path is not None:
        bag_player = ExecuteProcess(
            cmd=[
                'ros2',
                'bag',
                'play',
                str(bag_path),
                '--clock',
                '100',
                '--disable-keyboard-controls',
            ],
            output='screen',
        )
        actions.append(RegisterEventHandler(
            OnProcessExit(
                target_action=bag_player,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason='Source rosbag playback finished'
                        )
                    ),
                ],
            )
        ))
        actions.append(TimerAction(
            period=2.0,
            actions=[bag_player],
        ))

    return actions


def generate_launch_description():
    """Create the local trustworthiness launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'tracked_objects_topic',
            default_value='/vehicle/perception/tracked_objects',
            description='Canonical Autoware TrackedObjects input topic.',
        ),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/vehicle/lidar/points',
            description='Synchronized PointCloud2 input for point counting.',
        ),
        DeclareLaunchArgument(
            'sensor_frame',
            default_value='vehicle_lidar',
            description='Lidar frame used for distance and SDSM reference.',
        ),
        DeclareLaunchArgument(
            'global_frame',
            default_value='map',
            description='Global ENU frame used by the SDSM payload.',
        ),
        DeclareLaunchArgument(
            'score_topic',
            default_value='/local_trust_estimation/score',
            description='UUID-keyed combined local-trust output topic.',
        ),
        DeclareLaunchArgument(
            'sdsm_topic',
            default_value='/perception/global_trustworthiness/sdsm',
            description='Final SDSM output topic.',
        ),
        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description=(
                'Rosbag directory to play after startup. Supports ~. Leave '
                'empty to start only the nodes.'
            ),
        ),
        DeclareLaunchArgument(
            'output_bag_path',
            default_value='',
            description=(
                'New bag directory for all original and generated topics. '
                'Supports ~. Leave empty to disable recording.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='auto',
            description=(
                'Use ROS simulation time: auto enables it when bag_path is '
                'set; true/false force an explicit clock mode.'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
