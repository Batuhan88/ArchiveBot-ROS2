#!/usr/bin/env python3
"""ArchiveBot V11 all-in-one Gazebo + ROS 2 launch.

The V11 navigation/vision scripts and robot files are not modified.
The simulation must be stopped before launching a fresh instance.
"""

import tempfile
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('archivebot_description'))
    world = share / 'worlds' / 'archive_room.sdf'
    model = share / 'models' / 'archivebot_v9.urdf.xacro'
    bridge_config = share / 'config' / 'bridge.yaml'

    # "create" accepts expanded URDF; keep a temporary copy, not a modified Xacro.
    with tempfile.NamedTemporaryFile(
        mode='w', prefix='archivebot_', suffix='.urdf', delete=False
    ) as generated:
        generated.write(xacro.process_file(str(model)).toxml())
        urdf_path = generated.name

    start_navigation = LaunchConfiguration('start_navigation')

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '3', str(world), '--render-engine', 'ogre'],
        output='screen',
    )

    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='archivebot_sensor_bridge',
        parameters=[{'config_file': str(bridge_config)}],
        output='screen',
    )

    # V11 subscribes to this Gazebo ground-truth odometry topic.
    ground_truth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='archivebot_ground_truth_bridge',
        arguments=[
            '/archivebot/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen',
    )

    vision = Node(
        package='archivebot_description',
        executable='archivebot_vision.py',
        name='archivebot_vision',
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'archive_room',
            '-file', urdf_path,
            '-name', 'archivebot',
            '-x', '-4.5',
            '-y', '-3.3',
            '-z', '0.225',
        ],
        output='screen',
    )

    # The navigation node stays stopped until entity creation succeeds.
    def after_spawn(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg='ArchiveBot spawn failed; V11 navigation was not started.')
            ]
        return [
            LogInfo(msg='ArchiveBot spawned. Starting V11 in 5 seconds if enabled.'),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package='archivebot_description',
                        executable='archivebot_navigation_v11.py',
                        name='archivebot_navigation_v11',
                        condition=IfCondition(start_navigation),
                        output='screen',
                    )
                ],
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_navigation',
            default_value='true',
            description='If false, launch Gazebo, robot, bridges and vision without moving.',
        ),
        LogInfo(msg='ArchiveBot V11 launch: start_navigation can be set to false.'),
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_robot, on_exit=after_spawn)
        ),
        gazebo,
        sensor_bridge,
        ground_truth_bridge,
        vision,
        # Allow Gazebo to load the world before attempting entity creation.
        TimerAction(period=20.0, actions=[spawn_robot]),
    ])
