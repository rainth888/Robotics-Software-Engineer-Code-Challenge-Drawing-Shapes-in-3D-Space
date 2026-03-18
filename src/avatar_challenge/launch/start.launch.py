"""
start.launch.py — Launches xArm7 MoveIt2 simulation + shape_drawer node.

Usage (inside Docker container):
    ros2 launch avatar_challenge start.launch.py
    ros2 launch avatar_challenge start.launch.py config_file:=/path/to/custom.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('avatar_challenge')
    default_config = os.path.join(pkg_share, 'config', 'shapes.yaml')

    # Declare launch argument for custom shape config file
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to YAML shape definitions',
    )

    # Include xArm7 MoveIt2 fake simulation launch
    # This starts: robot_state_publisher, move_group, rviz2,
    #              ros2_control_node, joint_state_broadcaster, xarm7_traj_controller
    xarm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_moveit_config'),
                'launch',
                'xarm7_moveit_fake.launch.py',
            ])
        ),
    )

    # Delay shape_drawer by 10 seconds to let MoveIt fully initialise
    shape_drawer = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='avatar_challenge',
                executable='shape_drawer.py',
                name='shape_drawer',
                output='screen',
                parameters=[{
                    'config_file': LaunchConfiguration('config_file'),
                }],
            ),
        ],
    )

    return LaunchDescription([
        config_arg,
        xarm_moveit_launch,
        shape_drawer,
    ])
