#!/usr/bin/env python3
"""
Simple Bilateral Teleoperation Launch

This launch file starts only the bilateral teleoperation node.
Assumes the robots (rx150 and vx300s) are already running with MoveIt2.

Usage:
    ros2 launch bilateral_teleop bilateral_teleop.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package path
    bilateral_pkg = get_package_share_directory('bilateral_teleop')
    
    # Arguments
    leader_arg = DeclareLaunchArgument(
        'leader_robot', default_value='rx150',
        description='Leader robot name'
    )
    
    follower_arg = DeclareLaunchArgument(
        'follower_robot', default_value='vx300s',
        description='Follower robot name'
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate', default_value='30.0',
        description='Control loop rate (Hz)'
    )
    
    # Bilateral node
    bilateral_node = Node(
        package='bilateral_teleop',
        executable='moveit2_bilateral',
        name='moveit2_bilateral_teleop',
        output='screen',
        parameters=[
            os.path.join(bilateral_pkg, 'config', 'bilateral_params.yaml'),
            {
                'leader.robot_name': LaunchConfiguration('leader_robot'),
                'follower.robot_name': LaunchConfiguration('follower_robot'),
                'control_rate': LaunchConfiguration('rate'),
            }
        ],
    )
    
    return LaunchDescription([
        leader_arg,
        follower_arg,
        rate_arg,
        bilateral_node,
    ])
