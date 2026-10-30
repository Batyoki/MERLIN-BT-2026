#!/usr/bin/env python3
"""
Bilateral Teleoperation Launch File with Gazebo Simulation

This launch file sets up:
1. Gazebo simulation with ReactorX-150 and ViperX-300s
2. MoveIt2 for both robots
3. Bilateral teleoperation node (RX150 leader -> VX300s follower)

Usage:
    ros2 launch bilateral_teleop bilateral_gazebo.launch.py
    ros2 launch bilateral_teleop bilateral_gazebo.launch.py use_sim:=true  # Gazebo only
    ros2 launch bilateral_teleop bilateral_gazebo.launch.py use_sim:=false # Hardware
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ========== Launch Arguments ==========
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use Gazebo simulation (true) or hardware (false)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='Launch MoveIt2 for planning'
    )
    
    # Leader robot arguments
    leader_model_arg = DeclareLaunchArgument(
        'leader_model',
        default_value='rx150',
        description='Leader robot model (rx150, rx200)'
    )
    
    leader_name_arg = DeclareLaunchArgument(
        'leader_name',
        default_value='rx150',
        description='Leader robot namespace'
    )
    
    # Follower robot arguments
    follower_model_arg = DeclareLaunchArgument(
        'follower_model',
        default_value='vx300s',
        description='Follower robot model (vx300s, vx300)'
    )
    
    follower_name_arg = DeclareLaunchArgument(
        'follower_name',
        default_value='vx300s',
        description='Follower robot namespace'
    )
    
    # Get configurations
    use_sim = LaunchConfiguration('use_sim')
    use_rviz = LaunchConfiguration('use_rviz')
    use_moveit = LaunchConfiguration('use_moveit')
    leader_model = LaunchConfiguration('leader_model')
    leader_name = LaunchConfiguration('leader_name')
    follower_model = LaunchConfiguration('follower_model')
    follower_name = LaunchConfiguration('follower_name')
    
    # Package paths
    bilateral_pkg = get_package_share_directory('bilateral_teleop')
    
    # Try to find interbotix packages
    try:
        xsarm_sim_pkg = get_package_share_directory('interbotix_xsarm_sim')
        xsarm_control_pkg = get_package_share_directory('interbotix_xsarm_control')
        xsarm_moveit_pkg = get_package_share_directory('interbotix_xsarm_moveit')
        interbotix_available = True
    except Exception:
        interbotix_available = False
        xsarm_sim_pkg = None
        xsarm_control_pkg = None
        xsarm_moveit_pkg = None
    
    # ========== Build Launch Description ==========
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(use_sim_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(use_moveit_arg)
    ld.add_action(leader_model_arg)
    ld.add_action(leader_name_arg)
    ld.add_action(follower_model_arg)
    ld.add_action(follower_name_arg)
    
    if interbotix_available:
        # ========== Gazebo Simulation for Leader (RX150) ==========
        leader_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                xsarm_sim_pkg, '/launch/xsarm_gz_classic.launch.py'
            ]),
            launch_arguments={
                'robot_model': leader_model,
                'robot_name': leader_name,
                'use_rviz': 'false',
                'use_gazebo_gui': 'true',
                'world_filepath': '',
            }.items(),
            condition=IfCondition(use_sim)
        )
        ld.add_action(leader_sim)
        
        # ========== Gazebo Simulation for Follower (VX300s) ==========
        # Spawn second robot with offset position
        follower_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                xsarm_sim_pkg, '/launch/xsarm_gz_classic.launch.py'
            ]),
            launch_arguments={
                'robot_model': follower_model,
                'robot_name': follower_name,
                'use_rviz': 'false',
                'use_gazebo_gui': 'false',  # Already launched with leader
                'world_filepath': '',
                'xs_driver_logging_level': 'INFO',
            }.items(),
            condition=IfCondition(use_sim)
        )
        # Delay follower spawn to let leader spawn first
        ld.add_action(TimerAction(period=3.0, actions=[follower_sim]))
        
        # ========== MoveIt2 for Leader ==========
        leader_moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                xsarm_moveit_pkg, '/launch/xsarm_moveit.launch.py'
            ]),
            launch_arguments={
                'robot_model': leader_model,
                'robot_name': leader_name,
                'hardware_type': PythonExpression(["'gz_classic' if ", use_sim, " else 'actual'"]),
                'use_rviz': 'false',
            }.items(),
            condition=IfCondition(use_moveit)
        )
        ld.add_action(TimerAction(period=5.0, actions=[leader_moveit]))
        
        # ========== MoveIt2 for Follower ==========
        follower_moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                xsarm_moveit_pkg, '/launch/xsarm_moveit.launch.py'
            ]),
            launch_arguments={
                'robot_model': follower_model,
                'robot_name': follower_name,
                'hardware_type': PythonExpression(["'gz_classic' if ", use_sim, " else 'actual'"]),
                'use_rviz': 'false',
            }.items(),
            condition=IfCondition(use_moveit)
        )
        ld.add_action(TimerAction(period=7.0, actions=[follower_moveit]))
    
    # ========== Bilateral Teleoperation Node ==========
    bilateral_node = Node(
        package='bilateral_teleop',
        executable='moveit2_bilateral',
        name='moveit2_bilateral_teleop',
        output='screen',
        parameters=[
            os.path.join(bilateral_pkg, 'config', 'bilateral_params.yaml'),
            {
                'leader.robot_name': leader_name,
                'follower.robot_name': follower_name,
            }
        ],
    )
    # Delay to let MoveIt2 initialize
    ld.add_action(TimerAction(period=10.0, actions=[bilateral_node]))
    
    # ========== RViz Visualization ==========
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
    )
    ld.add_action(TimerAction(period=8.0, actions=[rviz_node]))
    
    return ld
