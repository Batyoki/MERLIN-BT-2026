from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to Interbotix robot launch files
    viperx300_path = os.path.join(
        get_package_share_directory('interbotix_xsarm_control'),
        'launch',
        'xsarm_control.launch.py'
    )
    reactorx150_path = os.path.join(
        get_package_share_directory('interbotix_xsarm_control'),
        'launch',
        'xsarm_control.launch.py'
    )

    # Follower (ViperX 300)
    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(viperx300_path),
        launch_arguments={
            'robot_model': 'vx300s',
            'use_sim': 'true',
            'use_rviz': 'true',
            'robot_name': 'viperx300'
        }.items()
    )

    # Leader (ReactorX 150)
    leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(reactorx150_path),
        launch_arguments={
            'robot_model': 'rx150',
            'use_sim': 'true',
            'use_rviz': 'true',
            'robot_name': 'reactorx150',
            'use_joint_pub_gui':'true',
        }.items()
    )

    # Bilateral node
    bilateral_node = Node(
        package='bilateral_node',
        executable='bilateral_node',
        name='bilateral_node',
        output='screen',
        parameters=[{'follower_ns': '/vx300', 'leader_ns': '/rx150'}]
    )

    return LaunchDescription([
        leader,
        follower,
        bilateral_node
    ])
