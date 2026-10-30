import rclpy
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

current_file=os.path.dirname(__file__)

def generate_launch_description():

    viper_model=LaunchConfiguration('viper_model',default='viperx300')
    reactor_model=LaunchConfiguration('reactor_model',default='reactorx150')

    viper_ns = LaunchConfiguration('viper_ns',default='viper')
    reactor_ns=LaunchConfiguration('reactor_ns',default='reactor')

    arm_bringup_launch=PathJoinSubstitution([current_file,'arm_bringup.launch.py'])

    viper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(arm_bringup_launch),
        launch_arguments={'robot_model': viper_model, 'namespace': viper_ns}.items()
    )

    reactor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(arm_bringup_launch),
        launch_arguments={'robot_model': reactor_model, 'namespace': reactor_ns}.items()
    )

    bilateral_node = Node(
        package='bilateral_node',           # change to your package name if packaging
        executable='bilateral_node',        # if you run as script: use Node with `entry_point` if installed; here Node can run a script if using `ros2 run` style packaging
        name='bilateral_node',
        output='screen',
        parameters=[os.path.join(os.path.dirname(__file__), '..', 'params', 'bilateral_params.yaml')]
    )

    return LaunchDescription([
        DeclareLaunchArgument('viper_model', default_value='viperx300'),
        DeclareLaunchArgument('reactor_model', default_value='reactorx150'),
        DeclareLaunchArgument('viper_ns', default_value='viper'),
        DeclareLaunchArgument('reactor_ns', default_value='reactor'),
        viper_launch,
        reactor_launch,
        bilateral_node
    ])



    # return LaunchDescription()