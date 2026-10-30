import rclpy
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    robot_model = LaunchConfiguration('robot_model',default='vx300')
    namespace=LaunchConfiguration('namespace',default='viper')


    bringup_Node=Node(
        package='bilateral_node',
        executable='bilateral_node',
        name='arm_bringup',
        namespace=namespace,
        parameters=[{'robot_model':robot_model}],
        output='screen'
    )

    

    ld = LaunchDescription([
        DeclareLaunchArgument('robot_model',default_value='vx300',description='Robot arm model'),
        DeclareLaunchArgument('namespace',default_value='viper',description='Namespace for arm'),
        bringup_Node
    ])

    return ld