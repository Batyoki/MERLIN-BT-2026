from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    output_root_arg = DeclareLaunchArgument(
        'output_root',
        default_value='/tmp/sessions',
        description='Root directory for storing session data'
    )

    return LaunchDescription([
        output_root_arg,
        Node(
            package='data_collection_pkg',
            executable='session_node', # The executable wrapping TimestampedRecorder
            name='session_recorder',
            output='screen',
            parameters=[
                {'output_root': LaunchConfiguration('output_root')},
                {'detect_tags': False},
                {'marker_length_m': 0.05}
            ]
        )
    ])