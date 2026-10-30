from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='data_collection_pkg',
            executable='writer_node', # The executable wrapping FrameWriter & JsonlStore
            name='data_writer',
            output='screen',
            parameters=[
                {'max_queue_size': 512},
                {'flush_timeout': 30.0}
            ]
        )
    ])