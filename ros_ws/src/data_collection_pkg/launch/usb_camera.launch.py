from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='data_collection_pkg',
            executable='usb_camera_node',  # The executable wrapping your CameraStream
            name='usb_camera',
            output='screen',
            parameters=[
                {'device_index': 0},
                {'width': 1920},
                {'height': 1080}
            ]
        )
    ])