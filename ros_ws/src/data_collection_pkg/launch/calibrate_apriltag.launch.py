from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='data_collection_pkg',
            executable='calibration_node', # The executable wrapping AprilTagCalibrator
            name='apriltag_calibrator',
            output='screen',
            parameters=[
                {'marker_length_m': 0.05},
                {'apriltag_family': 'tag36h11'}
            ]
        )
    ])