"""
Example usage of arm-bench modules
"""

# Example 1: Hardware Scanning
from arm_bench.scanner import scan_dynamixel_motors, scan_cameras

# Scan for motors (simulation mode)
motors = scan_dynamixel_motors(simulate=True)
print(f"Found {len(motors)} motors")

# Scan for cameras
cameras = scan_cameras()
print(f"Found {len(cameras)} cameras")


# Example 2: Bimanual Translation
from arm_bench.bimanual import create_translator

translator = create_translator()

# Reactor arm state (5DOF)
reactor_state = {
    "waist": 0.5,
    "shoulder": 1.0,
    "elbow": -0.5,
    "wrist_angle": 0.3,
    "wrist_rotate": 0.0
}

# Translate to Viper (6DOF)
viper_state = translator.reactor_to_viper(reactor_state)
print(f"Viper state: {viper_state}")


# Example 3: RLDS Recording
from arm_bench.rlds_recorder import create_recorder

recorder = create_recorder()

# Start recording
recorder.start_recording("my_demo_episode")

# ... perform robot actions ...
import time
time.sleep(2)  # Simulate some recording time

# Stop recording
recorder.stop_recording()

# List episodes
episodes = recorder.list_episodes()
print(f"Recorded episodes: {episodes}")


# Example 4: Teleoperation (manual control)
from arm_bench.teleoperation import DifferentialIKNode

ik_node = DifferentialIKNode()

# Delta end-effector command
delta_ee = {
    'x': 0.01,
    'y': 0.0,
    'z': 0.005,
    'roll': 0.0,
    'pitch': 0.0,
    'yaw': 0.1
}

# Compute joint velocities
joint_velocities = ik_node.compute_joint_velocities(delta_ee)
print(f"Joint velocities: {joint_velocities}")

# Send to robot
ik_node.send_to_robot(joint_velocities)


# Example 5: Visualization
from arm_bench.visualization import VisualizationLauncher

viz = VisualizationLauncher("simulation")

# This would launch RViz (requires ROS2)
# viz.launch_rviz()


# Example 6: Interbotix Workspace Builder
from arm_bench.interbotix_builder import InterbotixWorkspaceBuilder

builder = InterbotixWorkspaceBuilder()

# Check available robot models
robots = builder.get_available_robots()
print(f"Available robots: {robots}")

# Build workspace (commented out - takes time)
# builder.setup_full_workspace()


# Example 7: Camera Management
from arm_bench.rlds_recorder import create_camera_manager

cam_manager = create_camera_manager()
cameras = cam_manager.scan_cameras()

for cam in cameras:
    print(f"Camera {cam['id']}: {cam['name']}")
    
# Rename a camera
if cameras:
    cam_manager.rename_camera(0, "Front Camera")


print("\nAll examples completed successfully!")
