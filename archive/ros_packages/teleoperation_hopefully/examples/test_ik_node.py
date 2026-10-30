#!/usr/bin/env python3
"""
Test script for MoveIt2 IK Node.

This script demonstrates how to:
1. Publish end-effector poses to /rx150/ee_pose
2. Listen for computed joint commands
3. Verify IK results

Usage:
    ros2 run teleoperation_hopefully test_ik_node.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from interbotix_xs_msgs.msg import JointGroupCommand
import math
import time


class IKTestNode(Node):
    """Test node for IK computation."""
    
    def __init__(self):
        super().__init__('ik_test_node')
        
        # Publisher for target poses
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/rx150/ee_pose',
            qos_profile=10
        )
        
        # Subscriber for joint commands
        self.command_subscriber = self.create_subscription(
            JointGroupCommand,
            '/rx150/commands/joint_group',
            self.command_callback,
            qos_profile=10
        )
        
        self.get_logger().info("IK Test Node initialized")
        self.last_command = None
        
    def command_callback(self, msg: JointGroupCommand) -> None:
        """Callback for computed joint commands."""
        self.last_command = msg
        self.get_logger().info(
            f"Received command: {[f'{q:.3f}' for q in msg.cmd]}"
        )
    
    def publish_pose(self, x: float, y: float, z: float,
                    qx: float = 0.0, qy: float = 0.0, 
                    qz: float = 0.0, qw: float = 1.0) -> None:
        """
        Publish a target end-effector pose.
        
        Args:
            x, y, z: Position in meters
            qx, qy, qz, qw: Quaternion orientation (default: identity)
        """
        msg = PoseStamped()
        msg.header.frame_id = 'rx150/base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        
        self.get_logger().info(
            f"Publishing pose: pos=({x:.3f}, {y:.3f}, {z:.3f}), "
            f"quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})"
        )
        
        self.pose_publisher.publish(msg)
    
    def quaternion_from_euler(self, roll: float, pitch: float, 
                             yaw: float) -> tuple:
        """
        Convert Euler angles to quaternion (x, y, z, w).
        
        Args:
            roll, pitch, yaw: Angles in radians
            
        Returns:
            Tuple of (x, y, z, w)
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return (qx, qy, qz, qw)
    
    def test_case_1_identity_orientation(self):
        """Test 1: Simple pose with identity orientation."""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("Test 1: Identity Orientation")
        self.get_logger().info("="*50)
        
        # Publish pose
        self.publish_pose(
            x=0.3,
            y=0.1,
            z=0.2,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=1.0
        )
        
        # Wait for computation
        time.sleep(1.0)
        
        if self.last_command:
            self.get_logger().info("✓ Test 1 PASSED")
        else:
            self.get_logger().warn("✗ Test 1 FAILED - No command received")
    
    def test_case_2_rotated_orientation(self):
        """Test 2: Pose with 90-degree rotation around Y axis."""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("Test 2: Rotated Orientation (90° around Y)")
        self.get_logger().info("="*50)
        
        # Convert Euler angles to quaternion
        qx, qy, qz, qw = self.quaternion_from_euler(0, math.pi/2, 0)
        
        # Publish pose
        self.publish_pose(
            x=0.25,
            y=0.15,
            z=0.25,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw
        )
        
        # Wait for computation
        time.sleep(1.0)
        
        if self.last_command:
            self.get_logger().info("✓ Test 2 PASSED")
        else:
            self.get_logger().warn("✗ Test 2 FAILED - No command received")
    
    def test_case_3_unreachable_pose(self):
        """Test 3: Unreachable pose (should fail gracefully)."""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("Test 3: Unreachable Pose")
        self.get_logger().info("="*50)
        
        # Publish unreachable pose (too far away)
        self.publish_pose(
            x=2.0,
            y=2.0,
            z=2.0,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=1.0
        )
        
        # Wait for computation
        time.sleep(1.0)
        
        self.get_logger().info("Test 3: Observe error handling in logs")
    
    def test_case_4_trajectory(self):
        """Test 4: Generate a small trajectory."""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("Test 4: Trajectory (3 waypoints)")
        self.get_logger().info("="*50)
        
        waypoints = [
            (0.3, 0.0, 0.2),
            (0.3, 0.1, 0.2),
            (0.3, 0.2, 0.2),
        ]
        
        for i, (x, y, z) in enumerate(waypoints):
            self.get_logger().info(f"  Waypoint {i+1}: ({x}, {y}, {z})")
            self.publish_pose(x, y, z)
            time.sleep(1.0)
        
        self.get_logger().info("✓ Test 4 completed")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = IKTestNode()
    
    # Give node time to initialize
    time.sleep(1.0)
    
    try:
        # Run tests
        node.test_case_1_identity_orientation()
        time.sleep(2.0)
        
        node.test_case_2_rotated_orientation()
        time.sleep(2.0)
        
        node.test_case_3_unreachable_pose()
        time.sleep(2.0)
        
        node.test_case_4_trajectory()
        time.sleep(1.0)
        
        node.get_logger().info("\n" + "="*50)
        node.get_logger().info("All tests completed!")
        node.get_logger().info("="*50)
        
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
