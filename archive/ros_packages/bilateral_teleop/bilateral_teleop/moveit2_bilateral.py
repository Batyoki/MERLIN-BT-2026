#!/usr/bin/env python3
"""
MoveIt2-Based Bilateral Teleoperation Node

This node implements bilateral teleoperation between ReactorX-150 (5DOF leader)
and ViperX-300s (6DOF follower) using MoveIt2 for FK and IK.

Pipeline:
1. Subscribe to RX150 joint states (leader)
2. Compute FK to get leader end-effector pose (using MoveIt2)
3. Compute IK to find follower joint configuration (using MoveIt2)
4. Publish joint commands to VX300s (follower)

Uses MoveIt2's built-in compute_fk() and compute_ik() services.
"""

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from interbotix_xs_msgs.msg import JointGroupCommand
from typing import Optional, List, Tuple
import threading

from pymoveit2 import MoveIt2, MoveIt2State


class MoveIt2BilateralTeleop(Node):
    """
    ROS2 Node for bilateral teleoperation using MoveIt2.
    
    Maps RX150 (5DOF) leader arm movements to VX300s (6DOF) follower arm.
    """
    
    def __init__(self):
        super().__init__('moveit2_bilateral_teleop')
        
        # Callback group for MoveIt2 services
        self.callback_group = ReentrantCallbackGroup()
        
        # ============ ROS2 Parameters ============
        # Leader (RX150) parameters
        self.declare_parameter('leader.robot_name', 'rx150')
        self.declare_parameter('leader.group_name', 'interbotix_arm')
        self.declare_parameter('leader.base_link', 'rx150/base_link')
        self.declare_parameter('leader.end_effector_link', 'rx150/ee_gripper_link')
        self.declare_parameter('leader.joint_names', [
            'waist',
            'shoulder',
            'elbow',
            'wrist_angle',
            'wrist_rotate'
        ])
        
        # Follower (VX300s) parameters
        self.declare_parameter('follower.robot_name', 'vx300s')
        self.declare_parameter('follower.group_name', 'interbotix_arm')
        self.declare_parameter('follower.base_link', 'vx300s/base_link')
        self.declare_parameter('follower.end_effector_link', 'vx300s/ee_gripper_link')
        self.declare_parameter('follower.joint_names', [
            'waist',
            'shoulder',
            'elbow',
            'forearm_roll',
            'wrist_angle',
            'wrist_rotate'
        ])
        
        # Control parameters
        self.declare_parameter('control_rate', 30.0)  # Hz
        self.declare_parameter('ik_timeout', 0.1)  # seconds
        self.declare_parameter('position_scale', 1.0)  # Scale factor for position
        self.declare_parameter('use_move_group', False)  # Use move_group action
        
        # Get parameters
        self.leader_name = self.get_parameter('leader.robot_name').value
        self.leader_group = self.get_parameter('leader.group_name').value
        self.leader_base_link = self.get_parameter('leader.base_link').value
        self.leader_ee_link = self.get_parameter('leader.end_effector_link').value
        self.leader_joint_names = self.get_parameter('leader.joint_names').value
        
        self.follower_name = self.get_parameter('follower.robot_name').value
        self.follower_group = self.get_parameter('follower.group_name').value
        self.follower_base_link = self.get_parameter('follower.base_link').value
        self.follower_ee_link = self.get_parameter('follower.end_effector_link').value
        self.follower_joint_names = self.get_parameter('follower.joint_names').value
        
        self.control_rate = self.get_parameter('control_rate').value
        self.ik_timeout = self.get_parameter('ik_timeout').value
        self.position_scale = self.get_parameter('position_scale').value
        self.use_move_group = self.get_parameter('use_move_group').value
        
        # Prepend robot name to joint names
        self.leader_joint_names_full = [
            f"{self.leader_name}/{j}" for j in self.leader_joint_names
        ]
        self.follower_joint_names_full = [
            f"{self.follower_name}/{j}" for j in self.follower_joint_names
        ]
        
        # ============ State Variables ============
        self.leader_joint_state: Optional[JointState] = None
        self.follower_joint_state: Optional[JointState] = None
        self.leader_ee_pose: Optional[PoseStamped] = None
        self.last_ik_solution: Optional[List[float]] = None
        self.mutex = threading.Lock()
        
        # ============ MoveIt2 Interfaces ============
        self.get_logger().info("Initializing MoveIt2 for leader (RX150)...")
        self.leader_moveit2 = MoveIt2(
            node=self,
            joint_names=self.leader_joint_names_full,
            base_link_name=self.leader_base_link,
            end_effector_name=self.leader_ee_link,
            group_name=self.leader_group,
            callback_group=self.callback_group,
        )
        
        self.get_logger().info("Initializing MoveIt2 for follower (VX300s)...")
        self.follower_moveit2 = MoveIt2(
            node=self,
            joint_names=self.follower_joint_names_full,
            base_link_name=self.follower_base_link,
            end_effector_name=self.follower_ee_link,
            group_name=self.follower_group,
            callback_group=self.callback_group,
        )
        
        # ============ Subscribers ============
        self.leader_joint_sub = self.create_subscription(
            JointState,
            f'/{self.leader_name}/joint_states',
            self.leader_joint_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.follower_joint_sub = self.create_subscription(
            JointState,
            f'/{self.follower_name}/joint_states',
            self.follower_joint_callback,
            10,
            callback_group=self.callback_group
        )
        
        # ============ Publishers ============
        # Publish leader end-effector pose for debugging
        self.leader_ee_pub = self.create_publisher(
            PoseStamped,
            f'/{self.leader_name}/ee_pose',
            10
        )
        
        # Publish follower joint commands
        self.follower_cmd_pub = self.create_publisher(
            JointGroupCommand,
            f'/{self.follower_name}/commands/joint_group',
            10
        )
        
        # Also publish as JointState for MoveIt2
        self.follower_target_pub = self.create_publisher(
            JointState,
            f'/{self.follower_name}/target_joint_states',
            10
        )
        
        # ============ Control Timer ============
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop,
            callback_group=self.callback_group
        )
        
        # Statistics
        self.fk_success_count = 0
        self.ik_success_count = 0
        self.total_iterations = 0
        
        self.get_logger().info(
            f"\n{'='*60}\n"
            f"MoveIt2 Bilateral Teleoperation Node Initialized\n"
            f"{'='*60}\n"
            f"Leader:  {self.leader_name} (5DOF)\n"
            f"  - Base: {self.leader_base_link}\n"
            f"  - EE:   {self.leader_ee_link}\n"
            f"  - Joints: {self.leader_joint_names}\n"
            f"\nFollower: {self.follower_name} (6DOF)\n"
            f"  - Base: {self.follower_base_link}\n"
            f"  - EE:   {self.follower_ee_link}\n"
            f"  - Joints: {self.follower_joint_names}\n"
            f"\nControl Rate: {self.control_rate} Hz\n"
            f"IK Timeout: {self.ik_timeout} s\n"
            f"{'='*60}"
        )
    
    def leader_joint_callback(self, msg: JointState):
        """Store leader joint state."""
        with self.mutex:
            self.leader_joint_state = msg
    
    def follower_joint_callback(self, msg: JointState):
        """Store follower joint state."""
        with self.mutex:
            self.follower_joint_state = msg
    
    def compute_leader_fk(self) -> Optional[PoseStamped]:
        """
        Compute forward kinematics for the leader arm using MoveIt2.
        Returns end-effector pose.
        """
        with self.mutex:
            if self.leader_joint_state is None:
                return None
            joint_state = self.leader_joint_state
        
        try:
            # Use MoveIt2's compute_fk
            pose = self.leader_moveit2.compute_fk(joint_state=joint_state)
            
            if pose is not None:
                self.fk_success_count += 1
                self.get_logger().debug(
                    f"FK Success: pos=({pose.pose.position.x:.3f}, "
                    f"{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})"
                )
            return pose
            
        except Exception as e:
            self.get_logger().error(f"FK computation error: {e}")
            return None
    
    def compute_follower_ik(
        self, 
        target_pose: PoseStamped
    ) -> Optional[JointState]:
        """
        Compute inverse kinematics for the follower arm using MoveIt2.
        Returns joint configuration.
        """
        try:
            # Get current follower joint state as seed
            with self.mutex:
                seed_state = self.follower_joint_state
            
            # Scale position if needed
            position = target_pose.pose.position
            if self.position_scale != 1.0:
                position = Point(
                    x=position.x * self.position_scale,
                    y=position.y * self.position_scale,
                    z=position.z * self.position_scale
                )
            
            orientation = target_pose.pose.orientation
            
            # Use MoveIt2's compute_ik
            ik_solution = self.follower_moveit2.compute_ik(
                position=position,
                quat_xyzw=(
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w
                ),
                start_joint_state=seed_state,
                wait_for_server_timeout_sec=self.ik_timeout
            )
            
            if ik_solution is not None:
                self.ik_success_count += 1
                self.get_logger().debug(
                    f"IK Success: {[f'{q:.3f}' for q in ik_solution.position[:6]]}"
                )
            return ik_solution
            
        except Exception as e:
            self.get_logger().error(f"IK computation error: {e}")
            return None
    
    def publish_follower_command(self, joint_state: JointState):
        """Publish joint command to follower arm."""
        try:
            # Create JointGroupCommand message
            cmd = JointGroupCommand()
            cmd.name = "arm"
            
            # Extract only the arm joint values (exclude gripper)
            num_joints = len(self.follower_joint_names)
            cmd.cmd = list(joint_state.position[:num_joints])
            
            # Publish command
            self.follower_cmd_pub.publish(cmd)
            
            # Also publish as JointState for debugging
            target_state = JointState()
            target_state.header = Header()
            target_state.header.stamp = self.get_clock().now().to_msg()
            target_state.name = self.follower_joint_names_full
            target_state.position = list(joint_state.position[:num_joints])
            self.follower_target_pub.publish(target_state)
            
            self.get_logger().debug(f"Published command: {cmd.cmd}")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing command: {e}")
    
    def control_loop(self):
        """Main control loop: FK → IK → Publish."""
        self.total_iterations += 1
        
        # Step 1: Compute leader FK
        leader_pose = self.compute_leader_fk()
        if leader_pose is None:
            self.get_logger().debug("No leader FK available")
            return
        
        # Store and publish for debugging
        self.leader_ee_pose = leader_pose
        leader_pose.header.stamp = self.get_clock().now().to_msg()
        self.leader_ee_pub.publish(leader_pose)
        
        # Step 2: Compute follower IK
        follower_joints = self.compute_follower_ik(leader_pose)
        if follower_joints is None:
            self.get_logger().debug("IK failed, using last solution")
            return
        
        # Step 3: Publish follower command
        self.publish_follower_command(follower_joints)
        
        # Store last solution
        self.last_ik_solution = list(follower_joints.position)
        
        # Log statistics periodically
        if self.total_iterations % 100 == 0:
            fk_rate = self.fk_success_count / max(1, self.total_iterations) * 100
            ik_rate = self.ik_success_count / max(1, self.total_iterations) * 100
            self.get_logger().info(
                f"Stats: FK success={fk_rate:.1f}%, IK success={ik_rate:.1f}%"
            )


class MoveIt2BilateralTeleopDirect(Node):
    """
    Direct bilateral teleop using JointGroupCommand without MoveIt2 planning.
    Uses compute_fk and compute_ik only for mapping, no motion planning.
    """
    
    def __init__(self):
        super().__init__('moveit2_bilateral_direct')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Parameters - simplified
        self.declare_parameter('leader_robot', 'rx150')
        self.declare_parameter('follower_robot', 'vx300s')
        self.declare_parameter('rate', 50.0)
        
        self.leader = self.get_parameter('leader_robot').value
        self.follower = self.get_parameter('follower_robot').value
        self.rate = self.get_parameter('rate').value
        
        # Joint configurations
        self.leader_joints = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
        self.follower_joints = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        
        # State
        self.leader_state = None
        self.follower_state = None
        
        # MoveIt2 for leader FK
        self.leader_moveit = MoveIt2(
            node=self,
            joint_names=[f"{self.leader}/{j}" for j in self.leader_joints],
            base_link_name=f"{self.leader}/base_link",
            end_effector_name=f"{self.leader}/ee_gripper_link",
            group_name="interbotix_arm",
            callback_group=self.callback_group,
        )
        
        # MoveIt2 for follower IK
        self.follower_moveit = MoveIt2(
            node=self,
            joint_names=[f"{self.follower}/{j}" for j in self.follower_joints],
            base_link_name=f"{self.follower}/base_link",
            end_effector_name=f"{self.follower}/ee_gripper_link",
            group_name="interbotix_arm",
            callback_group=self.callback_group,
        )
        
        # Subscribers
        self.create_subscription(
            JointState, f'/{self.leader}/joint_states',
            self._leader_cb, 10, callback_group=self.callback_group
        )
        self.create_subscription(
            JointState, f'/{self.follower}/joint_states',
            self._follower_cb, 10, callback_group=self.callback_group
        )
        
        # Publishers
        self.ee_pose_pub = self.create_publisher(PoseStamped, f'/{self.leader}/ee_pose', 10)
        self.cmd_pub = self.create_publisher(JointGroupCommand, f'/{self.follower}/commands/joint_group', 10)
        
        # Timer
        self.create_timer(1.0/self.rate, self._loop, callback_group=self.callback_group)
        
        self.get_logger().info(f"Direct Bilateral: {self.leader} → {self.follower} @ {self.rate}Hz")
    
    def _leader_cb(self, msg): self.leader_state = msg
    def _follower_cb(self, msg): self.follower_state = msg
    
    def _loop(self):
        if self.leader_state is None:
            return
        
        # FK
        pose = self.leader_moveit.compute_fk(joint_state=self.leader_state)
        if pose is None:
            return
        
        self.ee_pose_pub.publish(pose)
        
        # IK
        ik = self.follower_moveit.compute_ik(
            position=pose.pose.position,
            quat_xyzw=(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            ),
            start_joint_state=self.follower_state,
            wait_for_server_timeout_sec=0.1
        )
        
        if ik is None:
            return
        
        # Publish
        cmd = JointGroupCommand()
        cmd.name = "arm"
        cmd.cmd = list(ik.position[:6])
        self.cmd_pub.publish(cmd)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = MoveIt2BilateralTeleop()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            
    finally:
        rclpy.shutdown()


def main_direct(args=None):
    """Entry point for direct control."""
    rclpy.init(args=args)
    
    try:
        node = MoveIt2BilateralTeleopDirect()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
