#!/usr/bin/env python3
"""
Direct Bilateral Teleoperation Node

This node implements direct bilateral teleoperation between ReactorX-150 (5DOF leader)
and ViperX-300s (6DOF follower) using direct joint mapping.

Since RX150 has 5DOF and VX300s has 6DOF, we map:
- Leader waist -> Follower waist
- Leader shoulder -> Follower shoulder  
- Leader elbow -> Follower elbow
- Leader wrist_angle -> Follower wrist_angle
- Leader wrist_rotate -> Follower wrist_rotate
- Follower forearm_roll is set to 0 (extra DOF)

For end-effector pose matching, we use the existing FK/IK nodes (reactor_fk + viper_ik).
"""

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from interbotix_xs_msgs.msg import JointGroupCommand
from typing import Optional, List
import threading
try:
    from pymoveit2.moveit2 import MoveIt2
    from geometry_msgs.msg import Point, Quaternion
    _HAS_PYMOVEIT2 = True
except Exception:
    MoveIt2 = None
    _HAS_PYMOVEIT2 = False


class DirectBilateralTeleop(Node):
    """
    Direct bilateral teleoperation with joint mapping.
    """
    
    def __init__(self):
        super().__init__('direct_bilateral_teleop')
        
        # Parameters
        self.declare_parameter('leader_robot', 'rx150')
        self.declare_parameter('follower_robot', 'vx300s')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('mode', 'joint')  # 'joint' or 'pose'
        
        self.leader = self.get_parameter('leader_robot').value
        self.follower = self.get_parameter('follower_robot').value
        self.rate = self.get_parameter('rate').value
        self.mode = self.get_parameter('mode').value
        
        # Joint names
        self.leader_joints = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
        self.follower_joints = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        
        # State
        self.leader_state: Optional[JointState] = None
        self.leader_pose: Optional[Pose] = None
        self.mutex = threading.Lock()
        
        # Subscribers
        self.leader_joint_sub = self.create_subscription(
            JointState,
            f'/{self.leader}/joint_states',
            self._leader_joint_cb,
            10
        )
        
        # If using pose mode, subscribe to FK output
        if self.mode == 'pose':
            self.leader_pose_sub = self.create_subscription(
                Pose,
                f'/{self.leader}/end_effector_pose',  # From reactor_fk node
                self._leader_pose_cb,
                10
            )
            # Initialize MoveIt2 wrappers for FK/IK if available
            if _HAS_PYMOVEIT2:
                # default base and ee link names (can be overridden later)
                leader_base = f'{self.leader}/base_link'
                leader_ee = f'{self.leader}/ee_arm_link'
                follower_base = f'{self.follower}/base_link'
                follower_ee = f'{self.follower}/ee_arm_link'

                try:
                    self.leader_moveit = MoveIt2(self, joint_names=self.leader_joints, base_link_name=leader_base, end_effector_name=leader_ee, group_name='arm')
                    self.follower_moveit = MoveIt2(self, joint_names=self.follower_joints, base_link_name=follower_base, end_effector_name=follower_ee, group_name='arm')
                except Exception as e:
                    self.get_logger().warn(f'Failed to initialize MoveIt2 wrappers: {e}')
                    self.leader_moveit = None
                    self.follower_moveit = None
            else:
                self.leader_moveit = None
                self.follower_moveit = None
        
        # Publishers
        self.follower_cmd_pub = self.create_publisher(
            JointGroupCommand,
            f'/{self.follower}/commands/joint_group',
            10
        )
        
        self.follower_joint_pub = self.create_publisher(
            JointState,
            f'/{self.follower}/target_joint_states',
            10
        )
        
        # If using pose mode, publish pose for IK
        if self.mode == 'pose':
            self.target_pose_pub = self.create_publisher(
                PoseStamped,
                f'/{self.leader}/ee_pose',  # For viper_ik node
                10
            )
        
        # Control timer
        self.create_timer(1.0 / self.rate, self._control_loop)
        
        self.get_logger().info(
            f"\n{'='*60}\n"
            f"Direct Bilateral Teleoperation Initialized\n"
            f"{'='*60}\n"
            f"Leader:  {self.leader} (5DOF)\n"
            f"Follower: {self.follower} (6DOF)\n"
            f"Mode: {self.mode}\n"
            f"Rate: {self.rate} Hz\n"
            f"{'='*60}"
        )
    
    def _leader_joint_cb(self, msg: JointState):
        with self.mutex:
            self.leader_state = msg
    
    def _leader_pose_cb(self, msg: Pose):
        with self.mutex:
            self.leader_pose = msg
    
    def _control_loop(self):
        """Main control loop."""
        if self.mode == 'joint':
            self._joint_mapping_control()
        else:
            self._pose_mapping_control()
    
    def _joint_mapping_control(self):
        """Direct joint-to-joint mapping."""
        with self.mutex:
            if self.leader_state is None:
                return
            leader_state = self.leader_state
        
        # Extract leader joint positions
        try:
            leader_positions = self._extract_joint_positions(
                leader_state, 
                self.leader, 
                self.leader_joints
            )
            if leader_positions is None:
                return
        except Exception as e:
            self.get_logger().error(f"Error extracting leader joints: {e}")
            return
        
        # Map to follower joints (5DOF -> 6DOF)
        # Insert 0.0 for forearm_roll (index 3)
        follower_positions = [
            leader_positions[0],  # waist
            leader_positions[1],  # shoulder
            leader_positions[2],  # elbow
            0.0,                   # forearm_roll (extra DOF)
            leader_positions[3],  # wrist_angle
            leader_positions[4],  # wrist_rotate
        ]
        
        # Publish follower command
        cmd = JointGroupCommand()
        cmd.name = "arm"
        cmd.cmd = follower_positions
        self.follower_cmd_pub.publish(cmd)
        
        self.get_logger().debug(f"Leader: {leader_positions} -> Follower: {follower_positions}")
    
    def _pose_mapping_control(self):
        """Pose-based mapping using FK/IK nodes."""
        # Prefer computing FK from leader and using MoveIt2 to move follower
        with self.mutex:
            if self.leader_state is None:
                return
            leader_state = self.leader_state

        # If MoveIt2 is available, compute FK on leader and command follower via move_to_pose
        if getattr(self, 'leader_moveit', None) is not None and getattr(self, 'follower_moveit', None) is not None:
            try:
                # Compute FK for leader end-effector
                fk_result = self.leader_moveit.compute_fk(joint_state=leader_state, fk_link_names=[f'{self.leader}/ee_arm_link'])
                if fk_result is None:
                    self.get_logger().warn('FK result is None')
                    return

                # fk_result can be a PoseStamped or list
                if isinstance(fk_result, list):
                    pose_stamped = fk_result[0]
                elif isinstance(fk_result, PoseStamped):
                    pose_stamped = fk_result
                else:
                    # try to build PoseStamped if plain Pose returned
                    if isinstance(fk_result, Pose):
                        pose_stamped = PoseStamped()
                        pose_stamped.header.stamp = self.get_clock().now().to_msg()
                        pose_stamped.header.frame_id = f'{self.leader}/base_link'
                        pose_stamped.pose = fk_result
                    else:
                        self.get_logger().warn(f'Unexpected FK result type: {type(fk_result)}')
                        return

                # Command follower to move to that pose
                try:
                    self.follower_moveit.move_to_pose(pose=pose_stamped)
                except Exception as e:
                    self.get_logger().error(f'Failed to command follower MoveIt2: {e}')
            except Exception as e:
                self.get_logger().error(f'Error during FK/IK pose mapping: {e}')
            return

        # Fallback: publish pose for IK node (existing behavior)
        with self.mutex:
            if self.leader_pose is None:
                return
            pose = self.leader_pose

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = f'{self.leader}/base_link'
        pose_stamped.pose = pose
        self.target_pose_pub.publish(pose_stamped)
    
    def _extract_joint_positions(
        self, 
        joint_state: JointState, 
        robot_name: str,
        joint_names: List[str]
    ) -> Optional[List[float]]:
        """Extract joint positions from JointState message."""
        positions = []
        for joint in joint_names:
            # Try with and without namespace prefix
            full_name = f"{robot_name}/{joint}"
            try:
                if full_name in joint_state.name:
                    idx = joint_state.name.index(full_name)
                elif joint in joint_state.name:
                    idx = joint_state.name.index(joint)
                else:
                    self.get_logger().warn(f"Joint {joint} not found in state")
                    return None
                positions.append(joint_state.position[idx])
            except (ValueError, IndexError) as e:
                self.get_logger().warn(f"Error getting joint {joint}: {e}")
                return None
        return positions


class ScaledBilateralTeleop(DirectBilateralTeleop):
    """
    Bilateral teleoperation with joint scaling.
    Accounts for different joint limits between robots.
    """
    
    def __init__(self):
        super().__init__()
        
        # RX150 joint limits (radians)
        self.leader_limits = {
            'waist': (-3.14, 3.14),
            'shoulder': (-1.88, 1.99),
            'elbow': (-2.15, 1.61),
            'wrist_angle': (-1.75, 2.15),
            'wrist_rotate': (-3.14, 3.14),
        }
        
        # VX300s joint limits (radians)
        self.follower_limits = {
            'waist': (-3.14, 3.14),
            'shoulder': (-1.85, 1.99),
            'elbow': (-2.15, 1.61),
            'forearm_roll': (-3.14, 3.14),
            'wrist_angle': (-1.75, 2.15),
            'wrist_rotate': (-3.14, 3.14),
        }
    
    def _scale_joint(self, value: float, from_limits: tuple, to_limits: tuple) -> float:
        """Scale a joint value from one limit range to another."""
        from_min, from_max = from_limits
        to_min, to_max = to_limits
        
        # Normalize to [0, 1]
        normalized = (value - from_min) / (from_max - from_min)
        # Scale to target range
        scaled = to_min + normalized * (to_max - to_min)
        # Clamp to target limits
        return max(to_min, min(to_max, scaled))
    
    def _joint_mapping_control(self):
        """Direct joint-to-joint mapping with scaling."""
        with self.mutex:
            if self.leader_state is None:
                return
            leader_state = self.leader_state
        
        try:
            leader_positions = self._extract_joint_positions(
                leader_state, 
                self.leader, 
                self.leader_joints
            )
            if leader_positions is None:
                return
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            return
        
        # Scale and map joints
        follower_positions = []
        for i, (leader_joint, follower_joint) in enumerate([
            ('waist', 'waist'),
            ('shoulder', 'shoulder'),
            ('elbow', 'elbow'),
        ]):
            scaled = self._scale_joint(
                leader_positions[i],
                self.leader_limits[leader_joint],
                self.follower_limits[follower_joint]
            )
            follower_positions.append(scaled)
        
        # Extra DOF: forearm_roll = 0
        follower_positions.append(0.0)
        
        # Remaining joints
        for i, (leader_joint, follower_joint) in enumerate([
            ('wrist_angle', 'wrist_angle'),
            ('wrist_rotate', 'wrist_rotate'),
        ], start=3):
            scaled = self._scale_joint(
                leader_positions[i],
                self.leader_limits[leader_joint],
                self.follower_limits[follower_joint]
            )
            follower_positions.append(scaled)
        
        # Publish
        cmd = JointGroupCommand()
        cmd.name = "arm"
        cmd.cmd = follower_positions
        self.follower_cmd_pub.publish(cmd)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = DirectBilateralTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


def main_scaled(args=None):
    """Entry point for scaled control."""
    rclpy.init(args=args)
    
    try:
        node = ScaledBilateralTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
