# #!/usr/bin/env python3
# """
# MoveIt2 Inverse Kinematics Node for Interbotix RX150 arm.

# This node:
# 1. Subscribes to /rx150/ee_pose topic (PoseStamped format)
# 2. Computes inverse kinematics using MoveIt2
# 3. Publishes computed joint angles to a command topic

# """

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped, Point, Quaternion
# from sensor_msgs.msg import JointState
# from interbotix_xs_msgs.msg import JointGroupCommand
# import numpy as np
# from typing import Optional, List

# from pymoveit2 import MoveIt2, MoveIt2State
# # from pymoveit2.pymoveit2.utils import enum_to_str
# from pymoveit2 import utils 
# from moveit_msgs.msg import MoveItErrorCodes




# class MoveIt2IKNode(Node):
#     """
#     ROS2 Node for computing Inverse Kinematics using MoveIt2 interface.
#     Subscribes to end effector pose and publishes joint commands.
#     """
    
#     def __init__(self):
#         super().__init__('moveit2_ik_node')
        
#         # ============ ROS2 Parameters ============
#         self.declare_parameter('group_name', 'interbotix_arm')
#         self.declare_parameter('base_link', 'vx300s/base_link')
#         self.declare_parameter('end_effector_link', 'vx300s/ee_gripper_link')
#         self.declare_parameter('joint_names', [
#             'vx300s/waist',
#             'vx300s/shoulder', 
#             'vx300s/elbow',
#             'vx300s/forearm_roll',
#             'vx300s/wrist_angle',
#             'vx300s/wrist_rotate'
#         ])
#         self.declare_parameter('ik_timeout', 1.0)
        
#         # Get parameters
#         self.group_name = self.get_parameter('group_name').value
#         self.base_link = self.get_parameter('base_link').value
#         self.end_effector_link = self.get_parameter('end_effector_link').value
#         self.joint_names = self.get_parameter('joint_names').value
#         self.ik_timeout = self.get_parameter('ik_timeout').value
        
#         # ============ Subscribers ============
#         self.ee_pose_subscription = self.create_subscription(
#             PoseStamped,
#             '/rx150/ee_pose',
#             self.ee_pose_callback,
#             qos_profile=10
#         )
        
#         self.joint_state_subscription = self.create_subscription(
#             JointState,
#             '/vx300s/joint_states',
#             self.joint_state_callback,
#             qos_profile=10
#         )
        
#         # ============ Publishers ============
#         self.joint_command_publisher = self.create_publisher(
#             JointGroupCommand,
#             '/vx300s/commands/joint_group',
#             qos_profile=10
#         )
        
#         # ============ State Variables ============
#         self.current_joint_state: Optional[JointState] = None
#         self.last_computed_ik: Optional[JointState] = None
#         self.ik_computation_success = False
        
#         # ============ MoveIt2 Interface ============
#         self.moveit2 = MoveIt2(
#             node=self,
#             joint_names=self.joint_names,
#             base_link_name=self.base_link,
#             end_effector_name=self.end_effector_link,
#             group_name=self.group_name,
#         )

#         home_command = JointGroupCommand()
#         home_command.name="arm"
#         home_command.cmd = np.array([0,0,0,0,0,0],dtype=float).tolist()
#         self.joint_command_publisher.publish(home_command)
        
#         self.get_logger().info(
#             f"MoveIt2 IK Node initialized:\n"
#             f"  Group: {self.group_name}\n"
#             f"  Base link: {self.base_link}\n"
#             f"  EE link: {self.end_effector_link}\n"
#             f"  Joints: {self.joint_names}"
#         )
    
#     def ee_pose_callback(self, msg: PoseStamped) -> None:
#         """
#         Callback for end effector pose subscription.
#         Computes IK and publishes joint commands.
#         """
#         self.get_logger().debug(
#             f"Received EE pose: position=({msg.pose.position.x:.3f}, "
#             f"{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})"
#         )
        
#         # Extract position and orientation
#         position = msg.pose.position
#         orientation = msg.pose.orientation
        
#         # Compute IK
#         solution = self.compute_inverse_kinematics(
#             position=position,
#             orientation=orientation,
#             frame_id=msg.header.frame_id
#         )
        
#         if solution is not None:
#             self.ik_computation_success = True
#             self.last_computed_ik = solution
            
#             # Publish joint commands
#             self.publish_joint_command(solution)
            
#             self.get_logger().info(
#                 f"IK successful! Joint angles: {[f'{q:.3f}' for q in solution.position]}"
#             )
#         else:
#             self.ik_computation_success = False
#             self.get_logger().warn("IK computation failed for the given pose")
    
#     def joint_state_callback(self, msg: JointState) -> None:
#         """
#         Callback for joint state subscription.
#         Stores current joint configuration for use as seed in IK.
#         """
#         self.current_joint_state = msg
    
#     def compute_inverse_kinematics(
#         self,
#         position,
#         orientation,
#         frame_id: Optional[str] = None,
#         seed_joint_state: Optional[List[float]] = None
#     ) -> Optional[JointState]:
#         """
#         Compute inverse kinematics for the given pose.
        
#         Args:
#             position: geometry_msgs.msg.Point with target position
#             orientation: geometry_msgs.msg.Quaternion with target orientation [x, y, z, w]
#             frame_id: frame reference (defaults to base_link)
#             seed_joint_state: Optional seed configuration for IK solver
            
#         Returns:
#             JointState with computed joint angles, or None if IK fails
#         """
        
#         # Use current joint state as seed if not provided
#         if seed_joint_state is None and self.current_joint_state is not None:
#             seed_joint_state = list(self.current_joint_state.position)
        
#         # Use default frame if not specified
#         if frame_id is None:
#             frame_id = self.base_link
        
#         try:
#             # Call MoveIt2 IK service
#             ik_solution = self.moveit2.compute_ik(
#                 position=position,
#                 quat_xyzw=(
#                     orientation.x,
#                     orientation.y, 
#                     orientation.z,
#                     orientation.w
#                 ),
#                 ik_link_name=self.end_effector_link,
#                 start_joint_state=seed_joint_state,
#                 wait_for_server_timeout_sec=self.ik_timeout
#             )
            
#             if ik_solution is not None:
#                 return ik_solution
#             else:
#                 self.get_logger().warn("MoveIt2 IK solver returned no solution")
#                 return None
                
#         except Exception as e:
#             self.get_logger().error(f"Exception in IK computation: {str(e)}")
#             return None
    
#     def publish_joint_command(self, joint_state: JointState) -> None:
#         """
#         Publish computed joint angles as a command to the robot.
        
#         Args:
#             joint_state: JointState message with computed joint positions
#         """
#         try:
#             # Create command message
#             cmd = JointGroupCommand()
#             cmd.name = self.group_name
#             cmd.cmd = list(joint_state.position[:len(self.joint_names)])
            
#             # Publish
#             # self.joint_command_publisher.publish(cmd)
#             self.moveit2.move_to_configuration(
#                 joint_positions  = joint_state.position[:len(self.joint_names)]
#             )
            
#             self.get_logger().debug(f"Published joint command: {cmd.cmd}")
            
#         except Exception as e:
#             self.get_logger().error(f"Error publishing joint command: {str(e)}")


# def main(args=None):
#     """Main entry point for the MoveIt2 IK node."""
#     rclpy.init(args=args)
    
#     try:
#         node = MoveIt2IKNode()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
"""
MoveIt2 Inverse Kinematics Node for Interbotix RX150 arm.

This node:
1. Subscribes to /rx150/ee_pose topic (PoseStamped format)
2. Computes inverse kinematics using MoveIt2
3. Moves the arm to the computed joint configuration

"""
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointGroupCommand
from typing import Optional, List

from pymoveit2.moveit2 import MoveIt2, MoveIt2State
from pymoveit2.utils import enum_to_str
from moveit_msgs.msg import MoveItErrorCodes


class MoveIt2IKNode(Node):
    """
    ROS2 Node for computing Inverse Kinematics using MoveIt2 interface.
    Subscribes to end effector pose and moves arm to computed configuration.
    """
    
    def __init__(self):
        super().__init__('moveit2_ik_node')
        
        # ============ ROS2 Parameters ============
        self.declare_parameter('group_name', 'interbotix_arm')
        self.declare_parameter('base_link', 'vx300s/base_link')
        self.declare_parameter('end_effector_link', 'vx300s/ee_gripper_link')
        self.declare_parameter('joint_names', [
            'vx300s/waist',
            'vx300s/shoulder', 
            'vx300s/elbow',
            'vx300s/forearm_roll',
            'vx300s/wrist_angle',
            'vx300s/wrist_rotate'
        ])
        self.declare_parameter('ik_timeout', 5.0)
        self.declare_parameter('use_move_group_action', False)
        
        # Get parameters
        self.group_name = self.get_parameter('group_name').value
        self.base_link = self.get_parameter('base_link').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.joint_names = self.get_parameter('joint_names').value
        self.ik_timeout = self.get_parameter('ik_timeout').value
        use_move_group_action = self.get_parameter('use_move_group_action').value
        
        # ============ Subscribers ============
        self.ee_pose_subscription = self.create_subscription(
            PoseStamped,
            '/rx150/ee_pose',
            self.ee_pose_callback,
            qos_profile=10
        )
        
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/vx300s/joint_states',
            self.joint_state_callback,
            qos_profile=10
        )
        
        # ============ Publishers ============
        self.joint_command_publisher = self.create_publisher(
            JointGroupCommand,
            '/vx300s/commands/joint_group',
            qos_profile=10
        )
        
        # ============ State Variables ============
        self.current_joint_state: Optional[JointState] = None
        self.last_computed_ik: Optional[JointState] = None
        self.ik_computation_success = False
        
        # ============ MoveIt2 Interface ============
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name=self.base_link,
            end_effector_name=self.end_effector_link,
            group_name=self.group_name,
            use_move_group_action=use_move_group_action,
        )

        # Send home command
        self.send_home_command()
        
        self.get_logger().info(
            f"MoveIt2 IK Node initialized:\n"
            f"  Group: {self.group_name}\n"
            f"  Base link: {self.base_link}\n"
            f"  EE link: {self.end_effector_link}\n"
            f"  Joints: {self.joint_names}"
        )
    
    def send_home_command(self) -> None:
        """Send home command to arm."""
        home_command = JointGroupCommand()
        home_command.name = "arm"
        home_command.cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_command_publisher.publish(home_command)
        self.get_logger().info("Home command sent")

    def ee_pose_callback(self, msg: PoseStamped) -> None:
        """
        Callback for end effector pose subscription.
        Computes IK and moves arm to target pose.
        """
        self.get_logger().debug(
            f"Received EE pose: position=({msg.pose.position.x:.3f}, "
            f"{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})"
        )
        
        # Extract position and orientation
        position = msg.pose.position
        orientation = msg.pose.orientation
        
        # Compute IK
        self.get_logger().info("Computing IK...")
        solution = self.compute_inverse_kinematics(
            position=position,
            orientation=orientation,
            frame_id=msg.header.frame_id
        )
        
        if solution is not None:
            self.get_logger().info(
                f"IK Solution found! Joint positions: {list(solution.position)}"
            )
            self.last_computed_ik = solution
            self.ik_computation_success = True
            
            # Move arm to computed configuration
            self.move_to_configuration(solution)
        else:
            self.get_logger().warn("No IK solution found for the given pose")
            self.ik_computation_success = False

    def joint_state_callback(self, msg: JointState) -> None:
        """Callback for joint state subscription."""
        # Store current joint state
        self.current_joint_state = msg

    def compute_inverse_kinematics(
        self,
        position,
        orientation,
        frame_id: Optional[str] = None,
        seed_joint_state: Optional[List[float]] = None
    ) -> Optional[JointState]:
        """
        Compute inverse kinematics for the given pose.
        
        Args:
            position: geometry_msgs.msg.Point or tuple (x, y, z)
            orientation: geometry_msgs.msg.Quaternion or tuple (x, y, z, w)
            frame_id: Reference frame (defaults to base link)
            seed_joint_state: Starting joint configuration for IK solver
            
        Returns:
            JointState with IK solution or None if no solution found
        """
        
        # Use current joint state as seed if not provided
        if seed_joint_state is None and self.current_joint_state is not None:
            seed_joint_state = list(self.current_joint_state.position)
        
        if frame_id is None:
            frame_id = self.base_link
        
        try:
            self.get_logger().debug("Computing IK...")
            
            # Call MoveIt2 IK service
            ik_solution = self.moveit2.compute_ik(
                position=(position.x, position.y, position.z),
                quat_xyzw=(
                    orientation.x,
                    orientation.y, 
                    orientation.z,
                    orientation.w
                ),
                ik_link_name=self.end_effector_link,
                start_joint_state=seed_joint_state,
                wait_for_server_timeout_sec=self.ik_timeout
            )

            self.get_logger().info("Computed IK...")
            
            if ik_solution is not None:
                self.get_logger().info("IK computation successful")
                return ik_solution
            else:
                self.get_logger().warn("MoveIt2 IK solver returned no solution")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Exception in IK computation: {str(e)}")
            return None

    def move_to_configuration(self, joint_state: JointState) -> None:
        """
        Move the arm to the specified joint configuration.
        
        Args:
            joint_state: Target joint configuration (JointState message)
        """
        try:
            self.get_logger().info("Moving arm to computed configuration...")
            
            # Extract joint positions from the solution
            joint_positions = list(joint_state.position)
            
            # Use MoveIt2 to move to configuration
            self.moveit2.move_to_configuration(
                joint_positions=joint_positions,
                joint_names=self.joint_names,
                tolerance=0.001,
                weight=1.0
            )
            
            # Wait for execution to complete
            success = self.moveit2.wait_until_executed()
            
            if success:
                self.get_logger().info("✓ Arm moved successfully to target configuration")
            else:
                self.get_logger().warn("✗ Failed to move arm to target configuration")
                error_code = self.moveit2.get_last_execution_error_code()
                if error_code:
                    self.get_logger().warn(f"Error code: {enum_to_str(MoveItErrorCodes, error_code.val)}")
                    
        except Exception as e:
            self.get_logger().error(f"Exception while moving arm: {str(e)}")


def main(args=None):
    """Main entry point for the MoveIt2 IK node."""
    rclpy.init(args=args)
    
    node = MoveIt2IKNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down IK node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()