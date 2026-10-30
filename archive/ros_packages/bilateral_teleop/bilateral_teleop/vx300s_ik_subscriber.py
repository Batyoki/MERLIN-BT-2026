#!/usr/bin/env python3
"""
IK Subscriber Node for ViperX-300s
Subscribes to end-effector pose, computes IK for ViperX-300s, and publishes joint states.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import threading
import sys

# Add the pymoveit2 module to path
sys.path.insert(0, '/home/yash_sai/Yash/Arm/ros_ws/src/pymoveit2')

try:
    from pymoveit2.moveit2 import MoveIt2
    _HAS_PYMOVEIT2 = True
except Exception:
    _HAS_PYMOVEIT2 = False
    print("Warning: PyMoveIt2 not available. IK computation will not work.")


class ViperIKSubscriber(Node):
    """
    ROS2 node that:
    1. Subscribes to end-effector pose (typically from /rx150/ee_pose)
    2. Computes IK for ViperX-300s (6 DOF)
    3. Publishes the resulting joint states to /vx300s/ik_joint_states
    """

    def __init__(self):
        super().__init__('viper_ik_subscriber')

        # Parameters
        self.declare_parameter('robot_name', 'vx300s')
        self.declare_parameter('pose_topic', '/rx150/ee_pose')
        self.declare_parameter('publish_rate', 30.0)  # Hz

        self.robot_name = self.get_parameter('robot_name').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        # ViperX-300s joint names (6 DOF)
        self.viper_joints = [
            "waist",
            "shoulder",
            "elbow",
            "forearm_roll",
            "wrist_angle",
            "wrist_rotate"
        ]

        # Thread-safe storage for pose
        self.pose_mutex = threading.Lock()
        self.latest_pose = None
        self.pose_received = False

        self.get_logger().info(f"Initializing IK Subscriber for {self.robot_name}...")

        # Subscribe to end-effector pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

        # Publisher for computed joint states
        self.joint_state_pub = self.create_publisher(
            JointState,
            f'/{self.robot_name}/ik_joint_states',
            10
        )

        # Initialize MoveIt2 for ViperX-300s if available
        self.viper_moveit = None
        if _HAS_PYMOVEIT2:
            try:
                self.viper_moveit = MoveIt2(
                    node=self,
                    joint_names=self.viper_joints,
                    base_link_name=f"{self.robot_name}/base_link",
                    end_effector_name=f"{self.robot_name}/ee_gripper_link",
                    group_name="interbotix_arm",
                )
                self.get_logger().info("MoveIt2 interface initialized for ViperX-300s")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize MoveIt2: {e}")
                self.viper_moveit = None
        else:
            self.get_logger().warn("PyMoveIt2 not available, IK computation disabled")

        # Timer for publishing IK results
        if self.viper_moveit is not None:
            timer_period = 1.0 / self.publish_rate
            self.timer = self.create_timer(timer_period, self.publish_ik)
            self.get_logger().info(f"IK Subscriber started at {self.publish_rate} Hz")
            self.get_logger().info(f"Listening to pose topic: {self.pose_topic}")
        else:
            self.get_logger().error("Cannot start IK subscriber without MoveIt2")

    def pose_callback(self, msg: PoseStamped):
        """Callback for receiving end-effector poses"""
        with self.pose_mutex:
            self.latest_pose = msg
            self.pose_received = True

    def publish_ik(self):
        """Compute IK and publish joint states"""
        if self.viper_moveit is None:
            return

        # Get latest pose
        with self.pose_mutex:
            if not self.pose_received:
                return
            pose = self.latest_pose

        if pose is None:
            return

        try:
            # Extract position and orientation
            position = (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z
            )
            quat_xyzw = (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            )

            # Compute IK
            ik_result = self.viper_moveit.compute_ik(
                position=position,
                quat_xyzw=quat_xyzw
            )

            if ik_result is not None:
                # Create and publish JointState message
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.header.frame_id = f"{self.robot_name}/base_link"
                joint_state_msg.name = self.viper_joints
                joint_state_msg.position = list(ik_result.position[:len(self.viper_joints)])
                joint_state_msg.velocity = []
                joint_state_msg.effort = []

                self.joint_state_pub.publish(joint_state_msg)

                # Log occasionally (every 2 seconds)
                if not hasattr(self, '_last_log_time'):
                    self._last_log_time = self.get_clock().now()
                    self._log_counter = 0
                
                self._log_counter += 1
                if self._log_counter >= (self.publish_rate * 2.0):  # Every 2 seconds
                    joint_str = ", ".join([f"{j:.3f}" for j in joint_state_msg.position])
                    self.get_logger().info(f"Publishing IK joints: [{joint_str}]")
                    self._log_counter = 0
            else:
                self.get_logger().warn("IK computation failed")

        except Exception as e:
            self.get_logger().error(f"Error in IK computation: {e}")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ViperIKSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
