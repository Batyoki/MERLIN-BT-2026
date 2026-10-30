#!/usr/bin/env python3
"""
FK Publisher Node for ReactorX-150
Subscribes to ReactorX-150 joint states, computes FK, and publishes end-effector pose.
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
    print("Warning: PyMoveIt2 not available. FK computation will not work.")


class ReactorFKPublisher(Node):
    """
    ROS2 node that:
    1. Subscribes to ReactorX-150 joint states
    2. Computes FK to get end-effector pose
    3. Publishes the pose on /rx150/ee_pose
    """

    def __init__(self):
        super().__init__('reactor_fk_publisher')

        # Parameters
        self.declare_parameter('robot_name', 'rx150')
        self.declare_parameter('publish_rate', 100.0)  # Hz

        self.robot_name = self.get_parameter('robot_name').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        # ReactorX-150 joint names (5 DOF)
        self.reactor_joints = [
            "waist",
            "shoulder",
            "elbow",
            "wrist_angle",
            "wrist_rotate"
        ]

        # Thread-safe storage for joint state
        self.joint_state_mutex = threading.Lock()
        self.latest_joint_state = None
        self.joint_state_received = False

        self.get_logger().info(f"Initializing FK Publisher for {self.robot_name}...")

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            f'/{self.robot_name}/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for end-effector pose
        self.ee_pose_pub = self.create_publisher(
            PoseStamped,
            f'/{self.robot_name}/ee_pose',
            10
        )

        # Initialize MoveIt2 for ReactorX-150 if available
        self.reactor_moveit = None
        if _HAS_PYMOVEIT2:
            try:
                self.reactor_moveit = MoveIt2(
                    node=self,
                    joint_names=self.reactor_joints,
                    base_link_name=f"{self.robot_name}/base_link",
                    end_effector_name=f"{self.robot_name}/ee_gripper_link",
                    group_name="interbotix_arm",
                )
                self.get_logger().info("MoveIt2 interface initialized for ReactorX-150")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize MoveIt2: {e}")
                self.reactor_moveit = None
        else:
            self.get_logger().warn("PyMoveIt2 not available, FK computation disabled")

        # Timer for publishing FK results
        if self.reactor_moveit is not None:
            timer_period = 1.0 / self.publish_rate
            self.timer = self.create_timer(timer_period, self.publish_fk)
            self.get_logger().info(f"FK Publisher started at {self.publish_rate} Hz")
        else:
            self.get_logger().error("Cannot start FK publisher without MoveIt2")

    def joint_state_callback(self, msg: JointState):
        """Callback for receiving joint states"""
        with self.joint_state_mutex:
            self.latest_joint_state = msg
            self.get_logger().info(f"Received joint state for FK computation. {msg}")
            self.joint_state_received = True

    def publish_fk(self):
        """Compute FK and publish end-effector pose"""
        if self.reactor_moveit is None:
            return

        # Get latest joint state
        with self.joint_state_mutex:
            if not self.joint_state_received:
                return
            joint_state = self.latest_joint_state

        if joint_state is None:
            return
        self.get_logger().info("Received joint state for FK computation.")
        try:
            # Extract joint positions for the reactor joints
            joint_positions = []
            for joint_name in self.reactor_joints:
                if joint_name in joint_state.name:
                    idx = joint_state.name.index(joint_name)
                    joint_positions.append(joint_state.position[idx])
                else:
                    self.get_logger().warn(f"Joint {joint_name} not found in joint state")
                    return

            # Compute FK
            fk_result = self.reactor_moveit.compute_fk(joint_state=joint_positions)

            self.get_logger().info("FK computation completed.")

            if fk_result is not None:
                # Publish the pose
                self.ee_pose_pub.publish(fk_result)
                
                # Log occasionally (every 2 seconds)
                if not hasattr(self, '_last_log_time'):
                    self._last_log_time = self.get_clock().now()
                    self._log_counter = 0
                
                self._log_counter += 1
                if self._log_counter >= (self.publish_rate * 2.0):  # Every 2 seconds
                    self.get_logger().info(
                        f"Publishing EE pose: [{fk_result.pose.position.x:.3f}, "
                        f"{fk_result.pose.position.y:.3f}, "
                        f"{fk_result.pose.position.z:.3f}]"
                    )
                    self._log_counter = 0
            else:
                self.get_logger().warn("FK computation failed")

        except Exception as e:
            self.get_logger().error(f"Error in FK computation: {e}")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ReactorFKPublisher()
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
