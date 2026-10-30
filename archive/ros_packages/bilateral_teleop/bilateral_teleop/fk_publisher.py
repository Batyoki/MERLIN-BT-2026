#!/usr/bin/env python3
"""
FK Publisher Node for ReactorX-150
Subscribes to joint states, computes FK, and publishes end-effector pose
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
except ImportError:
    _HAS_PYMOVEIT2 = False
    print("Warning: PyMoveIt2 not available. FK computation will not work.")


class FKPublisher(Node):
    """
    ROS2 node that:
    1. Subscribes to ReactorX-150 joint states
    2. Computes FK to get end-effector pose
    3. Publishes the pose on /rx150/ee_pose topic
    """
    
    def __init__(self):
        super().__init__('fk_publisher')
        
        # Declare parameters
        self.declare_parameter('robot_name', 'rx150')
        self.declare_parameter('publish_rate', 30.0)  # Hz
        
        robot_name = self.get_parameter('robot_name').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # ReactorX-150 joint names (5 DOF)
        self.joint_names = [
            "waist",
            "shoulder",
            "elbow",
            "wrist_angle",
            "wrist_rotate"
        ]
        
        # Thread-safe storage for joint states
        self.joint_state_mutex = threading.Lock()
        self.current_joint_state = None
        self.new_joint_state_available = False
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            f'/{robot_name}/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher for end-effector pose
        self.ee_pose_pub = self.create_publisher(
            PoseStamped,
            f'/{robot_name}/ee_pose',
            10
        )
        
        # Initialize MoveIt2 interface if available
        self.moveit2 = None
        if _HAS_PYMOVEIT2:
            self.get_logger().info(f"Initializing MoveIt2 interface for {robot_name}...")
            try:
                self.moveit2 = MoveIt2(
                    node=self,
                    joint_names=self.joint_names,
                    base_link_name=f"{robot_name}/base_link",
                    end_effector_name=f"{robot_name}/ee_gripper_link",
                    group_name="interbotix_arm",
                )
                self.get_logger().info("MoveIt2 interface initialized successfully!")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize MoveIt2: {e}")
                self.moveit2 = None
        else:
            self.get_logger().error("PyMoveIt2 not available. Cannot compute FK.")
        
        # Create timer for publishing at fixed rate
        if self.moveit2 is not None:
            timer_period = 1.0 / self.publish_rate
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.get_logger().info(
                f"FK Publisher started for {robot_name} at {self.publish_rate} Hz"
            )
        else:
            self.get_logger().error("FK Publisher cannot start without MoveIt2.")
    
    def joint_state_callback(self, msg: JointState):
        """Callback for receiving joint states"""
        # Extract only the joints we care about, in the correct order
        joint_positions = []
        
        for joint_name in self.joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                joint_positions.append(msg.position[idx])
            else:
                self.get_logger().warn(
                    f"Joint {joint_name} not found in joint state message",
                    throttle_duration_sec=5.0
                )
                return
        
        # Store joint state in thread-safe manner
        with self.joint_state_mutex:
            self.current_joint_state = joint_positions
            self.new_joint_state_available = True
    
    def timer_callback(self):
        """Timer callback to compute FK and publish pose"""
        # Get current joint state
        with self.joint_state_mutex:
            if not self.new_joint_state_available or self.current_joint_state is None:
                return
            
            joint_state = self.current_joint_state.copy()
            self.new_joint_state_available = False
        
        # Compute FK
        try:
            fk_result = self.moveit2.compute_fk(joint_state=joint_state)
            
            if fk_result is not None:
                # Publish the pose
                self.ee_pose_pub.publish(fk_result)
                
                self.get_logger().debug(
                    f"Published EE pose: pos=({fk_result.pose.position.x:.3f}, "
                    f"{fk_result.pose.position.y:.3f}, {fk_result.pose.position.z:.3f})",
                    throttle_duration_sec=1.0
                )
            else:
                self.get_logger().warn(
                    "FK computation returned None",
                    throttle_duration_sec=5.0
                )
                
        except Exception as e:
            self.get_logger().error(
                f"Error computing FK: {e}",
                throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FKPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Exception in FK Publisher: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
