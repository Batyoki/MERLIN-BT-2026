"""
ROS2 Bridge for arm-bench WebXR Teleoperation
Connects the WebXR server with ROS2 nodes for bilateral control

This module provides:
- Publishing WebXR pose data to ROS2 topics
- Subscribing to robot state topics
- Bilateral control via ROS2
"""
import json
import threading
import time
from typing import Any, Dict, Optional, Callable
import numpy as np

# Try to import ROS2
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Pose, Twist
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Header
    ROS2_AVAILABLE = True
except ImportError:
    rclpy = None
    Node = None
    SingleThreadedExecutor = None
    Pose = Any
    Twist = Any
    JointState = Any
    Bool = Any
    Header = Any

# Try to import Interbotix messages
INTERBOTIX_AVAILABLE = False
try:
    from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
    INTERBOTIX_AVAILABLE = True
except ImportError:
    JointGroupCommand = Any
    JointSingleCommand = Any


class WebXRControlMessage:
    """Custom message for WebXR control data"""
    def __init__(self):
        self.header = None
        self.pose = None
        self.gripper_open = False
        self.move_enabled = False


class WebXRROS2Bridge:
    """
    Bridge between WebXR server and ROS2
    
    Publishes WebXR pose data to ROS2 and subscribes to robot state.
    """
    
    def __init__(self, node_name: str = "webxr_ros2_bridge"):
        """
        Initialize the ROS2 bridge
        
        Args:
            node_name: Name for the ROS2 node
        """
        if not ROS2_AVAILABLE:
            raise ImportError("ROS2 not available. Please source your ROS2 workspace.")
        
        self.node_name = node_name
        self.node: Optional[Node] = None
        self.executor: Optional[SingleThreadedExecutor] = None
        self.spin_thread: Optional[threading.Thread] = None
        self.running = False
        
        # Publishers
        self.webxr_pose_pub = None
        self.webxr_delta_pub = None
        self.gripper_cmd_pub = None
        self.follower_cmd_pub = None
        
        # Subscribers
        self.leader_state_sub = None
        self.follower_state_sub = None
        
        # State
        self.leader_joint_state: Optional[JointState] = None
        self.follower_joint_state: Optional[JointState] = None
        self.latest_webxr_pose: Optional[Dict] = None
        self.reference_pose: Optional[Dict] = None
        
        # Callbacks
        self.state_callback: Optional[Callable] = None
        
    def start(self):
        """Start the ROS2 bridge"""
        if self.running:
            return
        
        print("Starting WebXR ROS2 Bridge...")
        
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
        
        # Create node
        self.node = Node(self.node_name)
        
        # Create publishers
        self.webxr_pose_pub = self.node.create_publisher(
            Pose,
            '/webxr/pose',
            10
        )
        
        self.webxr_delta_pub = self.node.create_publisher(
            Twist,
            '/webxr/delta_ee',
            10
        )
        
        self.gripper_cmd_pub = self.node.create_publisher(
            Bool,
            '/webxr/gripper_open',
            10
        )
        
        # Interbotix-specific publishers
        if INTERBOTIX_AVAILABLE:
            self.follower_cmd_pub = self.node.create_publisher(
                JointGroupCommand,
                '/vx300s/commands/joint_group',
                10
            )
        
        # Create subscribers
        self.leader_state_sub = self.node.create_subscription(
            JointState,
            '/rx150/joint_states',
            self._leader_state_callback,
            10
        )
        
        self.follower_state_sub = self.node.create_subscription(
            JointState,
            '/vx300s/joint_states',
            self._follower_state_callback,
            10
        )
        
        # Create executor
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
        # Start spin thread
        self.running = True
        self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.spin_thread.start()
        
        self.node.get_logger().info("WebXR ROS2 Bridge started")
        
    def stop(self):
        """Stop the ROS2 bridge"""
        self.running = False
        
        if self.spin_thread:
            self.spin_thread.join(timeout=2.0)
            self.spin_thread = None
        
        if self.executor:
            self.executor.shutdown()
            self.executor = None
        
        if self.node:
            self.node.destroy_node()
            self.node = None
        
        print("WebXR ROS2 Bridge stopped")
        
    def _spin_loop(self):
        """ROS2 spin loop"""
        while self.running and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.01)
            
    def _leader_state_callback(self, msg: JointState):
        """Handle leader joint state"""
        self.leader_joint_state = msg
        
    def _follower_state_callback(self, msg: JointState):
        """Handle follower joint state"""
        self.follower_joint_state = msg
        
    def publish_webxr_pose(self, pose_data: Dict):
        """
        Publish WebXR pose data to ROS2 topics
        
        Args:
            pose_data: Dictionary with 'position', 'orientation', 'control' keys
        """
        if not self.running or self.node is None:
            return
        
        self.latest_webxr_pose = pose_data
        
        try:
            # Publish pose
            pose_msg = Pose()
            
            position = pose_data.get('position', {})
            pose_msg.position.x = float(position.get('x', 0.0))
            pose_msg.position.y = float(position.get('y', 0.0))
            pose_msg.position.z = float(position.get('z', 0.0))
            
            orientation = pose_data.get('orientation', {})
            pose_msg.orientation.x = float(orientation.get('x', 0.0))
            pose_msg.orientation.y = float(orientation.get('y', 0.0))
            pose_msg.orientation.z = float(orientation.get('z', 0.0))
            pose_msg.orientation.w = float(orientation.get('w', 1.0))
            
            self.webxr_pose_pub.publish(pose_msg)
            
            # Compute and publish delta
            delta = self._compute_delta(pose_data)
            if delta is not None:
                delta_msg = Twist()
                delta_msg.linear.x = delta[0]
                delta_msg.linear.y = delta[1]
                delta_msg.linear.z = delta[2]
                delta_msg.angular.x = delta[3]
                delta_msg.angular.y = delta[4]
                delta_msg.angular.z = delta[5]
                self.webxr_delta_pub.publish(delta_msg)
            
            # Publish gripper state
            control = pose_data.get('control', {})
            gripper_msg = Bool()
            gripper_msg.data = control.get('gripperOpen', False)
            self.gripper_cmd_pub.publish(gripper_msg)
            
            # Log
            self.node.get_logger().debug(
                f"Published pose: ({pose_msg.position.x:.3f}, "
                f"{pose_msg.position.y:.3f}, {pose_msg.position.z:.3f})"
            )
            
        except Exception as e:
            self.node.get_logger().error(f"Error publishing pose: {e}")
            
    def _compute_delta(self, current_pose: Dict) -> Optional[np.ndarray]:
        """Compute delta from reference pose"""
        if self.reference_pose is None:
            self.reference_pose = current_pose.copy()
            return None
        
        # Only compute delta if move is enabled
        control = current_pose.get('control', {})
        if not control.get('moveEnabled', False):
            self.reference_pose = current_pose.copy()
            return None
        
        current_pos = current_pose.get('position', {})
        ref_pos = self.reference_pose.get('position', {})
        
        delta = np.array([
            float(current_pos.get('x', 0)) - float(ref_pos.get('x', 0)),
            float(current_pos.get('y', 0)) - float(ref_pos.get('y', 0)),
            float(current_pos.get('z', 0)) - float(ref_pos.get('z', 0)),
            0.0, 0.0, 0.0  # Angular velocity would need quaternion diff
        ])
        
        self.reference_pose = current_pose.copy()
        
        return delta
        
    def publish_follower_command(self, joint_positions: np.ndarray):
        """
        Publish joint command to follower arm
        
        Args:
            joint_positions: Joint positions in radians
        """
        if not INTERBOTIX_AVAILABLE or self.follower_cmd_pub is None:
            return
        
        msg = JointGroupCommand()
        msg.name = "arm"
        msg.cmd = joint_positions.tolist()
        
        self.follower_cmd_pub.publish(msg)
        
    def get_leader_state(self) -> Optional[Dict]:
        """Get current leader arm state"""
        if self.leader_joint_state is None:
            return None
        
        return {
            'names': list(self.leader_joint_state.name),
            'positions': list(self.leader_joint_state.position),
            'velocities': list(self.leader_joint_state.velocity) if self.leader_joint_state.velocity else [],
            'timestamp': self.leader_joint_state.header.stamp.sec + self.leader_joint_state.header.stamp.nanosec * 1e-9
        }
        
    def get_follower_state(self) -> Optional[Dict]:
        """Get current follower arm state"""
        if self.follower_joint_state is None:
            return None
        
        return {
            'names': list(self.follower_joint_state.name),
            'positions': list(self.follower_joint_state.position),
            'velocities': list(self.follower_joint_state.velocity) if self.follower_joint_state.velocity else [],
            'timestamp': self.follower_joint_state.header.stamp.sec + self.follower_joint_state.header.stamp.nanosec * 1e-9
        }


class BilateralROS2Controller:
    """
    ROS2-based bilateral controller
    Combines WebXR bridge with bilateral control logic
    """
    
    def __init__(
        self,
        control_rate: float = 50.0,
        position_gain: float = 1.0,
        orientation_gain: float = 0.5
    ):
        """
        Initialize bilateral ROS2 controller
        
        Args:
            control_rate: Control loop rate in Hz
            position_gain: Position error gain
            orientation_gain: Orientation error gain
        """
        self.bridge = WebXRROS2Bridge()
        self.control_rate = control_rate
        self.dt = 1.0 / control_rate
        
        self.position_gain = position_gain
        self.orientation_gain = orientation_gain
        
        self.running = False
        self.control_thread: Optional[threading.Thread] = None
        self.move_enabled = False
        
        # Import kinematics
        try:
            from arm_bench.kinematics import (
                BilateralController,
                get_rx150_config,
                get_vx300s_config
            )
            self.kinematics_controller = BilateralController(
                leader_config=get_rx150_config(),
                follower_config=get_vx300s_config(),
                control_rate=control_rate,
                position_gain=position_gain,
                orientation_gain=orientation_gain
            )
            self.kinematics_available = True
        except ImportError:
            self.kinematics_controller = None
            self.kinematics_available = False
            
    def start(self):
        """Start the bilateral controller"""
        if self.running:
            return
        
        # Start ROS2 bridge
        self.bridge.start()
        
        # Start control loop
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        
        print(f"Bilateral ROS2 Controller started at {self.control_rate} Hz")
        
    def stop(self):
        """Stop the controller"""
        self.running = False
        
        if self.control_thread:
            self.control_thread.join(timeout=2.0)
            self.control_thread = None
        
        self.bridge.stop()
        
    def enable_movement(self, enabled: bool = True):
        """Enable or disable movement"""
        self.move_enabled = enabled
        print(f"Movement {'enabled' if enabled else 'disabled'}")
        
    def _control_loop(self):
        """Main control loop"""
        print("Bilateral control loop started")
        
        while self.running:
            loop_start = time.time()
            
            try:
                # Get leader state
                leader_state = self.bridge.get_leader_state()
                
                if leader_state is not None and self.kinematics_available:
                    # Extract joint positions in correct order
                    expected_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
                    leader_q = np.zeros(5)
                    
                    for i, name in enumerate(expected_names):
                        if name in leader_state['names']:
                            idx = leader_state['names'].index(name)
                            leader_q[i] = leader_state['positions'][idx]
                    
                    # Update bilateral controller
                    follower_q_cmd, info = self.kinematics_controller.update(leader_q)
                    
                    # Publish command if movement enabled
                    if self.move_enabled:
                        self.bridge.publish_follower_command(follower_q_cmd)
                        
            except Exception as e:
                print(f"Control loop error: {e}")
            
            # Maintain rate
            elapsed = time.time() - loop_start
            if elapsed < self.dt:
                time.sleep(self.dt - elapsed)
                
        print("Bilateral control loop ended")
        
    def update_from_webxr(self, pose_data: Dict):
        """
        Update controller from WebXR pose
        
        Args:
            pose_data: WebXR pose data
        """
        self.bridge.publish_webxr_pose(pose_data)
        
        # Check if movement is enabled from WebXR
        control = pose_data.get('control', {})
        self.move_enabled = control.get('moveEnabled', False)


class IntegratedWebXRServer:
    """
    Integrated WebXR server with ROS2 bridge
    Combines the WebXR server with ROS2 communication
    """
    
    def __init__(self, port: int = 8443):
        """
        Initialize integrated server
        
        Args:
            port: Server port
        """
        self.port = port
        self.ros2_controller: Optional[BilateralROS2Controller] = None
        self.webxr_server = None
        
    def start(self):
        """Start the integrated server"""
        # Start ROS2 controller
        if ROS2_AVAILABLE:
            try:
                self.ros2_controller = BilateralROS2Controller()
                self.ros2_controller.start()
            except Exception as e:
                print(f"Warning: Could not start ROS2 controller: {e}")
                self.ros2_controller = None
        
        # Create WebXR server with ROS2 callback
        from arm_bench.webxr.server import WebXRTeleoperationServer
        
        self.webxr_server = WebXRTeleoperationServer(
            port=self.port,
            callback=self._webxr_callback
        )
        
        print("Starting integrated WebXR + ROS2 server...")
        self.webxr_server.run()
        
    def _webxr_callback(self, delta_ee: Dict):
        """Handle WebXR pose updates"""
        if self.ros2_controller:
            # Convert delta_ee to pose format expected by ROS2 controller
            pose_data = {
                'position': {
                    'x': delta_ee.get('x', 0),
                    'y': delta_ee.get('y', 0),
                    'z': delta_ee.get('z', 0)
                },
                'orientation': {
                    'x': 0, 'y': 0, 'z': 0, 'w': 1
                },
                'control': {
                    'gripperOpen': delta_ee.get('gripper_open', False),
                    'moveEnabled': delta_ee.get('move_enabled', False)
                }
            }
            self.ros2_controller.update_from_webxr(pose_data)
        
    def stop(self):
        """Stop the integrated server"""
        if self.webxr_server:
            self.webxr_server.stop()
        
        if self.ros2_controller:
            self.ros2_controller.stop()


def launch_webxr_ros2(port: int = 8443):
    """
    Launch integrated WebXR + ROS2 server
    
    Args:
        port: Server port
    """
    server = IntegratedWebXRServer(port=port)
    try:
        server.start()
    except KeyboardInterrupt:
        print("\nShutting down...")
        server.stop()


if __name__ == "__main__":
    launch_webxr_ros2()
