"""
Bilateral Control Module for arm-bench
Provides bilateral teleoperation between leader and follower arms

Supports:
- ReactorX 5DOF (leader) -> ViperX 6DOF (follower)
- Hardware-to-Hardware bilateral control
- Hardware-to-Simulation bilateral control
- WebXR-based teleoperation with delta EE control
"""
import threading
import time
import numpy as np
from typing import Dict, Optional, Callable, List, Tuple
from dataclasses import dataclass
from enum import Enum

from arm_bench.kinematics import (
    BilateralController,
    WebXRBilateralController,
    forward_kinematics,
    space_jacobian,
    damped_pseudoinverse,
    get_rx150_config,
    get_vx300s_config,
    compute_pose_error,
    se3_to_pose
)
from arm_bench.dynamixel_control import DynamixelController, GroupSyncController


class ControlMode(Enum):
    """Control modes for bilateral operation"""
    POSITION = "position"
    VELOCITY = "velocity"
    DIFFERENTIAL = "differential"


@dataclass
class ArmState:
    """State of a robot arm"""
    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    ee_position: np.ndarray
    ee_orientation: np.ndarray
    gripper_position: float
    timestamp: float


class HardwareArm:
    """
    Hardware arm interface using Dynamixel motors
    """
    
    def __init__(
        self,
        controller: DynamixelController,
        motor_ids: List[int],
        joint_names: List[str],
        gripper_id: Optional[int] = None
    ):
        """
        Initialize hardware arm
        
        Args:
            controller: DynamixelController instance
            motor_ids: List of motor IDs for arm joints
            joint_names: Names of joints
            gripper_id: Motor ID for gripper (optional)
        """
        self.controller = controller
        self.motor_ids = motor_ids
        self.joint_names = joint_names
        self.gripper_id = gripper_id
        self.dof = len(motor_ids)
        
        # Position conversion (raw position -> radians)
        # Dynamixel uses 0-4095 for 0-360 degrees
        self.pos_to_rad = (2 * np.pi) / 4095
        self.rad_to_pos = 4095 / (2 * np.pi)
        self.center_position = 2048  # Center position (180 degrees)
        
    def read_joint_positions(self) -> np.ndarray:
        """Read current joint positions in radians"""
        positions = np.zeros(self.dof)
        
        for i, motor_id in enumerate(self.motor_ids):
            raw_pos = self.controller.read_position(motor_id)
            if raw_pos is not None:
                # Convert to radians (centered at 0)
                positions[i] = (raw_pos - self.center_position) * self.pos_to_rad
        
        return positions
    
    def write_joint_positions(self, positions: np.ndarray) -> bool:
        """
        Write joint positions in radians
        
        Args:
            positions: Joint positions in radians
            
        Returns:
            True if successful
        """
        if len(positions) != self.dof:
            print(f"Error: Expected {self.dof} positions, got {len(positions)}")
            return False
        
        success = True
        
        for i, motor_id in enumerate(self.motor_ids):
            # Convert from radians to raw position
            raw_pos = int(positions[i] * self.rad_to_pos + self.center_position)
            raw_pos = max(0, min(4095, raw_pos))  # Clamp to valid range
            
            if not self.controller.set_goal_position(motor_id, raw_pos):
                success = False
        
        return success
    
    def read_gripper_position(self) -> float:
        """Read gripper position (0.0 = closed, 1.0 = open)"""
        if self.gripper_id is None:
            return 0.5
        
        raw_pos = self.controller.read_position(self.gripper_id)
        if raw_pos is not None:
            # Map gripper range to 0-1
            return (raw_pos - 1500) / 1500  # Assuming range 1500-3000
        return 0.5
    
    def write_gripper_position(self, position: float) -> bool:
        """
        Write gripper position
        
        Args:
            position: 0.0 = closed, 1.0 = open
        """
        if self.gripper_id is None:
            return True
        
        raw_pos = int(1500 + position * 1500)  # Map to 1500-3000
        raw_pos = max(1500, min(3000, raw_pos))
        
        return self.controller.set_goal_position(self.gripper_id, raw_pos)
    
    def get_state(self, kinematics_config=None) -> ArmState:
        """Get current arm state"""
        joint_pos = self.read_joint_positions()
        
        # Compute FK if config provided
        if kinematics_config is not None:
            T = forward_kinematics(joint_pos, kinematics_config)
            ee_pos, ee_quat = se3_to_pose(T)
        else:
            ee_pos = np.zeros(3)
            ee_quat = np.array([0, 0, 0, 1])
        
        return ArmState(
            joint_positions=joint_pos,
            joint_velocities=np.zeros(self.dof),  # Would need velocity reading
            ee_position=ee_pos,
            ee_orientation=ee_quat,
            gripper_position=self.read_gripper_position(),
            timestamp=time.time()
        )


class SimulatedArm:
    """
    Simulated arm for testing without hardware
    """
    
    def __init__(
        self,
        dof: int,
        joint_names: List[str],
        kinematics_config=None
    ):
        """
        Initialize simulated arm
        
        Args:
            dof: Degrees of freedom
            joint_names: Names of joints
            kinematics_config: Kinematics configuration
        """
        self.dof = dof
        self.joint_names = joint_names
        self.kinematics_config = kinematics_config
        
        # State
        self.joint_positions = np.zeros(dof)
        self.joint_velocities = np.zeros(dof)
        self.gripper_position = 0.5
        
    def read_joint_positions(self) -> np.ndarray:
        """Read current joint positions"""
        return self.joint_positions.copy()
    
    def write_joint_positions(self, positions: np.ndarray) -> bool:
        """Write joint positions"""
        self.joint_positions = np.asarray(positions).flatten()
        return True
    
    def read_gripper_position(self) -> float:
        """Read gripper position"""
        return self.gripper_position
    
    def write_gripper_position(self, position: float) -> bool:
        """Write gripper position"""
        self.gripper_position = position
        return True
    
    def get_state(self, kinematics_config=None) -> ArmState:
        """Get current arm state"""
        config = kinematics_config or self.kinematics_config
        
        if config is not None:
            T = forward_kinematics(self.joint_positions, config)
            ee_pos, ee_quat = se3_to_pose(T)
        else:
            ee_pos = np.zeros(3)
            ee_quat = np.array([0, 0, 0, 1])
        
        return ArmState(
            joint_positions=self.joint_positions.copy(),
            joint_velocities=self.joint_velocities.copy(),
            ee_position=ee_pos,
            ee_orientation=ee_quat,
            gripper_position=self.gripper_position,
            timestamp=time.time()
        )


class BilateralSystem:
    """
    Complete bilateral teleoperation system
    Manages leader and follower arms with FK/IK-based control
    """
    
    def __init__(
        self,
        leader_arm,
        follower_arm,
        leader_config=None,
        follower_config=None,
        control_rate: float = 50.0
    ):
        """
        Initialize bilateral system
        
        Args:
            leader_arm: Leader arm interface (HardwareArm or SimulatedArm)
            follower_arm: Follower arm interface
            leader_config: Leader kinematics config
            follower_config: Follower kinematics config
            control_rate: Control loop frequency in Hz
        """
        self.leader = leader_arm
        self.follower = follower_arm
        
        self.leader_config = leader_config or get_rx150_config()
        self.follower_config = follower_config or get_vx300s_config()
        
        self.control_rate = control_rate
        self.dt = 1.0 / control_rate
        
        # Bilateral controller
        self.controller = BilateralController(
            leader_config=self.leader_config,
            follower_config=self.follower_config,
            control_rate=control_rate
        )
        
        # State
        self.running = False
        self.thread = None
        self.move_enabled = False
        
        # Callbacks
        self.state_callback: Optional[Callable] = None
        
        # Statistics
        self.loop_count = 0
        self.last_error = None
        
    def start(self):
        """Start bilateral control loop"""
        if self.running:
            print("Bilateral system already running")
            return
        
        print("Starting bilateral control system...")
        
        # Initialize controller with current positions
        leader_q = self.leader.read_joint_positions()
        follower_q = self.follower.read_joint_positions()
        self.controller.initialize(leader_q, follower_q)
        
        self.running = True
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()
        
        print(f"Bilateral control running at {self.control_rate} Hz")
        
    def stop(self):
        """Stop bilateral control loop"""
        if not self.running:
            return
        
        print("Stopping bilateral control...")
        self.running = False
        
        if self.thread:
            self.thread.join(timeout=2.0)
            self.thread = None
        
        print("Bilateral control stopped")
        
    def enable_movement(self, enabled: bool = True):
        """Enable or disable movement"""
        self.move_enabled = enabled
        print(f"Movement {'enabled' if enabled else 'disabled'}")
        
    def _control_loop(self):
        """Main control loop"""
        print("Control loop started")
        
        while self.running:
            loop_start = time.time()
            
            try:
                # Read leader state
                leader_q = self.leader.read_joint_positions()
                
                # Update bilateral controller
                follower_q_cmd, info = self.controller.update(leader_q)
                
                # Write to follower if movement enabled
                if self.move_enabled:
                    self.follower.write_joint_positions(follower_q_cmd)
                
                # Read gripper and mirror
                gripper_pos = self.leader.read_gripper_position()
                if self.move_enabled:
                    self.follower.write_gripper_position(gripper_pos)
                
                # Call state callback if set
                if self.state_callback:
                    leader_state = self.leader.get_state(self.leader_config)
                    follower_state = self.follower.get_state(self.follower_config)
                    self.state_callback(leader_state, follower_state, info)
                
                self.loop_count += 1
                
            except Exception as e:
                self.last_error = str(e)
                print(f"Control loop error: {e}")
            
            # Maintain loop rate
            elapsed = time.time() - loop_start
            sleep_time = self.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        print("Control loop ended")
        
    def get_stats(self) -> Dict:
        """Get system statistics"""
        return {
            "running": self.running,
            "move_enabled": self.move_enabled,
            "loop_count": self.loop_count,
            "control_rate": self.control_rate,
            "last_error": self.last_error
        }


class BimanualSystem:
    """
    Bimanual teleoperation system (2 leaders, 2 followers)
    ALOHA-style bilateral control
    """
    
    def __init__(
        self,
        left_leader,
        left_follower,
        right_leader,
        right_follower,
        leader_config=None,
        follower_config=None,
        control_rate: float = 50.0
    ):
        """
        Initialize bimanual system
        
        Args:
            left_leader: Left leader arm
            left_follower: Left follower arm
            right_leader: Right leader arm
            right_follower: Right follower arm
            leader_config: Leader kinematics config
            follower_config: Follower kinematics config
            control_rate: Control loop frequency
        """
        # Create two bilateral systems
        self.left_system = BilateralSystem(
            left_leader, left_follower,
            leader_config, follower_config,
            control_rate
        )
        self.right_system = BilateralSystem(
            right_leader, right_follower,
            leader_config, follower_config,
            control_rate
        )
        
        self.running = False
        
    def start(self):
        """Start bimanual control"""
        self.left_system.start()
        self.right_system.start()
        self.running = True
        
    def stop(self):
        """Stop bimanual control"""
        self.left_system.stop()
        self.right_system.stop()
        self.running = False
        
    def enable_movement(self, enabled: bool = True):
        """Enable movement on both systems"""
        self.left_system.enable_movement(enabled)
        self.right_system.enable_movement(enabled)


class WebXRBilateralSystem:
    """
    WebXR-based bilateral control
    Uses delta end-effector pose from WebXR for control
    """
    
    def __init__(
        self,
        follower_arm,
        follower_config=None,
        control_rate: float = 50.0
    ):
        """
        Initialize WebXR bilateral system
        
        Args:
            follower_arm: Follower arm interface
            follower_config: Follower kinematics config
            control_rate: Control loop frequency
        """
        self.follower = follower_arm
        self.follower_config = follower_config or get_vx300s_config()
        self.control_rate = control_rate
        self.dt = 1.0 / control_rate
        
        # WebXR controller
        self.controller = WebXRBilateralController(
            follower_config=self.follower_config
        )
        
        # State
        self.running = False
        self.thread = None
        self.latest_webxr_pose: Optional[Dict] = None
        self.pose_lock = threading.Lock()
        
    def start(self):
        """Start WebXR control loop"""
        if self.running:
            return
        
        # Initialize controller
        follower_q = self.follower.read_joint_positions()
        self.controller.initialize(follower_q)
        
        self.running = True
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()
        
        print(f"WebXR bilateral control started at {self.control_rate} Hz")
        
    def stop(self):
        """Stop control loop"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
            self.thread = None
            
    def update_pose(self, webxr_pose: Dict):
        """
        Update with new WebXR pose
        
        Args:
            webxr_pose: Dictionary with 'position', 'orientation', 'control' keys
        """
        with self.pose_lock:
            self.latest_webxr_pose = webxr_pose.copy()
            
    def _control_loop(self):
        """Main control loop"""
        while self.running:
            loop_start = time.time()
            
            try:
                # Get latest pose
                with self.pose_lock:
                    pose = self.latest_webxr_pose
                
                if pose is not None:
                    # Update controller
                    follower_q, updated = self.controller.update(pose, self.dt)
                    
                    if updated:
                        self.follower.write_joint_positions(follower_q)
                    
                    # Handle gripper
                    control = pose.get('control', {})
                    if control.get('gripperOpen', False):
                        self.follower.write_gripper_position(1.0)
                    else:
                        self.follower.write_gripper_position(0.0)
                        
            except Exception as e:
                print(f"WebXR control error: {e}")
            
            # Maintain rate
            elapsed = time.time() - loop_start
            if elapsed < self.dt:
                time.sleep(self.dt - elapsed)


# ====================== Factory Functions ======================

def create_hardware_arm(
    controller: DynamixelController,
    arm_type: str
) -> HardwareArm:
    """
    Create a hardware arm interface
    
    Args:
        controller: DynamixelController instance
        arm_type: Type of arm ('rx150', 'vx300s', etc.)
        
    Returns:
        HardwareArm instance
    """
    arm_type = arm_type.lower()
    
    if arm_type in ['rx150', 'reactor_x_150']:
        motor_ids = [1, 2, 3, 4, 5]  # Adjust based on your setup
        joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
        gripper_id = 6
    elif arm_type in ['vx300s', 'viper_x_300s']:
        motor_ids = [1, 2, 3, 4, 5, 6]  # Adjust based on your setup
        joint_names = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        gripper_id = 7
    else:
        raise ValueError(f"Unknown arm type: {arm_type}")
    
    return HardwareArm(controller, motor_ids, joint_names, gripper_id)


def create_simulated_arm(arm_type: str) -> SimulatedArm:
    """
    Create a simulated arm interface
    
    Args:
        arm_type: Type of arm
        
    Returns:
        SimulatedArm instance
    """
    arm_type = arm_type.lower()
    
    if arm_type in ['rx150', 'reactor_x_150']:
        config = get_rx150_config()
        dof = 5
        joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
    elif arm_type in ['vx300s', 'viper_x_300s']:
        config = get_vx300s_config()
        dof = 6
        joint_names = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
    else:
        raise ValueError(f"Unknown arm type: {arm_type}")
    
    return SimulatedArm(dof, joint_names, config)


def create_bilateral_system(
    leader_type: str = 'rx150',
    follower_type: str = 'vx300s',
    leader_hardware: bool = False,
    follower_hardware: bool = False,
    controller: Optional[DynamixelController] = None,
    control_rate: float = 50.0
) -> BilateralSystem:
    """
    Create a bilateral teleoperation system
    
    Args:
        leader_type: Leader arm type
        follower_type: Follower arm type
        leader_hardware: Use hardware for leader
        follower_hardware: Use hardware for follower
        controller: DynamixelController instance (required if using hardware)
        control_rate: Control frequency
        
    Returns:
        BilateralSystem instance
    """
    # Create leader arm
    if leader_hardware:
        if controller is None:
            raise ValueError("Controller required for hardware arm")
        leader_arm = create_hardware_arm(controller, leader_type)
    else:
        leader_arm = create_simulated_arm(leader_type)
    
    # Create follower arm
    if follower_hardware:
        if controller is None:
            raise ValueError("Controller required for hardware arm")
        follower_arm = create_hardware_arm(controller, follower_type)
    else:
        follower_arm = create_simulated_arm(follower_type)
    
    # Get configs
    from arm_bench.kinematics import get_arm_config
    leader_config = get_arm_config(leader_type)
    follower_config = get_arm_config(follower_type)
    
    return BilateralSystem(
        leader_arm, follower_arm,
        leader_config, follower_config,
        control_rate
    )


def test_bilateral():
    """Test bilateral control with simulated arms"""
    print("Testing bilateral control with simulated arms...")
    
    # Create simulated arms
    system = create_bilateral_system(
        leader_type='rx150',
        follower_type='vx300s',
        leader_hardware=False,
        follower_hardware=False,
        control_rate=50.0
    )
    
    # State callback for visualization
    def state_callback(leader_state, follower_state, info):
        print(f"Leader EE: {leader_state.ee_position}")
        print(f"Follower EE: {follower_state.ee_position}")
        print(f"Position error: {info.get('position_error', 0):.4f} m")
        print("---")
    
    system.state_callback = state_callback
    
    # Start system
    system.start()
    system.enable_movement(True)
    
    # Simulate leader movement
    print("\nSimulating leader movement...")
    for i in range(10):
        # Move leader joints
        leader_q = np.array([
            0.5 * np.sin(i * 0.2),  # waist
            -0.5 + 0.2 * np.sin(i * 0.1),  # shoulder
            0.3 * np.sin(i * 0.15),  # elbow
            0.1 * np.sin(i * 0.25),  # wrist_angle
            0.0  # wrist_rotate
        ])
        system.leader.write_joint_positions(leader_q)
        time.sleep(0.2)
    
    # Stop
    system.stop()
    
    print("\nTest complete!")
    print(f"Stats: {system.get_stats()}")


if __name__ == "__main__":
    test_bilateral()
