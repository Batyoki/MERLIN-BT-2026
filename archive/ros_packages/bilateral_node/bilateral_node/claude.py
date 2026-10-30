#!/usr/bin/env python3
"""
Bilateral Teleoperation Wrapper for ReactorX 150 (5 DOF Leader) and ViperX 300s (6 DOF Follower)
Supports position mirroring and haptic feedback based on follower arm forces/torques
"""

import numpy as np
from typing import Tuple, Optional, Dict
import time
from dataclasses import dataclass
from enum import Enum

# You'll need to install interbotix_xs_modules:
# pip install interbotix_xs_modules
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


class ControlMode(Enum):
    POSITION = "position"
    VELOCITY = "velocity"
    CURRENT = "current"


@dataclass
class ArmState:
    """State representation for robotic arm"""
    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    joint_efforts: np.ndarray
    timestamp: float


@dataclass
class HapticConfig:
    """Configuration for haptic feedback"""
    force_scale: float = 1.0  # Scale factor for force reflection
    damping_coeff: float = 0.1  # Damping coefficient for stability
    max_force: float = 5.0  # Maximum feedback force (N)
    enable_filtering: bool = True
    filter_alpha: float = 0.3  # Low-pass filter coefficient


class BilateralTeleoperationWrapper:
    """
    Bilateral teleoperation wrapper for 5 DOF leader and 6 DOF follower arms
    """
    
    def __init__(
        self,
        leader_name: str = "rx150",
        follower_name: str = "vx300s",
        control_rate: float = 100.0,  # Hz
        haptic_config: Optional[HapticConfig] = None
    ):
        """
        Initialize bilateral teleoperation system
        
        Args:
            leader_name: Robot name for leader arm (ReactorX 150)
            follower_name: Robot name for follower arm (ViperX 300s)
            control_rate: Control loop frequency in Hz
            haptic_config: Haptic feedback configuration
        """
        self.control_rate = control_rate
        self.dt = 1.0 / control_rate
        
        # Initialize arms
        print(f"Initializing leader arm: {leader_name}")
        self.leader = InterbotixManipulatorXS(
            robot_model=leader_name,
            group_name="arm",
            gripper_name="gripper"
        )
        
        print(f"Initializing follower arm: {follower_name}")
        self.follower = InterbotixManipulatorXS(
            robot_model=follower_name,
            group_name="arm",
            gripper_name="gripper"
        )
        
        # Haptic configuration
        self.haptic_config = haptic_config or HapticConfig()
        
        # State tracking
        self.leader_state: Optional[ArmState] = None
        self.follower_state: Optional[ArmState] = None
        self.filtered_feedback = np.zeros(5)
        
        # DOF information
        self.leader_dof = 5
        self.follower_dof = 6
        
        # Mapping matrix from 5 DOF to 6 DOF (modify based on your setup)
        # Default: replicate wrist pitch to both follower wrist joints
        self.dof_mapping_matrix = np.array([
            [1, 0, 0, 0, 0],  # Waist
            [0, 1, 0, 0, 0],  # Shoulder
            [0, 0, 1, 0, 0],  # Elbow
            [0, 0, 0, 1, 0],  # Wrist angle (lower)
            [0, 0, 0, 0.5, 1],  # Wrist angle (upper) - split with wrist rotate
            [0, 0, 0, 0.5, 0],  # Wrist rotate - gets half of leader wrist angle
        ])
        
        # Safety limits
        self.position_tolerance = 0.1  # radians
        self.velocity_limit = 2.0  # rad/s
        
        print("Bilateral teleoperation system initialized")
    
    def _read_leader_state(self) -> ArmState:
        """Read current state of leader arm"""
        positions = self.leader.arm.get_joint_positions()
        velocities = self.leader.arm.get_joint_velocities()
        efforts = self.leader.arm.get_joint_efforts()
        
        return ArmState(
            joint_positions=np.array(positions),
            joint_velocities=np.array(velocities),
            joint_efforts=np.array(efforts),
            timestamp=time.time()
        )
    
    def _read_follower_state(self) -> ArmState:
        """Read current state of follower arm"""
        positions = self.follower.arm.get_joint_positions()
        velocities = self.follower.arm.get_joint_velocities()
        efforts = self.follower.arm.get_joint_efforts()
        
        return ArmState(
            joint_positions=np.array(positions),
            joint_velocities=np.array(velocities),
            joint_efforts=np.array(efforts),
            timestamp=time.time()
        )
    
    def map_5dof_to_6dof(self, leader_positions: np.ndarray) -> np.ndarray:
        """
        Map 5 DOF leader positions to 6 DOF follower positions
        
        Args:
            leader_positions: 5-element array of leader joint positions
            
        Returns:
            6-element array of follower joint positions
        """
        return self.dof_mapping_matrix @ leader_positions
    
    def map_6dof_to_5dof(self, follower_efforts: np.ndarray) -> np.ndarray:
        """
        Map 6 DOF follower efforts back to 5 DOF leader for haptic feedback
        Uses pseudo-inverse for mapping forces back
        
        Args:
            follower_efforts: 6-element array of follower joint efforts
            
        Returns:
            5-element array of leader joint efforts for haptic feedback
        """
        # Use pseudo-inverse of mapping matrix for force reflection
        mapping_pinv = np.linalg.pinv(self.dof_mapping_matrix)
        return mapping_pinv @ follower_efforts
    
    def compute_haptic_feedback(self, follower_state: ArmState) -> np.ndarray:
        """
        Compute haptic feedback torques for leader arm based on follower state
        
        Args:
            follower_state: Current state of follower arm
            
        Returns:
            5-element array of feedback torques for leader arm
        """
        # Map follower efforts to leader space
        raw_feedback = self.map_6dof_to_5dof(follower_state.joint_efforts)
        
        # Scale feedback
        scaled_feedback = raw_feedback * self.haptic_config.force_scale
        
        # Apply damping for stability
        if self.leader_state is not None:
            damping_term = -self.haptic_config.damping_coeff * self.leader_state.joint_velocities
            scaled_feedback += damping_term
        
        # Low-pass filter for smoothness
        if self.haptic_config.enable_filtering:
            alpha = self.haptic_config.filter_alpha
            self.filtered_feedback = alpha * scaled_feedback + (1 - alpha) * self.filtered_feedback
            feedback = self.filtered_feedback
        else:
            feedback = scaled_feedback
        
        # Clamp to maximum force
        feedback = np.clip(
            feedback,
            -self.haptic_config.max_force,
            self.haptic_config.max_force
        )
        
        return feedback
    
    def send_leader_command(self, efforts: np.ndarray):
        """Send torque commands to leader arm for haptic feedback"""
        self.leader.arm.set_joint_efforts(efforts.tolist())
    
    def send_follower_command(self, positions: np.ndarray):
        """Send position commands to follower arm"""
        self.follower.arm.set_joint_positions(positions.tolist())
    
    def teleoperation_step(self) -> Dict:
        """
        Execute one teleoperation control cycle
        
        Returns:
            Dictionary with state information and debug data
        """
        # Read leader arm state
        self.leader_state = self._read_leader_state()
        
        # Map leader positions to follower space
        follower_target = self.map_5dof_to_6dof(self.leader_state.joint_positions)
        
        # Send position command to follower
        self.send_follower_command(follower_target)
        
        # Read follower arm state
        self.follower_state = self._read_follower_state()
        
        # Compute haptic feedback
        haptic_efforts = self.compute_haptic_feedback(self.follower_state)
        
        # Send haptic feedback to leader
        self.send_leader_command(haptic_efforts)
        
        # Return debug information
        return {
            'leader_positions': self.leader_state.joint_positions,
            'follower_positions': self.follower_state.joint_positions,
            'follower_target': follower_target,
            'follower_efforts': self.follower_state.joint_efforts,
            'haptic_feedback': haptic_efforts,
            'timestamp': time.time()
        }
    
    def run_teleoperation(self, duration: Optional[float] = None):
        """
        Run continuous teleoperation loop
        
        Args:
            duration: Run duration in seconds (None for infinite)
        """
        print("Starting teleoperation loop...")
        print(f"Control rate: {self.control_rate} Hz")
        print("Press Ctrl+C to stop")
        
        start_time = time.time()
        iteration = 0
        
        try:
            while True:
                loop_start = time.time()
                
                # Execute teleoperation step
                debug_info = self.teleoperation_step()
                
                # Check duration
                if duration is not None and (time.time() - start_time) > duration:
                    break
                
                # Print debug info periodically
                if iteration % 100 == 0:
                    print(f"\nIteration {iteration}:")
                    print(f"  Leader pos: {np.round(debug_info['leader_positions'], 3)}")
                    print(f"  Follower target: {np.round(debug_info['follower_target'], 3)}")
                    print(f"  Haptic feedback: {np.round(debug_info['haptic_feedback'], 3)}")
                
                # Maintain control rate
                elapsed = time.time() - loop_start
                sleep_time = self.dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif iteration % 100 == 0:
                    print(f"  Warning: Loop time ({elapsed:.4f}s) exceeds target ({self.dt:.4f}s)")
                
                iteration += 1
                
        except KeyboardInterrupt:
            print("\nTeleoperation stopped by user")
        finally:
            self.shutdown()
    
    def set_dof_mapping(self, mapping_matrix: np.ndarray):
        """
        Set custom DOF mapping matrix
        
        Args:
            mapping_matrix: 6x5 matrix for mapping leader to follower joints
        """
        assert mapping_matrix.shape == (6, 5), "Mapping matrix must be 6x5"
        self.dof_mapping_matrix = mapping_matrix
        print("DOF mapping matrix updated")
    
    def set_haptic_config(self, config: HapticConfig):
        """Update haptic feedback configuration"""
        self.haptic_config = config
        self.filtered_feedback = np.zeros(5)
        print("Haptic configuration updated")
    
    def shutdown(self):
        """Safely shutdown both arms"""
        print("Shutting down teleoperation system...")
        
        # Set arms to sleep mode
        self.leader.arm.set_trajectory_time(2.0)
        self.leader.arm.go_to_sleep_pose()
        
        self.follower.arm.set_trajectory_time(2.0)
        self.follower.arm.go_to_sleep_pose()
        
        time.sleep(2.5)
        
        self.leader.shutdown()
        self.follower.shutdown()
        
        print("Shutdown complete")


# Example usage
if __name__ == "__main__":
    # Create haptic configuration
    haptic_cfg = HapticConfig(
        force_scale=0.8,
        damping_coeff=0.15,
        max_force=3.0,
        enable_filtering=True,
        filter_alpha=0.25
    )
    
    # Initialize teleoperation system
    teleop = BilateralTeleoperationWrapper(
        leader_name="rx150",
        follower_name="vx300s",
        control_rate=100.0,
        haptic_config=haptic_cfg
    )
    
    # Optional: Customize DOF mapping
    # custom_mapping = np.array([...])
    # teleop.set_dof_mapping(custom_mapping)
    
    # Move to home position
    print("Moving arms to home position...")
    teleop.leader.arm.go_to_home_pose()
    teleop.follower.arm.go_to_home_pose()
    time.sleep(2.0)
    
    # Run teleoperation for 60 seconds (or indefinitely with duration=None)
    teleop.run_teleoperation(duration=60.0)