"""
Bimanual manipulation interface
Provides abstraction for translating between different robot arm configurations
Specifically handles reactor (5DOF) to viper (6DOF) translation
"""
from typing import List, Dict, Optional
import numpy as np


class ArmConfiguration:
    """Base class for robot arm configuration"""
    
    def __init__(self, name: str, dof: int, joint_names: List[str]):
        self.name = name
        self.dof = dof
        self.joint_names = joint_names
        
    def validate_joint_state(self, joint_state: Dict) -> bool:
        """Validate that joint state matches configuration"""
        return all(joint in joint_state for joint in self.joint_names)


class Reactor5DOF(ArmConfiguration):
    """Configuration for 5DOF Reactor arm"""
    
    def __init__(self):
        joint_names = [
            "waist",
            "shoulder",
            "elbow",
            "wrist_angle",
            "wrist_rotate"
        ]
        super().__init__("reactor", 5, joint_names)


class Viper6DOF(ArmConfiguration):
    """Configuration for 6DOF Viper arm"""
    
    def __init__(self):
        joint_names = [
            "waist",
            "shoulder",
            "elbow",
            "forearm_roll",
            "wrist_angle",
            "wrist_rotate"
        ]
        super().__init__("viper", 6, joint_names)


class BimanualTranslator:
    """
    Translates control commands and states between different arm configurations
    Handles 5DOF to 6DOF translation and vice versa
    """
    
    def __init__(self):
        self.reactor = Reactor5DOF()
        self.viper = Viper6DOF()
        
    def reactor_to_viper(self, reactor_state: Dict) -> Dict:
        """
        Translate Reactor (5DOF) joint state to Viper (6DOF)
        
        The main difference is the forearm_roll joint in Viper.
        We insert it between elbow and wrist_angle.
        
        Args:
            reactor_state: Dictionary with reactor joint positions
            
        Returns:
            Dictionary with viper joint positions
        """
        if not self.reactor.validate_joint_state(reactor_state):
            raise ValueError("Invalid reactor joint state")
        
        # Map reactor joints to viper joints
        # The forearm_roll is set to a neutral position (0) or
        # could be inferred from wrist orientation
        viper_state = {
            "waist": reactor_state["waist"],
            "shoulder": reactor_state["shoulder"],
            "elbow": reactor_state["elbow"],
            "forearm_roll": 0.0,  # Neutral position for missing DOF
            "wrist_angle": reactor_state["wrist_angle"],
            "wrist_rotate": reactor_state["wrist_rotate"]
        }
        
        return viper_state
    
    def viper_to_reactor(self, viper_state: Dict) -> Dict:
        """
        Translate Viper (6DOF) joint state to Reactor (5DOF)
        
        The forearm_roll joint is dropped as Reactor doesn't have it.
        
        Args:
            viper_state: Dictionary with viper joint positions
            
        Returns:
            Dictionary with reactor joint positions
        """
        if not self.viper.validate_joint_state(viper_state):
            raise ValueError("Invalid viper joint state")
        
        # Map viper joints to reactor joints (drop forearm_roll)
        reactor_state = {
            "waist": viper_state["waist"],
            "shoulder": viper_state["shoulder"],
            "elbow": viper_state["elbow"],
            "wrist_angle": viper_state["wrist_angle"],
            "wrist_rotate": viper_state["wrist_rotate"]
        }
        
        print(f"Warning: Dropping forearm_roll joint (value: {viper_state['forearm_roll']})")
        
        return reactor_state
    
    def translate_trajectory(
        self,
        trajectory: List[Dict],
        source: str,
        target: str
    ) -> List[Dict]:
        """
        Translate entire trajectory from source arm to target arm
        
        Args:
            trajectory: List of joint states
            source: Source arm type ('reactor' or 'viper')
            target: Target arm type ('reactor' or 'viper')
            
        Returns:
            Translated trajectory
        """
        if source == target:
            return trajectory
        
        translated = []
        
        if source == "reactor" and target == "viper":
            for state in trajectory:
                translated.append(self.reactor_to_viper(state))
        elif source == "viper" and target == "reactor":
            for state in trajectory:
                translated.append(self.viper_to_reactor(state))
        else:
            raise ValueError(f"Unknown arm types: {source} -> {target}")
        
        return translated
    
    def scale_workspace(
        self,
        joint_state: Dict,
        source_arm_length: float,
        target_arm_length: float
    ) -> Dict:
        """
        Scale joint angles to account for different arm lengths
        
        This is useful when translating between arms of different sizes
        but similar kinematic structure.
        
        Args:
            joint_state: Joint positions to scale
            source_arm_length: Total length of source arm
            target_arm_length: Total length of target arm
            
        Returns:
            Scaled joint positions
        """
        scale_factor = target_arm_length / source_arm_length
        
        # Shoulder and elbow angles may need adjustment
        # Waist and wrist rotations typically stay the same
        scaled_state = joint_state.copy()
        
        # Apply scaling to relevant joints (this is simplified)
        if "shoulder" in scaled_state:
            scaled_state["shoulder"] *= scale_factor
        if "elbow" in scaled_state:
            scaled_state["elbow"] *= scale_factor
        
        return scaled_state


class BimanualCoordinator:
    """
    Coordinates control of two arms for bimanual manipulation
    Handles synchronized movements and relative positioning
    """
    
    def __init__(self, left_arm_type: str, right_arm_type: str):
        self.left_arm_type = left_arm_type
        self.right_arm_type = right_arm_type
        self.translator = BimanualTranslator()
        
    def sync_arms(
        self,
        left_state: Dict,
        right_state: Dict,
        sync_mode: str = "mirror"
    ) -> tuple:
        """
        Synchronize two arms based on sync mode
        
        Args:
            left_state: Left arm joint state
            right_state: Right arm joint state
            sync_mode: 'mirror', 'parallel', or 'independent'
            
        Returns:
            Tuple of (left_command, right_command)
        """
        if sync_mode == "mirror":
            # Mirror movements (opposite waist angles)
            right_command = right_state.copy()
            right_command["waist"] = -left_state["waist"]
            return left_state, right_command
            
        elif sync_mode == "parallel":
            # Parallel movements (same joint angles)
            return left_state, left_state.copy()
            
        else:  # independent
            return left_state, right_state
    
    def coordinate_grasp(
        self,
        object_width: float,
        object_position: List[float]
    ) -> tuple:
        """
        Coordinate both arms to grasp an object
        
        Args:
            object_width: Width of object to grasp
            object_position: [x, y, z] position of object
            
        Returns:
            Tuple of (left_arm_pose, right_arm_pose)
        """
        # Simplified grasp coordination
        # In practice, this would use IK to compute joint angles
        
        grasp_offset = object_width / 2 + 0.05  # 5cm clearance
        
        left_pose = {
            "position": [
                object_position[0] - grasp_offset,
                object_position[1],
                object_position[2]
            ],
            "orientation": [0, 0, 0, 1]  # quaternion
        }
        
        right_pose = {
            "position": [
                object_position[0] + grasp_offset,
                object_position[1],
                object_position[2]
            ],
            "orientation": [0, 0, 0, 1]
        }
        
        return left_pose, right_pose


def create_translator() -> BimanualTranslator:
    """Create a bimanual translator instance"""
    return BimanualTranslator()


def create_coordinator(left_arm: str, right_arm: str) -> BimanualCoordinator:
    """Create a bimanual coordinator instance"""
    return BimanualCoordinator(left_arm, right_arm)
