#!/usr/bin/env python3
"""
PyBullet Simulation for Interbotix ReactorX 150 and ViperX 300s Arms
Provides realistic physics simulation for bilateral teleoperation testing
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
import os


@dataclass
class JointInfo:
    """Information about a robot joint"""
    index: int
    name: str
    type: int
    lower_limit: float
    upper_limit: float
    max_force: float
    max_velocity: float


class InterbotixArmSim:
    """Simulated Interbotix arm in PyBullet"""
    
    def __init__(
        self,
        urdf_path: str,
        base_position: List[float],
        base_orientation: List[float],
        physics_client: int,
        name: str = "arm"
    ):
        """
        Initialize simulated arm
        
        Args:
            urdf_path: Path to URDF file
            base_position: [x, y, z] position
            base_orientation: [x, y, z, w] quaternion
            physics_client: PyBullet physics client ID
            name: Arm identifier
        """
        self.name = name
        self.physics_client = physics_client
        
        # Load robot
        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=base_position,
            baseOrientation=base_orientation,
            useFixedBase=True,
            physicsClientId=self.physics_client
        )
        
        # Get joint information
        self.num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.physics_client)
        self.joints: Dict[str, JointInfo] = {}
        self.controllable_joints: List[int] = []
        
        self._parse_joints()
        
        # State tracking
        self.target_positions = np.zeros(len(self.controllable_joints))
        self.current_positions = np.zeros(len(self.controllable_joints))
        self.current_velocities = np.zeros(len(self.controllable_joints))
        self.current_efforts = np.zeros(len(self.controllable_joints))
        
        print(f"Loaded {self.name} with {len(self.controllable_joints)} controllable joints")
    
    def _parse_joints(self):
        """Parse and store joint information"""
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot_id, i, physicsClientId=self.physics_client)
            
            joint = JointInfo(
                index=i,
                name=joint_info[1].decode('utf-8'),
                type=joint_info[2],
                lower_limit=joint_info[8],
                upper_limit=joint_info[9],
                max_force=joint_info[10],
                max_velocity=joint_info[11]
            )
            
            self.joints[joint.name] = joint
            
            # Only revolute and prismatic joints are controllable
            if joint.type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                self.controllable_joints.append(i)
    
    def get_joint_positions(self) -> np.ndarray:
        """Get current joint positions"""
        joint_states = p.getJointStates(
            self.robot_id,
            self.controllable_joints,
            physicsClientId=self.physics_client
        )
        self.current_positions = np.array([state[0] for state in joint_states])
        return self.current_positions
    
    def get_joint_velocities(self) -> np.ndarray:
        """Get current joint velocities"""
        joint_states = p.getJointStates(
            self.robot_id,
            self.controllable_joints,
            physicsClientId=self.physics_client
        )
        self.current_velocities = np.array([state[1] for state in joint_states])
        return self.current_velocities
    
    def get_joint_efforts(self) -> np.ndarray:
        """Get current joint efforts (torques)"""
        joint_states = p.getJointStates(
            self.robot_id,
            self.controllable_joints,
            physicsClientId=self.physics_client
        )
        self.current_efforts = np.array([state[3] for state in joint_states])
        return self.current_efforts
    
    def set_joint_positions(self, positions: np.ndarray, gains: Optional[Tuple[float, float]] = None):
        """
        Set target joint positions using position control
        
        Args:
            positions: Target joint positions
            gains: Optional (Kp, Kd) gains for PD control
        """
        self.target_positions = positions
        
        if gains is None:
            gains = (0.1, 0.5)  # Default PD gains
        
        kp, kd = gains
        
        for i, joint_idx in enumerate(self.controllable_joints):
            p.setJointMotorControl2(
                self.robot_id,
                joint_idx,
                p.POSITION_CONTROL,
                targetPosition=positions[i],
                positionGain=kp,
                velocityGain=kd,
                physicsClientId=self.physics_client
            )
    
    def set_joint_efforts(self, efforts: np.ndarray):
        """
        Set joint efforts (torques) using torque control
        
        Args:
            efforts: Target joint torques
        """
        for i, joint_idx in enumerate(self.controllable_joints):
            # Enable torque control
            p.setJointMotorControl2(
                self.robot_id,
                joint_idx,
                p.TORQUE_CONTROL,
                force=efforts[i],
                physicsClientId=self.physics_client
            )
    
    def apply_external_force(self, link_index: int, force: List[float], position: List[float]):
        """Apply external force to a link (for haptic simulation)"""
        p.applyExternalForce(
            self.robot_id,
            link_index,
            force,
            position,
            p.WORLD_FRAME,
            physicsClientId=self.physics_client
        )
    
    def reset_to_home(self):
        """Reset arm to home position"""
        home_position = np.zeros(len(self.controllable_joints))
        self.set_joint_positions(home_position)
    
    def get_end_effector_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get end effector position and orientation"""
        # Assume last link is end effector
        ee_link = self.controllable_joints[-1]
        link_state = p.getLinkState(
            self.robot_id,
            ee_link,
            physicsClientId=self.physics_client
        )
        position = np.array(link_state[0])
        orientation = np.array(link_state[1])
        return position, orientation


class BilateralSimulation:
    """Complete bilateral teleoperation simulation environment"""
    
    def __init__(
        self,
        use_gui: bool = True,
        time_step: float = 0.01,
        gravity: float = -9.81
    ):
        """
        Initialize simulation environment
        
        Args:
            use_gui: Use GUI mode (True) or DIRECT mode (False)
            time_step: Physics simulation time step
            gravity: Gravity acceleration
        """
        # Initialize PyBullet
        if use_gui:
            self.physics_client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        else:
            self.physics_client = p.connect(p.DIRECT)
        
        # Set up physics
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, gravity, physicsClientId=self.physics_client)
        p.setTimeStep(time_step, physicsClientId=self.physics_client)
        
        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.physics_client)
        
        # Load table (optional)
        self.table_id = self._create_table()
        
        # Arms will be initialized separately
        self.leader_arm: Optional[InterbotixArmSim] = None
        self.follower_arm: Optional[InterbotixArmSim] = None
        
        # Interaction objects
        self.objects: List[int] = []
        
        print("Simulation environment initialized")
    
    def _create_table(self) -> int:
        """Create a table for the arms"""
        table_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[0.4, 0.4, 0.02],
            physicsClientId=self.physics_client
        )
        table_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.4, 0.4, 0.02],
            rgbaColor=[0.6, 0.4, 0.2, 1],
            physicsClientId=self.physics_client
        )
        table_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=table_collision,
            baseVisualShapeIndex=table_visual,
            basePosition=[0, 0, 0.3],
            physicsClientId=self.physics_client
        )
        return table_id
    
    def load_leader_arm(self, urdf_path: str, position: Optional[List[float]] = None):
        """Load leader (ReactorX 150) arm"""
        if position is None:
            position = [-0.3, 0, 0.32]
        
        self.leader_arm = InterbotixArmSim(
            urdf_path=urdf_path,
            base_position=position,
            base_orientation=[0, 0, 0, 1],
            physics_client=self.physics_client,
            name="leader_rx150"
        )
        return self.leader_arm
    
    def load_follower_arm(self, urdf_path: str, position: Optional[List[float]] = None):
        """Load follower (ViperX 300s) arm"""
        if position is None:
            position = [0.3, 0, 0.32]
        
        self.follower_arm = InterbotixArmSim(
            urdf_path=urdf_path,
            base_position=position,
            base_orientation=[0, 0, 0, 1],
            physics_client=self.physics_client,
            name="follower_vx300s"
        )
        return self.follower_arm
    
    def add_object(self, shape: str = "box", position: Optional[List[float]] = None, 
                   size: float = 0.05, color: Optional[List[float]] = None) -> int:
        """Add an interactable object to the scene"""
        if position is None:
            position = [0, 0, 0.4]
        if color is None:
            color = [1, 0, 0, 1]
        
        if shape == "box":
            collision_shape = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[size/2, size/2, size/2],
                physicsClientId=self.physics_client
            )
            visual_shape = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[size/2, size/2, size/2],
                rgbaColor=color,
                physicsClientId=self.physics_client
            )
        elif shape == "sphere":
            collision_shape = p.createCollisionShape(
                p.GEOM_SPHERE,
                radius=size/2,
                physicsClientId=self.physics_client
            )
            visual_shape = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=size/2,
                rgbaColor=color,
                physicsClientId=self.physics_client
            )
        else:
            raise ValueError(f"Unknown shape: {shape}")
        
        object_id = p.createMultiBody(
            baseMass=0.1,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position,
            physicsClientId=self.physics_client
        )
        
        self.objects.append(object_id)
        return object_id
    
    def step(self):
        """Advance simulation by one step"""
        p.stepSimulation(physicsClientId=self.physics_client)
    
    def reset(self):
        """Reset simulation"""
        if self.leader_arm:
            self.leader_arm.reset_to_home()
        if self.follower_arm:
            self.follower_arm.reset_to_home()
        
        # Remove all objects
        for obj_id in self.objects:
            p.removeBody(obj_id, physicsClientId=self.physics_client)
        self.objects.clear()
    
    def disconnect(self):
        """Disconnect from physics server"""
        p.disconnect(physicsClientId=self.physics_client)
        print("Simulation disconnected")


# Example standalone usage
if __name__ == "__main__":
    # Create simulation
    sim = BilateralSimulation(use_gui=True)
    
    # Note: You need actual URDF files for the robots
    # These can be obtained from Interbotix's ROS packages
    # For now, this will show an error without the URDFs
    
    try:
        # Load arms (replace with actual URDF paths)
        leader = sim.load_leader_arm("/home/sp/Workspaces/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf/rx150.urdf.xacro")  
        follower = sim.load_follower_arm("/home/sp/Workspaces/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf/vx300s.urdf.xacro")
        
        # Add some test objects
        sim.add_object("box", [0, 0.2, 0.4], size=0.05, color=[1, 0, 0, 1])
        sim.add_object("sphere", [0, -0.2, 0.4], size=0.05, color=[0, 1, 0, 1])
        
        # Run simulation
        for i in range(10000):
            # Example: Move leader, follower follows
            time_val = i * 0.01
            leader_pos = leader.get_joint_positions()
            leader_pos[0] = 0.5 * np.sin(time_val)  # Oscillate base joint
            leader.set_joint_positions(leader_pos)
            
            # Follower mirrors (with DOF mapping in real implementation)
            follower.set_joint_positions(np.pad(leader_pos, (0, 1), 'constant'))
            
            sim.step()
            time.sleep(0.01)
            
    except Exception as e:
        print(f"Error: {e}")
        print("\nTo use this simulation, you need URDF files for the robots.")
        print("Download them from: https://github.com/Interbotix/interbotix_ros_manipulators")
    
    finally:
        sim.disconnect()