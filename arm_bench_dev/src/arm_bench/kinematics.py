"""
Kinematics module for arm-bench
Provides Forward Kinematics (FK) and Inverse Kinematics (IK) for Interbotix arms
Supports bilateral control between ReactorX (5DOF) and ViperX (6DOF)
"""
import numpy as np
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum


class ArmType(Enum):
    """Supported arm types"""
    REACTOR_X_150 = "rx150"
    REACTOR_X_200 = "rx200"
    VIPER_X_300S = "vx300s"
    VIPER_X_300 = "vx300"
    WIDOW_X_250 = "wx250"
    WIDOW_X_250_6DOF = "wx250s"


@dataclass
class ArmKinematicsConfig:
    """Kinematics configuration for an arm"""
    name: str
    dof: int
    joint_names: List[str]
    # Screw axes (6 x N matrix, each column is [w; v])
    Slist: np.ndarray
    # Home configuration (4x4 SE3 matrix)
    M: np.ndarray
    # Joint limits (Nx2 matrix, [min, max] for each joint)
    joint_limits: np.ndarray


# ====================== Math Utilities ======================

def skew(v: np.ndarray) -> np.ndarray:
    """Return the 3x3 skew-symmetric matrix of a 3-vector."""
    x, y, z = v.flatten()
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0]
    ])


def matrix_exp3(omega: np.ndarray, theta: float) -> np.ndarray:
    """
    Exponential map for SO(3) - Rodrigues' formula
    omega: 3-vector rotation axis (unit or scaled)
    theta: rotation angle (if omega is unit) or scale factor
    """
    omega = np.asarray(omega, dtype=float).flatten()
    omega_norm = np.linalg.norm(omega)
    
    if omega_norm < 1e-10:
        return np.eye(3)
    
    omega_hat = omega / omega_norm
    omega_skew = skew(omega_hat)
    theta_eff = theta * omega_norm
    
    R = (np.eye(3) + 
         np.sin(theta_eff) * omega_skew + 
         (1 - np.cos(theta_eff)) * (omega_skew @ omega_skew))
    
    return R


def matrix_exp6(S: np.ndarray, theta: float) -> np.ndarray:
    """
    Exponential map for SE(3)
    S: 6-vector twist [omega; v]
    theta: joint angle
    Returns: 4x4 transformation matrix
    """
    S = np.asarray(S, dtype=float).flatten()
    omega = S[0:3]
    v = S[3:6]
    
    omega_norm = np.linalg.norm(omega)
    
    if omega_norm < 1e-10:
        # Pure translation
        T = np.eye(4)
        T[0:3, 3] = v * theta
        return T
    
    omega_hat = omega / omega_norm
    omega_skew = skew(omega_hat)
    theta_eff = theta * omega_norm
    
    # Rotation
    R = (np.eye(3) + 
         np.sin(theta_eff) * omega_skew + 
         (1 - np.cos(theta_eff)) * (omega_skew @ omega_skew))
    
    # Translation
    G = (np.eye(3) * theta_eff + 
         (1 - np.cos(theta_eff)) * omega_skew + 
         (theta_eff - np.sin(theta_eff)) * (omega_skew @ omega_skew))
    p = G @ (v / omega_norm)
    
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = p
    
    return T


def matrix_log3(R: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    Logarithm map for SO(3)
    Returns: (axis, angle)
    """
    trace = np.clip(np.trace(R), -1.0, 3.0)
    angle = np.arccos((trace - 1.0) / 2.0)
    
    if angle < 1e-10:
        return np.zeros(3), 0.0
    
    if np.abs(angle - np.pi) < 1e-6:
        # Special case: angle near pi
        for i in range(3):
            if R[i, i] > -1 + 1e-6:
                axis = R[0:3, i] + np.eye(3)[0:3, i]
                axis = axis / np.linalg.norm(axis)
                return axis, angle
    
    axis = (1.0 / (2.0 * np.sin(angle))) * np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ])
    
    return axis, angle


def adjoint(T: np.ndarray) -> np.ndarray:
    """
    Adjoint representation of SE(3) transformation
    T: 4x4 transformation matrix
    Returns: 6x6 adjoint matrix
    """
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    p_skew = skew(p)
    
    Ad = np.zeros((6, 6))
    Ad[0:3, 0:3] = R
    Ad[3:6, 3:6] = R
    Ad[3:6, 0:3] = p_skew @ R
    
    return Ad


def rotation_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion [x, y, z, w]"""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    
    return np.array([x, y, z, w])


def quaternion_to_rotation(q: np.ndarray) -> np.ndarray:
    """Convert quaternion [x, y, z, w] to 3x3 rotation matrix"""
    x, y, z, w = q
    
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])
    
    return R


def pose_to_se3(position: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    """Convert position + quaternion to 4x4 SE3 transformation"""
    T = np.eye(4)
    T[0:3, 0:3] = quaternion_to_rotation(quaternion)
    T[0:3, 3] = position
    return T


def se3_to_pose(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Convert 4x4 SE3 transformation to position + quaternion"""
    position = T[0:3, 3].copy()
    quaternion = rotation_to_quaternion(T[0:3, 0:3])
    return position, quaternion


def compute_pose_error(T_target: np.ndarray, T_current: np.ndarray) -> np.ndarray:
    """
    Compute pose error as a 6-vector twist [v_linear; v_angular]
    """
    # Position error
    pos_error = T_target[0:3, 3] - T_current[0:3, 3]
    
    # Orientation error (using rotation matrix)
    R_target = T_target[0:3, 0:3]
    R_current = T_current[0:3, 0:3]
    R_error = R_target @ R_current.T
    
    axis, angle = matrix_log3(R_error)
    angular_error = angle * axis
    
    return np.concatenate([pos_error, angular_error])


# ====================== Arm Configurations ======================

def get_rx150_config() -> ArmKinematicsConfig:
    """Get ReactorX-150 kinematics configuration (5DOF)"""
    # Home configuration
    M = np.array([
        [1.0, 0.0, 0.0, 0.358575],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.25457],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=float)
    
    # Screw axes (each row is a screw axis [w; v])
    Slist = np.array([
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],       # waist
        [0.0, 1.0, 0.0, -0.10457, 0.0, 0.0],  # shoulder
        [0.0, 1.0, 0.0, -0.25457, 0.0, 0.05], # elbow
        [0.0, 1.0, 0.0, -0.25457, 0.0, 0.2],  # wrist_angle
        [1.0, 0.0, 0.0, 0.0, 0.25457, 0.0]    # wrist_rotate
    ], dtype=float).T  # Transpose to get 6xN
    
    joint_limits = np.array([
        [-3.14, 3.14],   # waist
        [-1.85, 1.85],   # shoulder
        [-1.50, 1.50],   # elbow
        [-1.80, 2.15],   # wrist_angle
        [-3.14, 3.14]    # wrist_rotate
    ])
    
    return ArmKinematicsConfig(
        name="ReactorX-150",
        dof=5,
        joint_names=["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"],
        Slist=Slist,
        M=M,
        joint_limits=joint_limits
    )


def get_vx300s_config() -> ArmKinematicsConfig:
    """Get ViperX-300s kinematics configuration (6DOF)"""
    # Home configuration
    M = np.array([
        [1.0, 0.0, 0.0, 0.536494],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.42705],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=float)
    
    # Screw axes (each row is a screw axis [w; v])
    Slist = np.array([
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],           # waist
        [0.0, 1.0, 0.0, -0.12705, 0.0, 0.0],      # shoulder
        [0.0, 1.0, 0.0, -0.42705, 0.0, 0.05955],  # elbow
        [1.0, 0.0, 0.0, 0.0, 0.42705, 0.0],       # forearm_roll
        [0.0, 1.0, 0.0, -0.42705, 0.0, 0.35955],  # wrist_angle
        [1.0, 0.0, 0.0, 0.0, 0.42705, 0.0]        # wrist_rotate
    ], dtype=float).T  # Transpose to get 6xN
    
    joint_limits = np.array([
        [-3.14, 3.14],   # waist
        [-1.88, 1.99],   # shoulder
        [-2.15, 1.60],   # elbow
        [-3.14, 3.14],   # forearm_roll
        [-1.74, 2.15],   # wrist_angle
        [-3.14, 3.14]    # wrist_rotate
    ])
    
    return ArmKinematicsConfig(
        name="ViperX-300s",
        dof=6,
        joint_names=["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"],
        Slist=Slist,
        M=M,
        joint_limits=joint_limits
    )


def get_arm_config(arm_type: str) -> ArmKinematicsConfig:
    """Get kinematics configuration for an arm type"""
    arm_type = arm_type.lower()
    
    if arm_type in ["rx150", "reactor_x_150", "reactorx150"]:
        return get_rx150_config()
    elif arm_type in ["vx300s", "viper_x_300s", "viperx300s"]:
        return get_vx300s_config()
    else:
        raise ValueError(f"Unknown arm type: {arm_type}")


# ====================== Forward Kinematics ======================

def forward_kinematics(q: np.ndarray, config: ArmKinematicsConfig) -> np.ndarray:
    """
    Compute forward kinematics using Product of Exponentials (PoE) formula
    
    Args:
        q: Joint angles (N,)
        config: Arm kinematics configuration
        
    Returns:
        T: 4x4 SE3 transformation matrix of end-effector
    """
    q = np.asarray(q, dtype=float).flatten()
    
    if len(q) != config.dof:
        raise ValueError(f"Expected {config.dof} joint angles, got {len(q)}")
    
    T = np.eye(4)
    
    for i in range(config.dof):
        S_i = config.Slist[:, i]
        T = T @ matrix_exp6(S_i, q[i])
    
    return T @ config.M


def space_jacobian(q: np.ndarray, config: ArmKinematicsConfig) -> np.ndarray:
    """
    Compute space Jacobian
    
    Args:
        q: Joint angles (N,)
        config: Arm kinematics configuration
        
    Returns:
        J: 6xN Jacobian matrix [J_omega; J_v]
    """
    q = np.asarray(q, dtype=float).flatten()
    n = config.dof
    
    J = np.zeros((6, n))
    J[:, 0] = config.Slist[:, 0]
    
    T = np.eye(4)
    
    for i in range(1, n):
        S_i = config.Slist[:, i - 1]
        T = T @ matrix_exp6(S_i, q[i - 1])
        J[:, i] = adjoint(T) @ config.Slist[:, i]
    
    return J


# ====================== Inverse Kinematics ======================

def damped_pseudoinverse(J: np.ndarray, damping: float = 0.05) -> np.ndarray:
    """
    Compute damped pseudo-inverse of Jacobian (Damped Least Squares)
    J_pinv = J^T (J J^T + λ^2 I)^{-1}
    """
    m, n = J.shape
    JJT = J @ J.T
    damping_matrix = damping**2 * np.eye(m)
    J_pinv = J.T @ np.linalg.inv(JJT + damping_matrix)
    return J_pinv


def inverse_kinematics_differential(
    T_target: np.ndarray,
    q_init: np.ndarray,
    config: ArmKinematicsConfig,
    max_iterations: int = 100,
    position_tolerance: float = 0.001,
    orientation_tolerance: float = 0.01,
    damping: float = 0.05,
    step_size: float = 0.1
) -> Tuple[np.ndarray, bool]:
    """
    Solve inverse kinematics using damped least squares
    
    Args:
        T_target: 4x4 target transformation
        q_init: Initial joint angles
        config: Arm kinematics configuration
        max_iterations: Maximum iterations
        position_tolerance: Position error tolerance (meters)
        orientation_tolerance: Orientation error tolerance (radians)
        damping: Damping factor for pseudo-inverse
        step_size: Step size for iterations
        
    Returns:
        q: Solution joint angles
        success: Whether solution was found
    """
    q = np.asarray(q_init, dtype=float).flatten().copy()
    
    for iteration in range(max_iterations):
        # Compute current end-effector pose
        T_current = forward_kinematics(q, config)
        
        # Compute pose error
        error = compute_pose_error(T_target, T_current)
        
        # Check convergence
        pos_error = np.linalg.norm(error[0:3])
        ori_error = np.linalg.norm(error[3:6])
        
        if pos_error < position_tolerance and ori_error < orientation_tolerance:
            return q, True
        
        # Compute Jacobian and its pseudo-inverse
        J = space_jacobian(q, config)
        J_pinv = damped_pseudoinverse(J, damping)
        
        # Compute joint velocity
        dq = step_size * J_pinv @ error
        
        # Update joint angles
        q = q + dq
        
        # Apply joint limits
        for i in range(config.dof):
            q[i] = np.clip(q[i], config.joint_limits[i, 0], config.joint_limits[i, 1])
    
    # Check final error
    T_current = forward_kinematics(q, config)
    error = compute_pose_error(T_target, T_current)
    pos_error = np.linalg.norm(error[0:3])
    ori_error = np.linalg.norm(error[3:6])
    
    success = pos_error < position_tolerance * 10 and ori_error < orientation_tolerance * 10
    
    return q, success


def compute_joint_velocities(
    delta_pose: np.ndarray,
    q_current: np.ndarray,
    config: ArmKinematicsConfig,
    damping: float = 0.05
) -> np.ndarray:
    """
    Compute joint velocities from delta end-effector pose
    
    Args:
        delta_pose: 6-vector [dx, dy, dz, d_roll, d_pitch, d_yaw]
        q_current: Current joint angles
        config: Arm kinematics configuration
        damping: Damping factor
        
    Returns:
        dq: Joint velocities
    """
    J = space_jacobian(q_current, config)
    J_pinv = damped_pseudoinverse(J, damping)
    
    dq = J_pinv @ delta_pose
    
    return dq


# ====================== Bilateral Control ======================

class BilateralController:
    """
    Bilateral control between leader (ReactorX 5DOF) and follower (ViperX 6DOF)
    
    The controller:
    1. Reads leader joint positions
    2. Computes leader FK to get end-effector pose
    3. Computes delta pose from previous pose
    4. Uses differential IK to compute follower joint commands
    """
    
    def __init__(
        self,
        leader_config: ArmKinematicsConfig = None,
        follower_config: ArmKinematicsConfig = None,
        control_rate: float = 50.0,
        position_gain: float = 1.0,
        orientation_gain: float = 0.5,
        damping: float = 0.05,
        max_joint_velocity: float = 1.0
    ):
        """
        Initialize bilateral controller
        
        Args:
            leader_config: Leader arm configuration (default: ReactorX-150)
            follower_config: Follower arm configuration (default: ViperX-300s)
            control_rate: Control loop rate in Hz
            position_gain: Gain for position tracking
            orientation_gain: Gain for orientation tracking
            damping: Damping for IK pseudo-inverse
            max_joint_velocity: Maximum joint velocity (rad/s)
        """
        self.leader_config = leader_config or get_rx150_config()
        self.follower_config = follower_config or get_vx300s_config()
        
        self.control_rate = control_rate
        self.dt = 1.0 / control_rate
        
        self.position_gain = position_gain
        self.orientation_gain = orientation_gain
        self.damping = damping
        self.max_joint_velocity = max_joint_velocity
        
        # State
        self.leader_q_prev = None
        self.leader_T_prev = None
        self.follower_q = None
        
        # Preferred null-space posture for follower (6DOF)
        self.preferred_follower_q = np.array([0.0, -0.5, 0.8, 0.0, 0.0, 0.0])
        
        # Smoothing filter
        self.velocity_filter_alpha = 0.3
        self.filtered_velocity = np.zeros(6)
        
    def initialize(self, leader_q: np.ndarray, follower_q: np.ndarray):
        """
        Initialize the controller with current joint positions
        
        Args:
            leader_q: Current leader joint positions
            follower_q: Current follower joint positions
        """
        self.leader_q_prev = np.asarray(leader_q, dtype=float).flatten()
        self.leader_T_prev = forward_kinematics(self.leader_q_prev, self.leader_config)
        self.follower_q = np.asarray(follower_q, dtype=float).flatten()
        self.filtered_velocity = np.zeros(6)
        
    def update(self, leader_q: np.ndarray) -> Tuple[np.ndarray, Dict]:
        """
        Update the controller with new leader joint positions
        
        Args:
            leader_q: Current leader joint positions
            
        Returns:
            follower_q_cmd: Commanded follower joint positions
            info: Dictionary with debug info
        """
        leader_q = np.asarray(leader_q, dtype=float).flatten()
        
        # Initialize if needed
        if self.leader_q_prev is None:
            self.leader_q_prev = leader_q.copy()
            self.leader_T_prev = forward_kinematics(leader_q, self.leader_config)
            if self.follower_q is None:
                self.follower_q = self.preferred_follower_q.copy()
            return self.follower_q.copy(), {"initialized": True}
        
        # Compute leader FK
        T_leader = forward_kinematics(leader_q, self.leader_config)
        
        # Compute delta pose (change in end-effector position/orientation)
        delta_pose = compute_pose_error(T_leader, self.leader_T_prev)
        
        # Apply gains
        delta_pose[0:3] *= self.position_gain
        delta_pose[3:6] *= self.orientation_gain
        
        # Low-pass filter the velocity
        self.filtered_velocity = (
            self.velocity_filter_alpha * delta_pose + 
            (1 - self.velocity_filter_alpha) * self.filtered_velocity
        )
        
        # Compute follower Jacobian
        J_follower = space_jacobian(self.follower_q, self.follower_config)
        J_pinv = damped_pseudoinverse(J_follower, self.damping)
        
        # Compute joint velocities
        dq = J_pinv @ self.filtered_velocity
        
        # Null-space projection for posture control
        # dq_null = (I - J_pinv J) * alpha * (q_preferred - q_current)
        null_space_projector = np.eye(self.follower_config.dof) - J_pinv @ J_follower
        null_space_velocity = 0.1 * (self.preferred_follower_q - self.follower_q)
        dq += null_space_projector @ null_space_velocity
        
        # Limit joint velocities
        dq_norm = np.linalg.norm(dq)
        if dq_norm > self.max_joint_velocity:
            dq = dq * (self.max_joint_velocity / dq_norm)
        
        # Integrate to get new joint positions
        self.follower_q = self.follower_q + dq * self.dt
        
        # Apply joint limits
        for i in range(self.follower_config.dof):
            self.follower_q[i] = np.clip(
                self.follower_q[i],
                self.follower_config.joint_limits[i, 0],
                self.follower_config.joint_limits[i, 1]
            )
        
        # Update previous state
        self.leader_q_prev = leader_q.copy()
        self.leader_T_prev = T_leader.copy()
        
        # Compute error info
        T_follower = forward_kinematics(self.follower_q, self.follower_config)
        tracking_error = compute_pose_error(T_leader, T_follower)
        
        info = {
            "leader_pose": T_leader,
            "follower_pose": T_follower,
            "delta_pose": delta_pose,
            "joint_velocities": dq,
            "position_error": np.linalg.norm(tracking_error[0:3]),
            "orientation_error": np.linalg.norm(tracking_error[3:6])
        }
        
        return self.follower_q.copy(), info
    
    def reset(self):
        """Reset the controller state"""
        self.leader_q_prev = None
        self.leader_T_prev = None
        self.filtered_velocity = np.zeros(6)


class WebXRBilateralController:
    """
    Controller for WebXR-based bilateral control
    Uses delta end-effector pose from WebXR
    """
    
    def __init__(
        self,
        follower_config: ArmKinematicsConfig = None,
        position_scale: float = 1.0,
        orientation_scale: float = 0.5,
        damping: float = 0.05,
        max_joint_velocity: float = 1.0
    ):
        """
        Initialize WebXR bilateral controller
        
        Args:
            follower_config: Follower arm configuration (default: ViperX-300s)
            position_scale: Scale factor for position changes
            orientation_scale: Scale factor for orientation changes
            damping: Damping for IK
            max_joint_velocity: Maximum joint velocity
        """
        self.follower_config = follower_config or get_vx300s_config()
        
        self.position_scale = position_scale
        self.orientation_scale = orientation_scale
        self.damping = damping
        self.max_joint_velocity = max_joint_velocity
        
        # State
        self.follower_q = None
        self.reference_pose = None
        self.move_enabled = False
        
        # Preferred posture
        self.preferred_q = np.array([0.0, -0.5, 0.8, 0.0, 0.0, 0.0])
        
    def initialize(self, follower_q: np.ndarray):
        """Initialize with current joint positions"""
        self.follower_q = np.asarray(follower_q, dtype=float).flatten()
        self.reference_pose = None
        
    def update(self, webxr_pose: Dict, dt: float = 0.02) -> Tuple[np.ndarray, bool]:
        """
        Update controller with WebXR pose data
        
        Args:
            webxr_pose: Dictionary with 'position', 'orientation', 'control' keys
            dt: Time step
            
        Returns:
            follower_q: Commanded joint positions
            updated: Whether a command was generated
        """
        # Check if movement is enabled
        control = webxr_pose.get('control', {})
        self.move_enabled = control.get('moveEnabled', False)
        
        if not self.move_enabled:
            return self.follower_q.copy() if self.follower_q is not None else self.preferred_q.copy(), False
        
        # Initialize if needed
        if self.follower_q is None:
            self.follower_q = self.preferred_q.copy()
        
        # Extract pose
        position = webxr_pose.get('position', {})
        orientation = webxr_pose.get('orientation', {})
        
        # Build current WebXR pose
        current_pose = np.array([
            position.get('x', 0.0),
            position.get('y', 0.0),
            position.get('z', 0.0),
            orientation.get('x', 0.0),
            orientation.get('y', 0.0),
            orientation.get('z', 0.0)
        ])
        
        # Compute delta from reference
        if self.reference_pose is None:
            self.reference_pose = current_pose.copy()
            return self.follower_q.copy(), False
        
        delta_pose = current_pose - self.reference_pose
        self.reference_pose = current_pose.copy()
        
        # Scale delta
        delta_pose[0:3] *= self.position_scale
        delta_pose[3:6] *= self.orientation_scale
        
        # Compute Jacobian and joint velocities
        J = space_jacobian(self.follower_q, self.follower_config)
        J_pinv = damped_pseudoinverse(J, self.damping)
        
        dq = J_pinv @ delta_pose
        
        # Limit velocity
        dq_norm = np.linalg.norm(dq)
        if dq_norm > self.max_joint_velocity * dt:
            dq = dq * (self.max_joint_velocity * dt / dq_norm)
        
        # Update joint positions
        self.follower_q = self.follower_q + dq
        
        # Apply limits
        for i in range(self.follower_config.dof):
            self.follower_q[i] = np.clip(
                self.follower_q[i],
                self.follower_config.joint_limits[i, 0],
                self.follower_config.joint_limits[i, 1]
            )
        
        return self.follower_q.copy(), True
    
    def reset(self):
        """Reset controller state"""
        self.reference_pose = None


# ====================== Utility Functions ======================

def create_bilateral_controller(
    leader_type: str = "rx150",
    follower_type: str = "vx300s",
    **kwargs
) -> BilateralController:
    """
    Create a bilateral controller
    
    Args:
        leader_type: Leader arm type
        follower_type: Follower arm type
        **kwargs: Additional arguments for BilateralController
        
    Returns:
        BilateralController instance
    """
    leader_config = get_arm_config(leader_type)
    follower_config = get_arm_config(follower_type)
    
    return BilateralController(
        leader_config=leader_config,
        follower_config=follower_config,
        **kwargs
    )


def test_kinematics():
    """Test kinematics functions"""
    print("Testing ReactorX-150 FK...")
    rx150_config = get_rx150_config()
    q_home = np.zeros(5)
    T = forward_kinematics(q_home, rx150_config)
    print(f"Home position:\n{T}")
    
    pos, quat = se3_to_pose(T)
    print(f"Position: {pos}")
    print(f"Quaternion: {quat}")
    
    print("\nTesting ViperX-300s FK...")
    vx300s_config = get_vx300s_config()
    q_home = np.zeros(6)
    T = forward_kinematics(q_home, vx300s_config)
    print(f"Home position:\n{T}")
    
    pos, quat = se3_to_pose(T)
    print(f"Position: {pos}")
    print(f"Quaternion: {quat}")
    
    print("\nTesting Jacobian...")
    J = space_jacobian(q_home, vx300s_config)
    print(f"Jacobian shape: {J.shape}")
    print(f"Jacobian:\n{J}")
    
    print("\nTesting IK...")
    # Target pose (small offset from home)
    T_target = T.copy()
    T_target[0, 3] -= 0.05  # Move 5cm in -X
    
    q_init = np.zeros(6)
    q_solution, success = inverse_kinematics_differential(T_target, q_init, vx300s_config)
    
    print(f"IK success: {success}")
    print(f"Solution: {q_solution}")
    
    T_achieved = forward_kinematics(q_solution, vx300s_config)
    error = compute_pose_error(T_target, T_achieved)
    print(f"Position error: {np.linalg.norm(error[0:3]):.6f} m")
    print(f"Orientation error: {np.linalg.norm(error[3:6]):.6f} rad")


if __name__ == "__main__":
    test_kinematics()
