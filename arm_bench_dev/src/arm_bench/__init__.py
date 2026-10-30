"""
arm-bench: Robot Arm Benchmarking and Control Toolkit

This package provides:
- Bilateral teleoperation between different robot arms
- Forward and Inverse Kinematics for Interbotix arms
- WebXR-based teleoperation
- ROS2 integration
- Hardware motor control via Dynamixel
- Visualization and simulation
"""

__version__ = "0.2.0"

# Import key modules for easy access
from arm_bench.kinematics import (
    forward_kinematics,
    space_jacobian,
    inverse_kinematics_differential,
    compute_joint_velocities,
    BilateralController,
    WebXRBilateralController,
    get_rx150_config,
    get_vx300s_config,
    get_arm_config,
    create_bilateral_controller
)

from arm_bench.dynamixel_control import (
    DynamixelController,
    GroupSyncController
)

from arm_bench.bilateral_control import (
    BilateralSystem,
    BimanualSystem,
    WebXRBilateralSystem,
    HardwareArm,
    SimulatedArm,
    create_bilateral_system,
    create_hardware_arm,
    create_simulated_arm
)

from arm_bench.bimanual import (
    BimanualTranslator,
    BimanualCoordinator,
    create_translator,
    create_coordinator
)

# Conditional imports for optional dependencies
try:
    from arm_bench.ros2_bridge import (
        WebXRROS2Bridge,
        BilateralROS2Controller,
        IntegratedWebXRServer,
        launch_webxr_ros2
    )
    ROS2_BRIDGE_AVAILABLE = True
except ImportError:
    ROS2_BRIDGE_AVAILABLE = False

try:
    from arm_bench.webxr.server import (
        WebXRTeleoperationServer,
        create_webxr_server,
        start_webxr_server
    )
    WEBXR_AVAILABLE = True
except ImportError:
    WEBXR_AVAILABLE = False

__all__ = [
    # Version
    '__version__',
    
    # Kinematics
    'forward_kinematics',
    'space_jacobian',
    'inverse_kinematics_differential',
    'compute_joint_velocities',
    'BilateralController',
    'WebXRBilateralController',
    'get_rx150_config',
    'get_vx300s_config',
    'get_arm_config',
    'create_bilateral_controller',
    
    # Hardware control
    'DynamixelController',
    'GroupSyncController',
    
    # Bilateral control
    'BilateralSystem',
    'BimanualSystem',
    'WebXRBilateralSystem',
    'HardwareArm',
    'SimulatedArm',
    'create_bilateral_system',
    'create_hardware_arm',
    'create_simulated_arm',
    
    # Bimanual
    'BimanualTranslator',
    'BimanualCoordinator',
    'create_translator',
    'create_coordinator',
    
    # Feature flags
    'ROS2_BRIDGE_AVAILABLE',
    'WEBXR_AVAILABLE',
]
