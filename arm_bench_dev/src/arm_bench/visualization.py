"""
Visualization module for arm-bench
Provides robot visualization using PyBullet simulation and RViz
Supports 2 or 4 robot configurations for bilateral/bimanual modes
"""
import subprocess
import os
import sys
import threading
import time
import numpy as np
from typing import Optional, Dict, List, Tuple, Callable
from dataclasses import dataclass

# Try to import PyBullet for visualization
PYBULLET_AVAILABLE = False
try:
    import pybullet as p
    import pybullet_data
    PYBULLET_AVAILABLE = True
except ImportError:
    pass


def prepare_rviz_environment() -> Dict[str, str]:
    """Return a Qt-safe environment for launching RViz."""
    env = os.environ.copy()

    # OpenCV can inject a Qt plugin path that breaks RViz on Linux.
    for key in ("QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH"):
        env.pop(key, None)

    env.setdefault("QT_QPA_PLATFORM", "xcb")
    return env


@dataclass
class RobotVisualization:
    """Configuration for a visualized robot"""
    name: str
    urdf_path: str
    base_position: Tuple[float, float, float]
    base_orientation: Tuple[float, float, float, float]
    robot_id: int = -1
    joint_ids: List[int] = None
    ee_link_id: int = -1
    is_leader: bool = False


class PyBulletVisualizer:
    """
    PyBullet-based visualizer for robot arms
    Supports multiple robots in the same scene
    """
    
    def __init__(
        self,
        gui: bool = True,
        real_time: bool = True,
        camera_distance: float = 2.0,
        camera_yaw: float = 45.0,
        camera_pitch: float = -30.0
    ):
        """
        Initialize PyBullet visualizer
        
        Args:
            gui: Use GUI mode (vs headless)
            real_time: Enable real-time simulation
            camera_distance: Camera distance from target
            camera_yaw: Camera yaw angle
            camera_pitch: Camera pitch angle
        """
        if not PYBULLET_AVAILABLE:
            raise ImportError("PyBullet not installed. Install with: pip install pybullet")
        
        self.gui = gui
        self.real_time = real_time
        self.camera_distance = camera_distance
        self.camera_yaw = camera_yaw
        self.camera_pitch = camera_pitch
        
        self.physics_client = -1
        self.robots: Dict[str, RobotVisualization] = {}
        self.running = False
        self.update_thread: Optional[threading.Thread] = None
        
        # Joint state callbacks
        self.joint_state_callback: Optional[Callable] = None
        
    def start(self):
        """Start the visualizer"""
        if self.running:
            return
        
        print("Starting PyBullet visualizer...")
        
        # Connect to PyBullet
        if self.gui:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)
        
        # Set up simulation
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        if self.real_time:
            p.setRealTimeSimulation(1)
        
        # Load ground plane
        p.loadURDF("plane.urdf")
        
        # Set camera
        p.resetDebugVisualizerCamera(
            cameraDistance=self.camera_distance,
            cameraYaw=self.camera_yaw,
            cameraPitch=self.camera_pitch,
            cameraTargetPosition=[0, 0, 0.3]
        )
        
        self.running = True
        print("PyBullet visualizer started")
        
    def stop(self):
        """Stop the visualizer"""
        if not self.running:
            return
        
        self.running = False
        
        if self.update_thread:
            self.update_thread.join(timeout=2.0)
            self.update_thread = None
        
        if self.physics_client >= 0:
            p.disconnect(self.physics_client)
            self.physics_client = -1
        
        print("PyBullet visualizer stopped")
        
    def add_robot(
        self,
        name: str,
        urdf_path: str,
        base_position: Tuple[float, float, float] = (0, 0, 0),
        base_orientation: Tuple[float, float, float, float] = (0, 0, 0, 1),
        is_leader: bool = False
    ) -> bool:
        """
        Add a robot to the visualization
        
        Args:
            name: Unique name for the robot
            urdf_path: Path to URDF file
            base_position: Base position (x, y, z)
            base_orientation: Base orientation (quaternion)
            is_leader: Whether this is a leader arm
            
        Returns:
            True if successful
        """
        if not self.running:
            print("Visualizer not running")
            return False
        
        if not os.path.exists(urdf_path):
            print(f"URDF not found: {urdf_path}")
            return False
        
        try:
            # Load the robot
            robot_id = p.loadURDF(
                urdf_path,
                basePosition=base_position,
                baseOrientation=base_orientation,
                useFixedBase=True
            )
            
            # Get joint info
            num_joints = p.getNumJoints(robot_id)
            joint_ids = []
            ee_link_id = -1
            
            for i in range(num_joints):
                joint_info = p.getJointInfo(robot_id, i)
                joint_type = joint_info[2]
                joint_name = joint_info[1].decode('utf-8')
                
                # Collect revolute joints (arm joints)
                if joint_type == p.JOINT_REVOLUTE:
                    joint_ids.append(i)
                
                # Find end-effector link
                if 'ee' in joint_name.lower() or 'gripper' in joint_name.lower():
                    ee_link_id = i
            
            # If no EE found, use last joint
            if ee_link_id < 0 and joint_ids:
                ee_link_id = joint_ids[-1]
            
            # Store robot info
            self.robots[name] = RobotVisualization(
                name=name,
                urdf_path=urdf_path,
                base_position=base_position,
                base_orientation=base_orientation,
                robot_id=robot_id,
                joint_ids=joint_ids,
                ee_link_id=ee_link_id,
                is_leader=is_leader
            )
            
            print(f"Added robot '{name}' with {len(joint_ids)} joints")
            return True
            
        except Exception as e:
            print(f"Error adding robot: {e}")
            return False
            
    def set_joint_positions(self, robot_name: str, joint_positions: np.ndarray):
        """
        Set joint positions for a robot
        
        Args:
            robot_name: Name of the robot
            joint_positions: Joint positions in radians
        """
        if robot_name not in self.robots:
            return
        
        robot = self.robots[robot_name]
        
        for i, joint_id in enumerate(robot.joint_ids):
            if i < len(joint_positions):
                p.resetJointState(robot.robot_id, joint_id, joint_positions[i])
                
    def get_joint_positions(self, robot_name: str) -> Optional[np.ndarray]:
        """
        Get current joint positions for a robot
        
        Args:
            robot_name: Name of the robot
            
        Returns:
            Joint positions array or None
        """
        if robot_name not in self.robots:
            return None
        
        robot = self.robots[robot_name]
        positions = []
        
        for joint_id in robot.joint_ids:
            state = p.getJointState(robot.robot_id, joint_id)
            positions.append(state[0])
        
        return np.array(positions)
        
    def get_ee_pose(self, robot_name: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Get end-effector pose for a robot
        
        Args:
            robot_name: Name of the robot
            
        Returns:
            Tuple of (position, orientation) or None
        """
        if robot_name not in self.robots:
            return None
        
        robot = self.robots[robot_name]
        
        if robot.ee_link_id < 0:
            return None
        
        state = p.getLinkState(robot.robot_id, robot.ee_link_id)
        position = np.array(state[0])
        orientation = np.array(state[1])
        
        return position, orientation
        
    def set_robot_color(self, robot_name: str, color: Tuple[float, float, float, float]):
        """
        Set color for a robot
        
        Args:
            robot_name: Name of the robot
            color: RGBA color (0-1 range)
        """
        if robot_name not in self.robots:
            return
        
        robot = self.robots[robot_name]
        
        # Set color for all links
        for i in range(-1, p.getNumJoints(robot.robot_id)):
            p.changeVisualShape(robot.robot_id, i, rgbaColor=color)
            
    def add_debug_line(
        self,
        start: Tuple[float, float, float],
        end: Tuple[float, float, float],
        color: Tuple[float, float, float] = (1, 0, 0),
        width: float = 2.0,
        lifetime: float = 0.0
    ) -> int:
        """Add a debug line to the visualization"""
        return p.addUserDebugLine(
            start, end, color, 
            lineWidth=width, 
            lifeTime=lifetime
        )
        
    def step_simulation(self):
        """Step the simulation (if not real-time)"""
        if not self.real_time:
            p.stepSimulation()


class VisualizationLauncher:
    """Manages RViz and visualization tools"""
    
    def __init__(self, mode: str = "simulation"):
        self.mode = mode
        self.rviz_process = None
        self.pybullet_viz: Optional[PyBulletVisualizer] = None
        
    def launch_rviz(self, config_file: Optional[str] = None):
        """
        Launch RViz visualization
        
        Args:
            config_file: Path to RViz config file (optional)
        """
        try:
            # Check if ROS2 is available
            ros_distro = os.environ.get('ROS_DISTRO', None)
            
            if not ros_distro:
                print("Warning: ROS_DISTRO not set. Please source ROS2 workspace.")
                print("Example: source /opt/ros/humble/setup.bash")
                return False
            
            # Build RViz command
            cmd = ['ros2', 'run', 'rviz2', 'rviz2']
            
            if config_file and os.path.exists(config_file):
                cmd.extend(['-d', config_file])
            
            print(f"Launching RViz for {self.mode} mode...")
            self.rviz_process = subprocess.Popen(cmd, env=prepare_rviz_environment())
            
            print(f"RViz launched with PID: {self.rviz_process.pid}")
            return True
            
        except FileNotFoundError:
            print("Error: RViz not found. Please install ROS2 and RViz.")
            print("Installation: sudo apt install ros-<distro>-rviz2")
            return False
        except Exception as e:
            print(f"Error launching RViz: {e}")
            return False
    
    def stop_rviz(self):
        """Stop RViz if running"""
        if self.rviz_process and self.rviz_process.poll() is None:
            print("Stopping RViz...")
            self.rviz_process.terminate()
            self.rviz_process.wait(timeout=5)
            print("RViz stopped")
        else:
            print("RViz is not running")

    def open_rviz_launcher(self, robot_name: Optional[str] = None):
        """Open the RViz Launch Manager GUI as a separate process.

        This will spawn the `rviz_launch_gui.py` script included in the package so it
        runs independently of the current process. `robot_name` can be provided by
        the caller (e.g., when the user clicks an arm) but is only a hint the GUI
        can use — the user still chooses the package/launch and args.
        """
        try:
            script_path = os.path.join(os.path.dirname(__file__), 'rviz_launch_gui.py')
            if not os.path.exists(script_path):
                print(f"RViz launcher script not found: {script_path}")
                return False
            cmd = [sys.executable, script_path]
            env = prepare_rviz_environment()
            # Launch detached GUI process
            subprocess.Popen(cmd, env=env)
            return True
        except Exception as e:
            print(f"Failed to open RViz launcher GUI: {e}")
            return False
    
    def launch_pybullet(
        self,
        num_robots: int = 2,
        robot_type: str = "vx300s",
        show_leaders: bool = True
    ) -> Optional[PyBulletVisualizer]:
        """
        Launch PyBullet visualization with specified number of robots
        
        Args:
            num_robots: Number of follower robots (2 or 4)
            robot_type: Type of robot to visualize
            show_leaders: Whether to show leader arms
            
        Returns:
            PyBulletVisualizer instance or None
        """
        if not PYBULLET_AVAILABLE:
            print("PyBullet not available. Install with: pip install pybullet")
            return None
        
        try:
            self.pybullet_viz = PyBulletVisualizer(gui=True)
            self.pybullet_viz.start()
            
            # Find URDF paths
            urdf_paths = self._find_urdf_paths(robot_type)
            
            if not urdf_paths:
                print(f"No URDF found for {robot_type}")
                return self.pybullet_viz
            
            # Add robots based on configuration
            if num_robots == 2:
                # Bilateral setup: 1 leader, 1 follower
                if show_leaders:
                    self.pybullet_viz.add_robot(
                        "leader_1",
                        urdf_paths.get('rx150', urdf_paths.get(robot_type)),
                        base_position=(-0.5, 0, 0),
                        is_leader=True
                    )
                    self.pybullet_viz.set_robot_color("leader_1", (0.2, 0.6, 1.0, 1.0))
                
                self.pybullet_viz.add_robot(
                    "follower_1",
                    urdf_paths.get('vx300s', urdf_paths.get(robot_type)),
                    base_position=(0.5, 0, 0),
                    is_leader=False
                )
                self.pybullet_viz.set_robot_color("follower_1", (0.2, 1.0, 0.3, 1.0))
                
            elif num_robots == 4:
                # Bimanual setup: 2 leaders, 2 followers
                if show_leaders:
                    self.pybullet_viz.add_robot(
                        "left_leader",
                        urdf_paths.get('rx150', urdf_paths.get(robot_type)),
                        base_position=(-0.5, 0.5, 0),
                        is_leader=True
                    )
                    self.pybullet_viz.set_robot_color("left_leader", (0.2, 0.6, 1.0, 1.0))
                    
                    self.pybullet_viz.add_robot(
                        "right_leader",
                        urdf_paths.get('rx150', urdf_paths.get(robot_type)),
                        base_position=(-0.5, -0.5, 0),
                        is_leader=True
                    )
                    self.pybullet_viz.set_robot_color("right_leader", (0.2, 0.6, 1.0, 1.0))
                
                self.pybullet_viz.add_robot(
                    "left_follower",
                    urdf_paths.get('vx300s', urdf_paths.get(robot_type)),
                    base_position=(0.5, 0.5, 0),
                    is_leader=False
                )
                self.pybullet_viz.set_robot_color("left_follower", (0.2, 1.0, 0.3, 1.0))
                
                self.pybullet_viz.add_robot(
                    "right_follower",
                    urdf_paths.get('vx300s', urdf_paths.get(robot_type)),
                    base_position=(0.5, -0.5, 0),
                    is_leader=False
                )
                self.pybullet_viz.set_robot_color("right_follower", (0.2, 1.0, 0.3, 1.0))
            
            print(f"Visualization launched with {num_robots} robots")
            return self.pybullet_viz
            
        except Exception as e:
            print(f"Error launching PyBullet: {e}")
            return None
    
    def _find_urdf_paths(self, robot_type: str) -> Dict[str, str]:
        """Find URDF paths for robots"""
        urdf_paths = {}
        
        # Search paths
        search_paths = [
            os.path.expanduser("~/interbotix_ws/src"),
            "/opt/interbotix",
            os.path.expanduser("~/ros2_ws/src"),
            os.path.dirname(os.path.abspath(__file__))
        ]
        
        # Robot URDF patterns
        patterns = {
            'rx150': ['rx150.urdf', 'rx150.urdf.xacro'],
            'vx300s': ['vx300s.urdf', 'vx300s.urdf.xacro'],
            'wx250': ['wx250.urdf', 'wx250.urdf.xacro'],
        }
        
        for search_path in search_paths:
            for robot_name, file_patterns in patterns.items():
                for pattern in file_patterns:
                    for root, dirs, files in os.walk(search_path):
                        for file in files:
                            if file == pattern or pattern in file:
                                urdf_paths[robot_name] = os.path.join(root, file)
                                break
        
        return urdf_paths
    
    def stop_pybullet(self):
        """Stop PyBullet visualization"""
        if self.pybullet_viz:
            self.pybullet_viz.stop()
            self.pybullet_viz = None
    
    def launch_controller(self, robot_name: str = "interbotix_arm"):
        """
        Launch robot controller interface
        
        Args:
            robot_name: Name of the robot to control
        """
        try:
            # Check for ROS2
            ros_distro = os.environ.get('ROS_DISTRO', None)
            
            if not ros_distro:
                print("Warning: ROS_DISTRO not set. Please source ROS2 workspace.")
                return False
            
            # Launch controller
            cmd = [
                'ros2', 'launch',
                f'{robot_name}_control',
                'control.launch.py'
            ]
            
            print(f"Launching controller for {robot_name}...")
            controller_process = subprocess.Popen(cmd)
            
            print(f"Controller launched with PID: {controller_process.pid}")
            return True
            
        except FileNotFoundError:
            print(f"Error: Controller launch file not found for {robot_name}")
            print("Make sure the robot package is installed and sourced")
            return False
        except Exception as e:
            print(f"Error launching controller: {e}")
            return False


def create_default_rviz_config(output_path: str):
    """
    Create a default RViz configuration file
    
    Args:
        output_path: Where to save the config file
    """
    config = """
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 1.0
      Class: rviz_default_plugins/RobotModel
      Description Topic: /robot_description
      Enabled: true
      Name: RobotModel
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Name: TF
    - Class: rviz_default_plugins/Axes
      Enabled: true
      Length: 0.3
      Name: World Frame
      Radius: 0.01
      Reference Frame: world

  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: world
    Frame Rate: 30

  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.0
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1.0
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Pitch: 0.5
      Target Frame: world
      Yaw: 0.785
"""
    
    with open(output_path, 'w') as f:
        f.write(config)
    
    print(f"Default RViz config saved to {output_path}")


def launch_bilateral_visualization(
    num_robots: int = 2,
    show_leaders: bool = True,
    robot_type: str = "vx300s"
) -> Optional[PyBulletVisualizer]:
    """
    Launch bilateral visualization with specified configuration
    
    Args:
        num_robots: Number of robots (2 for bilateral, 4 for bimanual)
        show_leaders: Whether to show leader arms
        robot_type: Type of robot
        
    Returns:
        PyBulletVisualizer instance or None
    """
    launcher = VisualizationLauncher()
    return launcher.launch_pybullet(num_robots, robot_type, show_leaders)

