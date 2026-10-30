"""
Interbotix workspace builder and utilities
Handles building and managing Interbotix robot arm workspace
"""
import subprocess
import os
from typing import Optional
from pathlib import Path


class InterbotixWorkspaceBuilder:
    """Build and manage Interbotix ROS2 workspace"""
    
    def __init__(self, workspace_path: Optional[str] = None):
        self.workspace_path = workspace_path or os.path.expanduser("~/interbotix_ws")
        self.src_path = os.path.join(self.workspace_path, "src")
        
    def check_ros2_installed(self) -> bool:
        """Check if ROS2 is installed"""
        ros_distro = os.environ.get('ROS_DISTRO', None)
        
        if not ros_distro:
            print("ROS2 not detected. Please install ROS2 and source the setup file.")
            print("Example: source /opt/ros/humble/setup.bash")
            return False
        
        print(f"ROS2 detected: {ros_distro}")
        return True
    
    def create_workspace(self) -> bool:
        """Create ROS2 workspace directory structure"""
        try:
            os.makedirs(self.src_path, exist_ok=True)
            print(f"Workspace created at: {self.workspace_path}")
            return True
        except Exception as e:
            print(f"Error creating workspace: {e}")
            return False
    
    def clone_interbotix_repos(self) -> bool:
        """Clone Interbotix ROS packages"""
        repos = [
            {
                "name": "interbotix_ros_manipulators",
                "url": "https://github.com/Interbotix/interbotix_ros_manipulators.git",
                "branch": "rolling"  # or humble, jazzy depending on ROS2 version
            },
            {
                "name": "interbotix_ros_core",
                "url": "https://github.com/Interbotix/interbotix_ros_core.git",
                "branch": "rolling"
            }
        ]
        
        print("Cloning Interbotix repositories...")
        
        for repo in repos:
            repo_path = os.path.join(self.src_path, repo["name"])
            
            if os.path.exists(repo_path):
                print(f"Repository {repo['name']} already exists, skipping...")
                continue
            
            try:
                cmd = [
                    "git", "clone",
                    "-b", repo["branch"],
                    repo["url"],
                    repo_path
                ]
                
                result = subprocess.run(cmd, check=True, capture_output=True, text=True)
                print(f"Cloned {repo['name']}")
                
            except subprocess.CalledProcessError as e:
                print(f"Error cloning {repo['name']}: {e}")
                return False
        
        return True
    
    def install_dependencies(self) -> bool:
        """Install ROS dependencies using rosdep"""
        if not self.check_ros2_installed():
            return False
        
        print("Installing dependencies with rosdep...")
        
        try:
            # Initialize rosdep if needed
            subprocess.run(["sudo", "rosdep", "init"], capture_output=True)
            subprocess.run(["rosdep", "update"], check=True, capture_output=True)
            
            # Install dependencies
            cmd = [
                "rosdep", "install",
                "--from-paths", self.src_path,
                "--ignore-src",
                "-r", "-y"
            ]
            
            result = subprocess.run(
                cmd,
                cwd=self.workspace_path,
                check=True,
                capture_output=True,
                text=True
            )
            
            print("Dependencies installed successfully")
            return True
            
        except subprocess.CalledProcessError as e:
            print(f"Error installing dependencies: {e}")
            print(e.stderr)
            return False
    
    def build_workspace(self) -> bool:
        """Build the ROS2 workspace"""
        if not self.check_ros2_installed():
            return False
        
        print(f"Building workspace: {self.workspace_path}")
        
        try:
            cmd = ["colcon", "build", "--symlink-install"]
            
            result = subprocess.run(
                cmd,
                cwd=self.workspace_path,
                check=True,
                capture_output=True,
                text=True
            )
            
            print("Workspace built successfully!")
            print(f"\nTo use the workspace, run:")
            print(f"source {self.workspace_path}/install/setup.bash")
            
            return True
            
        except subprocess.CalledProcessError as e:
            print(f"Error building workspace: {e}")
            print(e.stderr)
            return False
    
    def setup_full_workspace(self) -> bool:
        """Complete workspace setup process"""
        print("=== Interbotix Workspace Setup ===")
        
        if not self.check_ros2_installed():
            return False
        
        steps = [
            ("Creating workspace", self.create_workspace),
            ("Cloning repositories", self.clone_interbotix_repos),
            ("Installing dependencies", self.install_dependencies),
            ("Building workspace", self.build_workspace),
        ]
        
        for step_name, step_func in steps:
            print(f"\n{step_name}...")
            if not step_func():
                print(f"Failed at: {step_name}")
                return False
        
        print("\n=== Setup Complete! ===")
        print(f"Workspace location: {self.workspace_path}")
        print(f"\nNext steps:")
        print(f"1. source {self.workspace_path}/install/setup.bash")
        print(f"2. Launch your robot with appropriate launch file")
        
        return True
    
    def get_available_robots(self) -> list:
        """List available Interbotix robot models"""
        # Common Interbotix arms
        robots = [
            "px100",
            "px150",
            "rx150",
            "rx200",
            "vx250",
            "vx300",
            "vx300s",
            "wx200",
            "wx250",
            "wx250s"
        ]
        
        return robots
    
    def launch_robot(self, robot_model: str, use_sim: bool = False) -> bool:
        """
        Launch Interbotix robot
        
        Args:
            robot_model: Robot model name (e.g., 'vx300s')
            use_sim: If True, launch in simulation mode
        """
        if not self.check_ros2_installed():
            return False
        
        # Source workspace
        setup_file = os.path.join(self.workspace_path, "install", "setup.bash")
        
        if not os.path.exists(setup_file):
            print(f"Workspace not built. Please run setup_full_workspace() first.")
            return False
        
        try:
            launch_cmd = [
                "ros2", "launch",
                "interbotix_xsarm_control",
                "xsarm_control.launch.py",
                f"robot_model:={robot_model}"
            ]
            
            if use_sim:
                launch_cmd.append("use_sim:=true")
            
            print(f"Launching {robot_model}...")
            print(f"Command: {' '.join(launch_cmd)}")
            
            # This would launch in background
            process = subprocess.Popen(launch_cmd)
            
            print(f"Robot launched with PID: {process.pid}")
            return True
            
        except Exception as e:
            print(f"Error launching robot: {e}")
            return False


def build_interbotix_workspace(workspace_path: Optional[str] = None) -> bool:
    """Convenience function to build Interbotix workspace"""
    builder = InterbotixWorkspaceBuilder(workspace_path)
    return builder.setup_full_workspace()
