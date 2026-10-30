"""
Teleoperation module for arm-bench
Supports joystick and WebXR camera-based control

Based on manipulator_teleop WebXR implementation for AR/VR control
"""
import threading
import time
import json
import asyncio
from typing import Callable, Optional, Tuple, Dict, List
import numpy as np

# Try to import optional dependencies
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False

try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.staticfiles import StaticFiles
    from fastapi.responses import HTMLResponse, RedirectResponse
    import uvicorn
    FASTAPI_AVAILABLE = True
except ImportError:
    FASTAPI_AVAILABLE = False


class TeleoperationController:
    """Base class for teleoperation controllers"""
    
    def __init__(self, callback: Optional[Callable] = None):
        self.callback = callback
        self.running = False
        self.thread = None
        
    def start(self):
        """Start teleoperation"""
        if self.running:
            print("Teleoperation already running")
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        print(f"{self.__class__.__name__} started")
    
    def stop(self):
        """Stop teleoperation"""
        if not self.running:
            return
        
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        print(f"{self.__class__.__name__} stopped")
    
    def _run_loop(self):
        """Main teleoperation loop - override in subclasses"""
        raise NotImplementedError


class JoystickTeleoperation(TeleoperationController):
    """Joystick-based teleoperation using pygame"""
    
    def __init__(self, callback: Optional[Callable] = None, device_id: int = 0):
        super().__init__(callback)
        self.device_id = device_id
        self.joystick = None
        self.gripper_state = False  # Closed by default
        self.move_enabled = False
        self.deadzone = 0.1  # Joystick deadzone
        
    def _initialize_joystick(self) -> bool:
        """Initialize joystick device"""
        if not PYGAME_AVAILABLE:
            print("Error: pygame not installed")
            print("Install with: pip install pygame")
            return False
            
        try:
            pygame.init()
            pygame.joystick.init()
            
            if pygame.joystick.get_count() == 0:
                print("No joystick detected")
                print("Available devices: None")
                return False
            
            print(f"Found {pygame.joystick.get_count()} joystick(s):")
            for i in range(pygame.joystick.get_count()):
                js = pygame.joystick.Joystick(i)
                js.init()
                print(f"  [{i}] {js.get_name()}")
                js.quit()
            
            self.joystick = pygame.joystick.Joystick(self.device_id)
            self.joystick.init()
            
            print(f"\nUsing joystick: {self.joystick.get_name()}")
            print(f"  Axes: {self.joystick.get_numaxes()}")
            print(f"  Buttons: {self.joystick.get_numbuttons()}")
            print(f"  Hats: {self.joystick.get_numhats()}")
            
            return True
            
        except Exception as e:
            print(f"Error initializing joystick: {e}")
            return False
    
    def _apply_deadzone(self, value: float) -> float:
        """Apply deadzone to joystick axis"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def _run_loop(self):
        """Main joystick control loop"""
        if not self._initialize_joystick():
            self.running = False
            return
        
        try:
            print("\n" + "="*50)
            print("Joystick Teleoperation Active")
            print("="*50)
            print("Controls:")
            print("  Left Stick X:  Side movement (X)")
            print("  Left Stick Y:  Forward/Back (Y)")
            print("  Right Stick X: Rotation (Yaw)")
            print("  Right Stick Y: Up/Down (Z)")
            print("  Button 0 (A):  Toggle Gripper")
            print("  Button 1 (B):  Toggle Move Enable")
            print("  Button 6 (Back): Stop teleoperation")
            print("="*50 + "\n")
            
            while self.running:
                pygame.event.pump()
                
                # Read joystick axes
                num_axes = min(6, self.joystick.get_numaxes())
                axes = [self._apply_deadzone(self.joystick.get_axis(i)) for i in range(num_axes)]
                
                # Pad axes if fewer than 4
                while len(axes) < 4:
                    axes.append(0.0)
                
                # Map joystick to delta end-effector pose
                # These scale factors should be tuned for your robot
                delta_ee = {
                    'x': axes[0] * 0.01,       # Left stick X -> side movement
                    'y': -axes[1] * 0.01,      # Left stick Y -> forward/back (inverted)
                    'z': -axes[3] * 0.01,      # Right stick Y -> up/down (inverted)
                    'roll': 0.0,
                    'pitch': 0.0,
                    'yaw': axes[2] * 0.05,     # Right stick X -> rotation
                    'gripper_open': self.gripper_state,
                    'move_enabled': self.move_enabled
                }
                
                # Only send updates if there's significant movement or state change
                if self.move_enabled and any(abs(v) > 0.01 for k, v in delta_ee.items() if k in ['x', 'y', 'z', 'yaw']):
                    self._send_delta_ee(delta_ee)
                
                # Check buttons
                for i in range(min(10, self.joystick.get_numbuttons())):
                    if self.joystick.get_button(i):
                        self._handle_button(i)
                
                time.sleep(0.01)  # 100Hz update rate
                
        except Exception as e:
            print(f"Error in joystick loop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            pygame.quit()
    
    def _send_delta_ee(self, delta_ee: dict):
        """Send delta end-effector command"""
        if self.callback:
            self.callback(delta_ee)
    
    def _handle_button(self, button_id: int):
        """Handle button press"""
        time.sleep(0.2)  # Debounce
        
        if button_id == 0:  # A button - toggle gripper
            self.gripper_state = not self.gripper_state
            print(f"Gripper: {'OPEN' if self.gripper_state else 'CLOSED'}")
        elif button_id == 1:  # B button - toggle move enable
            self.move_enabled = not self.move_enabled
            print(f"Movement: {'ENABLED' if self.move_enabled else 'DISABLED'}")
        elif button_id == 6:  # Back button - stop
            print("Stop button pressed")
            self.running = False


class WebXRTeleoperation(TeleoperationController):
    """
    WebXR camera-based teleoperation
    Uses FastAPI WebSocket server for AR/VR hand tracking
    Based on manipulator_teleop/webxr_api_server
    """
    
    def __init__(self, callback: Optional[Callable] = None, host: str = "0.0.0.0", port: int = 8080):
        super().__init__(callback)
        self.host = host
        self.port = port
        self.server = None
        self.latest_pose = {}
        self.active_connections: List[WebSocket] = []
        self.move_enabled = False
        self.gripper_open = False
        
    def _run_loop(self):
        """Main WebXR control loop"""
        if not FASTAPI_AVAILABLE:
            print("Error: FastAPI not installed")
            print("Install with: pip install fastapi uvicorn")
            self.running = False
            return
            
        print("\n" + "="*60)
        print("WebXR Teleoperation Server")
        print("="*60)
        print(f"Server URL: http://{self._get_local_ip()}:{self.port}")
        print(f"WebSocket: ws://{self._get_local_ip()}:{self.port}/ws")
        print("")
        print("Open this URL on your AR/VR device to control the robot.")
        print("Use hand tracking or controllers for pose input.")
        print("="*60 + "\n")
        
        # Create FastAPI app
        app = FastAPI()
        
        @app.get("/")
        async def root():
            return {"message": "arm-bench WebXR Teleoperation Server", "status": "running"}
        
        @app.get("/server-info")
        async def server_info():
            return {
                "message": "arm-bench WebXR Teleop Server",
                "local_ip": self._get_local_ip(),
                "clients_connected": len(self.active_connections),
                "server_status": "running"
            }
        
        @app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            self.active_connections.append(websocket)
            print(f"WebXR client connected. Total: {len(self.active_connections)}")
            
            try:
                while self.running:
                    # Receive pose data from client
                    data = await websocket.receive_text()
                    pose_data = json.loads(data)
                    
                    # Update latest pose
                    self.latest_pose = pose_data
                    
                    # Extract control states
                    control = pose_data.get('control', {})
                    self.gripper_open = control.get('gripperOpen', False)
                    self.move_enabled = control.get('moveEnabled', False)
                    
                    # Convert to delta EE command
                    if self.move_enabled and 'position' in pose_data:
                        delta_ee = self._pose_to_delta_ee(pose_data)
                        self._send_delta_ee(delta_ee)
                    
            except WebSocketDisconnect:
                self.active_connections.remove(websocket)
                print(f"WebXR client disconnected. Total: {len(self.active_connections)}")
            except Exception as e:
                print(f"WebSocket error: {e}")
                if websocket in self.active_connections:
                    self.active_connections.remove(websocket)
        
        @app.get("/latest_pose")
        async def get_latest_pose():
            return self.latest_pose
        
        # Run the server
        try:
            uvicorn.run(app, host=self.host, port=self.port, log_level="info")
        except Exception as e:
            print(f"Server error: {e}")
        
    def _get_local_ip(self) -> str:
        """Get local IP address for display"""
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "127.0.0.1"
    
    def _pose_to_delta_ee(self, pose_data: dict) -> dict:
        """Convert WebXR pose to delta EE command"""
        position = pose_data.get('position', {})
        orientation = pose_data.get('orientation', {})
        control = pose_data.get('control', {})
        
        # Scale pose changes to appropriate velocities
        # These should be tuned based on the tracking space
        scale = 0.01
        
        delta_ee = {
            'x': position.get('x', 0) * scale,
            'y': position.get('y', 0) * scale,
            'z': position.get('z', 0) * scale,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,  # Could be computed from quaternion
            'gripper_open': control.get('gripperOpen', False),
            'move_enabled': control.get('moveEnabled', False)
        }
        
        return delta_ee
    
    def _send_delta_ee(self, delta_ee: dict):
        """Send delta end-effector command"""
        if self.callback:
            self.callback(delta_ee)


class DifferentialIKNode:
    """
    Abstraction for differential IK computation
    Receives delta EE commands and computes joint velocities
    Uses MInK (Minimum Inverse Kinematics) library for computation
    """
    
    def __init__(self, robot_description: str = None):
        self.robot_description = robot_description
        self.jacobian = None
        self.current_joint_state = None
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # Try to initialize MInK if available
        try:
            import mink
            self.mink_available = True
            print("MInK IK solver available")
        except ImportError:
            self.mink_available = False
            print("MInK not available - using simple mapping")
        
    def compute_joint_velocities(self, delta_ee: dict) -> dict:
        """
        Compute joint velocities from delta end-effector pose
        
        Args:
            delta_ee: Dictionary with x, y, z, roll, pitch, yaw deltas
            
        Returns:
            Dictionary with joint velocities
        """
        if self.mink_available:
            return self._compute_with_mink(delta_ee)
        else:
            return self._compute_simple(delta_ee)
    
    def _compute_with_mink(self, delta_ee: dict) -> dict:
        """Compute using MInK differential IK"""
        # This would use the MInK library for proper differential IK
        # For now, fall back to simple computation
        return self._compute_simple(delta_ee)
    
    def _compute_simple(self, delta_ee: dict) -> dict:
        """Simple mapping from delta EE to joint velocities"""
        # Simple proportional mapping (should be replaced with proper IK)
        joint_velocities = {}
        
        # Map delta EE to joint velocities
        # This is a simplified approximation
        joint_velocities['joint_1'] = delta_ee.get('yaw', 0) * 2.0   # Base rotation
        joint_velocities['joint_2'] = delta_ee.get('z', 0) * 1.0    # Shoulder
        joint_velocities['joint_3'] = delta_ee.get('y', 0) * 1.0    # Elbow
        joint_velocities['joint_4'] = delta_ee.get('x', 0) * 1.0    # Wrist 1
        joint_velocities['joint_5'] = delta_ee.get('pitch', 0) * 0.5  # Wrist 2
        joint_velocities['joint_6'] = delta_ee.get('roll', 0) * 0.5   # Wrist 3
        
        return joint_velocities
    
    def send_to_robot(self, joint_velocities: dict):
        """Send joint velocities to robot"""
        # This would publish to ROS2 topic or send to robot controller
        print(f"[Robot Command] Joint velocities: {joint_velocities}")


class ArmBenchTeleoperator:
    """
    High-level teleoperation manager for arm-bench
    Integrates joystick and WebXR control with robot control
    """
    
    def __init__(self, controller=None):
        """
        Initialize teleoperator
        
        Args:
            controller: DynamixelController instance for motor control
        """
        self.controller = controller
        self.ik_node = DifferentialIKNode()
        self.current_teleop = None
        self.teleop_mode = None
        
    def start_joystick(self, device_id: int = 0):
        """Start joystick teleoperation"""
        if self.current_teleop:
            self.current_teleop.stop()
        
        self.current_teleop = JoystickTeleoperation(
            callback=self._handle_delta_ee,
            device_id=device_id
        )
        self.teleop_mode = 'joystick'
        self.current_teleop.start()
    
    def start_webxr(self, port: int = 8080):
        """Start WebXR teleoperation"""
        if self.current_teleop:
            self.current_teleop.stop()
        
        self.current_teleop = WebXRTeleoperation(
            callback=self._handle_delta_ee,
            port=port
        )
        self.teleop_mode = 'webxr'
        self.current_teleop.start()
    
    def stop(self):
        """Stop current teleoperation"""
        if self.current_teleop:
            self.current_teleop.stop()
            self.current_teleop = None
            self.teleop_mode = None
    
    def _handle_delta_ee(self, delta_ee: dict):
        """Handle delta EE command from teleoperation"""
        # Only move if movement is enabled
        if not delta_ee.get('move_enabled', True):
            return
        
        # Compute joint velocities
        joint_velocities = self.ik_node.compute_joint_velocities(delta_ee)
        
        # Send to robot
        if self.controller:
            self._send_to_controller(joint_velocities)
        else:
            self.ik_node.send_to_robot(joint_velocities)
        
        # Handle gripper
        if 'gripper_open' in delta_ee:
            self._handle_gripper(delta_ee['gripper_open'])
    
    def _send_to_controller(self, joint_velocities: dict):
        """Send joint velocities to Dynamixel controller"""
        if not self.controller or not self.controller.is_connected:
            return
        
        # Convert velocities to position increments
        # This is a simple integration - real implementation would use velocity control mode
        for joint_name, velocity in joint_velocities.items():
            joint_idx = int(joint_name.split('_')[1])  # Extract joint number
            if joint_idx in self.controller.motors:
                current_pos = self.controller.read_position(joint_idx) or 2048
                new_pos = int(current_pos + velocity * 100)  # Scale velocity to position delta
                new_pos = max(0, min(4095, new_pos))  # Clamp to valid range
                self.controller.set_goal_position(joint_idx, new_pos)
    
    def _handle_gripper(self, open_state: bool):
        """Handle gripper open/close command"""
        if not self.controller or not self.controller.is_connected:
            print(f"Gripper: {'OPEN' if open_state else 'CLOSED'}")
            return
        
        # Gripper is typically motor ID 8 or 9
        gripper_id = 8
        if gripper_id in self.controller.motors:
            if open_state:
                self.controller.set_goal_position(gripper_id, 3000)  # Open position
            else:
                self.controller.set_goal_position(gripper_id, 2048)  # Closed position


def start_joystick_teleop(callback: Optional[Callable] = None) -> JoystickTeleoperation:
    """Start joystick teleoperation"""
    teleop = JoystickTeleoperation(callback)
    teleop.start()
    return teleop


def start_webxr_teleop(callback: Optional[Callable] = None, port: int = 8080) -> WebXRTeleoperation:
    """Start WebXR teleoperation"""
    teleop = WebXRTeleoperation(callback, port=port)
    teleop.start()
    return teleop

