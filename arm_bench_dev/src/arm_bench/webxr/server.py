"""
WebXR Teleoperation Server for arm-bench
Based on manipulator_teleop/webxr_api_server implementation

Provides camera-based AR teleoperation using mobile devices with WebXR support
"""

# Postponed annotations: the WebSocket type hints below must not be evaluated
# when fastapi is missing, otherwise importing arm_bench fails entirely.
from __future__ import annotations

import json
import asyncio
import ssl
import os
import socket
import logging
import subprocess
from pathlib import Path
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass, field

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Check for FastAPI
try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.staticfiles import StaticFiles
    from fastapi.responses import HTMLResponse, RedirectResponse, JSONResponse
    import uvicorn
    FASTAPI_AVAILABLE = True
except ImportError:
    FASTAPI_AVAILABLE = False
    logger.warning("FastAPI not installed. Install with: pip install fastapi uvicorn")


@dataclass
class WebXRPose:
    """Represents a WebXR pose with position and orientation"""
    position: Dict[str, float] = field(default_factory=lambda: {'x': 0.0, 'y': 0.0, 'z': 0.0})
    orientation: Dict[str, float] = field(default_factory=lambda: {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})
    gripper_open: bool = False
    move_enabled: bool = False
    timestamp: str = ""


class ConnectionManager:
    """Manages WebSocket connections"""
    
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.latest_pose: Dict = {}
        self.connection_count: int = 0

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        self.connection_count += 1
        logger.info(f"New client connected. Total clients: {self.connection_count}")

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
            self.connection_count -= 1
            logger.info(f"Client disconnected. Total clients: {self.connection_count}")

    async def broadcast(self, message: Dict):
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except RuntimeError:
                disconnected.append(connection)
        
        for conn in disconnected:
            self.disconnect(conn)
    
    def update_pose(self, pose_data: Dict):
        self.latest_pose = pose_data

    def get_latest_pose(self) -> Dict:
        return self.latest_pose
        
    async def disconnect_all(self):
        logger.info(f"Disconnecting all {len(self.active_connections)} clients")
        for connection in self.active_connections[:]:
            try:
                await connection.close()
            except Exception:
                pass
            self.disconnect(connection)


class WebXRTeleoperationServer:
    """
    WebXR Teleoperation Server for arm-bench
    
    Provides AR-based control for robot arms using mobile devices with WebXR support.
    """
    
    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 8443,
        callback: Optional[Callable] = None,
        controller = None,
        use_https: bool = True
    ):
        """
        Initialize WebXR server
        
        Args:
            host: Server host address
            port: Server port (default 8443 for HTTPS)
            callback: Callback function for pose updates
            controller: DynamixelController instance for motor control
            use_https: Use HTTPS (required for WebXR on non-localhost)
        """
        if not FASTAPI_AVAILABLE:
            raise ImportError("FastAPI not installed. Install with: pip install fastapi uvicorn")
        
        self.host = host
        self.port = port
        self.callback = callback
        self.controller = controller
        self.use_https = use_https
        
        self.manager = ConnectionManager()
        self.app = None
        self.server_task = None
        self.running = False
        
        # Get static files directory
        self.static_dir = Path(__file__).parent / "static"
        
        # Ensure static directory exists
        self.static_dir.mkdir(parents=True, exist_ok=True)
        
        # Certificate paths
        self.certs_dir = Path(__file__).parent / "certs"
        self.cert_file = self.certs_dir / "cert.pem"
        self.key_file = self.certs_dir / "key.pem"
        
        # Create the FastAPI app
        self._create_app()
    
    def _create_app(self):
        """Create the FastAPI application"""
        self.app = FastAPI(title="arm-bench WebXR Teleoperation")
        
        # Mount static files
        if self.static_dir.exists():
            self.app.mount("/static", StaticFiles(directory=str(self.static_dir)), name="static")
        
        # Routes
        @self.app.get("/", response_class=HTMLResponse)
        async def root():
            return RedirectResponse(url="/static/index.html")
        
        @self.app.get("/server-info")
        async def server_info():
            return {
                "message": "arm-bench WebXR Teleop Server",
                "local_ip": self._get_local_ip(),
                "clients_connected": self.manager.connection_count,
                "server_status": "running",
                "controller_status": "connected" if self.controller and self.controller.is_connected else "not connected"
            }
        
        @self.app.get("/latest_pose")
        async def get_latest_pose():
            return self.manager.get_latest_pose()
        
        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            client_ip = websocket.client.host if websocket.client else "unknown"
            logger.info(f"WebSocket connection attempt from: {client_ip}")
            await self.manager.connect(websocket)
            
            try:
                while True:
                    # Receive pose data from client
                    data = await websocket.receive_text()
                    pose_data = json.loads(data)
                    
                    # Update the latest pose
                    self.manager.update_pose(pose_data)
                    
                    # Process the pose data
                    self._process_pose(pose_data)
                    
                    # Broadcast to other clients
                    await self.manager.broadcast(pose_data)
                    
            except WebSocketDisconnect:
                self.manager.disconnect(websocket)
                logger.info(f"Client {client_ip} disconnected")
            except Exception as e:
                logger.error(f"Error with client {client_ip}: {str(e)}")
                self.manager.disconnect(websocket)
    
    def _process_pose(self, pose_data: Dict):
        """Process received pose data"""
        try:
            # Extract data
            position = pose_data.get('position', {})
            orientation = pose_data.get('orientation', {})
            control = pose_data.get('control', {})
            
            gripper_open = control.get('gripperOpen', False)
            move_enabled = control.get('moveEnabled', False)
            
            # Log the pose
            logger.debug(
                f"Pose: pos({position.get('x', 0):.3f}, {position.get('y', 0):.3f}, {position.get('z', 0):.3f}) "
                f"gripper={'OPEN' if gripper_open else 'CLOSED'} move={'ON' if move_enabled else 'OFF'}"
            )
            
            # Call the callback if provided
            if self.callback:
                delta_ee = self._pose_to_delta_ee(pose_data)
                self.callback(delta_ee)
            
            # Send to controller if connected and move is enabled
            if self.controller and self.controller.is_connected and move_enabled:
                self._send_to_controller(pose_data)
            
            # Handle gripper
            if self.controller and self.controller.is_connected:
                self._handle_gripper(gripper_open)
                
        except Exception as e:
            logger.error(f"Error processing pose: {e}")
    
    def _pose_to_delta_ee(self, pose_data: Dict) -> Dict:
        """Convert WebXR pose to delta end-effector command"""
        position = pose_data.get('position', {})
        orientation = pose_data.get('orientation', {})
        control = pose_data.get('control', {})
        
        # Scale pose changes to appropriate velocities
        scale = 0.01
        
        delta_ee = {
            'x': float(position.get('x', 0)) * scale,
            'y': float(position.get('y', 0)) * scale,
            'z': float(position.get('z', 0)) * scale,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'gripper_open': control.get('gripperOpen', False),
            'move_enabled': control.get('moveEnabled', False)
        }
        
        return delta_ee
    
    def _send_to_controller(self, pose_data: Dict):
        """Send pose to Dynamixel controller"""
        if not self.controller:
            return
        
        # This would implement the actual motor control
        # For now, just log
        logger.debug("Sending pose to controller")
    
    def _handle_gripper(self, open_state: bool):
        """Handle gripper open/close"""
        if not self.controller:
            return
        
        # Gripper is typically motor ID 8
        gripper_id = 8
        if gripper_id in self.controller.motors:
            if open_state:
                self.controller.set_goal_position(gripper_id, 3000)  # Open
            else:
                self.controller.set_goal_position(gripper_id, 2048)  # Closed
    
    def _get_local_ip(self) -> str:
        """Get local IP address"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"
    
    def _setup_https(self) -> bool:
        """Generate self-signed certificates for HTTPS"""
        self.certs_dir.mkdir(parents=True, exist_ok=True)
        
        if self.cert_file.exists() and self.key_file.exists():
            logger.info("Using existing SSL certificates")
            return True
        
        logger.info("Generating self-signed SSL certificates...")
        
        try:
            # Generate self-signed certificate using openssl
            cmd = [
                "openssl", "req", "-x509", "-newkey", "rsa:4096",
                "-keyout", str(self.key_file),
                "-out", str(self.cert_file),
                "-days", "365",
                "-nodes",
                "-subj", f"/CN={self._get_local_ip()}"
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                logger.info("SSL certificates generated successfully")
                return True
            else:
                logger.error(f"Failed to generate certificates: {result.stderr}")
                return False
                
        except FileNotFoundError:
            logger.error("OpenSSL not found. Please install OpenSSL or disable HTTPS.")
            return False
        except Exception as e:
            logger.error(f"Error generating certificates: {e}")
            return False
    
    def run(self):
        """Run the server (blocking)"""
        self.running = True
        
        local_ip = self._get_local_ip()
        protocol = "https" if self.use_https else "http"
        
        print("\n" + "=" * 60)
        print("arm-bench WebXR Teleoperation Server")
        print("=" * 60)
        print(f"Address: {local_ip}:{self.port}")
        print("")
        print("Access URLs:")
        print(f"  - Main App: {protocol}://{local_ip}:{self.port}/static/index.html")
        print(f"  - Server Info: {protocol}://{local_ip}:{self.port}/server-info")
        print("")
        print("Instructions:")
        print("  1. Open the URL on your mobile device (same network)")
        print("  2. Accept the security warning (self-signed certificate)")
        print("  3. Connect to WebSocket and start AR session")
        print("  4. Use hand movements to control the robot")
        print("=" * 60 + "\n")
        
        # Setup HTTPS if needed
        ssl_context = None
        if self.use_https:
            if self._setup_https():
                ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
                ssl_context.load_cert_chain(str(self.cert_file), str(self.key_file))
            else:
                logger.warning("Failed to setup HTTPS, falling back to HTTP")
                self.use_https = False
        
        # Run the server
        try:
            if ssl_context:
                uvicorn.run(
                    self.app,
                    host=self.host,
                    port=self.port,
                    ssl_keyfile=str(self.key_file),
                    ssl_certfile=str(self.cert_file),
                    log_level="info"
                )
            else:
                uvicorn.run(
                    self.app,
                    host=self.host,
                    port=self.port,
                    log_level="info"
                )
        except Exception as e:
            logger.error(f"Server error: {e}")
        finally:
            self.running = False
    
    async def run_async(self):
        """Run the server asynchronously"""
        self.running = True
        
        config = uvicorn.Config(
            self.app,
            host=self.host,
            port=self.port,
            log_level="info"
        )
        
        if self.use_https and self._setup_https():
            config.ssl_keyfile = str(self.key_file)
            config.ssl_certfile = str(self.cert_file)
        
        server = uvicorn.Server(config)
        await server.serve()
    
    def stop(self):
        """Stop the server"""
        self.running = False


def create_webxr_server(
    host: str = "0.0.0.0",
    port: int = 8443,
    callback: Callable = None,
    controller = None,
    use_https: bool = True
) -> WebXRTeleoperationServer:
    """
    Create a WebXR teleoperation server
    
    Args:
        host: Server host
        port: Server port
        callback: Callback for pose updates
        controller: DynamixelController instance
        use_https: Use HTTPS (required for WebXR)
    
    Returns:
        WebXRTeleoperationServer instance
    """
    return WebXRTeleoperationServer(
        host=host,
        port=port,
        callback=callback,
        controller=controller,
        use_https=use_https
    )


def start_webxr_server(
    port: int = 8443,
    callback: Callable = None,
    controller = None
):
    """Start the WebXR server (blocking)"""
    server = create_webxr_server(port=port, callback=callback, controller=controller)
    server.run()


if __name__ == "__main__":
    # Test the server
    start_webxr_server()
