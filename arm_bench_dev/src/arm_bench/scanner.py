"""
Hardware scanner for arm-bench
Scans for Dynamixel motors using Protocol 2.0 and cameras
"""
import yaml
import os
import glob
from typing import List, Dict, Optional


def scan_serial_ports() -> List[str]:
    """Scan for available serial ports"""
    ports = []
    
    # Check for Interbotix-specific udev rule first
    if os.path.exists('/dev/ttyDXL'):
        ports.append('/dev/ttyDXL')
        return ports
    
    # Check common serial port patterns
    patterns = [
        '/dev/ttyUSB*',  # Linux USB
        '/dev/ttyACM*',  # Linux ACM
        '/dev/cu.usbserial*',  # macOS USB serial
        '/dev/cu.usbmodem*',  # macOS USB modem
    ]
    
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    
    return sorted(ports)


def scan_dynamixel_motors(port: str = None, simulate: bool = False, baudrate: int = 1000000) -> List[Dict]:
    """
    Scan for Dynamixel motors using Protocol 2.0
    
    Args:
        port: Specific port to scan, or None to scan all
        simulate: If True, return simulated motors
        baudrate: Baud rate for serial communication (default: 1000000)
        
    Returns:
        List of detected motors with their properties
    """
    if simulate:
        # Simulated Interbotix ViperX 300 6DOF arm (9 motors total)
        return [
            {
                "id": 1,
                "model": "XM430-W350",
                "port": "SIM_PORT",
                "position": 2048,
                "velocity": 0,
                "torque": 0,
                "temperature": 32
            },
            {
                "id": 2,
                "model": "XM430-W350",
                "port": "SIM_PORT",
                "position": 2048,
                "velocity": 0,
                "torque": 0,
                "temperature": 33
            },
            {
                "id": 3,
                "model": "XM430-W350",
                "port": "SIM_PORT",
                "position": 2048,
                "velocity": 0,
                "torque": 0,
                "temperature": 31
            },
            {
                "id": 4,
                "model": "XM430-W350",
                "port": "SIM_PORT",
                "position": 2048,
                "velocity": 0,
                "torque": 0,
                "temperature": 30
            },
            {
                "id": 5,
                "model": "XM430-W350",
                "port": "SIM_PORT",
                "position": 2048,
                "velocity": 0,
                "torque": 0,
                "temperature": 29
            },
            {
                "id": 6,
                "model": "XM430-W350",
                "port": "SIM_PORT",
                "position": 2048,
                "velocity": 0,
                "torque": 0,
                "temperature": 28
            },
            {
                "id": 7,
                "model": "XM430-W350",
                "port": "SIM_PORT",
                "position": 2048,
                "velocity": 0,
                "torque": 0,
                "temperature": 30
            },
            {
                "id": 8,
                "model": "XM430-W210",
                "port": "SIM_PORT",
                "position": 2048,
                "velocity": 0,
                "torque": 0,
                "temperature": 34
            },
            {
                "id": 9,
                "model": "XM430-W210",
                "port": "SIM_PORT",
                "position": 2048,
                "velocity": 0,
                "torque": 0,
                "temperature": 33
            }
        ]
    
    motors = []
    ports_to_scan = [port] if port else scan_serial_ports()
    
    try:
        import dynamixel_sdk as dxl
        
        # Protocol version
        PROTOCOL_VERSION = 2.0
        
        # Control table addresses for XM/XL series
        ADDR_MODEL_NUMBER = 0
        ADDR_PRESENT_POSITION = 132
        ADDR_PRESENT_VELOCITY = 128
        ADDR_PRESENT_CURRENT = 126
        ADDR_PRESENT_TEMPERATURE = 146
        
        # Model number mapping
        MODEL_NAMES = {
            1020: "XM430-W210",
            1030: "XM430-W350",
            1060: "XL430-W250",
            1080: "XL330-M077",
            1090: "XL330-M288",
            1120: "XM430-W350",  # Alternative model number
        }
        
        for port_path in ports_to_scan:
            try:
                print(f"Scanning port: {port_path}")
                
                # Initialize PortHandler and PacketHandler
                port_handler = dxl.PortHandler(port_path)
                packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)
                
                # Open port
                if not port_handler.openPort():
                    print(f"Failed to open port {port_path}")
                    continue
                
                # Set baudrate
                if not port_handler.setBaudRate(baudrate):
                    print(f"Failed to set baudrate on {port_path}")
                    port_handler.closePort()
                    continue
                
                print(f"Successfully opened {port_path} at {baudrate} baud")
                
                # Scan for motors (IDs 1-9 for ViperX 300)
                for motor_id in range(1, 10):
                    # Try to ping motor
                    dxl_model_number, dxl_comm_result, dxl_error = packet_handler.ping(port_handler, motor_id)
                    
                    if dxl_comm_result == dxl.COMM_SUCCESS:
                        print(f"  Found motor ID {motor_id}")
                        
                        # Read motor properties
                        model_name = MODEL_NAMES.get(dxl_model_number, f"Model-{dxl_model_number}")
                        
                        # Read position
                        position, _, _ = packet_handler.read4ByteTxRx(port_handler, motor_id, ADDR_PRESENT_POSITION)
                        
                        # Read velocity
                        velocity, _, _ = packet_handler.read4ByteTxRx(port_handler, motor_id, ADDR_PRESENT_VELOCITY)
                        
                        # Read temperature
                        temperature, _, _ = packet_handler.read1ByteTxRx(port_handler, motor_id, ADDR_PRESENT_TEMPERATURE)
                        
                        motors.append({
                            "id": motor_id,
                            "model": model_name,
                            "model_number": dxl_model_number,
                            "port": port_path,
                            "position": position,
                            "velocity": velocity,
                            "torque": 0,
                            "temperature": temperature
                        })
                
                # Close port
                port_handler.closePort()
                
                # If we found motors on this port, don't scan others
                if motors:
                    break
                    
            except Exception as e:
                print(f"Error scanning port {port_path}: {e}")
                
    except ImportError:
        print("Dynamixel SDK not installed. Install with: pip install dynamixel-sdk")
        print("Using simulation mode instead.")
        return scan_dynamixel_motors(simulate=True)
    
    return motors


def scan_cameras() -> List[Dict]:
    """
    Scan for available cameras
    
    Returns:
        List of detected cameras
    """
    cameras = []
    
    # Try to detect cameras (0-10 is usually sufficient)
    try:
        import cv2
        import os
        
        # Suppress OpenCV warnings temporarily
        os.environ['OPENCV_LOG_LEVEL'] = 'ERROR'
        
        # Check which video devices actually exist before trying to open
        video_devices = []
        for idx in range(10):
            device_path = f"/dev/video{idx}"
            if os.path.exists(device_path):
                video_devices.append(idx)
        
        for idx in video_devices:
            try:
                # Use V4L2 backend explicitly and set timeout
                cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                if cap.isOpened():
                    # Try to read a frame to verify it's a real camera
                    ret, _ = cap.read()
                    if ret:
                        # Get camera properties
                        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        fps = int(cap.get(cv2.CAP_PROP_FPS))
                        
                        cameras.append({
                            "id": idx,
                            "name": f"Camera {idx}",
                            "resolution": f"{width}x{height}",
                            "fps": fps if fps > 0 else 30,
                            "device": f"/dev/video{idx}"
                        })
                    cap.release()
            except Exception:
                # Silently skip problematic cameras
                pass
                
    except ImportError:
        print("OpenCV not installed. Cannot scan cameras (pip install opencv-python).")
    except Exception as e:
        print(f"Error scanning cameras: {e}")
    
    return cameras


def save_hardware_config(motors: List[Dict], cameras: List[Dict] = None):
    """Save hardware configuration to file"""
    path = os.path.expanduser("~/.arm_bench_config.yaml")
    
    config = {
        "motors": motors,
        "cameras": cameras or []
    }
    
    with open(path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)
    
    print(f"Hardware configuration saved to {path}")


def load_hardware_config() -> Optional[Dict]:
    """Load hardware configuration from file"""
    path = os.path.expanduser("~/.arm_bench_config.yaml")
    
    if not os.path.exists(path):
        return None
    
    with open(path, 'r') as f:
        config = yaml.safe_load(f)
    
    return config


def scan_ports(simulate=False):
    """Legacy function for compatibility"""
    return scan_dynamixel_motors(simulate=simulate)


def save_config(motors):
    """Legacy function for compatibility"""
    save_hardware_config(motors)