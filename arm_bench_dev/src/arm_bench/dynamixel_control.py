"""
Dynamixel Motor Control Module for arm-bench
Provides low-level motor control using Dynamixel SDK Protocol 2.0
"""

from typing import List, Dict, Optional, Tuple
import time


# Control table addresses for XM/XL series (Protocol 2.0)
class ControlTable:
    # EEPROM Area
    MODEL_NUMBER = 0
    MODEL_INFORMATION = 2
    FIRMWARE_VERSION = 6
    ID = 7
    BAUD_RATE = 8
    RETURN_DELAY_TIME = 9
    DRIVE_MODE = 10
    OPERATING_MODE = 11
    SECONDARY_ID = 12
    PROTOCOL_TYPE = 13
    HOMING_OFFSET = 20
    MOVING_THRESHOLD = 24
    TEMPERATURE_LIMIT = 31
    MAX_VOLTAGE_LIMIT = 32
    MIN_VOLTAGE_LIMIT = 34
    PWM_LIMIT = 36
    CURRENT_LIMIT = 38
    VELOCITY_LIMIT = 44
    MAX_POSITION_LIMIT = 48
    MIN_POSITION_LIMIT = 52
    
    # RAM Area
    TORQUE_ENABLE = 64
    LED = 65
    STATUS_RETURN_LEVEL = 68
    REGISTERED_INSTRUCTION = 69
    HARDWARE_ERROR_STATUS = 70
    VELOCITY_I_GAIN = 76
    VELOCITY_P_GAIN = 78
    POSITION_D_GAIN = 80
    POSITION_I_GAIN = 82
    POSITION_P_GAIN = 84
    FEEDFORWARD_2ND_GAIN = 88
    FEEDFORWARD_1ST_GAIN = 90
    BUS_WATCHDOG = 98
    GOAL_PWM = 100
    GOAL_CURRENT = 102
    GOAL_VELOCITY = 104
    PROFILE_ACCELERATION = 108
    PROFILE_VELOCITY = 112
    GOAL_POSITION = 116
    REALTIME_TICK = 120
    MOVING = 122
    MOVING_STATUS = 123
    PRESENT_PWM = 124
    PRESENT_CURRENT = 126
    PRESENT_VELOCITY = 128
    PRESENT_POSITION = 132
    VELOCITY_TRAJECTORY = 136
    POSITION_TRAJECTORY = 140
    PRESENT_INPUT_VOLTAGE = 144
    PRESENT_TEMPERATURE = 146


class DynamixelController:
    """
    Controller for Dynamixel motors using Protocol 2.0
    """
    
    def __init__(self, port: str = '/dev/ttyDXL', baudrate: int = 1000000):
        """
        Initialize the controller
        
        Args:
            port: Serial port path
            baudrate: Communication baud rate
        """
        self.port = port
        self.baudrate = baudrate
        self.port_handler = None
        self.packet_handler = None
        self.is_connected = False
        self.motors: Dict[int, Dict] = {}
        
        # Try to import dynamixel_sdk
        try:
            import dynamixel_sdk as dxl
            self.dxl = dxl
            self.PROTOCOL_VERSION = 2.0
        except ImportError:
            raise ImportError("Dynamixel SDK not installed. Install with: pip install dynamixel-sdk")
    
    def connect(self) -> bool:
        """
        Connect to the Dynamixel bus
        
        Returns:
            True if connection successful
        """
        try:
            self.port_handler = self.dxl.PortHandler(self.port)
            self.packet_handler = self.dxl.PacketHandler(self.PROTOCOL_VERSION)
            
            # Open port
            if not self.port_handler.openPort():
                print(f"Failed to open port {self.port}")
                return False
            
            # Set baudrate
            if not self.port_handler.setBaudRate(self.baudrate):
                print(f"Failed to set baudrate {self.baudrate}")
                self.port_handler.closePort()
                return False
            
            self.is_connected = True
            print(f"Connected to Dynamixel bus on {self.port} at {self.baudrate} baud")
            return True
            
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the Dynamixel bus"""
        if self.port_handler is not None:
            # Disable torque on all motors first
            for motor_id in self.motors:
                self.disable_torque(motor_id)
            
            self.port_handler.closePort()
            self.is_connected = False
            print("Disconnected from Dynamixel bus")
    
    def scan_motors(self, id_range: range = range(1, 20)) -> List[int]:
        """
        Scan for connected motors
        
        Args:
            id_range: Range of IDs to scan
            
        Returns:
            List of found motor IDs
        """
        if not self.is_connected:
            print("Not connected to bus")
            return []
        
        found_ids = []
        
        for motor_id in id_range:
            # Try to ping
            model_number, comm_result, error = self.packet_handler.ping(
                self.port_handler, motor_id
            )
            
            if comm_result == self.dxl.COMM_SUCCESS:
                found_ids.append(motor_id)
                self.motors[motor_id] = {
                    'id': motor_id,
                    'model_number': model_number,
                    'model': self._get_model_name(model_number)
                }
                print(f"Found motor ID {motor_id}: {self.motors[motor_id]['model']}")
        
        return found_ids
    
    def _get_model_name(self, model_number: int) -> str:
        """Get model name from model number"""
        model_names = {
            1020: "XM430-W210",
            1030: "XM430-W350",
            1060: "XL430-W250",
            1080: "XL330-M077",
            1090: "XL330-M288",
            1120: "XM430-W350",
        }
        return model_names.get(model_number, f"Model-{model_number}")
    
    def enable_torque(self, motor_id: int) -> bool:
        """
        Enable torque on a motor
        
        Args:
            motor_id: Motor ID to enable
            
        Returns:
            True if successful
        """
        if not self.is_connected:
            print("Not connected to bus")
            return False
        
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ControlTable.TORQUE_ENABLE, 1
        )
        
        if result != self.dxl.COMM_SUCCESS:
            print(f"Failed to enable torque on motor {motor_id}: {self.packet_handler.getTxRxResult(result)}")
            return False
        
        if error != 0:
            print(f"Motor {motor_id} error: {self.packet_handler.getRxPacketError(error)}")
            return False
        
        print(f"Torque enabled on motor {motor_id}")
        return True
    
    def disable_torque(self, motor_id: int) -> bool:
        """
        Disable torque on a motor
        
        Args:
            motor_id: Motor ID to disable
            
        Returns:
            True if successful
        """
        if not self.is_connected:
            return False
        
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ControlTable.TORQUE_ENABLE, 0
        )
        
        if result != self.dxl.COMM_SUCCESS:
            print(f"Failed to disable torque on motor {motor_id}")
            return False
        
        print(f"Torque disabled on motor {motor_id}")
        return True
    
    def enable_all_torque(self) -> bool:
        """Enable torque on all discovered motors"""
        success = True
        for motor_id in self.motors:
            if not self.enable_torque(motor_id):
                success = False
        return success
    
    def disable_all_torque(self) -> bool:
        """Disable torque on all discovered motors"""
        success = True
        for motor_id in self.motors:
            if not self.disable_torque(motor_id):
                success = False
        return success
    
    def enable_position_hold(self, motor_id: int) -> bool:
        """
        Enable position hold mode on a motor.
        This sets the motor to position control mode and enables torque
        so the motor holds its current position.
        
        Args:
            motor_id: Motor ID to enable position hold
            
        Returns:
            True if successful
        """
        if not self.is_connected:
            print("Not connected to bus")
            return False
        
        # First, read current position
        current_pos = self.read_position(motor_id)
        if current_pos is None:
            print(f"Failed to read position from motor {motor_id}")
            return False
        
        # Disable torque before changing operating mode
        self.disable_torque(motor_id)
        
        # Set to position control mode (mode 3)
        if not self.set_operating_mode(motor_id, 3):
            print(f"Failed to set position control mode on motor {motor_id}")
            return False
        
        # Set profile velocity for smooth motion
        self.set_profile_velocity(motor_id, 50)  # Reasonable velocity
        self.set_profile_acceleration(motor_id, 10)  # Smooth acceleration
        
        # Enable torque
        if not self.enable_torque(motor_id):
            return False
        
        # Set goal position to current position (hold in place)
        if not self.set_goal_position(motor_id, current_pos):
            print(f"Failed to set goal position on motor {motor_id}")
            return False
        
        print(f"Position hold enabled on motor {motor_id} at position {current_pos}")
        return True
    
    def enable_all_position_hold(self) -> bool:
        """
        Enable position hold mode on all discovered motors.
        Motors will hold their current position.
        
        Returns:
            True if all motors successfully enabled
        """
        print("Enabling position hold on all motors...")
        success = True
        for motor_id in self.motors:
            if not self.enable_position_hold(motor_id):
                success = False
                print(f"Warning: Failed to enable position hold on motor {motor_id}")
        
        if success:
            print("All motors are now in position hold mode")
        return success
    
    def set_stiff_mode(self, motor_id: int, stiffness: int = 800) -> bool:
        """
        Set motor to stiff mode with high position gains.
        This makes the motor resist manual movement.
        
        Args:
            motor_id: Motor ID
            stiffness: P gain value (0-16383, higher = stiffer)
            
        Returns:
            True if successful
        """
        if not self.is_connected:
            return False
        
        # Set high position P gain for stiffness
        result = self.set_pid_gains(motor_id, p=stiffness, i=0, d=100)
        
        if result:
            print(f"Stiff mode enabled on motor {motor_id} (P={stiffness})")
        
        return result
    
    def set_all_stiff_mode(self, stiffness: int = 800) -> bool:
        """Enable stiff mode on all motors"""
        success = True
        for motor_id in self.motors:
            if not self.set_stiff_mode(motor_id, stiffness):
                success = False
        return success
    
    def set_operating_mode(self, motor_id: int, mode: int) -> bool:
        """
        Set operating mode for a motor
        
        Args:
            motor_id: Motor ID
            mode: Operating mode:
                  0 = Current Control
                  1 = Velocity Control
                  3 = Position Control (default)
                  4 = Extended Position Control
                  5 = Current-based Position Control
                  16 = PWM Control
                  
        Returns:
            True if successful
        """
        # Must disable torque first
        self.disable_torque(motor_id)
        
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ControlTable.OPERATING_MODE, mode
        )
        
        if result != self.dxl.COMM_SUCCESS:
            print(f"Failed to set operating mode on motor {motor_id}")
            return False
        
        print(f"Operating mode set to {mode} on motor {motor_id}")
        return True
    
    def set_goal_position(self, motor_id: int, position: int) -> bool:
        """
        Set goal position for a motor
        
        Args:
            motor_id: Motor ID
            position: Goal position (0-4095 for 360 degrees)
            
        Returns:
            True if successful
        """
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, ControlTable.GOAL_POSITION, position
        )
        
        if result != self.dxl.COMM_SUCCESS:
            print(f"Failed to set goal position on motor {motor_id}")
            return False
        
        return True
    
    def set_goal_velocity(self, motor_id: int, velocity: int) -> bool:
        """
        Set goal velocity for a motor
        
        Args:
            motor_id: Motor ID
            velocity: Goal velocity (units depend on motor model)
            
        Returns:
            True if successful
        """
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, ControlTable.GOAL_VELOCITY, velocity
        )
        
        if result != self.dxl.COMM_SUCCESS:
            print(f"Failed to set goal velocity on motor {motor_id}")
            return False
        
        return True
    
    def set_goal_current(self, motor_id: int, current: int) -> bool:
        """
        Set goal current (torque) for a motor
        
        Args:
            motor_id: Motor ID
            current: Goal current in mA
            
        Returns:
            True if successful
        """
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler, motor_id, ControlTable.GOAL_CURRENT, current
        )
        
        if result != self.dxl.COMM_SUCCESS:
            print(f"Failed to set goal current on motor {motor_id}")
            return False
        
        return True
    
    def set_profile_velocity(self, motor_id: int, velocity: int) -> bool:
        """
        Set profile velocity (max velocity during position control)
        
        Args:
            motor_id: Motor ID
            velocity: Profile velocity
            
        Returns:
            True if successful
        """
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, ControlTable.PROFILE_VELOCITY, velocity
        )
        
        if result != self.dxl.COMM_SUCCESS:
            print(f"Failed to set profile velocity on motor {motor_id}")
            return False
        
        return True
    
    def set_profile_acceleration(self, motor_id: int, acceleration: int) -> bool:
        """
        Set profile acceleration
        
        Args:
            motor_id: Motor ID
            acceleration: Profile acceleration
            
        Returns:
            True if successful
        """
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, ControlTable.PROFILE_ACCELERATION, acceleration
        )
        
        if result != self.dxl.COMM_SUCCESS:
            print(f"Failed to set profile acceleration on motor {motor_id}")
            return False
        
        return True
    
    def read_position(self, motor_id: int) -> Optional[int]:
        """
        Read current position of a motor
        
        Args:
            motor_id: Motor ID
            
        Returns:
            Current position or None if failed
        """
        position, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor_id, ControlTable.PRESENT_POSITION
        )
        
        if result != self.dxl.COMM_SUCCESS:
            return None
        
        return position
    
    def read_velocity(self, motor_id: int) -> Optional[int]:
        """
        Read current velocity of a motor
        
        Args:
            motor_id: Motor ID
            
        Returns:
            Current velocity or None if failed
        """
        velocity, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor_id, ControlTable.PRESENT_VELOCITY
        )
        
        if result != self.dxl.COMM_SUCCESS:
            return None
        
        return velocity
    
    def read_current(self, motor_id: int) -> Optional[int]:
        """
        Read current current (torque) of a motor
        
        Args:
            motor_id: Motor ID
            
        Returns:
            Current in mA or None if failed
        """
        current, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, ControlTable.PRESENT_CURRENT
        )
        
        if result != self.dxl.COMM_SUCCESS:
            return None
        
        # Convert from unsigned to signed
        if current > 32767:
            current = current - 65536
        
        return current
    
    def read_temperature(self, motor_id: int) -> Optional[int]:
        """
        Read temperature of a motor
        
        Args:
            motor_id: Motor ID
            
        Returns:
            Temperature in Celsius or None if failed
        """
        temperature, result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor_id, ControlTable.PRESENT_TEMPERATURE
        )
        
        if result != self.dxl.COMM_SUCCESS:
            return None
        
        return temperature
    
    def read_all_motor_states(self) -> Dict[int, Dict]:
        """
        Read state of all motors
        
        Returns:
            Dictionary of motor states
        """
        states = {}
        
        for motor_id in self.motors:
            states[motor_id] = {
                'position': self.read_position(motor_id),
                'velocity': self.read_velocity(motor_id),
                'current': self.read_current(motor_id),
                'temperature': self.read_temperature(motor_id)
            }
        
        return states
    
    def set_led(self, motor_id: int, on: bool) -> bool:
        """
        Turn LED on/off
        
        Args:
            motor_id: Motor ID
            on: True to turn on, False to turn off
            
        Returns:
            True if successful
        """
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ControlTable.LED, 1 if on else 0
        )
        
        return result == self.dxl.COMM_SUCCESS
    
    def set_pid_gains(self, motor_id: int, p: int = None, i: int = None, d: int = None) -> bool:
        """
        Set PID gains for position control
        
        Args:
            motor_id: Motor ID
            p: Position P gain (0-16383)
            i: Position I gain (0-16383)
            d: Position D gain (0-16383)
            
        Returns:
            True if all successful
        """
        success = True
        
        if p is not None:
            result, _ = self.packet_handler.write2ByteTxRx(
                self.port_handler, motor_id, ControlTable.POSITION_P_GAIN, p
            )
            if result != self.dxl.COMM_SUCCESS:
                success = False
        
        if i is not None:
            result, _ = self.packet_handler.write2ByteTxRx(
                self.port_handler, motor_id, ControlTable.POSITION_I_GAIN, i
            )
            if result != self.dxl.COMM_SUCCESS:
                success = False
        
        if d is not None:
            result, _ = self.packet_handler.write2ByteTxRx(
                self.port_handler, motor_id, ControlTable.POSITION_D_GAIN, d
            )
            if result != self.dxl.COMM_SUCCESS:
                success = False
        
        return success


class GroupSyncController:
    """
    Controller for synchronized read/write to multiple motors
    """
    
    def __init__(self, controller: DynamixelController):
        """
        Initialize with an existing controller
        
        Args:
            controller: DynamixelController instance
        """
        self.controller = controller
        self.dxl = controller.dxl
        
        # Group sync instances
        self.group_sync_write_position = None
        self.group_sync_read_position = None
        
    def setup_sync_write_position(self, motor_ids: List[int]):
        """Setup synchronized position write for multiple motors"""
        self.group_sync_write_position = self.dxl.GroupSyncWrite(
            self.controller.port_handler,
            self.controller.packet_handler,
            ControlTable.GOAL_POSITION,
            4  # 4 bytes for position
        )
        
        for motor_id in motor_ids:
            self.group_sync_write_position.addParam(motor_id, [0, 0, 0, 0])
    
    def setup_sync_read_position(self, motor_ids: List[int]):
        """Setup synchronized position read for multiple motors"""
        self.group_sync_read_position = self.dxl.GroupSyncRead(
            self.controller.port_handler,
            self.controller.packet_handler,
            ControlTable.PRESENT_POSITION,
            4  # 4 bytes for position
        )
        
        for motor_id in motor_ids:
            self.group_sync_read_position.addParam(motor_id)
    
    def sync_write_positions(self, positions: Dict[int, int]) -> bool:
        """
        Write positions to multiple motors simultaneously
        
        Args:
            positions: Dictionary mapping motor ID to goal position
            
        Returns:
            True if successful
        """
        if self.group_sync_write_position is None:
            return False
        
        for motor_id, position in positions.items():
            # Convert position to byte array
            param = [
                self.dxl.DXL_LOBYTE(self.dxl.DXL_LOWORD(position)),
                self.dxl.DXL_HIBYTE(self.dxl.DXL_LOWORD(position)),
                self.dxl.DXL_LOBYTE(self.dxl.DXL_HIWORD(position)),
                self.dxl.DXL_HIBYTE(self.dxl.DXL_HIWORD(position))
            ]
            self.group_sync_write_position.changeParam(motor_id, param)
        
        result = self.group_sync_write_position.txPacket()
        return result == self.dxl.COMM_SUCCESS
    
    def sync_read_positions(self, motor_ids: List[int]) -> Dict[int, int]:
        """
        Read positions from multiple motors simultaneously
        
        Args:
            motor_ids: List of motor IDs to read
            
        Returns:
            Dictionary mapping motor ID to position
        """
        if self.group_sync_read_position is None:
            return {}
        
        result = self.group_sync_read_position.txRxPacket()
        
        if result != self.dxl.COMM_SUCCESS:
            return {}
        
        positions = {}
        for motor_id in motor_ids:
            if self.group_sync_read_position.isAvailable(motor_id, ControlTable.PRESENT_POSITION, 4):
                positions[motor_id] = self.group_sync_read_position.getData(
                    motor_id, ControlTable.PRESENT_POSITION, 4
                )
        
        return positions


def test_controller():
    """Test the Dynamixel controller"""
    import os
    
    port = '/dev/ttyDXL' if os.path.exists('/dev/ttyDXL') else '/dev/ttyUSB0'
    
    controller = DynamixelController(port=port)
    
    if not controller.connect():
        print("Failed to connect")
        return
    
    try:
        # Scan for motors
        motor_ids = controller.scan_motors(range(1, 10))
        print(f"\nFound {len(motor_ids)} motors: {motor_ids}")
        
        if motor_ids:
            # Enable torque on first motor
            motor_id = motor_ids[0]
            print(f"\nEnabling torque on motor {motor_id}...")
            controller.enable_torque(motor_id)
            
            # Read state
            print(f"\nMotor {motor_id} state:")
            print(f"  Position: {controller.read_position(motor_id)}")
            print(f"  Velocity: {controller.read_velocity(motor_id)}")
            print(f"  Current: {controller.read_current(motor_id)} mA")
            print(f"  Temperature: {controller.read_temperature(motor_id)}°C")
            
            # Wait for user
            input("\nPress Enter to disable torque and exit...")
            
            controller.disable_torque(motor_id)
    
    finally:
        controller.disconnect()


if __name__ == "__main__":
    test_controller()
