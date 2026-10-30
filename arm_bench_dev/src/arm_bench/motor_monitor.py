"""
Live motor status for the Motor Status tab — only motors that actually answer.

Two sources, tried in this order:

1. ROS (preferred): the Interbotix driver is running. Positions, velocities and
   currents come from every ``*/joint_states`` topic (ros_state.py). The driver
   owns the serial port, so the GUI never touches it.
2. Serial (fallback): no driver running. The selected port is scanned with the
   Dynamixel SDK (Protocol 2.0) and each responding motor is read directly.
   This is READ-ONLY: no torque enable, no mode change, no goal writes.

Nothing here is hardcoded or simulated; an empty list means nothing answered.
"""

import glob
import math
import os
import threading
from typing import Dict, List, Optional

from arm_bench.ros_state import RosJointStateSource

# X-series control table (Protocol 2.0)
ADDR_MODEL_NUMBER = 0
ADDR_PRESENT_CURRENT = 126       # 2 bytes, signed, 2.69 mA/unit (XL430: Present Load, 0.1 %)
ADDR_PRESENT_VELOCITY = 128      # 4 bytes, signed, 0.229 rev/min per unit
ADDR_PRESENT_POSITION = 132      # 4 bytes, 4096 counts/rev
ADDR_PRESENT_INPUT_VOLTAGE = 144  # 2 bytes, 0.1 V
ADDR_PRESENT_TEMPERATURE = 146   # 1 byte, °C

MODEL_NAMES = {
    1020: "XM430-W350", 1030: "XM430-W210", 1060: "XL430-W250", 1090: "2XL430-W250",
    1120: "XM540-W270", 1130: "XM540-W150", 1000: "XH430-W350", 1010: "XH430-W210",
    1200: "XC430-W150", 1070: "XC430-W240",
}
LOAD_MODELS = {1060, 1090}       # report Present Load instead of current

COLUMNS = ("Source", "Arm / Port", "ID / Joint", "Model", "Position (rad)",
           "Velocity (rad/s)", "Current (mA)", "Voltage (V)", "Temp (°C)")


def list_serial_ports() -> List[str]:
    ports = ["/dev/ttyDXL"] if os.path.exists("/dev/ttyDXL") else []
    ports += sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    return ports


def _signed(value: int, bits: int) -> int:
    return value - (1 << bits) if value >= (1 << (bits - 1)) else value


class SerialMotorReader:
    """Read-only Dynamixel bus reader for one port."""

    def __init__(self, port: str, baudrate: int = 1_000_000):
        self.port = port
        self.baudrate = baudrate
        self.ids: Dict[int, int] = {}      # id -> model number
        self._ph = self._pk = None
        self._lock = threading.Lock()

    def open(self) -> Optional[str]:
        """Returns an error string, or None on success."""
        try:
            from dynamixel_sdk import PacketHandler, PortHandler
        except ImportError:
            return "dynamixel-sdk is not installed (pip install dynamixel-sdk)"
        self._ph = PortHandler(self.port)
        self._pk = PacketHandler(2.0)
        try:
            if not self._ph.openPort():
                return f"Cannot open {self.port} (is the Interbotix driver using it?)"
            if not self._ph.setBaudRate(self.baudrate):
                return f"Cannot set baud rate {self.baudrate} on {self.port}"
        except Exception as exc:
            return f"Cannot open {self.port}: {exc}"
        return None

    def scan(self, id_range=range(0, 32)) -> Dict[int, int]:
        from dynamixel_sdk import COMM_SUCCESS
        self.ids = {}
        with self._lock:
            for dxl_id in id_range:
                model, result, error = self._pk.ping(self._ph, dxl_id)
                if result == COMM_SUCCESS and error == 0:
                    self.ids[dxl_id] = model
        return self.ids

    def read(self) -> List[dict]:
        from dynamixel_sdk import COMM_SUCCESS
        rows = []
        with self._lock:
            for dxl_id, model in self.ids.items():
                def rd(addr, size):
                    fn = {1: self._pk.read1ByteTxRx, 2: self._pk.read2ByteTxRx, 4: self._pk.read4ByteTxRx}[size]
                    value, result, error = fn(self._ph, dxl_id, addr)
                    return value if result == COMM_SUCCESS and error == 0 else None
                pos = rd(ADDR_PRESENT_POSITION, 4)
                vel = rd(ADDR_PRESENT_VELOCITY, 4)
                cur = rd(ADDR_PRESENT_CURRENT, 2)
                volt = rd(ADDR_PRESENT_INPUT_VOLTAGE, 2)
                temp = rd(ADDR_PRESENT_TEMPERATURE, 1)
                if model in LOAD_MODELS:
                    cur_txt = f"load {(_signed(cur, 16) * 0.1):.1f} %" if cur is not None else "–"
                else:
                    cur_txt = f"{_signed(cur, 16) * 2.69:.0f}" if cur is not None else "–"
                rows.append({
                    "Source": "serial",
                    "Arm / Port": self.port,
                    "ID / Joint": str(dxl_id),
                    "Model": MODEL_NAMES.get(model, f"model {model}"),
                    "Position (rad)": f"{(_signed(pos, 32) - 2048) * 2 * math.pi / 4096:.3f}" if pos is not None else "–",
                    "Velocity (rad/s)": f"{_signed(vel, 32) * 0.229 * 2 * math.pi / 60:.3f}" if vel is not None else "–",
                    "Current (mA)": cur_txt,
                    "Voltage (V)": f"{volt * 0.1:.1f}" if volt is not None else "–",
                    "Temp (°C)": str(temp) if temp is not None else "–",
                })
        return rows

    def close(self):
        if self._ph is not None:
            try:
                self._ph.closePort()
            except Exception:
                pass


class MotorMonitor:
    """Chooses ROS when joint states are flowing, else the selected serial port."""

    def __init__(self):
        self.ros = RosJointStateSource.shared()
        self.serial: Optional[SerialMotorReader] = None

    def ros_active(self) -> bool:
        return self.ros.available and bool(self.ros.arms())

    def connect_serial(self, port: str, baudrate: int = 1_000_000) -> str:
        """Open and scan a port; returns a status message."""
        self.disconnect_serial()
        reader = SerialMotorReader(port, baudrate)
        err = reader.open()
        if err:
            return err
        ids = reader.scan()
        self.serial = reader
        if not ids:
            return f"No motors answered on {port} at {baudrate} baud"
        return f"{len(ids)} motor(s) on {port}: IDs {sorted(ids)}"

    def disconnect_serial(self):
        if self.serial:
            self.serial.close()
            self.serial = None

    def rows(self) -> List[dict]:
        if self.ros_active():
            rows = []
            for arm, st in self.ros.latest().items():
                for i, name in enumerate(st.names):
                    def at(arr, fmt):
                        return fmt.format(arr[i]) if i < len(arr) else "–"
                    rows.append({
                        "Source": "ROS", "Arm / Port": arm, "ID / Joint": name, "Model": "–",
                        "Position (rad)": at(st.position, "{:.3f}"),
                        "Velocity (rad/s)": at(st.velocity, "{:.3f}"),
                        "Current (mA)": at(st.effort_mA, "{:.0f}"),
                        "Voltage (V)": "–", "Temp (°C)": "–",
                    })
            return rows
        if self.serial:
            return self.serial.read()
        return []
