"""MERLIN main window.

Opens directly on the Control tab (no arm / operation-mode selection windows).
Tabs: Control, Motor Status, Cameras, Recording.
"""
import os
import sys

# CRITICAL: Clean Qt environment BEFORE importing any GUI libraries
# This prevents segmentation faults caused by Qt plugin conflicts
if 'QT_PLUGIN_PATH' in os.environ:
    del os.environ['QT_PLUGIN_PATH']
if 'QT_QPA_PLATFORM_PLUGIN_PATH' in os.environ:
    del os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']

# Set Qt platform to xcb for Linux
os.environ['QT_QPA_PLATFORM'] = 'xcb'

import subprocess
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk
from typing import Optional

from arm_bench.controller_launcher import (CONTROL_MODES, default_workspace,
                                           launch_mpc_controller, stop_process)
from arm_bench.joint_controller import JointCommandWindow
from arm_bench.motor_monitor import COLUMNS as MOTOR_COLUMNS
from arm_bench.motor_monitor import MotorMonitor, list_serial_ports
from arm_bench.ros_state import RosJointStateSource

APP_NAME = "MERLIN"

# DO NOT import OpenCV here - delay until needed to avoid Qt conflicts
# OpenCV will be imported lazily when camera features are used
OPENCV_AVAILABLE = False
cv2 = None
Image = None
ImageTk = None


def _lazy_import_opencv():
    """Lazy import of OpenCV to avoid conflicts at startup"""
    global OPENCV_AVAILABLE, cv2, Image, ImageTk
    if cv2 is None:
        try:
            import cv2 as cv2_module
            from PIL import Image as PIL_Image, ImageTk as PIL_ImageTk
            cv2 = cv2_module
            Image = PIL_Image
            ImageTk = PIL_ImageTk
            OPENCV_AVAILABLE = True
        except ImportError:
            OPENCV_AVAILABLE = False
    return OPENCV_AVAILABLE


def add_navigation_bar(root, owner, bg="#1e1e1e", back_handler=None, home_handler=None):
    """Create a small top navigation bar with Back and Home buttons."""
    nav_frame = tk.Frame(root, bg=bg)
    nav_frame.pack(fill='x', padx=10, pady=(10, 0))

    def create_btn(text, cmd, side):
        btn = tk.Button(
            nav_frame,
            text=text,
            command=cmd,
            bg="#2d2d2d",
            fg="white",
            relief=tk.FLAT,
            padx=10,
            pady=4,
            activebackground="#007acc",
            activeforeground="white"
        )
        btn.pack(side=side, padx=4)
        return btn

    if back_handler is not None:
        create_btn("← Back", back_handler, tk.LEFT)
    if home_handler is not None:
        create_btn("⌂ Home", home_handler, tk.RIGHT)
    return nav_frame


# Interbotix arm configurations
INTERBOTIX_ARMS = {
    "vx300s": {"name": "ViperX 300 6DOF", "dof": 6,
               "joints": ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]},
    "vx300": {"name": "ViperX 300 5DOF", "dof": 5,
              "joints": ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]},
    "wx250": {"name": "WidowX 250", "dof": 5,
              "joints": ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]},
    "wx250s": {"name": "WidowX 250 6DOF", "dof": 6,
               "joints": ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]},
    "rx150": {"name": "ReactorX 150", "dof": 5,
              "joints": ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]},
    "rx200": {"name": "ReactorX 200", "dof": 5,
              "joints": ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]},
}
NON_ARM_JOINTS = {"gripper", "left_finger", "right_finger"}


class MonitoringInterface:
    """MERLIN main window."""

    def __init__(self, mode: str = "", motors: list = None, cameras: list = None,
                 arm_type: Optional[str] = None):
        self.mode = mode
        self.cameras = cameras or []          # [{"id": int, "name": str, ...}]
        self.arm_type = arm_type

        _lazy_import_opencv()

        self.root = tk.Tk()
        self.root.title(APP_NAME)
        try:
            self.root.geometry("1400x900")
        except Exception:
            self.root.geometry("1200x800")  # Fallback size
        self.root.configure(bg="#1e1e1e")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Live data sources
        self.ros = RosJointStateSource.shared()
        self.motor_monitor = MotorMonitor()

        # Camera state
        self.camera_frames = {}        # camera id -> cv2.VideoCapture
        self.camera_labels = {}
        self.camera_calib_labels = {}
        self.camera_failed_count = {}
        self.latest_frames = {}        # camera name -> (BGR frame, capture time)
        self._frames_lock = threading.Lock()

        # Processes / recorder
        self.controller_process = None
        self.matlab_process = None
        self.recorder = None
        self.control_config_path = None
        self.update_interval = 500  # ms, motor table refresh

        # Colors
        self.colors = {
            'bg': '#1e1e1e',
            'fg': '#ffffff',
            'panel_bg': '#2d2d2d',
            'accent': '#007acc',
            'success': '#4ec9b0',
            'warning': '#dcdcaa',
            'error': '#f48771'
        }

        self._setup_ui()
        self.root.after(self.update_interval, self._motor_update_loop)
        self.root.after(1000, self._controller_watch)

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------
    def _setup_ui(self):
        style = ttk.Style()
        try:
            style.theme_use('clam')
        except Exception as e:
            # Fallback to default theme if clam fails
            print(f"Warning: Could not set theme 'clam': {e}")
        style.configure('TNotebook', background=self.colors['bg'])
        style.configure('TNotebook.Tab', background=self.colors['panel_bg'], foreground=self.colors['fg'], padding=[20, 10])
        style.map('TNotebook.Tab', background=[('selected', self.colors['accent'])])

        # Title Bar
        title_frame = tk.Frame(self.root, bg=self.colors['accent'], height=60)
        title_frame.pack(fill='x', side='top')
        title_frame.pack_propagate(False)

        tk.Label(
            title_frame,
            text=f" {APP_NAME}",
            font=("Helvetica", 18, "bold"),
            bg=self.colors['accent'],
            fg=self.colors['fg']
        ).pack(pady=15)

        # System Info Bar
        info_frame = tk.Frame(self.root, bg=self.colors['panel_bg'], height=50)
        info_frame.pack(fill='x', side='top')
        info_frame.pack_propagate(False)

        self.info_label = tk.Label(
            info_frame,
            text="",
            font=("Helvetica", 11),
            bg=self.colors['panel_bg'],
            fg=self.colors['success']
        )
        self.info_label.pack(pady=10)
        self._update_info()

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill='both', expand=True, padx=10, pady=10)
        self.notebook = notebook

        control_frame = tk.Frame(notebook, bg=self.colors['bg'])
        notebook.add(control_frame, text="Control")
        self._setup_control_tab(control_frame)

        motor_frame = tk.Frame(notebook, bg=self.colors['bg'])
        notebook.add(motor_frame, text="Motor Status")
        self._setup_motor_tab(motor_frame)

        camera_frame = tk.Frame(notebook, bg=self.colors['bg'])
        notebook.add(camera_frame, text="Cameras")
        self._setup_camera_tab(camera_frame)

        recording_frame = tk.Frame(notebook, bg=self.colors['bg'])
        notebook.add(recording_frame, text=" Recording")
        self._setup_recording_tab(recording_frame)

        notebook.select(control_frame)

    def _update_info(self, status: str = "Ready"):
        arms = self.ros.arms() if self.ros.available else []
        arm_txt = ", ".join(arms) if arms else "none"
        ctrl = "running" if self.controller_process and self.controller_process.poll() is None else "stopped"
        self.info_label.config(
            text=f"Arms (ROS): {arm_txt} | Cameras: {len(self.cameras)} | Controller: {ctrl} | Status: {status}")

    def _button(self, parent, text, command, color, fg=None, size=11, padx=25, pady=12):
        return tk.Button(parent, text=text, command=command, bg=color, fg=fg or self.colors['fg'],
                         font=("Helvetica", size), relief=tk.FLAT, padx=padx, pady=pady)

    def _panel(self, parent, title):
        panel = tk.LabelFrame(parent, text=title, font=("Helvetica", 12, "bold"),
                              bg=self.colors['panel_bg'], fg=self.colors['fg'], padx=30, pady=20)
        panel.pack(fill='x', padx=40, pady=10)
        return panel

    # ------------------------------------------------------------------
    # Control tab
    # ------------------------------------------------------------------
    def _setup_control_tab(self, parent):
        parent.configure(bg=self.colors['bg'])

        # Visualization: one entry point for RViz, launch files and nodes
        viz_panel = self._panel(parent, "Visualization")
        self._button(viz_panel, "Run ROS 2 Files (RViz / launch files / nodes)", self._launch_rviz,
                     self.colors['success']).pack(pady=5)

        # Teleoperation: one button per input mode
        teleop_panel = self._panel(parent, "Teleoperation")
        teleop_btn_frame = tk.Frame(teleop_panel, bg=self.colors['panel_bg'])
        teleop_btn_frame.pack()
        self._button(teleop_btn_frame, "Joint Sliders", self._open_joint_controller,
                     "#4CAF50").grid(row=0, column=0, padx=10, pady=5)
        self._button(teleop_btn_frame, "Joystick", self._start_joystick_teleop,
                     self.colors['accent']).grid(row=0, column=1, padx=10, pady=5)
        self._button(teleop_btn_frame, "WebXR (phone / headset)", self._start_camera_teleop,
                     self.colors['accent']).grid(row=0, column=2, padx=10, pady=5)

        # Bilateral master–slave teleoperation (MPC controller)
        bil = self._panel(parent, "Bilateral Teleoperation (Master–Slave, MPC)")
        row1 = tk.Frame(bil, bg=self.colors['panel_bg'])
        row1.pack(anchor='w', pady=5)
        tk.Label(row1, text="Control mode:", font=("Helvetica", 11), bg=self.colors['panel_bg'],
                 fg=self.colors['fg']).pack(side='left', padx=5)
        self.control_mode_var = tk.StringVar(value=next(iter(CONTROL_MODES)))
        ttk.Combobox(row1, textvariable=self.control_mode_var, values=list(CONTROL_MODES),
                     state="readonly", width=28).pack(side='left', padx=5)
        tk.Label(row1, text="Control Frequency (Hz):", font=("Helvetica", 10), bg=self.colors['panel_bg'],
                 fg=self.colors['fg']).pack(side='left', padx=(20, 5))
        self.freq_var = tk.StringVar(value="50")
        tk.Entry(row1, textvariable=self.freq_var, width=10, font=("Helvetica", 10)).pack(side='left', padx=5)

        row2 = tk.Frame(bil, bg=self.colors['panel_bg'])
        row2.pack(anchor='w', pady=5)
        self.launch_drivers_var = tk.BooleanVar(value=True)
        self.use_rviz_var = tk.BooleanVar(value=True)
        for text, var in (("Start both arm drivers", self.launch_drivers_var),
                          ("Open RViz with the robot", self.use_rviz_var)):
            tk.Checkbutton(row2, text=text, variable=var, bg=self.colors['panel_bg'], fg=self.colors['fg'],
                           selectcolor=self.colors['panel_bg'], font=("Helvetica", 10)).pack(side='left', padx=10)

        row3 = tk.Frame(bil, bg=self.colors['panel_bg'])
        row3.pack(pady=10)
        self._button(row3, "Load Control Config", self._load_control_file, self.colors['accent'],
                     size=10, padx=20, pady=10).pack(side='left', padx=5)
        self._button(row3, "Start Control", self._start_control, self.colors['success'],
                     size=10, padx=20, pady=10).pack(side='left', padx=5)
        self._button(row3, "Stop Control", self._stop_control, self.colors['error'],
                     size=10, padx=20, pady=10).pack(side='left', padx=5)
        self.control_status = tk.Label(bil, text="Config: default (config/controller.yaml) | Stopped",
                                       font=("Helvetica", 10), bg=self.colors['panel_bg'],
                                       fg=self.colors['warning'], justify='left')
        self.control_status.pack(anchor='w')

        # Custom control algorithms: MATLAB parameter tuning only
        custom = self._panel(parent, "Custom Control Algorithms")
        self._button(custom, "MATLAB Parameter Tuning", self._open_matlab_tuning, self.colors['warning'],
                     fg='black', size=10, padx=20, pady=10).pack(pady=5)

    # ---------------- visualization / teleop -------------------------
    def _launch_rviz(self):
        """Open the ROS 2 launch manager (RViz, launch files, single nodes)."""
        try:
            from arm_bench.visualization import VisualizationLauncher
            launcher = VisualizationLauncher(mode=self.mode)
            if not launcher.open_rviz_launcher(robot_name=self.arm_type):
                messagebox.showwarning(
                    "Run ROS 2 Files",
                    "Failed to open the launch manager.\nMake sure ROS 2 is sourced.")
        except Exception as e:
            messagebox.showerror("Run ROS 2 Files", f"Error opening the launch manager:\n{e}")

    def _choose_arm(self) -> Optional[str]:
        arms = self.ros.arms() if self.ros.available else []
        if len(arms) == 1:
            return arms[0]
        hint = f"Detected: {', '.join(arms)}" if arms else "No arm detected on ROS (enter its namespace)"
        return simpledialog.askstring("Joint Sliders", f"Arm namespace (e.g. slaveR)\n{hint}",
                                      parent=self.root, initialvalue=arms[0] if arms else (self.arm_type or ""))

    def _open_joint_controller(self):
        """Slider window publishing to <arm>/commands/joint_group with the arm's real joint names."""
        arm = self._choose_arm()
        if not arm:
            return
        st = self.ros.get(arm) if self.ros.available else None
        if st is not None:
            names = [n for n in st.names if n not in NON_ARM_JOINTS]
        else:
            model = next((m for m in INTERBOTIX_ARMS if m in arm), None)
            names = INTERBOTIX_ARMS.get(model or "vx300s")["joints"]
        try:
            win = JointCommandWindow(self.root, topic=f"/{arm.strip('/')}/commands/joint_group",
                                     joint_names=names, joint_count=len(names), group_name="arm")
            win.title(f"{APP_NAME} - Joint Sliders ({arm})")
        except Exception as exc:
            messagebox.showerror("Joint Sliders", f"Failed to open joint sliders:\n{exc}")

    def _start_joystick_teleop(self):
        """Start joystick teleoperation"""
        try:
            from arm_bench.teleoperation import ArmBenchTeleoperator

            # Check for pygame
            try:
                import pygame  # noqa: F401
            except ImportError:
                messagebox.showerror(
                    "Joystick Error",
                    "pygame is not installed.\n\nInstall with:\npip install pygame"
                )
                return

            # Create teleoperator with Dynamixel controller if available
            try:
                from arm_bench.dynamixel_control import DynamixelController

                port = '/dev/ttyDXL' if os.path.exists('/dev/ttyDXL') else '/dev/ttyUSB0'
                controller = DynamixelController(port=port)

                if controller.connect():
                    controller.scan_motors(range(1, 10))
                    controller.enable_all_torque()
                    teleoperator = ArmBenchTeleoperator(controller=controller)
                else:
                    teleoperator = ArmBenchTeleoperator()
                    messagebox.showwarning("Joystick", "Running without hardware.\nDynamixel controller not available.")
            except Exception as e:
                teleoperator = ArmBenchTeleoperator()
                print(f"Running without hardware: {e}")

            self.teleoperator = teleoperator
            self.teleoperator.start_joystick()

            messagebox.showinfo(
                "Joystick Control",
                "Joystick teleoperation started!\n\n"
                "Controls:\n"
                "  Left Stick: XY movement\n"
                "  Right Stick: Z and rotation\n"
                "  Button A: Toggle gripper\n"
                "  Button B: Toggle move enable\n"
                "  Button Back: Stop\n\n"
                "Check terminal for output."
            )

        except ImportError as e:
            messagebox.showerror("Joystick Error", f"Failed to import teleoperation module:\n{e}")
        except Exception as e:
            messagebox.showerror("Joystick Error", f"Failed to start joystick control:\n{e}")

    def _start_camera_teleop(self):
        """Start camera-based teleoperation (WebXR)"""
        try:
            try:
                import fastapi  # noqa: F401
                import uvicorn  # noqa: F401
                import websockets  # noqa: F401
            except ImportError:
                messagebox.showerror(
                    "WebXR Error",
                    "Required packages not installed.\n\nInstall with:\npip install fastapi uvicorn websockets"
                )
                return

            try:
                from arm_bench.webxr import WebXRTeleoperationServer
            except ImportError as e:
                messagebox.showerror(
                    "WebXR Error",
                    f"Failed to import WebXR module:\n{e}\n\nMake sure the webxr module is installed."
                )
                return

            # HTTPS port (required for WebXR on mobile)
            server_port = 8443

            try:
                from arm_bench.dynamixel_control import DynamixelController

                port = '/dev/ttyDXL' if os.path.exists('/dev/ttyDXL') else '/dev/ttyUSB0'
                controller = DynamixelController(port=port)

                if controller.connect():
                    controller.scan_motors(range(1, 10))
                    controller.enable_all_torque()
                    self.webxr_server = WebXRTeleoperationServer(port=server_port, controller=controller)
                else:
                    self.webxr_server = WebXRTeleoperationServer(port=server_port)
            except Exception as e:
                print(f"Running WebXR without hardware: {e}")
                self.webxr_server = WebXRTeleoperationServer(port=server_port)

            def start_webxr():
                try:
                    self.webxr_server.run()
                except Exception as e:
                    print(f"WebXR server error: {e}")

            threading.Thread(target=start_webxr, daemon=True).start()
            time.sleep(1.0)

            import socket
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80))
                local_ip = s.getsockname()[0]
                s.close()
            except Exception:
                local_ip = "localhost"

            messagebox.showinfo(
                "WebXR Control",
                f"WebXR teleoperation server started!\n\n"
                f"Open this URL on your AR/VR device:\n"
                f"https://{local_ip}:{server_port}/static/index.html\n\n"
                f"Requirements:\n"
                f"• Chrome/Edge with WebXR support\n"
                f"• AR-capable device (phone/headset)\n"
                f"• Same WiFi network as this PC\n"
                f"• Accept the security warning (self-signed cert)\n\n"
                f"Controls:\n"
                f"• Click 'Connect to Server' first\n"
                f"• Click 'Start AR' to begin tracking\n"
                f"• Hold MOVE button to track arm position\n"
                f"• Click GRIPPER button to toggle gripper\n\n"
                f"Check terminal for server output."
            )

        except Exception as e:
            import traceback
            traceback.print_exc()
            messagebox.showerror("WebXR Error", f"Failed to start WebXR control:\n{e}")

    # ---------------- bilateral controller ---------------------------
    def _load_control_file(self):
        """Choose a controller YAML (gains, mode, limits) — see config/controller.yaml."""
        start_dir = os.path.join(default_workspace(), "src", "controller", "config")
        filename = filedialog.askopenfilename(
            title="Select controller config (YAML)", initialdir=start_dir if os.path.isdir(start_dir) else None,
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")])
        if not filename:
            return
        try:
            import yaml
            with open(filename) as f:
                data = yaml.safe_load(f) or {}
            if "controller" not in data or "ros__parameters" not in data.get("controller", {}):
                raise ValueError("expected a 'controller: ros__parameters:' section (see config/controller.yaml)")
        except Exception as exc:
            messagebox.showerror("Control Config", f"Not a valid controller config:\n{exc}")
            return
        self.control_config_path = filename
        self._set_control_status()

    def _set_control_status(self):
        cfg = os.path.basename(self.control_config_path) if self.control_config_path else \
            "default (config/controller.yaml)"
        running = self.controller_process is not None and self.controller_process.poll() is None
        state = f"Running — log: {self.controller_process.log_path}" if running else "Stopped"
        self.control_status.config(text=f"Config: {cfg} | {state}",
                                   fg=self.colors['success'] if running else self.colors['warning'])
        self._update_info()

    def _start_control(self):
        if self.controller_process is not None and self.controller_process.poll() is None:
            messagebox.showinfo("Control", "The controller is already running.")
            return
        try:
            freq = float(self.freq_var.get())
            if not 1.0 <= freq <= 500.0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Control", "Control frequency must be a number between 1 and 500 Hz.")
            return
        mode = CONTROL_MODES[self.control_mode_var.get()]
        try:
            self.controller_process = launch_mpc_controller(
                default_workspace(), mode=mode, params_file=self.control_config_path,
                launch_drivers=self.launch_drivers_var.get(), use_rviz=self.use_rviz_var.get(),
                loop_rate_hz=freq)
        except Exception as exc:
            messagebox.showerror("Control", f"Failed to start the controller:\n{exc}")
            return
        self._set_control_status()

    def _stop_control(self):
        if self.controller_process is None or self.controller_process.poll() is not None:
            messagebox.showinfo("Control", "The controller is not running.")
            return
        stop_process(self.controller_process)
        self._set_control_status()

    def _controller_watch(self):
        """Notice when the launch process exits on its own (e.g. a node crashed)."""
        p = self.controller_process
        if p is not None and p.poll() is not None and getattr(p, "_reported", False) is False:
            p._reported = True
            self._set_control_status()
            if p.returncode not in (0, -2, 130):
                messagebox.showwarning("Control", f"The controller exited (code {p.returncode}).\n"
                                                  f"See the log: {p.log_path}")
        self.root.after(1000, self._controller_watch)

    # ---------------- MATLAB tuning ----------------------------------
    def _open_matlab_tuning(self):
        """Open the parameter-tuning window; MERLIN comes back when it is closed."""
        if self.matlab_process is not None and self.matlab_process.poll() is None:
            messagebox.showinfo("MATLAB Tuning", "The tuning window is already open.")
            return
        try:
            # Separate process: Qt and Tk event loops cannot share one process
            self.matlab_process = subprocess.Popen(
                [sys.executable, "-m", "arm_bench.matlab_integration_enhanced"])
        except Exception as e:
            messagebox.showerror("MATLAB Tuning Error", f"Failed to launch the tuning interface:\n\n{e}")
            return
        self.root.withdraw()
        self.root.after(500, self._wait_matlab)

    def _wait_matlab(self):
        if self.matlab_process is not None and self.matlab_process.poll() is None:
            self.root.after(500, self._wait_matlab)
            return
        self.root.deiconify()
        self.root.lift()

    # ------------------------------------------------------------------
    # Motor status tab
    # ------------------------------------------------------------------
    def _setup_motor_tab(self, parent):
        parent.configure(bg=self.colors['bg'])

        button_frame = tk.Frame(parent, bg=self.colors['bg'])
        button_frame.pack(fill='x', padx=10, pady=10)

        tk.Label(button_frame, text="Serial port:", font=("Helvetica", 10), bg=self.colors['bg'],
                 fg=self.colors['fg']).pack(side='left', padx=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(button_frame, textvariable=self.port_var, width=18)
        self.port_combo.pack(side='left', padx=5)
        tk.Label(button_frame, text="Baud:", font=("Helvetica", 10), bg=self.colors['bg'],
                 fg=self.colors['fg']).pack(side='left', padx=5)
        self.baud_var = tk.StringVar(value="1000000")
        ttk.Combobox(button_frame, textvariable=self.baud_var, width=10,
                     values=["57600", "115200", "1000000", "2000000", "3000000", "4000000"]).pack(side='left', padx=5)
        self._button(button_frame, "Refresh Ports", self._refresh_ports, self.colors['accent'],
                     size=10, padx=20, pady=8).pack(side='left', padx=5)
        self._button(button_frame, "Scan Port", self._scan_port, self.colors['success'],
                     size=10, padx=20, pady=8).pack(side='left', padx=5)
        self._button(button_frame, "Disconnect", self._disconnect_port, self.colors['error'],
                     size=10, padx=20, pady=8).pack(side='left', padx=5)

        self.motor_source_label = tk.Label(parent, text="", font=("Helvetica", 10), bg=self.colors['bg'],
                                           fg=self.colors['warning'], anchor='w', justify='left')
        self.motor_source_label.pack(fill='x', padx=15)

        tree_frame = tk.Frame(parent, bg=self.colors['bg'])
        tree_frame.pack(fill='both', expand=True, padx=10, pady=10)
        self.motor_tree = ttk.Treeview(tree_frame, columns=MOTOR_COLUMNS, show='headings', height=15)
        for col in MOTOR_COLUMNS:
            self.motor_tree.heading(col, text=col, anchor='center')
            self.motor_tree.column(col, width=130, anchor='center')

        vsb = ttk.Scrollbar(tree_frame, orient="vertical", command=self.motor_tree.yview)
        hsb = ttk.Scrollbar(tree_frame, orient="horizontal", command=self.motor_tree.xview)
        self.motor_tree.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)
        self.motor_tree.grid(row=0, column=0, sticky='nsew')
        vsb.grid(row=0, column=1, sticky='ns')
        hsb.grid(row=1, column=0, sticky='ew')
        tree_frame.grid_rowconfigure(0, weight=1)
        tree_frame.grid_columnconfigure(0, weight=1)

        style = ttk.Style()
        style.configure("Treeview", background=self.colors['panel_bg'],
                        foreground=self.colors['fg'], fieldbackground=self.colors['panel_bg'])
        style.configure("Treeview.Heading", background=self.colors['accent'],
                        foreground=self.colors['fg'], font=("Helvetica", 10, "bold"))
        self._refresh_ports()
        self._refresh_motor_data()

    def _refresh_ports(self):
        ports = list_serial_ports()
        self.port_combo.config(values=ports)
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _scan_port(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("Motor Status", "Select or type a serial port first.")
            return
        if self.motor_monitor.ros_active():
            if not messagebox.askyesno("Motor Status", "Motors are already being read from ROS (the "
                                                       "driver owns the port). Scan the port anyway?"):
                return
        self.motor_source_label.config(text=f"Scanning {port}…")
        self.root.update_idletasks()
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            baud = 1000000
        msg = self.motor_monitor.connect_serial(port, baud)
        self.motor_source_label.config(text=msg)
        self._refresh_motor_data()

    def _disconnect_port(self):
        self.motor_monitor.disconnect_serial()
        self._refresh_motor_data()

    def _refresh_motor_data(self):
        rows = self.motor_monitor.rows()
        for item in self.motor_tree.get_children():
            self.motor_tree.delete(item)
        for r in rows:
            self.motor_tree.insert('', 'end', values=[r.get(c, "–") for c in MOTOR_COLUMNS])
        if self.motor_monitor.ros_active():
            src = "Live from ROS: " + ", ".join(self.ros.arms())
        elif self.motor_monitor.serial is not None:
            src = f"Live from {self.motor_monitor.serial.port} (read-only)"
        else:
            src = ("No motors connected. Start the Interbotix driver (or the bilateral controller), "
                   "or choose a serial port and press 'Scan Port'.")
        if not self.motor_source_label.cget("text").startswith(("Scanning", "No motors answered",
                                                                 "Cannot", "dynamixel")) or rows:
            self.motor_source_label.config(text=src)

    def _motor_update_loop(self):
        try:
            self._refresh_motor_data()
            self._update_info()
        finally:
            self.root.after(self.update_interval, self._motor_update_loop)

    # ------------------------------------------------------------------
    # Cameras tab
    # ------------------------------------------------------------------
    def _setup_camera_tab(self, parent):
        parent.configure(bg=self.colors['bg'])
        control_frame = tk.Frame(parent, bg=self.colors['bg'])
        control_frame.pack(fill='x', padx=10, pady=10)

        tk.Label(control_frame, text="Live Camera Feeds", font=("Helvetica", 14, "bold"),
                 bg=self.colors['bg'], fg=self.colors['fg']).pack(side='left', padx=10)
        self._button(control_frame, "Scan Cameras", self._scan_cameras, self.colors['accent'],
                     size=10, padx=20, pady=8).pack(side='left', padx=5)
        self._button(control_frame, "Start All Cameras", self._start_all_cameras, self.colors['success'],
                     size=10, padx=20, pady=8).pack(side='left', padx=5)
        self._button(control_frame, "Stop All Cameras", self._stop_all_cameras, self.colors['error'],
                     size=10, padx=20, pady=8).pack(side='left', padx=5)

        self.camera_grid_frame = tk.Frame(parent, bg=self.colors['bg'])
        self.camera_grid_frame.pack(fill='both', expand=True, padx=10, pady=10)
        self._build_camera_grid()

    def _camera_name(self, camera) -> str:
        return camera.get('name', f"Camera {camera['id']}")

    def _build_camera_grid(self):
        for w in self.camera_grid_frame.winfo_children():
            w.destroy()
        self.camera_labels.clear()
        self.camera_calib_labels.clear()
        if not self.cameras:
            tk.Label(self.camera_grid_frame, text="No cameras detected. Connect a camera and press 'Scan Cameras'.",
                     font=("Helvetica", 12), bg=self.colors['bg'], fg=self.colors['warning']).pack(pady=50)
            return
        from arm_bench.camera_calibration import CameraCalibration
        cameras_per_row = 2
        for idx, camera in enumerate(self.cameras):
            row, col = divmod(idx, cameras_per_row)
            name = self._camera_name(camera)
            frame = tk.LabelFrame(self.camera_grid_frame, text=name, font=("Helvetica", 11, "bold"),
                                  bg=self.colors['panel_bg'], fg=self.colors['fg'], padx=10, pady=10)
            frame.grid(row=row, column=col, padx=10, pady=10, sticky='nsew')
            self.camera_grid_frame.grid_rowconfigure(row, weight=1)
            self.camera_grid_frame.grid_columnconfigure(col, weight=1)
            video_label = tk.Label(frame, text="Camera feed will appear here", bg='black', fg='white')
            video_label.pack()
            self.camera_labels[camera['id']] = video_label
            bottom = tk.Frame(frame, bg=self.colors['panel_bg'])
            bottom.pack(fill='x', pady=5)
            calib = CameraCalibration.load(name)
            calib_txt = ("Calibrated: intrinsics" + (" + extrinsics" if calib.extrinsics_source != "none" else "")
                         if calib else "Not calibrated")
            info = f"Resolution: {camera.get('resolution', 'Unknown')} | FPS: {camera.get('fps', 'N/A')} | {calib_txt}"
            lbl = tk.Label(bottom, text=info, font=("Helvetica", 9), bg=self.colors['panel_bg'],
                           fg=self.colors['warning'])
            lbl.pack(side='left', pady=5)
            self.camera_calib_labels[camera['id']] = lbl
            self._button(bottom, "Calibrate", lambda c=camera: self._calibrate_camera(c),
                         self.colors['accent'], size=10, padx=15, pady=5).pack(side='right')

    def _scan_cameras(self):
        from arm_bench.scanner import scan_cameras
        self._stop_all_cameras()
        names = {c['id']: c.get('name') for c in self.cameras}
        self.cameras = scan_cameras()
        for c in self.cameras:                          # keep user-given names
            if names.get(c['id']):
                c['name'] = names[c['id']]
        self._build_camera_grid()
        self._refresh_recording_camera_list()
        self._update_info()

    def _start_all_cameras(self):
        """Start all camera feeds"""
        if not OPENCV_AVAILABLE:
            messagebox.showwarning("OpenCV Required", "Please install opencv-python and pillow:\npip install opencv-python pillow")
            return
        if not self.cameras:
            messagebox.showwarning("Cameras", "No cameras. Press 'Scan Cameras' first.")
            return

        successful_cameras, failed_cameras = [], []
        for camera in self.cameras:
            camera_id = camera['id']
            if self.camera_frames.get(camera_id) is not None:
                continue
            # MJPEG uses much less USB bandwidth than YUYV with several cameras
            cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 15)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                time.sleep(0.1)
                ret = False
                for _ in range(3):
                    ret, _frame = cap.read()
                    if ret:
                        break
                    time.sleep(0.1)
                if ret:
                    self.camera_frames[camera_id] = cap
                    self.camera_failed_count[camera_id] = 0
                    successful_cameras.append(camera_id)
                    continue
                cap.release()
                msg = f"Camera {camera_id} - Cannot Read Frames\nCheck camera connection"
            else:
                msg = f"Camera {camera_id} - Failed to Open\nCamera may be in use"
            failed_cameras.append(camera_id)
            if camera_id in self.camera_labels:
                self.camera_labels[camera_id].config(text=msg, fg='red')

        if successful_cameras:
            self._update_camera_feeds()
            self._update_info(f"{len(successful_cameras)}/{len(self.cameras)} cameras running")
            if failed_cameras:
                messagebox.showwarning(
                    "Camera Warning",
                    f"Started {len(successful_cameras)} camera(s) successfully.\n\n"
                    f"Failed cameras: {', '.join(map(str, failed_cameras))}\n\n"
                    f"These cameras may be in use by another application.")
        elif failed_cameras:
            messagebox.showerror("Camera Error", "Failed to start any cameras.\nAll cameras are unavailable or in use.")

    def _stop_all_cameras(self):
        for cap in self.camera_frames.values():
            if cap is not None:
                cap.release()
        self.camera_frames.clear()
        with self._frames_lock:
            self.latest_frames.clear()

    def _update_camera_feeds(self):
        """Read every running camera once; display it and keep it as the latest frame."""
        if not OPENCV_AVAILABLE or Image is None or ImageTk is None:
            return
        names = {c['id']: self._camera_name(c) for c in self.cameras}
        to_remove = []
        for camera_id, cap in list(self.camera_frames.items()):
            if cap is None or not cap.isOpened():
                continue
            ret, frame = cap.read()
            if ret:
                self.camera_failed_count[camera_id] = 0
                with self._frames_lock:
                    self.latest_frames[names.get(camera_id, f"Camera {camera_id}")] = (frame, time.time())
                if camera_id in self.camera_labels:
                    disp = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), (640, 480))
                    photo = ImageTk.PhotoImage(image=Image.fromarray(disp))
                    label = self.camera_labels[camera_id]
                    label.config(image=photo)
                    label.image = photo
            else:
                self.camera_failed_count[camera_id] = self.camera_failed_count.get(camera_id, 0) + 1
                if self.camera_failed_count[camera_id] >= 10:
                    to_remove.append(camera_id)
                    if camera_id in self.camera_labels:
                        self.camera_labels[camera_id].config(
                            text=f"Camera {camera_id} - Connection Lost\nToo many read failures", fg='red')
        for camera_id in to_remove:
            self.camera_frames.pop(camera_id).release()
        if self.camera_frames:
            self.root.after(33, self._update_camera_feeds)

    def _latest_frame_of(self, name):
        with self._frames_lock:
            item = self.latest_frames.get(name)
        return item[0] if item else None

    def _recording_frames(self):
        with self._frames_lock:
            return dict(self.latest_frames)

    def _calibrate_camera(self, camera):
        if self.camera_frames.get(camera['id']) is None:
            self._start_all_cameras()
            if self.camera_frames.get(camera['id']) is None:
                return
        from arm_bench.calibration_window import CalibrationWindow
        name = self._camera_name(camera)
        win = CalibrationWindow(self.root, name, lambda: self._latest_frame_of(name), self.colors)
        win.bind("<Destroy>", lambda e: self._build_camera_grid() if e.widget is win else None)

    # ------------------------------------------------------------------
    # Recording tab
    # ------------------------------------------------------------------
    def _setup_recording_tab(self, parent):
        tk.Label(parent, text="RLDS Dataset Recording", font=("Arial", 14, "bold")).pack(pady=10)
        self.recording_status = tk.Label(parent, text="Status: Not Recording", font=("Arial", 11))
        self.recording_status.pack(pady=10)

        cam_frame = tk.LabelFrame(parent, text="Camera Configuration", padx=10, pady=10)
        cam_frame.pack(padx=20, pady=10, fill='x')
        tk.Button(cam_frame, text="Scan Cameras", command=self._scan_cameras).pack(pady=5)
        self.camera_listbox = tk.Listbox(cam_frame, height=5)
        self.camera_listbox.pack(fill='x', pady=5)
        tk.Button(cam_frame, text="Rename Selected Camera", command=self._rename_camera).pack(pady=5)
        self._refresh_recording_camera_list()

        rate_frame = tk.Frame(parent)
        rate_frame.pack(pady=5)
        tk.Label(rate_frame, text="Recording rate (Hz):").pack(side='left', padx=5)
        self.record_rate_var = tk.StringVar(value="30")
        tk.Entry(rate_frame, textvariable=self.record_rate_var, width=8).pack(side='left', padx=5)

        control_frame = tk.Frame(parent)
        control_frame.pack(pady=20)
        tk.Button(control_frame, text="Start Recording", command=self._start_recording,
                  bg="#4CAF50", fg="white", width=15).grid(row=0, column=0, padx=5)
        tk.Button(control_frame, text="Stop Recording", command=self._stop_recording,
                  bg="#f44336", fg="white", width=15).grid(row=0, column=1, padx=5)

    def _refresh_recording_camera_list(self):
        if not hasattr(self, "camera_listbox"):
            return
        self.camera_listbox.delete(0, tk.END)
        for camera in self.cameras:
            self.camera_listbox.insert(tk.END, f"{self._camera_name(camera)} - "
                                               f"{camera.get('resolution', 'Unknown')} (/dev/video{camera['id']})")

    def _rename_camera(self):
        selection = self.camera_listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a camera first")
            return
        new_name = simpledialog.askstring("Rename Camera", "Enter new camera name:")
        if new_name:
            self.cameras[selection[0]]['name'] = new_name
            self._refresh_recording_camera_list()
            self._build_camera_grid()

    def _start_recording(self):
        from arm_bench.episode_recorder import EpisodeRecorder
        if self.recorder is not None and self.recorder.recording:
            messagebox.showwarning("Recording", "Already recording!")
            return
        try:
            rate = float(self.record_rate_var.get())
        except ValueError:
            rate = 30.0
        if self.cameras and not self.camera_frames:
            if messagebox.askyesno("Start Cameras?", "Cameras are not running. Start them for recording?\n\n"
                                                     "'No' records joint states only."):
                self._start_all_cameras()
        arms = self.ros.arms() if self.ros.available else []
        if not arms and not self.camera_frames:
            messagebox.showerror("Recording", "Nothing to record: no */joint_states on ROS and no camera running.")
            return
        if not arms and not messagebox.askyesno(
                "Recording", "No joint states are being received (Interbotix driver not running).\n"
                             "Record camera frames only?"):
            return
        output_dir = filedialog.askdirectory(title="Select Output Directory for Dataset")
        if not output_dir:
            return
        self.recorder = EpisodeRecorder(output_dir, rate_hz=rate, state_source=self.ros,
                                        frame_source=self._recording_frames)
        path = self.recorder.start(metadata={"control_mode": self.control_mode_var.get(),
                                             "arms_at_start": arms})
        self.recording_status.config(text=f"Status: Recording → {path}", fg="red")
        self._recording_tick()

    def _recording_tick(self):
        if self.recorder is None or not self.recorder.recording:
            return
        self.recording_status.config(
            text=f"Recording... Step {self.recorder.step} | Time: {self.recorder.step / self.recorder.rate_hz:.1f}s"
                 f" | Frames written: {self.recorder.frames_written}", fg="red")
        self.root.after(500, self._recording_tick)

    def _stop_recording(self):
        if self.recorder is None or not self.recorder.recording:
            messagebox.showwarning("Recording", "Not currently recording!")
            return
        s = self.recorder.stop()
        self.recording_status.config(text="Status: Not Recording", fg="black")
        messagebox.showinfo(
            "Recording",
            f"Stopped recording. Dataset saved.\n\nLocation: {s['episode_dir']}\n"
            f"Steps: {s['num_steps']} | Duration: {s['duration_s']:.1f}s\n"
            f"Arms: {', '.join(s['arms']) or 'none'}\nCamera frames: {s['frames_written']}"
            f" (dropped {s['dropped_frames']})\nCalibrated cameras: {', '.join(s['calibrated_cameras']) or 'none'}\n"
            f"HDF5: {s['hdf5']}")

    # ------------------------------------------------------------------
    def _on_close(self):
        if self.recorder is not None and self.recorder.recording:
            self.recorder.stop()
        if self.controller_process is not None and self.controller_process.poll() is None:
            if messagebox.askyesno(APP_NAME, "Stop the running controller before closing?"):
                stop_process(self.controller_process)
        self._stop_all_cameras()
        self.motor_monitor.disconnect_serial()
        self.ros.shutdown()
        self.root.destroy()

    def show(self):
        self.root.lift()
        self.root.attributes('-topmost', True)
        self.root.after_idle(self.root.attributes, '-topmost', False)
        self.root.mainloop()


MerlinApp = MonitoringInterface


def launch_monitoring_interface(mode: str = "", motors: list = None, cameras: list = None,
                                arm_type: Optional[str] = None):
    """Open the MERLIN main window (blocks until it is closed)."""
    MonitoringInterface(mode, motors, cameras, arm_type=arm_type).show()
