"""Tkinter-based joint command controller for publishing Interbotix-style joint-group commands."""

import math
import tkinter as tk
from tkinter import ttk, messagebox
from typing import Iterable, List, Optional, Sequence


def build_joint_group_payload(
    joint_names: Sequence[str],
    joint_angles_radians: Sequence[float],
    group_name: str = "arm",
) -> dict:
    """Build a simple payload structure matching Interbotix JointGroupCommand semantics."""
    if len(joint_names) != len(joint_angles_radians):
        raise ValueError("joint_names and joint_angles_radians must have the same length")

    return {
        "name": group_name,
        "cmd": [float(angle) for angle in joint_angles_radians],
    }


def publish_joint_group_command(
    topic: str,
    joint_names: Sequence[str],
    joint_angles_radians: Sequence[float],
    group_name: str = "arm",
    node_name: str = "armbench_joint_controller",
) -> dict:
    """Publish a joint-group command to a ROS 2 topic using the Interbotix message type."""
    payload = build_joint_group_payload(joint_names, joint_angles_radians, group_name=group_name)

    try:
        import rclpy
        from rclpy.node import Node
        from interbotix_xs_msgs.msg import JointGroupCommand
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 or the Interbotix message package is not available. "
            "Please source the ROS 2 workspace before publishing."
        ) from exc

    topic_name = topic.strip() if topic else "/commands/joint_group"
    if not topic_name.startswith("/"):
        topic_name = f"/{topic_name}"

    initialized = False
    if not rclpy.ok():
        rclpy.init(args=None)
        initialized = True

    try:
        node = Node(node_name)
        publisher = node.create_publisher(JointGroupCommand, topic_name, 10)
        msg = JointGroupCommand()
        msg.name = payload["name"]
        msg.cmd = payload["cmd"]
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_publisher(publisher)
        node.destroy_node()
    finally:
        if initialized:
            rclpy.shutdown()

    return payload


class JointCommandWindow(tk.Toplevel):
    """Simple slider-based window for sending joint angles to a ROS 2 joint-group topic."""

    def __init__(
        self,
        parent: tk.Misc,
        topic: str = "/commands/joint_group",
        joint_names: Optional[Sequence[str]] = None,
        joint_count: int = 6,
        group_name: str = "arm",
    ):
        super().__init__(parent)
        self.title("Joint Controller")
        self.geometry("760x560")
        self.resizable(True, True)

        self.topic_var = tk.StringVar(value=topic or "/commands/joint_group")
        self.group_var = tk.StringVar(value=group_name or "arm")
        self.status_var = tk.StringVar(value="Ready")

        self.joint_names = list(joint_names or [f"joint_{i + 1}" for i in range(joint_count)])
        self.angle_vars: List[tk.DoubleVar] = []
        self._build_ui()

    def _build_ui(self) -> None:
        main = ttk.Frame(self, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        ttk.Label(main, text="Joint Controller", font=("Helvetica", 16, "bold")).pack(anchor=tk.W)
        ttk.Label(
            main,
            text="Use the sliders below to command the selected arm and publish directly to a joint-group topic.",
            wraplength=700,
        ).pack(anchor=tk.W, pady=(4, 10))

        settings = ttk.LabelFrame(main, text="Publish Settings", padding=10)
        settings.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(settings, text="Topic:").grid(row=0, column=0, sticky=tk.W, padx=4, pady=4)
        ttk.Entry(settings, textvariable=self.topic_var, width=40).grid(row=0, column=1, sticky=tk.EW, padx=4, pady=4)
        ttk.Label(settings, text="Group name:").grid(row=1, column=0, sticky=tk.W, padx=4, pady=4)
        ttk.Entry(settings, textvariable=self.group_var, width=20).grid(row=1, column=1, sticky=tk.W, padx=4, pady=4)
        settings.columnconfigure(1, weight=1)

        controls = ttk.Frame(main)
        controls.pack(fill=tk.X, pady=(0, 8))
        ttk.Button(controls, text="Zero All", command=self._zero_all).pack(side=tk.LEFT, padx=4)
        ttk.Button(controls, text="Publish", command=self._publish).pack(side=tk.LEFT, padx=4)

        slider_frame = ttk.LabelFrame(main, text="Joint Angles", padding=10)
        slider_frame.pack(fill=tk.BOTH, expand=True)

        for idx, joint_name in enumerate(self.joint_names):
            row = ttk.Frame(slider_frame)
            row.pack(fill=tk.X, pady=4)

            angle_var = tk.DoubleVar(value=0.0)
            self.angle_vars.append(angle_var)

            ttk.Label(row, text=joint_name, width=16, anchor=tk.W).pack(side=tk.LEFT)
            scale = ttk.Scale(row, from_=-180.0, to=180.0, orient=tk.HORIZONTAL, variable=angle_var, length=420)
            scale.pack(side=tk.LEFT, padx=8, fill=tk.X, expand=True)
            value_label = ttk.Label(row, text="0°", width=10)
            value_label.pack(side=tk.LEFT)

            angle_var.trace_add("write", lambda *_args, idx=idx, label=value_label: self._update_label(idx, label))
            self._update_label(idx, value_label)

        status = ttk.Label(main, textvariable=self.status_var, foreground="#2f6b2f")
        status.pack(anchor=tk.W, pady=(8, 0))

    def _update_label(self, idx: int, label: ttk.Label) -> None:
        value = self.angle_vars[idx].get()
        label.config(text=f"{value:.1f}°")

    def _zero_all(self) -> None:
        for var in self.angle_vars:
            var.set(0.0)

    def _publish(self) -> None:
        topic = self.topic_var.get().strip()
        if not topic:
            messagebox.showwarning("Missing topic", "Please provide a topic name before publishing.")
            return

        group_name = self.group_var.get().strip() or "arm"
        angles_deg = [var.get() for var in self.angle_vars]
        angles_rad = [math.radians(angle) for angle in angles_deg]

        if not messagebox.askyesno(
            "Publish joint command",
            f"Publish {len(angles_deg)} joint values to {topic} using group '{group_name}'?",
        ):
            return

        try:
            publish_joint_group_command(topic, self.joint_names, angles_rad, group_name=group_name)
            self.status_var.set(f"Published {len(angles_deg)} values to {topic}")
        except Exception as exc:  # pragma: no cover - UI feedback path
            self.status_var.set("Publish failed")
            messagebox.showerror("Publish failed", str(exc))
