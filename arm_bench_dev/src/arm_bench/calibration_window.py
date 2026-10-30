"""
Camera calibration dialog (Cameras tab → "Calibrate").

Step 1 — intrinsics: hold a checkerboard in view at different angles and press
"Capture view" each time the corners are detected (15+ views recommended),
then "Compute intrinsics".
Step 2 — extrinsics: place the AprilTag that defines the world frame in view
and press "Detect AprilTag".
"Save" writes ~/.merlin/calibration/<camera>.json and the ray embeddings.
"""

import tkinter as tk
from tkinter import messagebox
from typing import Callable, Optional

import numpy as np

from arm_bench.camera_calibration import (
    CameraCalibration, apriltag_extrinsics, calibrate_intrinsics, find_checkerboard)


class CalibrationWindow(tk.Toplevel):
    def __init__(self, master, camera_name: str, get_frame: Callable[[], Optional[np.ndarray]],
                 colors: dict):
        super().__init__(master)
        self.title(f"MERLIN - Calibrate {camera_name}")
        self.configure(bg=colors['bg'])
        self.colors = colors
        self.camera_name = camera_name
        self.get_frame = get_frame
        self.corner_sets = []
        self.image_size = None
        existing = CameraCalibration.load(camera_name)
        self.calib: Optional[CameraCalibration] = existing
        self._last_corners = None
        self._running = True

        nav = tk.Frame(self, bg=colors['bg'])
        nav.pack(fill='x', padx=10, pady=(10, 0))
        tk.Button(nav, text="← Back", command=self.close, bg="#2d2d2d", fg="white",
                  relief=tk.FLAT, padx=10, pady=4, activebackground="#007acc",
                  activeforeground="white").pack(side=tk.LEFT, padx=4)

        body = tk.Frame(self, bg=colors['bg'])
        body.pack(fill='both', expand=True, padx=10, pady=10)
        self.preview = tk.Label(body, bg='black', fg='white', text="Waiting for camera frames…",
                                width=80, height=24)
        self.preview.pack(side='left', padx=5)

        side = tk.LabelFrame(body, text="Calibration", font=("Helvetica", 12, "bold"),
                             bg=colors['panel_bg'], fg=colors['fg'], padx=15, pady=10)
        side.pack(side='left', fill='y', padx=5)

        def field(label, default):
            tk.Label(side, text=label, bg=colors['panel_bg'], fg=colors['fg'],
                     font=("Helvetica", 10)).pack(anchor='w')
            var = tk.StringVar(value=default)
            tk.Entry(side, textvariable=var, width=12, font=("Helvetica", 10)).pack(anchor='w', pady=(0, 6))
            return var

        self.cols_var = field("Checkerboard inner corners (cols)", "9")
        self.rows_var = field("Checkerboard inner corners (rows)", "6")
        self.square_var = field("Square size (m)", "0.025")
        self.tag_var = field("AprilTag size (m)", "0.05")
        self.family_var = field("AprilTag family", "tag36h11")

        def button(text, cmd, color):
            tk.Button(side, text=text, command=cmd, bg=color, fg=colors['fg'], font=("Helvetica", 10),
                      relief=tk.FLAT, padx=20, pady=8).pack(fill='x', pady=3)

        button("Capture view", self.capture, colors['accent'])
        button("Compute intrinsics", self.compute_intrinsics, colors['accent'])
        button("Detect AprilTag", self.detect_tag, colors['accent'])
        button("Save", self.save, colors['success'])

        self.status = tk.Label(side, text=self._status_text(), justify='left', wraplength=260,
                               bg=colors['panel_bg'], fg=colors['warning'], font=("Helvetica", 10))
        self.status.pack(anchor='w', pady=10)
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.after(100, self._tick)

    # ------------------------------------------------------------------
    def _pattern(self):
        return int(self.cols_var.get()), int(self.rows_var.get())

    def _status_text(self, extra: str = "") -> str:
        c = self.calib
        txt = f"Views captured: {len(self.corner_sets)}\n"
        if c is not None:
            txt += (f"Intrinsics: fx={c.K[0][0]:.1f}, fy={c.K[1][1]:.1f}, "
                    f"RMS={c.reprojection_rms:.3f} px\n"
                    f"Extrinsics: {c.extrinsics_source}"
                    + (f" (tags {c.tags_used})" if c.tags_used else "") + "\n")
        else:
            txt += "No calibration yet\n"
        return txt + extra

    def _tick(self):
        if not self._running:
            return
        frame = self.get_frame()
        if frame is not None:
            import cv2
            from PIL import Image, ImageTk
            self.image_size = (frame.shape[1], frame.shape[0])
            vis = frame.copy()
            try:
                self._last_corners = find_checkerboard(frame, self._pattern())
            except Exception:
                self._last_corners = None
            if self._last_corners is not None:
                cv2.drawChessboardCorners(vis, self._pattern(), self._last_corners, True)
            vis = cv2.resize(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB), (640, 480))
            photo = ImageTk.PhotoImage(Image.fromarray(vis))
            self.preview.config(image=photo, width=640, height=480)
            self.preview.image = photo
        self.after(150, self._tick)

    def capture(self):
        if self._last_corners is None:
            messagebox.showwarning("Calibration", "Checkerboard not detected in the current frame.", parent=self)
            return
        self.corner_sets.append(self._last_corners.copy())
        self.status.config(text=self._status_text())

    def compute_intrinsics(self):
        try:
            K, dist, rms = calibrate_intrinsics(self.corner_sets, self._pattern(),
                                                float(self.square_var.get()), self.image_size)
        except Exception as exc:
            messagebox.showerror("Calibration", str(exc), parent=self)
            return
        prev = self.calib
        self.calib = CameraCalibration(self.camera_name, self.image_size, K.tolist(), dist.tolist(), rms)
        if prev is not None and prev.extrinsics_source != "none":
            self.calib.T_world_cam, self.calib.extrinsics_source = prev.T_world_cam, prev.extrinsics_source
            self.calib.tags_used = prev.tags_used
        self.status.config(text=self._status_text("Intrinsics computed — now detect the AprilTag or save."))

    def detect_tag(self):
        if self.calib is None:
            messagebox.showwarning("Calibration", "Compute (or load) intrinsics first.", parent=self)
            return
        frame = self.get_frame()
        if frame is None:
            return
        try:
            T, ids = apriltag_extrinsics(frame, self.calib.K, self.calib.dist,
                                         float(self.tag_var.get()), self.family_var.get())
        except Exception as exc:
            messagebox.showerror("Calibration", str(exc), parent=self)
            return
        if T is None:
            messagebox.showwarning("Calibration", "No AprilTag detected.", parent=self)
            return
        self.calib.T_world_cam, self.calib.extrinsics_source, self.calib.tags_used = T.tolist(), "apriltag", ids
        self.status.config(text=self._status_text(f"Camera position (world): {np.round(T[:3, 3], 3).tolist()} m"))

    def save(self):
        if self.calib is None:
            messagebox.showwarning("Calibration", "Nothing to save yet.", parent=self)
            return
        path = self.calib.save()
        self.status.config(text=self._status_text(f"Saved to {path}"))

    def close(self):
        self._running = False
        self.destroy()
