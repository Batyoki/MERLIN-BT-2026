"""
Episode recorder shared by the GUI Recording tab and ``arm-bench record``.

Records REAL data only:
  * joint states of every arm publishing ``*/joint_states`` (position [rad],
    velocity [rad/s], effort = motor current [mA] as published by xs_sdk),
    with the ROS message stamp;
  * camera frames (JPEG) with their capture time;
  * the camera calibration (K, distortion, T_world_cam, Plücker ray file) in
    effect at recording time.
If no joint states are available the episode says so in metadata.json; nothing
is synthesised.

Episode layout
  <output>/<episode>/
      metadata.json
      steps.jsonl              one line per step (streamed while recording)
      camera_frames/<camera>_<step>.jpg
      calibration/<camera>.json, <camera>_rays.npz
      episode.h5               written at stop (needs h5py):
          /time                                 (N,)   seconds since start
          /arms/<arm>/{position,velocity,effort_mA,stamp}   (N, J) / (N,)
              attrs: joint_names
          /cameras/<camera>/{frame_file, t_capture}          (N,)
              attrs: K, dist, T_world_cam (if calibrated)
"""

import json
import os
import queue
import shutil
import threading
import time
from datetime import datetime
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from arm_bench.camera_calibration import CALIB_DIR, CameraCalibration, _safe

FrameSource = Callable[[], Dict[str, Tuple[np.ndarray, float]]]


class CameraGrabber:
    """Continuously reads OpenCV cameras in threads; latest() returns newest frames."""

    def __init__(self, cameras: Dict[str, int], width: int = 640, height: int = 480, fps: int = 15):
        import cv2
        self._cv2 = cv2
        self._latest: Dict[str, Tuple[np.ndarray, float]] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._caps = {}
        for name, index in cameras.items():
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            if not cap.isOpened():
                print(f"Camera {name} (/dev/video{index}) could not be opened")
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._caps[name] = cap
            threading.Thread(target=self._loop, args=(name, cap), daemon=True).start()

    def _loop(self, name, cap):
        while not self._stop.is_set():
            ok, frame = cap.read()
            if ok:
                with self._lock:
                    self._latest[name] = (frame, time.time())
            else:
                time.sleep(0.01)

    def latest(self) -> Dict[str, Tuple[np.ndarray, float]]:
        with self._lock:
            return dict(self._latest)

    def close(self):
        self._stop.set()
        time.sleep(0.05)
        for cap in self._caps.values():
            cap.release()


class EpisodeRecorder:
    def __init__(self, output_dir: Optional[str] = None, rate_hz: float = 30.0,
                 state_source=None, frame_source: Optional[FrameSource] = None):
        self.output_dir = os.path.expanduser(output_dir or "~/merlin_datasets")
        self.rate_hz = rate_hz
        self.state_source = state_source
        self.frame_source = frame_source
        self.recording = False
        self.episode_dir = None
        self.step = 0
        self.frames_written = 0
        self._t0 = 0.0
        self._thread = None
        self._stop = threading.Event()
        self._jpeg_q: "queue.Queue" = queue.Queue(maxsize=256)
        self._writer = None
        self._steps_file = None
        self._arms_seen: Dict[str, List[str]] = {}
        self._actions_seen: Dict[str, int] = {}
        self._cams_seen = set()
        self.dropped_frames = 0

    # ------------------------------------------------------------------
    def start(self, episode_name: Optional[str] = None, metadata: Optional[dict] = None) -> str:
        if self.recording:
            raise RuntimeError("Already recording")
        name = episode_name or f"episode_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.episode_dir = os.path.join(self.output_dir, _safe(name))
        os.makedirs(os.path.join(self.episode_dir, "camera_frames"), exist_ok=True)
        self._meta = dict(metadata or {}, episode=name, rate_hz=self.rate_hz,
                          start_time=time.time(), units=dict(position="rad", velocity="rad/s",
                                                             effort="mA (xs_sdk present current)"))
        self._steps_file = open(os.path.join(self.episode_dir, "steps.jsonl"), "w")
        self.step = self.frames_written = self.dropped_frames = 0
        self._arms_seen, self._cams_seen, self._actions_seen = {}, set(), {}
        self._stop.clear()
        self._writer = threading.Thread(target=self._write_jpegs, daemon=True)
        self._writer.start()
        self._t0 = time.time()
        self.recording = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self.episode_dir

    def stop(self) -> dict:
        if not self.recording:
            return {}
        self.recording = False
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._jpeg_q.put(None)
        self._writer.join(timeout=10.0)
        self._steps_file.close()

        calib = self._copy_calibrations()
        self._meta.update(
            end_time=time.time(), duration_s=time.time() - self._t0, num_steps=self.step,
            frames_written=self.frames_written, dropped_frames=self.dropped_frames,
            arms={a: names for a, names in self._arms_seen.items()},
            actions_recorded=sorted(self._actions_seen),
            action_note=("action = last JointGroupCommand.cmd (rad) seen on <arm>/commands/joint_group within 0.5 s"
                         if self._actions_seen else
                         "no actions recorded (no */commands/joint_group topic seen or interbotix_xs_msgs missing)"),
            cameras=sorted(self._cams_seen), calibrated_cameras=calib,
            joint_state_source=("ros" if self._arms_seen else "none — no */joint_states received"),
        )
        with open(os.path.join(self.episode_dir, "metadata.json"), "w") as f:
            json.dump(self._meta, f, indent=2)
        self._meta["hdf5"] = self._write_hdf5(calib)
        with open(os.path.join(self.episode_dir, "metadata.json"), "w") as f:
            json.dump(self._meta, f, indent=2)
        return dict(self._meta, episode_dir=self.episode_dir)

    # ------------------------------------------------------------------
    def _loop(self):
        period = 1.0 / self.rate_hz
        next_t = time.time()
        while not self._stop.is_set():
            t = time.time()
            rec = {"step": self.step, "t": t - self._t0, "arms": {}, "actions": {}, "frames": {}}
            if self.state_source is not None and getattr(self.state_source, "available", False):
                for arm, st in self.state_source.latest(max_age=0.5).items():
                    rec["arms"][arm] = st.as_dict()
                    self._arms_seen.setdefault(arm, st.names)
            if self.state_source is not None and hasattr(self.state_source, "commands"):
                for arm, c in self.state_source.commands(max_age=0.5).items():
                    rec["actions"][arm] = dict(cmd=c["cmd"], age_s=t - c["received"])
                    self._actions_seen.setdefault(arm, len(c["cmd"]))
            if self.frame_source is not None:
                for cam, (frame, t_cap) in self.frame_source().items():
                    fname = f"{_safe(cam)}_{self.step:06d}.jpg"
                    try:
                        self._jpeg_q.put_nowait((os.path.join(self.episode_dir, "camera_frames", fname), frame))
                        rec["frames"][cam] = {"file": fname, "t_capture": t_cap - self._t0}
                        self._cams_seen.add(cam)
                    except queue.Full:
                        self.dropped_frames += 1
            self._steps_file.write(json.dumps(rec) + "\n")
            self.step += 1
            next_t += period
            time.sleep(max(0.0, next_t - time.time()))

    def _write_jpegs(self):
        import cv2
        while True:
            item = self._jpeg_q.get()
            if item is None:
                break
            path, frame = item
            if cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                self.frames_written += 1

    def _copy_calibrations(self) -> List[str]:
        out = []
        dst = os.path.join(self.episode_dir, "calibration")
        for cam in self._cams_seen:
            src = os.path.join(CALIB_DIR, f"{_safe(cam)}.json")
            if os.path.isfile(src):
                os.makedirs(dst, exist_ok=True)
                shutil.copy2(src, dst)
                rays = src.replace(".json", "_rays.npz")
                if os.path.isfile(rays):
                    shutil.copy2(rays, dst)
                out.append(cam)
        return out

    def _write_hdf5(self, calibrated: List[str]) -> str:
        try:
            import h5py
        except ImportError:
            return "skipped (h5py not installed; steps.jsonl holds all data)"
        steps = [json.loads(line) for line in open(os.path.join(self.episode_dir, "steps.jsonl"))]
        path = os.path.join(self.episode_dir, "episode.h5")
        with h5py.File(path, "w") as h5:
            h5.create_dataset("time", data=np.array([s["t"] for s in steps]))
            for arm, names in self._arms_seen.items():
                g = h5.create_group(f"arms/{arm}")
                g.attrs["joint_names"] = json.dumps(names)
                for key in ("position", "velocity", "effort_mA"):
                    arr = np.full((len(steps), len(names)), np.nan)
                    for i, s in enumerate(steps):
                        v = s["arms"].get(arm, {}).get(key, [])
                        arr[i, :len(v)] = v[:len(names)]
                    g.create_dataset(key, data=arr)
                g.create_dataset("stamp", data=np.array(
                    [s["arms"].get(arm, {}).get("stamp", np.nan) for s in steps]))
            for arm, n in self._actions_seen.items():
                arr = np.full((len(steps), n), np.nan)
                age = np.full(len(steps), np.nan)
                for i, s in enumerate(steps):
                    a = s.get("actions", {}).get(arm)
                    if a:
                        arr[i, :len(a["cmd"])] = a["cmd"][:n]
                        age[i] = a["age_s"]
                g = h5.require_group(f"arms/{arm}")
                g.create_dataset("action", data=arr)
                g.create_dataset("action_age_s", data=age)
            for cam in sorted(self._cams_seen):
                g = h5.create_group(f"cameras/{_safe(cam)}")
                g.create_dataset("frame_file", data=np.array(
                    [s["frames"].get(cam, {}).get("file", "") for s in steps], dtype=h5py.string_dtype()))
                g.create_dataset("t_capture", data=np.array(
                    [s["frames"].get(cam, {}).get("t_capture", np.nan) for s in steps]))
                if cam in calibrated:
                    c = CameraCalibration.load(cam)
                    g.attrs["K"] = np.array(c.K)
                    g.attrs["dist"] = np.array(c.dist)
                    g.attrs["T_world_cam"] = np.array(c.T_world_cam)
        return path


def list_episodes(output_dir: Optional[str] = None) -> List[str]:
    root = os.path.expanduser(output_dir or "~/merlin_datasets")
    if not os.path.isdir(root):
        return []
    return sorted(d for d in os.listdir(root) if os.path.isfile(os.path.join(root, d, "steps.jsonl")))
