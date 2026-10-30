from __future__ import annotations

import subprocess
import sys
from dataclasses import dataclass

import numpy as np


@dataclass(slots=True)
class CameraInfo:
    index: int
    name: str
    width: int
    height: int
    fps: float
    readable: bool


def backend() -> int:
    import cv2
    if sys.platform == "darwin":
        return cv2.CAP_AVFOUNDATION
    if sys.platform.startswith("linux"):
        return cv2.CAP_V4L2
    return cv2.CAP_ANY


def list_cameras(max_probe: int = 8) -> list[CameraInfo]:
    import cv2
    found: list[CameraInfo] = []
    for index in range(max_probe):
        cap = cv2.VideoCapture(index, backend())
        if not cap.isOpened():
            cap.release()
            continue
        ok = any(cap.read()[0] for _ in range(3))
        w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
        cap.release()
        found.append(CameraInfo(index, _name(index), w, h, fps, ok))
    return found


def _name(index: int) -> str:
    if sys.platform != "darwin":
        return f"camera_{index}"
    try:
        out = subprocess.check_output(["system_profiler", "SPCameraDataType"], text=True, stderr=subprocess.DEVNULL, timeout=5)
        names = [line.strip()[:-1] for line in out.splitlines() if line.strip().endswith(":") and "Camera" not in line[:8]]
        return names[index] if index < len(names) else f"camera_{index}"
    except (subprocess.SubprocessError, FileNotFoundError):
        return f"camera_{index}"


class CameraStream:
    """Single USB camera — buffer disabled for lowest latency."""

    def __init__(self, index: int, width: int | None = None, height: int | None = None) -> None:
        import cv2
        self.index = index
        self._cv2 = cv2
        self._cap = cv2.VideoCapture(index, backend())
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera {index}")
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if width:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    @property
    def size(self) -> tuple[int, int]:
        w = int(self._cap.get(self._cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(self._cv2.CAP_PROP_FRAME_HEIGHT))
        return w, h

    def read(self) -> tuple[bool, np.ndarray]:
        return self._cap.read()

    def release(self) -> None:
        self._cap.release()
