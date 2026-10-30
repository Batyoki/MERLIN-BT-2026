from __future__ import annotations

import json
import threading
from collections import deque
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np


class FrameWriter:
    """Async image writer — capture thread never waits on disk I/O."""

    def __init__(self, max_queue: int = 512) -> None:
        self._queue: deque[tuple[Path, np.ndarray]] = deque()
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def write(self, path: Path, image: np.ndarray) -> None:
        with self._lock:
            self._queue.append((path, image.copy()))
        self._event.set()

    def flush(self, timeout: float = 30.0) -> None:
        import time
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if not self._queue:
                    return
            time.sleep(0.01)
        raise TimeoutError("FrameWriter flush timeout")

    def close(self) -> None:
        self._stop.set()
        self._event.set()
        self._thread.join(timeout=10)

    def _run(self) -> None:
        import cv2
        while not self._stop.is_set() or self._queue:
            self._event.wait(timeout=0.05)
            self._event.clear()
            batch: list[tuple[Path, np.ndarray]] = []
            with self._lock:
                while self._queue:
                    batch.append(self._queue.popleft())
            for path, image in batch:
                path.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(path), image)


class JsonlStore:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._handle = path.open("a", encoding="utf-8")

    def append(self, record: dict[str, Any]) -> None:
        with self._lock:
            self._handle.write(json.dumps(record, ensure_ascii=False))
            self._handle.write("\n")
            self._handle.flush()

    def close(self) -> None:
        self._handle.close()


def save_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
