from __future__ import annotations

import threading
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from uuid import uuid4

import numpy as np

from .apriltag import AprilTagCalibrator, AprilTagLayout
from .store import CameraCalibration, calibration_dir
from .usb import CameraInfo, CameraStream, list_cameras
from .schema import FrameRecord, SessionManifest
from .writer import FrameWriter, JsonlStore, save_json


class TimestampedRecorder:
    """
    Multi-camera recorder with per-frame monotonic timestamps.

    - One thread per camera reads at max rate (no sleep, buffer size 1)
    - Timestamp captured immediately on frame read
    - Disk writes happen async so capture never blocks
    """

    def __init__(
        self,
        output_root: Path,
        camera_indices: list[int] | None = None,
        marker_length_m: float = 0.05,
        apriltag_family: str = "tag36h11",
        detect_tags: bool = False,
        session_name: str | None = None,
        tag_layout_path: Path | None = None,
    ) -> None:
        self.output_root = output_root
        self.camera_indices = camera_indices
        self.marker_length_m = marker_length_m
        self.apriltag_family = apriltag_family
        self.detect_tags = detect_tags
        self.tag_layout_path = tag_layout_path
        self.session_id = session_name or datetime.now(timezone.utc).strftime("session_%Y%m%d_%H%M%S")
        self.session_dir = output_root / self.session_id
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self._session_start_ns = time.time_ns()
        self._session_mono_start = time.perf_counter_ns()
        self._writer = FrameWriter()
        self._threads: list[threading.Thread] = []
        self._stop = threading.Event()
        self._frame_counts: dict[str, int] = {}
        self._jsonl: dict[str, JsonlStore] = {}
        self._calibrations: dict[str, CameraCalibration] = {}
        self._tag_layout = self._load_tag_layout()

    def calibrate_cameras(self, device_indices: list[int], intrinsics: dict[int, list[list[float]]] | None = None) -> list[CameraCalibration]:
        return [self.calibrate_camera(device_index, intrinsics=intrinsics.get(device_index) if intrinsics else None) for device_index in device_indices]

    def calibrate_camera(self, device_index: int, intrinsics: list[list[float]] | None = None) -> CameraCalibration:
        import cv2

        stream = CameraStream(device_index)
        width, height = stream.size
        matrix = intrinsics or self._load_intrinsics(device_index, width, height)
        calibrator = AprilTagCalibrator(marker_length_m=self.marker_length_m, family=self.apriltag_family, camera_matrix=matrix)
        ok, frame = stream.read()
        stream.release()
        if not ok:
            raise RuntimeError(f"Cannot read frame from camera {device_index}")

        result = calibrator.detect(frame, self._tag_layout)
        if not result.detections:
            raise RuntimeError(f"No AprilTag detected on camera {device_index}")

        usable_detections = result.detections
        if self._tag_layout is not None:
            layout_ids = set(self._tag_layout.keys())
            usable_detections = [det for det in result.detections if det.tag_id in layout_ids] or result.detections

        best = max(usable_detections, key=lambda d: d.confidence)
        extrinsics = best.cam_to_world
        tag_ids = [det.tag_id for det in usable_detections]
        if len(usable_detections) > 1:
            extrinsics = AprilTagCalibrator.estimate_camera_to_world(usable_detections).tolist()
        cameras = list_cameras()
        cam_info = next((c for c in cameras if c.index == device_index), None)
        camera_id = _safe_id(cam_info.name if cam_info else f"camera_{device_index}")

        cal = CameraCalibration(
            camera_id=camera_id,
            device_index=device_index,
            intrinsics=matrix,
            extrinsics=extrinsics,
            marker_length_m=self.marker_length_m,
            apriltag_family=self.apriltag_family,
            confidence=best.confidence,
            tag_id=best.tag_id,
            tag_ids=tag_ids,
        )
        cal.save(calibration_dir(self.session_dir) / f"{camera_id}.json")
        annotated = calibrator.annotate(frame, result)
        cv2.imwrite(str(self.session_dir / f"{camera_id}_calibration.png"), annotated)
        self._calibrations[camera_id] = cal
        return cal

    def record(self, duration_sec: float | None = None, max_frames: int | None = None) -> SessionManifest:
        cameras = list_cameras()
        indices = self.camera_indices or [c.index for c in cameras if c.readable]
        if not indices:
            raise RuntimeError("No readable cameras found")

        for index in indices:
            cal_path = self._find_calibration(index)
            if cal_path:
                self._calibrations[_safe_id(cal_path.stem)] = CameraCalibration.load(cal_path)

        for index in indices:
            cam_info = next((c for c in cameras if c.index == index), None)
            camera_id = _safe_id(cam_info.name if cam_info else f"camera_{index}")
            self._frame_counts[camera_id] = 0
            self._jsonl[camera_id] = JsonlStore(self.session_dir / camera_id / "frames.jsonl")
            thread = threading.Thread(target=self._capture_loop, args=(index, camera_id), daemon=True)
            thread.start()
            self._threads.append(thread)

        deadline = time.perf_counter() + duration_sec if duration_sec else None
        try:
            while not self._stop.is_set():
                if deadline and time.perf_counter() >= deadline:
                    break
                if max_frames and max(self._frame_counts.values() or [0]) >= max_frames:
                    break
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

        end_ns = time.time_ns()
        manifest = SessionManifest(
            session_id=self.session_id,
            started_at_ns=self._session_start_ns,
            cameras=[cal.to_dict() for cal in self._calibrations.values()],
            marker_length_m=self.marker_length_m,
            apriltag_family=self.apriltag_family,
            total_frames=sum(self._frame_counts.values()),
            duration_ns=end_ns - self._session_start_ns,
        )
        save_json(self.session_dir / "manifest.json", manifest.to_dict())
        episode_store = JsonlStore(self.session_dir / "episode_store.jsonl")
        episode_store.append({"type": "episode", "session_id": self.session_id, **manifest.to_dict()})
        episode_store.close()
        return manifest

    def stop(self) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=5)
        self._writer.flush()
        self._writer.close()
        for store in self._jsonl.values():
            store.close()

    def _capture_loop(self, device_index: int, camera_id: str) -> None:
        stream = CameraStream(device_index)
        width, height = stream.size
        cal = self._calibrations.get(camera_id)
        calibrator = None
        if self.detect_tags and cal:
            calibrator = AprilTagCalibrator(
                marker_length_m=self.marker_length_m,
                family=self.apriltag_family,
                camera_matrix=cal.intrinsics,
                dist_coeffs=cal.dist_coeffs,
            )

        while not self._stop.is_set():
            timestamp_ns = time.time_ns()
            session_time_ns = time.perf_counter_ns() - self._session_mono_start
            ok, frame = stream.read()
            if not ok:
                continue

            frame_index = self._frame_counts[camera_id]
            image_name = f"{timestamp_ns:019d}.png"
            image_path = self.session_dir / camera_id / "images" / image_name
            self._writer.write(image_path, frame)

            tags: list[dict[str, Any]] = []
            if calibrator:
                try:
                    tags = [asdict(d) for d in calibrator.detect(frame, self._tag_layout).detections]
                except Exception:
                    tags = []

            record = FrameRecord(
                timestamp_ns=timestamp_ns,
                session_time_ns=session_time_ns,
                camera_id=camera_id,
                device_index=device_index,
                frame_index=frame_index,
                image_path=str(image_path.relative_to(self.session_dir)),
                width=width,
                height=height,
                tags=tags,
            )
            self._jsonl[camera_id].append(asdict(record))
            self._frame_counts[camera_id] += 1

        stream.release()

    def _load_intrinsics(self, device_index: int, width: int, height: int) -> list[list[float]]:
        from .apriltag import default_intrinsics
        config = Path("config") / f"camera_{device_index}_intrinsics.json"
        if config.exists():
            import json
            payload = json.loads(config.read_text())
            return payload["intrinsics"]
        return default_intrinsics(width, height)

    def _find_calibration(self, device_index: int) -> Path | None:
        cal_dir = calibration_dir(self.session_dir)
        if not cal_dir.exists():
            return None
        for path in cal_dir.glob("*.json"):
            import json
            payload = json.loads(path.read_text())
            if payload.get("device_index") == device_index:
                return path
        return None

    def _load_tag_layout(self) -> AprilTagLayout | dict[int, list[list[float]]] | None:
        if self.tag_layout_path is None:
            return None
        return AprilTagLayout.load(self.tag_layout_path)


def _safe_id(name: str) -> str:
    return "".join(c if c.isalnum() or c in "-_" else "_" for c in name.strip()) or "camera"
