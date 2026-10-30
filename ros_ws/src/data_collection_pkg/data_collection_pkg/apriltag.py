from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np


@dataclass(slots=True)
class TagDetection:
    tag_id: int
    family: str
    corners: list[list[float]]
    tvec: list[float]
    rvec: list[float]
    tag_to_camera: list[list[float]]
    camera_to_tag: list[list[float]]
    cam_to_world: list[list[float]]
    distance_m: float
    confidence: float


@dataclass(slots=True)
class TagFrameResult:
    detections: list[TagDetection]
    image_size: tuple[int, int]


@dataclass(slots=True)
class AprilTagLayout:
    tags: dict[int, list[list[float]]]
    name: str | None = None

    @classmethod
    def load(cls, path: Path) -> AprilTagLayout:
        import json

        payload = json.loads(path.read_text(encoding="utf-8"))
        tags: dict[int, list[list[float]]] = {}

        if isinstance(payload, dict) and "tags" in payload:
            raw_tags = payload["tags"]
            if isinstance(raw_tags, list):
                for item in raw_tags:
                    tag_id = int(item["tag_id"])
                    tags[tag_id] = item["tag_to_world"]
            elif isinstance(raw_tags, dict):
                for key, value in raw_tags.items():
                    tags[int(key)] = value
        elif isinstance(payload, list):
            for item in payload:
                tag_id = int(item["tag_id"])
                tags[tag_id] = item["tag_to_world"]
        else:
            raise ValueError(f"Unsupported tag layout format in {path}")

        if not tags:
            raise ValueError(f"No tag poses found in {path}")

        name = payload.get("name") if isinstance(payload, dict) else None
        return cls(tags=tags, name=name)

    def as_matrices(self) -> dict[int, np.ndarray]:
        return {tag_id: np.array(matrix, dtype=np.float64) for tag_id, matrix in self.tags.items()}


class AprilTagCalibrator:
    def __init__(
        self,
        marker_length_m: float = 0.05,
        family: str = "tag36h11",
        camera_matrix: list[list[float]] | None = None,
        dist_coeffs: list[float] | None = None,
    ) -> None:
        self.marker_length_m = marker_length_m
        self.family = family
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs or [0.0, 0.0, 0.0, 0.0, 0.0]

    def detect(
        self,
        image: np.ndarray,
        tag_to_world: np.ndarray | dict[int, np.ndarray] | AprilTagLayout | None = None,
    ) -> TagFrameResult:
        import cv2
        import pupil_apriltags as apriltags

        if self.camera_matrix is None:
            raise ValueError("camera_matrix required — run `calibrate init` first or pass intrinsics")

        layout, single_world = _normalize_layout(tag_to_world)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        fx, fy = float(self.camera_matrix[0][0]), float(self.camera_matrix[1][1])
        cx, cy = float(self.camera_matrix[0][2]), float(self.camera_matrix[1][2])

        detector = apriltags.Detector(families=self.family)
        raw = detector.detect(gray, estimate_tag_pose=True, camera_params=(fx, fy, cx, cy), tag_size=self.marker_length_m)

        detections: list[TagDetection] = []
        for item in raw:
            rotation = np.array(item.pose_R, dtype=np.float64)
            translation = np.array(item.pose_t, dtype=np.float64).reshape(3)
            tag_to_camera = _transform(rotation, translation)
            camera_to_tag = np.linalg.inv(tag_to_camera)
            if single_world is not None:
                tag_world = single_world
            elif layout is not None:
                tag_world = layout.get(int(item.tag_id), np.eye(4, dtype=np.float64))
            else:
                tag_world = np.eye(4, dtype=np.float64)
            cam_to_world = tag_world @ camera_to_tag
            rvec, _ = cv2.Rodrigues(rotation)
            detections.append(
                TagDetection(
                    tag_id=int(item.tag_id),
                    family=self.family,
                    corners=item.corners.astype(float).tolist(),
                    tvec=translation.astype(float).tolist(),
                    rvec=rvec.reshape(3).astype(float).tolist(),
                    tag_to_camera=tag_to_camera.astype(float).tolist(),
                    camera_to_tag=camera_to_tag.astype(float).tolist(),
                    cam_to_world=cam_to_world.astype(float).tolist(),
                    distance_m=float(np.linalg.norm(translation)),
                    confidence=max(0.0, min(1.0, 1.0 - float(getattr(item, "pose_err", 0.0)))),
                )
            )
        h, w = gray.shape[:2]
        return TagFrameResult(detections=detections, image_size=(w, h))

    @staticmethod
    def estimate_camera_to_world(detections: list[TagDetection]) -> np.ndarray:
        if not detections:
            raise ValueError("No detections available to estimate camera pose")

        weights = np.array([max(det.confidence, 1e-6) for det in detections], dtype=np.float64)
        weights = weights / weights.sum()

        rotation_stack = np.zeros((3, 3), dtype=np.float64)
        translation_stack = np.zeros(3, dtype=np.float64)
        for det, weight in zip(detections, weights, strict=False):
            transform = np.array(det.cam_to_world, dtype=np.float64)
            rotation_stack += weight * transform[:3, :3]
            translation_stack += weight * transform[:3, 3]

        u, _, vt = np.linalg.svd(rotation_stack)
        rotation = u @ vt
        if np.linalg.det(rotation) < 0:
            u[:, -1] *= -1
            rotation = u @ vt

        return _transform(rotation, translation_stack)

    def annotate(self, image: np.ndarray, result: TagFrameResult | None = None) -> np.ndarray:
        import cv2

        if result is None:
            result = self.detect(image)
        out = image.copy()
        for det in result.detections:
            pts = np.array(det.corners, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(out, [pts], True, (0, 255, 0), 2)
            center = pts.mean(axis=0).astype(int).reshape(2)
            label = f"ID{det.tag_id} d={det.distance_m:.3f}m t=[{det.tvec[0]:+.2f},{det.tvec[1]:+.2f},{det.tvec[2]:+.2f}]"
            cv2.putText(out, label, (int(center[0]), int(center[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            if self.camera_matrix:
                cv2.drawFrameAxes(
                    out,
                    np.array(self.camera_matrix, dtype=np.float64),
                    np.array(self.dist_coeffs, dtype=np.float64),
                    np.array(det.rvec, dtype=np.float64).reshape(3, 1),
                    np.array(det.tvec, dtype=np.float64).reshape(3, 1),
                    self.marker_length_m * 0.5,
                )
        return out

    @staticmethod
    def detection_dict(det: TagDetection) -> dict[str, Any]:
        return asdict(det)


def _transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation.reshape(3)
    return transform


def _normalize_layout(
    tag_to_world: np.ndarray | dict[int, np.ndarray] | AprilTagLayout | None,
) -> tuple[dict[int, np.ndarray] | None, np.ndarray | None]:
    if tag_to_world is None:
        return None, None
    if isinstance(tag_to_world, AprilTagLayout):
        return tag_to_world.as_matrices(), None
    if isinstance(tag_to_world, dict):
        return {int(tag_id): np.array(matrix, dtype=np.float64) for tag_id, matrix in tag_to_world.items()}, None
    return None, np.array(tag_to_world, dtype=np.float64)


def default_intrinsics(width: int, height: int, fov_deg: float = 70.0) -> list[list[float]]:
    fx = width / (2.0 * np.tan(np.radians(fov_deg / 2.0)))
    fy = fx
    return [[fx, 0.0, width / 2.0], [0.0, fy, height / 2.0], [0.0, 0.0, 1.0]]
