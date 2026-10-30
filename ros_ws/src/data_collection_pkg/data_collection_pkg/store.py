from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class CameraCalibration:
    camera_id: str
    device_index: int
    intrinsics: list[list[float]]
    dist_coeffs: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0])
    extrinsics: list[list[float]] = field(default_factory=lambda: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    marker_length_m: float = 0.05
    apriltag_family: str = "tag36h11"
    confidence: float = 0.0
    tag_id: int | None = None
    tag_ids: list[int] = field(default_factory=list)

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path: Path) -> CameraCalibration:
        payload = json.loads(path.read_text(encoding="utf-8"))
        return cls(**payload)

    def to_dict(self) -> dict[str, Any]:
        return {
            "camera_id": self.camera_id,
            "device_index": self.device_index,
            "intrinsics": self.intrinsics,
            "dist_coeffs": self.dist_coeffs,
            "extrinsics": self.extrinsics,
            "marker_length_m": self.marker_length_m,
            "apriltag_family": self.apriltag_family,
            "confidence": self.confidence,
            "tag_id": self.tag_id,
            "tag_ids": self.tag_ids,
        }


def calibration_dir(session_dir: Path) -> Path:
    return session_dir / "calibration"
