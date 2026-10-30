from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(slots=True)
class FrameRecord:
    timestamp_ns: int
    session_time_ns: int
    camera_id: str
    device_index: int
    frame_index: int
    image_path: str
    width: int
    height: int
    tags: list[dict[str, Any]] = field(default_factory=list)


@dataclass(slots=True)
class SessionManifest:
    session_id: str
    started_at_ns: int
    cameras: list[dict[str, Any]]
    marker_length_m: float
    apriltag_family: str
    total_frames: int = 0
    duration_ns: int = 0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
