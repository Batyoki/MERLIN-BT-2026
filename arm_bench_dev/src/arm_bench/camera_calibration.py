"""
Camera calibration used by the Cameras tab and the recorder.

* Intrinsics K, distortion: checkerboard (OpenCV calibrateCamera).
* Extrinsics T_world_cam: AprilTag(s) with known pose in the world frame
  (default: a single tag whose frame IS the world frame).
* Ray embeddings: per-pixel Plücker rays (d, o × d) in the world frame,
  computed from K and T_world_cam (camera-geometry-aware policies, Jiang et al.).

Results are stored per camera in ~/.merlin/calibration/<camera>.json (+ .npz
for the ray embeddings) and copied into every recorded episode.
"""

import json
import os
import time
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

CALIB_DIR = os.path.expanduser(os.environ.get("MERLIN_CALIB_DIR", "~/.merlin/calibration"))


@dataclass
class CameraCalibration:
    camera: str
    image_size: Tuple[int, int]                     # (width, height)
    K: List[List[float]]
    dist: List[float]
    reprojection_rms: float = float("nan")
    T_world_cam: List[List[float]] = field(default_factory=lambda: np.eye(4).tolist())
    extrinsics_source: str = "none"                 # "none" | "apriltag"
    tags_used: List[int] = field(default_factory=list)
    created: float = field(default_factory=time.time)

    def path(self) -> str:
        return os.path.join(CALIB_DIR, f"{_safe(self.camera)}.json")

    def save(self, with_rays: bool = True, ray_step: int = 4) -> str:
        os.makedirs(CALIB_DIR, exist_ok=True)
        with open(self.path(), "w") as f:
            json.dump(asdict(self), f, indent=2)
        if with_rays:
            rays = plucker_rays(np.array(self.K), np.array(self.T_world_cam), *self.image_size, step=ray_step)
            np.savez_compressed(self.path().replace(".json", "_rays.npz"), rays=rays, step=ray_step)
        return self.path()

    @classmethod
    def load(cls, camera: str) -> Optional["CameraCalibration"]:
        p = os.path.join(CALIB_DIR, f"{_safe(camera)}.json")
        if not os.path.isfile(p):
            return None
        with open(p) as f:
            d = json.load(f)
        d["image_size"] = tuple(d["image_size"])
        return cls(**d)


def _safe(name: str) -> str:
    return "".join(c if c.isalnum() or c in "-_" else "_" for c in str(name))


# ---------------------------------------------------------------------------
# Intrinsics
# ---------------------------------------------------------------------------
def find_checkerboard(image_bgr, pattern: Tuple[int, int]):
    """pattern = inner corners (cols, rows). Returns refined corners or None."""
    import cv2
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY) if image_bgr.ndim == 3 else image_bgr
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)


def calibrate_intrinsics(corner_sets, pattern: Tuple[int, int], square_m: float,
                         image_size: Tuple[int, int]):
    """Returns (K, dist, rms). Needs >= 5 views, better 15+ at varied angles."""
    import cv2
    if len(corner_sets) < 5:
        raise ValueError("Need at least 5 checkerboard views")
    obj = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    obj[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_m
    rms, K, dist, _, _ = cv2.calibrateCamera([obj] * len(corner_sets), corner_sets, image_size, None, None)
    return K, dist.ravel(), float(rms)


# ---------------------------------------------------------------------------
# Extrinsics
# ---------------------------------------------------------------------------
def apriltag_extrinsics(image_bgr, K, dist, tag_size_m: float, family: str = "tag36h11",
                        tag_poses_world: Optional[Dict[int, np.ndarray]] = None):
    """
    Camera pose in the world frame from AprilTags.
    tag_poses_world: {tag_id: 4x4 T_world_tag}; default: first detected tag = world.
    Returns (T_world_cam 4x4, [tag ids]) or (None, []).
    """
    import cv2
    try:
        import pupil_apriltags
    except ImportError as exc:
        raise ImportError("pupil-apriltags is required (pip install pupil-apriltags)") from exc
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    und = cv2.undistort(gray, np.asarray(K), np.asarray(dist))
    fx, fy, cx, cy = K[0][0], K[1][1], K[0][2], K[1][2]
    det = pupil_apriltags.Detector(families=family)
    found = det.detect(und, estimate_tag_pose=True, camera_params=(fx, fy, cx, cy), tag_size=tag_size_m)
    if not found:
        return None, []
    estimates, ids = [], []
    for d in found:
        T_cam_tag = np.eye(4)
        T_cam_tag[:3, :3] = d.pose_R
        T_cam_tag[:3, 3] = d.pose_t.ravel()
        if tag_poses_world is None:
            T_world_tag = np.eye(4) if not estimates else None
        else:
            T_world_tag = tag_poses_world.get(d.tag_id)
        if T_world_tag is None:
            continue
        estimates.append(T_world_tag @ np.linalg.inv(T_cam_tag))
        ids.append(int(d.tag_id))
    if not estimates:
        return None, []
    T = estimates[0].copy()
    if len(estimates) > 1:          # average translations, project mean rotation onto SO(3)
        T[:3, 3] = np.mean([e[:3, 3] for e in estimates], axis=0)
        U, _, Vt = np.linalg.svd(np.sum([e[:3, :3] for e in estimates], axis=0))
        T[:3, :3] = U @ Vt
    return T, ids


# ---------------------------------------------------------------------------
# Ray embeddings
# ---------------------------------------------------------------------------
def plucker_rays(K, T_world_cam, width: int, height: int, step: int = 1) -> np.ndarray:
    """
    Per-pixel Plücker coordinates in the world frame, shape (H/step, W/step, 6):
    [d (unit direction), m = o × d], with o the camera centre.
    """
    K = np.asarray(K, float)
    T = np.asarray(T_world_cam, float)
    u, v = np.meshgrid(np.arange(0, width, step) + 0.5, np.arange(0, height, step) + 0.5)
    pix = np.stack([u, v, np.ones_like(u)], axis=-1)
    d_cam = pix @ np.linalg.inv(K).T
    d_world = d_cam @ T[:3, :3].T
    d_world /= np.linalg.norm(d_world, axis=-1, keepdims=True)
    o = T[:3, 3]
    m = np.cross(np.broadcast_to(o, d_world.shape), d_world)
    return np.concatenate([d_world, m], axis=-1).astype(np.float32)


def default_intrinsics(width: int, height: int, fov_deg: float = 70.0):
    """Only a placeholder when no calibration exists — flagged as uncalibrated."""
    f = 0.5 * width / np.tan(np.deg2rad(fov_deg) / 2)
    return [[f, 0, width / 2], [0, f, height / 2], [0, 0, 1]]
