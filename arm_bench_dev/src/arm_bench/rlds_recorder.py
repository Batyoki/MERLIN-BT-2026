"""
Dataset recording entry points for MERLIN (kept for backward compatibility).

The previous RLDSRecorder wrote SYNTHETIC joint data (sin/cos/random) and never
captured camera frames. Recording is now done by
arm_bench.episode_recorder.EpisodeRecorder, which stores real joint states from
ROS and real camera frames; this module only adapts the old API to it.
"""

from typing import Dict, List, Optional

from arm_bench.episode_recorder import CameraGrabber, EpisodeRecorder, list_episodes
from arm_bench.ros_state import RosJointStateSource


class RLDSRecorder(EpisodeRecorder):
    """Old API (start_recording / stop_recording / list_episodes) on the real recorder."""

    def __init__(self, output_dir: Optional[str] = None, cameras: Optional[Dict[str, int]] = None,
                 rate_hz: float = 30.0):
        self._grabber = CameraGrabber(cameras) if cameras else None
        super().__init__(output_dir, rate_hz, state_source=RosJointStateSource.shared(),
                         frame_source=self._grabber.latest if self._grabber else None)

    def start_recording(self, episode_name: Optional[str] = None, metadata: Optional[dict] = None) -> str:
        return self.start(episode_name, metadata)

    def stop_recording(self) -> dict:
        summary = self.stop()
        if self._grabber:
            self._grabber.close()
        return summary

    def list_episodes(self) -> List[str]:
        return list_episodes(self.output_dir)


class CameraManager:
    """Lists /dev/video* cameras that deliver frames, with user-defined names."""

    def __init__(self):
        self.cameras = {}
        self.camera_names = {}

    def scan_cameras(self) -> List[Dict]:
        from arm_bench.scanner import scan_cameras
        cameras = [dict(c, available=True) for c in scan_cameras()]
        self.cameras = {cam["id"]: cam for cam in cameras}
        return cameras

    def rename_camera(self, camera_id: int, new_name: str):
        if camera_id in self.cameras:
            self.cameras[camera_id]["name"] = new_name
            self.camera_names[camera_id] = new_name
        else:
            print(f"Camera {camera_id} not found")

    def get_camera_name(self, camera_id: int) -> Optional[str]:
        if camera_id in self.camera_names:
            return self.camera_names[camera_id]
        if camera_id in self.cameras:
            return self.cameras[camera_id]["name"]
        return None


def create_recorder(output_dir: Optional[str] = None, cameras: Optional[Dict[str, int]] = None,
                    rate_hz: float = 30.0) -> RLDSRecorder:
    return RLDSRecorder(output_dir, cameras, rate_hz)


def create_camera_manager() -> CameraManager:
    return CameraManager()
