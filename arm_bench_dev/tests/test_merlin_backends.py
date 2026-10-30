"""Functional tests for the GUI back-ends (no hardware, no ROS needed)."""

import json
import math
import os
import sys
import time
import types

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Motor monitor: serial reader against a simulated Dynamixel bus
# ---------------------------------------------------------------------------
class _FakeBus:
    """Mimics dynamixel_sdk PortHandler/PacketHandler for two X-series motors."""
    regs = {
        1: {0: 1020, 126: 37, 128: 10, 132: 3072, 144: 121, 146: 34},           # XM430
        2: {0: 1060, 126: (1 << 16) - 50, 128: 0, 132: 2048, 144: 120, 146: 31},  # XL430, load -5 %
    }


def _install_fake_sdk(monkeypatch):
    sdk = types.ModuleType("dynamixel_sdk")
    sdk.COMM_SUCCESS = 0

    class PortHandler:
        def __init__(self, port):
            self.port = port

        def openPort(self):
            return True

        def setBaudRate(self, b):
            return True

        def closePort(self):
            pass

    class PacketHandler:
        def __init__(self, proto):
            assert proto == 2.0

        def ping(self, ph, i):
            return (_FakeBus.regs[i][0], 0, 0) if i in _FakeBus.regs else (0, -3001, 0)

        def _rd(self, ph, i, addr):
            return _FakeBus.regs[i].get(addr, 0), 0, 0

        read1ByteTxRx = read2ByteTxRx = read4ByteTxRx = _rd

    sdk.PortHandler, sdk.PacketHandler = PortHandler, PacketHandler
    monkeypatch.setitem(sys.modules, "dynamixel_sdk", sdk)


def test_serial_reader_reads_only_answering_motors(monkeypatch):
    _install_fake_sdk(monkeypatch)
    from arm_bench.motor_monitor import MotorMonitor
    mm = MotorMonitor()
    msg = mm.connect_serial("/dev/ttyFAKE")
    assert "IDs [1, 2]" in msg
    rows = {r["ID / Joint"]: r for r in mm.rows()}
    assert set(rows) == {"1", "2"}
    assert rows["1"]["Model"] == "XM430-W350"
    assert float(rows["1"]["Position (rad)"]) == pytest.approx(math.pi / 2, abs=1e-3)
    assert float(rows["1"]["Current (mA)"]) == pytest.approx(37 * 2.69, abs=1)
    assert rows["1"]["Temp (°C)"] == "34"
    assert rows["2"]["Current (mA)"] == "load -5.0 %"          # XL430 reports load, not current
    mm.disconnect_serial()
    assert mm.rows() == []                                     # nothing simulated when disconnected


# ---------------------------------------------------------------------------
# Calibration: synthetic checkerboard views with a known camera
# ---------------------------------------------------------------------------
def _render_views(K, pattern=(9, 6), square=0.025, n=12, size=(640, 480)):
    cv2 = pytest.importorskip("cv2")
    sq_px = 40
    board = np.full(((pattern[1] + 3) * sq_px, (pattern[0] + 3) * sq_px), 255, np.uint8)
    for r in range(pattern[1] + 1):
        for c in range(pattern[0] + 1):
            if (r + c) % 2 == 0:
                board[(r + 1) * sq_px:(r + 2) * sq_px, (c + 1) * sq_px:(c + 2) * sq_px] = 0
    s = square / sq_px
    views, rng = [], np.random.default_rng(0)
    for _ in range(n):
        rvec = rng.normal(0, 0.25, 3)
        tvec = np.array([rng.normal(0, 0.03), rng.normal(0, 0.03), rng.uniform(0.35, 0.5)])
        R = cv2.Rodrigues(rvec)[0]
        # homography board-pixel -> image for the plane z=0 (board origin at its centre)
        off = np.array([[s, 0, -board.shape[1] * s / 2], [0, s, -board.shape[0] * s / 2], [0, 0, 1]])
        H = np.asarray(K) @ np.column_stack([R[:, 0], R[:, 1], tvec]) @ off
        img = cv2.warpPerspective(board, H, size, borderValue=255)
        views.append(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
    return views


def test_intrinsic_calibration_recovers_known_camera():
    from arm_bench.camera_calibration import calibrate_intrinsics, find_checkerboard
    K = np.array([[520.0, 0, 320], [0, 520.0, 240], [0, 0, 1]])
    corners = [c for c in (find_checkerboard(v, (9, 6)) for v in _render_views(K)) if c is not None]
    assert len(corners) >= 8
    K_est, dist, rms = calibrate_intrinsics(corners, (9, 6), 0.025, (640, 480))
    assert rms < 1.0
    assert K_est[0, 0] == pytest.approx(520, rel=0.03)
    assert K_est[0, 2] == pytest.approx(320, abs=10)


def test_apriltag_extrinsics_and_rays(tmp_path, monkeypatch):
    cv2 = pytest.importorskip("cv2")
    pytest.importorskip("pupil_apriltags")
    monkeypatch.setenv("MERLIN_CALIB_DIR", str(tmp_path))
    import importlib
    import arm_bench.camera_calibration as cc
    importlib.reload(cc)
    K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])
    # tag 0 (tag36h11), 0.08 m, facing the camera 0.5 m away, centred
    tag = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 0, 200)
    tag = cv2.copyMakeBorder(tag, 25, 25, 25, 25, cv2.BORDER_CONSTANT, value=255)   # quiet zone
    tag_m, z = 0.08, 0.5
    px = 600 * tag_m / z * (250 / 200)          # rendered size incl. white border
    img = np.full((480, 640), 255, np.uint8)
    t = cv2.resize(tag, (int(px), int(px)), interpolation=cv2.INTER_NEAREST)
    y0, x0 = 240 - t.shape[0] // 2, 320 - t.shape[1] // 2
    img[y0:y0 + t.shape[0], x0:x0 + t.shape[1]] = t
    T, ids = cc.apriltag_extrinsics(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), K, np.zeros(5), tag_m)
    assert ids == [0]
    # camera centre in the tag (world) frame: 0.5 m along the tag normal
    assert np.linalg.norm(T[:3, 3]) == pytest.approx(z, rel=0.05)

    rays = cc.plucker_rays(K, T, 640, 480, step=8)
    assert rays.shape == (60, 80, 6)
    assert np.allclose(np.linalg.norm(rays[..., :3], axis=-1), 1, atol=1e-5)
    assert np.allclose(np.sum(rays[..., :3] * rays[..., 3:], axis=-1), 0, atol=1e-5)   # Plücker constraint

    calib = cc.CameraCalibration("Front cam", (640, 480), K.tolist(), [0] * 5, 0.2, T.tolist(), "apriltag", ids)
    calib.save()
    loaded = cc.CameraCalibration.load("Front cam")
    assert loaded.extrinsics_source == "apriltag" and os.path.isfile(calib.path().replace(".json", "_rays.npz"))


# ---------------------------------------------------------------------------
# Recorder: real-data pipeline with a stub state source and frame source
# ---------------------------------------------------------------------------
class _StubState:
    available = True

    def __init__(self):
        from arm_bench.ros_state import ArmJointState
        self._mk = ArmJointState

    def latest(self, max_age=1.0):
        t = time.time()
        return {"slaveR": self._mk("slaveR", ["waist", "shoulder"], [math.sin(t), 0.1], [0.0, 0.0], [12.0, -5.0], t)}

    def arms(self, max_age=1.0):
        return ["slaveR"]


def test_episode_recorder_writes_real_streams(tmp_path, monkeypatch):
    pytest.importorskip("cv2")
    h5py = pytest.importorskip("h5py")
    monkeypatch.setenv("MERLIN_CALIB_DIR", str(tmp_path / "calib"))
    import importlib
    import arm_bench.camera_calibration as cc
    import arm_bench.episode_recorder as er
    importlib.reload(cc)
    importlib.reload(er)
    cc.CameraCalibration("Front", (64, 48), np.eye(3).tolist(), [0] * 5, 0.1).save(ray_step=8)
    frame = np.zeros((48, 64, 3), np.uint8)
    rec = er.EpisodeRecorder(str(tmp_path / "out"), rate_hz=50, state_source=_StubState(),
                             frame_source=lambda: {"Front": (frame, time.time())})
    path = rec.start("ep1")
    time.sleep(0.5)
    s = rec.stop()
    assert s["num_steps"] >= 15 and s["frames_written"] == s["num_steps"]
    assert s["arms"] == {"slaveR": ["waist", "shoulder"]} and s["calibrated_cameras"] == ["Front"]
    with h5py.File(os.path.join(path, "episode.h5")) as h5:
        assert h5["arms/slaveR/effort_mA"][0].tolist() == [12.0, -5.0]
        assert json.loads(h5["arms/slaveR"].attrs["joint_names"]) == ["waist", "shoulder"]
        assert h5["cameras/Front/frame_file"][0].decode().endswith(".jpg")
        assert h5["cameras/Front"].attrs["K"].shape == (3, 3)
    assert os.path.isfile(os.path.join(path, "calibration", "Front.json"))


class _StubStateWithActions(_StubState):
    def commands(self, max_age=1.0):
        return {"slaveR": dict(group="arm", cmd=[0.25, -0.5], received=time.time() - 0.01)}


def test_episode_recorder_stores_commanded_actions(tmp_path):
    h5py = pytest.importorskip("h5py")
    from arm_bench.episode_recorder import EpisodeRecorder
    rec = EpisodeRecorder(str(tmp_path), rate_hz=50, state_source=_StubStateWithActions(), frame_source=None)
    rec.start("act")
    time.sleep(0.3)
    s = rec.stop()
    assert s["actions_recorded"] == ["slaveR"] and "JointGroupCommand" in s["action_note"]
    step = json.loads(open(os.path.join(s["episode_dir"], "steps.jsonl")).readline())
    assert step["actions"]["slaveR"]["cmd"] == [0.25, -0.5]
    with h5py.File(os.path.join(s["episode_dir"], "episode.h5")) as h5:
        assert h5["arms/slaveR/action"][0].tolist() == [0.25, -0.5]
        assert 0.0 <= h5["arms/slaveR/action_age_s"][0] < 0.1


def test_recorder_without_command_source_says_so(tmp_path):
    from arm_bench.episode_recorder import EpisodeRecorder
    rec = EpisodeRecorder(str(tmp_path), rate_hz=50, state_source=_StubState(), frame_source=None)
    rec.start("noact")
    time.sleep(0.1)
    s = rec.stop()
    assert s["actions_recorded"] == [] and s["action_note"].startswith("no actions recorded")


def test_recorder_does_not_invent_joint_data(tmp_path):
    from arm_bench.episode_recorder import EpisodeRecorder
    rec = EpisodeRecorder(str(tmp_path), rate_hz=50, state_source=None, frame_source=None)
    rec.start("empty")
    time.sleep(0.1)
    s = rec.stop()
    assert s["arms"] == {} and s["joint_state_source"].startswith("none")
    line = open(os.path.join(s["episode_dir"], "steps.jsonl")).readline()
    assert json.loads(line)["arms"] == {}


# ---------------------------------------------------------------------------
# Controller launcher
# ---------------------------------------------------------------------------
def test_bilateral_launch_command_carries_gui_settings():
    from arm_bench.controller_launcher import build_mpc_launch_command
    cmd = build_mpc_launch_command("/tmp/ros_ws", mode="pf", params_file="/a b/gains.yaml",
                                   launch_drivers=True, use_rviz=True, loop_rate_hz=100)
    script = cmd[2]
    for part in ("control_mode:=pf", "'params_file:=/a b/gains.yaml'", "launch_drivers:=true",
                 "use_rviz:=true", "loop_rate_hz:=100.0"):
        assert part in script


def test_launcher_refuses_unbuilt_workspace(tmp_path):
    from arm_bench.controller_launcher import launch_mpc_controller
    with pytest.raises(FileNotFoundError):
        launch_mpc_controller(str(tmp_path))
