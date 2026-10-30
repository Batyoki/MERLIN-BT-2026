"""
Pre-flight checks for the hardware that cannot be simulated: cameras, joystick, WebXR phone link and the
ROS topics of the arms.

    python -m arm_bench.hardware_check cameras [--ids 0 2 4] [--seconds 5]
    python -m arm_bench.hardware_check joystick [--seconds 10]
    python -m arm_bench.hardware_check webxr                  # loopback self-test of the server code, no phone needed
    python -m arm_bench.hardware_check ros [--seconds 5]      # joint_states rates and command topics of the arms

Every command prints a table and exits 0 only when all checks passed, so it can be put in a launch script.
The numbers are MEASURED on the machine it runs on; none of them is assumed. The loopback test of the WebXR
server runs without a phone: it checks that the server code parses poses and calls the callback, not that the
phone's browser, certificate or WiFi work (open https://<ip>:8443 on the phone for that, see
docs/HARDWARE_VALIDATION.md).
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from typing import Dict, List, Optional, Sequence

import numpy as np


# ----------------------------------------------------------------------------------------------- cameras
def analyse_stream(stamps: Sequence[float], shape: Optional[tuple], expected_fps: float = 15.0) -> Dict[str, float]:
    """Frame-rate statistics of one camera from its capture timestamps."""
    t = np.asarray(stamps, float)
    if len(t) < 3:
        return dict(frames=len(t), fps=0.0, max_gap_ms=float("inf"), jitter_ms=float("inf"),
                    ok=False, shape=shape, reason="fewer than 3 frames")
    dt = np.diff(t)
    fps = (len(t) - 1) / (t[-1] - t[0])
    ok = fps >= 0.8 * expected_fps and dt.max() < 0.5
    return dict(frames=len(t), fps=float(fps), max_gap_ms=float(1e3 * dt.max()), jitter_ms=float(1e3 * dt.std()),
                ok=bool(ok), shape=shape, reason="" if ok else "frame rate below 80 % of expected or a gap > 0.5 s")


def camera_skew_ms(stamps_a: Sequence[float], stamps_b: Sequence[float]) -> float:
    """Median absolute time to the nearest frame of the other camera (ms). Frames of two free-running USB
    cameras are not synchronised; this is the offset to expect when you pair frames by time."""
    a, b = np.asarray(stamps_a, float), np.asarray(stamps_b, float)
    if len(a) == 0 or len(b) == 0:
        return float("nan")
    return float(1e3 * np.median([np.min(np.abs(b - x)) for x in a]))


def check_cameras(ids: Optional[List[int]], seconds: float, expected_fps: float = 15.0) -> bool:
    try:
        import cv2
    except ImportError:
        print("opencv-python is not installed: pip install opencv-python")
        return False
    if not ids:
        from arm_bench.scanner import scan_cameras
        ids = [c["id"] for c in scan_cameras()]
    if not ids:
        print("No camera found.")
        return False
    caps = {}
    for i in ids:
        cap = cv2.VideoCapture(i)
        if not cap.isOpened():
            print(f"camera {i}: cannot open")
            continue
        cap.set(cv2.CAP_PROP_FPS, expected_fps)
        caps[i] = cap
    stamps = {i: [] for i in caps}
    shapes = {i: None for i in caps}

    def grab(i):
        end = time.time() + seconds
        while time.time() < end:
            ok, frame = caps[i].read()
            if ok:
                stamps[i].append(time.time())
                shapes[i] = frame.shape
    threads = [threading.Thread(target=grab, args=(i,)) for i in caps]      # all cameras at the same time:
    [t.start() for t in threads]                                             # USB bandwidth problems only show up so
    [t.join() for t in threads]
    [c.release() for c in caps.values()]
    all_ok = len(caps) == len(ids)
    print(f"{'camera':>7} {'frames':>7} {'fps':>6} {'max gap ms':>11} {'jitter ms':>10} {'shape':>16}  result")
    for i in caps:
        r = analyse_stream(stamps[i], shapes[i], expected_fps)
        all_ok &= r["ok"]
        print(f"{i:>7} {r['frames']:>7} {r['fps']:>6.1f} {r['max_gap_ms']:>11.0f} {r['jitter_ms']:>10.1f} "
              f"{str(r['shape']):>16}  {'OK' if r['ok'] else 'FAIL: ' + r['reason']}")
    ids_ok = list(caps)
    for a in range(len(ids_ok)):
        for b in range(a + 1, len(ids_ok)):
            print(f"pair {ids_ok[a]}-{ids_ok[b]}: median offset to the nearest frame of the other camera "
                  f"{camera_skew_ms(stamps[ids_ok[a]], stamps[ids_ok[b]]):.1f} ms (cameras are NOT hardware-synchronised)")
    return all_ok


# ---------------------------------------------------------------------------------------------- joystick
def analyse_axes(samples: np.ndarray, deadzone: float = 0.1) -> Dict[str, object]:
    """samples: (n_samples, n_axes) recorded while the operator first leaves the stick alone, then sweeps it.
    Reports the idle offset (first 20 % of samples) and the range reached."""
    s = np.asarray(samples, float)
    n_idle = max(1, int(0.2 * len(s)))
    idle = np.abs(np.median(s[:n_idle], axis=0))
    rng = s.max(axis=0) - s.min(axis=0)
    return dict(idle_offset=idle, range=rng, idle_ok=bool(np.all(idle < deadzone)), moved=rng > 0.5)


def check_joystick(seconds: float, device: int = 0, deadzone: float = 0.1) -> bool:
    try:
        import pygame
    except ImportError:
        print("pygame is not installed: pip install pygame")
        return False
    pygame.init()
    pygame.joystick.init()
    n = pygame.joystick.get_count()
    if n == 0:
        print("No joystick detected.")
        return False
    for i in range(n):
        js = pygame.joystick.Joystick(i)
        print(f"[{i}] {js.get_name()}  axes={js.get_numaxes()} buttons={js.get_numbuttons()} hats={js.get_numhats()}")
    js = pygame.joystick.Joystick(device)
    print(f"Leave the stick alone for {0.2 * seconds:.0f} s, then move every axis to both ends ({seconds:.0f} s total).")
    rows, end = [], time.time() + seconds
    while time.time() < end:
        pygame.event.pump()
        rows.append([js.get_axis(a) for a in range(js.get_numaxes())])
        time.sleep(0.01)
    r = analyse_axes(np.array(rows), deadzone)
    for a in range(len(r["range"])):
        print(f"axis {a}: idle offset {r['idle_offset'][a]:+.3f}, range {r['range'][a]:.2f}  "
              f"{'moved' if r['moved'][a] else 'NOT MOVED'}")
    print(f"idle offsets {'below' if r['idle_ok'] else 'ABOVE'} the {deadzone} dead-zone"
          + ("" if r["idle_ok"] else ": raise the dead-zone or recalibrate the stick, otherwise the arm drifts"))
    return bool(r["idle_ok"] and r["moved"].any())


# -------------------------------------------------------------------------------------------------- WebXR
def check_webxr_loopback(n_msgs: int = 20) -> bool:
    """Drive the real WebXR server handler through a WebSocket test client (no phone, no network)."""
    try:
        from starlette.testclient import TestClient
        from arm_bench.webxr.server import FASTAPI_AVAILABLE, WebXRTeleoperationServer
    except ImportError as exc:
        print(f"cannot run the loopback test: {exc} (pip install fastapi httpx uvicorn)")
        return False
    if not FASTAPI_AVAILABLE:
        print("fastapi/uvicorn not installed")
        return False
    got: List[dict] = []
    srv = WebXRTeleoperationServer(callback=got.append, use_https=False)
    t0 = time.time()
    with TestClient(srv.app) as client:
        with client.websocket_connect("/ws") as ws:
            for k in range(n_msgs):
                ws.send_json(dict(position=dict(x=0.01 * k, y=0.0, z=-0.01 * k),
                                  orientation=dict(x=0, y=0, z=0, w=1),
                                  control=dict(gripperOpen=bool(k % 2), moveEnabled=True)))
                ws.receive_json()                                    # the server echoes to all clients
        latest = client.get("/latest_pose").json()
        info = client.get("/server-info").json()
    dt = time.time() - t0
    ok = len(got) == n_msgs and abs(latest["position"]["x"] - 0.01 * (n_msgs - 1)) < 1e-9
    print(f"server processed {len(got)}/{n_msgs} poses in {1e3 * dt:.0f} ms; /latest_pose x = {latest['position']['x']:.3f}; "
          f"/server-info status = {info['server_status']}")
    if got:
        print(f"callback delta_ee of the last pose: x={got[-1]['x']:.4f} z={got[-1]['z']:.4f} "
              f"gripper_open={got[-1]['gripper_open']} move_enabled={got[-1]['move_enabled']}")
    print("NOT tested here: the phone's WebXR session, the self-signed certificate, WiFi latency/jitter.")
    return bool(ok)


# ------------------------------------------------------------------------------------------------- ROS
def check_ros(seconds: float) -> bool:
    from arm_bench.ros_state import ROS_AVAILABLE, RosJointStateSource
    if not ROS_AVAILABLE:
        print("rclpy is not available: source your ROS 2 workspace first")
        return False
    src = RosJointStateSource(discovery_period=0.5)
    counts: Dict[str, List[float]] = {}
    end = time.time() + seconds
    while time.time() < end:
        for arm, st in src.latest(max_age=1.0).items():
            if not counts.get(arm) or counts[arm][-1] != st.stamp:
                counts.setdefault(arm, []).append(st.stamp)
        time.sleep(0.005)
    cmds = src.commands(max_age=2.0)
    src.shutdown()
    if not counts:
        print("no */joint_states topic received")
        return False
    for arm, ts in sorted(counts.items()):
        rate = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 2 else 0.0
        print(f"{arm}: {rate:.1f} Hz joint_states" + ("" if rate >= 40 else "   <- below the 50 Hz control rate"))
    print("commanded-action topics seen: " + (", ".join(sorted(cmds)) or "none (start the controller first)"))
    return all(len(ts) > 2 for ts in counts.values())


# ------------------------------------------------------------------------------------------------- main
def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("cameras")
    c.add_argument("--ids", type=int, nargs="*")
    c.add_argument("--seconds", type=float, default=5.0)
    c.add_argument("--fps", type=float, default=15.0)
    j = sub.add_parser("joystick")
    j.add_argument("--seconds", type=float, default=10.0)
    j.add_argument("--device", type=int, default=0)
    sub.add_parser("webxr")
    r = sub.add_parser("ros")
    r.add_argument("--seconds", type=float, default=5.0)
    a = ap.parse_args(argv)
    ok = {"cameras": lambda: check_cameras(a.ids, a.seconds, a.fps),
          "joystick": lambda: check_joystick(a.seconds, a.device),
          "webxr": check_webxr_loopback,
          "ros": lambda: check_ros(a.seconds)}[a.cmd]()
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
