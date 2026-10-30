"""Start / stop the bilateral controller (ros2 launch controller controller_misa.launch.py)."""

import os
import shlex
import signal
import subprocess
import time
from typing import Optional

CONTROL_MODES = {
    "Proposed (MISA + MPC)": "misa",
    "Position-force": "pf",
    "Position-position": "pp",
    "Unilateral (follow only)": "unilateral",
}

LOG_DIR = os.path.expanduser("~/.merlin/logs")


def default_workspace() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "ros_ws"))


def build_mpc_launch_command(workspace_path: str, mode: Optional[str] = None,
                             params_file: Optional[str] = None, launch_drivers: bool = False,
                             use_rviz: bool = False, loop_rate_hz: float = 0.0,
                             log_dir: Optional[str] = None) -> list:
    """bash command that sources ROS + the workspace and runs the controller launch file."""
    workspace_path = os.path.abspath(workspace_path or "")
    setup_script = os.path.join(workspace_path, "install", "setup.bash")
    args = []
    if mode:
        args.append(f"control_mode:={mode}")
    if params_file:
        args.append(f"params_file:={params_file}")
    if launch_drivers:
        args.append("launch_drivers:=true")
    if use_rviz:
        args.append("use_rviz:=true")
    if loop_rate_hz and loop_rate_hz > 0:
        args.append(f"loop_rate_hz:={float(loop_rate_hz)}")
    if log_dir:
        args.append(f"log_dir:={log_dir}")
    shell_script = (
        "source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null || true; "
        f"source {shlex.quote(setup_script)}; "
        "ros2 launch controller controller_misa.launch.py"
        + "".join(" " + shlex.quote(a) for a in args)
    )
    return ["bash", "-lc", shell_script]


def launch_mpc_controller(workspace_path: str, env: Optional[dict] = None, **kwargs):
    """
    Start the controller in its own process group. Output goes to
    ~/.merlin/logs/controller_<time>.log (a PIPE nobody reads would fill up and
    block ros2 launch). Returns the Popen; its .log_path attribute is the log file.
    """
    setup = os.path.join(os.path.abspath(workspace_path or ""), "install", "setup.bash")
    if not os.path.isfile(setup):
        raise FileNotFoundError(f"ROS workspace not built: {setup} not found (run colcon build)")
    command = build_mpc_launch_command(workspace_path, **kwargs)
    process_env = os.environ.copy()
    if env:
        process_env.update(env)
    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, f"controller_{time.strftime('%Y%m%d_%H%M%S')}.log")
    log = open(log_path, "w")
    proc = subprocess.Popen(command, cwd=os.path.abspath(workspace_path), env=process_env,
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    proc.log_path = log_path
    return proc


def stop_process(proc, timeout: float = 5.0):
    """SIGINT the whole launch process group (lets ros2 launch shut nodes down), then SIGKILL."""
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
