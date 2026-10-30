"""Utility to discover ROS2 packages and launch files and build launch commands."""
import os
import shlex
import subprocess
from typing import List, Tuple, Optional


# def get_ros2_packages() -> List[str]:
#     """Return a list of installed ROS2 package names using `ros2 pkg list`.
#     Falls back to an empty list if `ros2` isn't available.
#     """
#     try:
#         res = subprocess.run(["ros2", "pkg", "list"], capture_output=True, text=True, check=True)
#         pkgs = [p.strip() for p in res.stdout.splitlines() if p.strip()]
#         return pkgs
#     except Exception:
#         return []

def get_ros2_packages():
    try:
        res = subprocess.run(
            ["ros2", "pkg", "list"],
            capture_output=True,
            text=True,
            check=True,
            env=os.environ.copy()
        )
        return sorted(
            p.strip()
            for p in res.stdout.splitlines()
            if p.strip()
        )
    except Exception as e:
        print(e)
        return []


def get_package_share_dir(package: str) -> Optional[str]:
    """Try to find the package share directory.
    Prefer `ament_index_python` if available, otherwise try `ros2 pkg prefix`.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(package)
    except Exception:
        try:
            # `ros2 pkg prefix <pkg>` prints the prefix; share is usually under <prefix>/share/<pkg>
            res = subprocess.run(["ros2", "pkg", "prefix", package], capture_output=True, text=True, check=True)
            prefix = res.stdout.strip()
            candidate = os.path.join(prefix, "share", package)
            if os.path.isdir(candidate):
                return candidate
        except Exception:
            return None


def list_launch_files(package: str) -> List[Tuple[str, str]]:
    """Return list of (filename, fullpath) for launch files in the package's `launch/` folder."""
    share = get_package_share_dir(package)
    if not share:
        return []
    launch_dir = os.path.join(share, "launch")
    if not os.path.isdir(launch_dir):
        return []
    result = []
    for f in os.listdir(launch_dir):
        if f.endswith(".launch.py") or f.endswith(".py"):
            result.append((f, os.path.join(launch_dir, f)))
    return result


def build_ros2_launch_command(
    package: str,
    launch_file: str,
    extra_args: str = "",
    workspace_path: Optional[str] = None,
) -> List[str]:
    """Build a launch command from inputs.

    When a workspace path is provided, the command is wrapped in a shell that
    sources the workspace's install/setup.bash first so locally-built packages
    are available to ROS 2.
    `extra_args` should be a single string like `robot_model:=rx200 use_sim:=true`.
    """
    cmd = ["ros2", "launch", package, launch_file]
    if extra_args:
        # naive split on whitespace to allow multiple `key:=value` entries
        cmd.extend(extra_args.split())

    if not workspace_path:
        return cmd

    workspace_path = os.path.abspath(workspace_path or "")
    setup_script = os.path.join(workspace_path, "install", "setup.bash")
    if not os.path.isfile(setup_script):
        raise FileNotFoundError(f"Workspace setup script not found: {setup_script}")

    shell_script = (
        "source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null || true; "
        f"source {shlex.quote(setup_script)}; "
        f"{shlex.join(cmd)}"
    )
    return ["bash", "-lc", shell_script]


def list_executables(package: str) -> List[str]:
    """Executables of a package (``ros2 pkg executables``); empty if ROS is not sourced."""
    if not package:
        return []
    try:
        out = subprocess.run(["ros2", "pkg", "executables", package], capture_output=True,
                             text=True, timeout=10).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []
    return [line.split()[1] for line in out.splitlines() if len(line.split()) == 2]


def build_ros2_run_command(
    package: str,
    executable: str,
    extra_args: str = "",
    workspace_path: Optional[str] = None,
) -> List[str]:
    """``ros2 run <package> <executable> [--ros-args ...]``, sourcing the workspace if given."""
    cmd = ["ros2", "run", package, executable]
    if extra_args:
        cmd.extend(extra_args.split())
    if not workspace_path:
        return cmd
    setup_script = os.path.join(os.path.abspath(workspace_path), "install", "setup.bash")
    if not os.path.isfile(setup_script):
        raise FileNotFoundError(f"Workspace setup script not found: {setup_script}")
    return ["bash", "-lc",
            "source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null || true; "
            f"source {shlex.quote(setup_script)}; {shlex.join(cmd)}"]
