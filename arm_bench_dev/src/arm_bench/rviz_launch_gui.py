"""MERLIN - Run ROS 2 Files: open RViz, run a package's launch file, or run a single node.
Presets save/load and validation/confirmation of extra args before launching."""
import os
import json
import re
import shlex
import sys
import threading
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext, simpledialog
from typing import List

MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
if MODULE_DIR not in sys.path:
    sys.path.insert(0, MODULE_DIR)

try:
    from .launch_manager import (get_ros2_packages, list_launch_files, build_ros2_launch_command,
                                 list_executables, build_ros2_run_command)
    from .visualization import prepare_rviz_environment
except ImportError:
    from launch_manager import (get_ros2_packages, list_launch_files, build_ros2_launch_command,
                                list_executables, build_ros2_run_command)
    from visualization import prepare_rviz_environment


def _presets_path() -> str:
    base = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    cfg_dir = os.path.join(base, 'configs')
    os.makedirs(cfg_dir, exist_ok=True)
    return os.path.join(cfg_dir, 'rviz_presets.json')


def load_presets() -> dict:
    path = _presets_path()
    try:
        with open(path, 'r') as f:
            return json.load(f)
    except Exception:
        return {}


def save_presets(presets: dict):
    path = _presets_path()
    with open(path, 'w') as f:
        json.dump(presets, f, indent=2)


def validate_extra_args(arg_string: str) -> tuple[bool, List[str]]:
    """Validate extra args are in ROS2 `key:=value` format (space-separated).
    Returns (valid, invalid_tokens).
    """
    if not arg_string.strip():
        return True, []
    tokens = arg_string.split()
    invalid = []
    for t in tokens:
        # Accept key:=value (key non-empty) or single flags; require := if contains ':'
        if ':=' in t:
            parts = t.split(':=', 1)
            if not parts[0].strip() or parts[1] is None:
                invalid.append(t)
        else:
            # allow plain tokens but warn
            # treat tokens without := as potentially invalid but not fatal
            pass
    return (len(invalid) == 0), invalid


class RvizLaunchGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MERLIN - Run ROS 2 Files")
        self.geometry("900x620")

        self.packages = []
        self.launch_files = []
        self.processes = []
        self.workspace_envs = {}

        self.colors = {
            'bg': '#1e1e1e',
            'fg': '#ffffff',
            'panel_bg': '#2d2d2d',
            'accent': '#007acc',
            'success': '#4ec9b0',
            'warning': '#dcdcaa',
            'error': '#f48771'
        }

        top_nav = tk.Frame(self, bg="#1e1e1e")
        top_nav.pack(fill=tk.X, padx=8, pady=(8, 0))
        tk.Button(top_nav, text="← Back to MERLIN", command=self._go_back, bg="#2d2d2d", fg="white", relief=tk.FLAT).pack(side=tk.LEFT)

        top = ttk.Frame(self)
        top.pack(fill=tk.X, padx=8, pady=6)
        ttk.Button(top, text="Refresh Packages", command=self.refresh_packages).pack(side=tk.LEFT)

        self.pkg_var = tk.StringVar()
        self.pkg_combo = ttk.Combobox(top, textvariable=self.pkg_var, state="readonly", width=40)
        self.pkg_combo.pack(side=tk.LEFT, padx=6)
        self.pkg_combo.bind("<<ComboboxSelected>>", self.pkg_selected)

        ttk.Button(top, text="Refresh Launches", command=self.refresh_launches).pack(side=tk.LEFT, padx=6)

        

        # Presets
        self.presets = load_presets()
        self.preset_var = tk.StringVar()
        self.preset_combo = ttk.Combobox(top, textvariable=self.preset_var, state="readonly", width=30)
        self.preset_combo.pack(side=tk.LEFT, padx=6)
        self.preset_combo.bind("<<ComboboxSelected>>", lambda e: self.load_selected_preset())
        ttk.Button(top, text="Save Preset", command=self.save_current_preset).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Delete Preset", command=self.delete_selected_preset).pack(side=tk.LEFT, padx=2)

        mid = ttk.Frame(self)
        mid.pack(fill=tk.BOTH, expand=True, padx=8, pady=6)

        lists = ttk.Frame(mid)
        lists.pack(fill=tk.BOTH, expand=True, side=tk.LEFT)
        ttk.Label(lists, text="Launch files").pack(anchor=tk.W)
        self.launch_list = tk.Listbox(lists, height=8, exportselection=False)
        self.launch_list.pack(fill=tk.BOTH, expand=True)
        ttk.Label(lists, text="Nodes (executables)").pack(anchor=tk.W, pady=(6, 0))
        self.node_list = tk.Listbox(lists, height=6, exportselection=False)
        self.node_list.pack(fill=tk.BOTH, expand=True)
        self.executables = []

        right = ttk.Frame(mid)
        right.pack(fill=tk.Y, side=tk.LEFT, padx=8)

        ttk.Label(right, text="Workspace path (optional)").pack(anchor=tk.W)
        self.workspace_entry = ttk.Entry(right, width=30)
        self.workspace_entry.pack(pady=4)
        default_workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'ros_ws'))
        self.workspace_entry.insert(0, default_workspace)

        ttk.Button(right, text="Source Workspace", command=self.source_workspace).pack(fill=tk.X, pady=(0, 4))

        ttk.Label(right, text="Extra args (space separated) ").pack(anchor=tk.W)
        self.args_entry = ttk.Entry(right, width=30)
        self.args_entry.pack(pady=4)

        ttk.Button(right, text="Launch selected launch file", command=self.launch_selected).pack(fill=tk.X, pady=4)
        ttk.Button(right, text="Run selected node", command=self.run_selected_node).pack(fill=tk.X, pady=(0, 4))
        ttk.Button(right, text="Open RViz", command=self.open_rviz).pack(fill=tk.X, pady=(0, 4))
        ttk.Button(right, text="Kill Selected", command=self.kill_selected_process).pack(fill=tk.X)

        ttk.Label(right, text="Running processes").pack(anchor=tk.W, pady=(8, 2))
        self.process_listbox = tk.Listbox(right, height=6, width=35)
        self.process_listbox.pack(fill=tk.X)

        bottom = ttk.Frame(self)
        bottom.pack(fill=tk.BOTH, expand=True, padx=8, pady=6)

        ttk.Label(bottom, text="Output").pack(anchor=tk.W)
        self.output = scrolledtext.ScrolledText(bottom, height=12)
        self.output.pack(fill=tk.BOTH, expand=True)

        self.refresh_packages()
        self._refresh_preset_combo()
    
    def refresh_packages(self):
        self.packages = get_ros2_packages()
        self.pkg_combo['values'] = self.packages
        if self.packages:
            self.pkg_combo.current(0)
            self.pkg_selected()
        self._refresh_preset_combo()

    def _refresh_preset_combo(self):
        self.presets = load_presets()
        keys = list(self.presets.keys())
        self.preset_combo['values'] = keys
        if keys:
            self.preset_combo.current(0)

    def pkg_selected(self, _evt=None):
        pkg = self.pkg_var.get()
        self.launch_files = list_launch_files(pkg)
        self.launch_list.delete(0, tk.END)
        for name, path in self.launch_files:
            self.launch_list.insert(tk.END, name)
        self.executables = list_executables(pkg)
        self.node_list.delete(0, tk.END)
        for exe in self.executables:
            self.node_list.insert(tk.END, exe)

    def refresh_launches(self):
        self.pkg_selected()

    def append_output(self, text: str):
        self.output.insert(tk.END, text + "\n")
        self.output.see(tk.END)

    def _default_workspace_path(self) -> str:
        return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'ros_ws'))

    def _get_workspace_environment(self, workspace_path: str) -> dict:
        workspace_path = workspace_path.strip() if workspace_path else ""
        if not workspace_path:
            workspace_path = self._default_workspace_path()

        workspace_path = os.path.abspath(workspace_path)

        if workspace_path in self.workspace_envs:
            return self.workspace_envs[workspace_path]

        if not os.path.isdir(workspace_path):
            raise FileNotFoundError(f"Workspace directory not found: {workspace_path}")

        setup_script = os.path.join(workspace_path, "install", "setup.bash")
        print(setup_script)

        if not os.path.isfile(setup_script):
            raise FileNotFoundError(f"Workspace setup script not found: {setup_script}")

        cmd = [
            "bash",
            "-lc",
            f"source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null || true; "
            f"source {shlex.quote(setup_script)}; "
            f"env"
        ]

        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            env=os.environ.copy(),
            check=True,
        )

        env = os.environ.copy()
        for line in proc.stdout.splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                env[key] = value

        self.workspace_envs[workspace_path] = env
        return env

    def source_workspace(self):
        workspace_path = self.workspace_entry.get().strip()
        if not workspace_path:
            messagebox.showwarning("Workspace", "Please enter a workspace path first.")
            return
        try:
            env = self._get_workspace_environment(workspace_path)
            os.environ.clear()
            os.environ.update(env)
            print("AMENT_PREFIX_PATH =", os.environ.get("AMENT_PREFIX_PATH"))
            print("COLCON_PREFIX_PATH =", os.environ.get("COLCON_PREFIX_PATH"))
            self.append_output(f"Workspace environment loaded from {workspace_path}")
        except Exception as exc:
            messagebox.showwarning("Workspace", f"Unable to source workspace: {exc}")

    def _refresh_processes(self):
        running = []
        for proc_info in self.processes:
            proc = proc_info['proc']
            if proc.poll() is None:
                running.append(proc_info)
        self.processes = running
        self.process_listbox.delete(0, tk.END)
        for proc_info in self.processes:
            self.process_listbox.insert(tk.END, f"{proc_info['name']} (PID {proc_info['proc'].pid})")

    def _monitor_process(self, proc_info: dict):
        proc = proc_info['proc']
        name = proc_info['name']
        try:
            for line in proc.stdout:
                self.append_output(f"[{name}] {line.rstrip()}")
            proc.wait()
            self.append_output(f"[{name}] exited with {proc.returncode}")
        except Exception as exc:
            self.append_output(f"[{name}] monitoring failed: {exc}")
        finally:
            self._refresh_processes()

    def launch_selected(self):
        sel = self.launch_list.curselection()
        if not sel:
            messagebox.showwarning("No launch file", "Please select a launch file to run.")
            return
        idx = sel[0]
        launch_name, _path = self.launch_files[idx]
        pkg = self.pkg_var.get()
        extra = self.args_entry.get().strip()
        workspace_path = self.workspace_entry.get().strip() or self._default_workspace_path()
        try:
            cmd = build_ros2_launch_command(pkg, launch_name, extra, workspace_path)
            launch_env = self._get_workspace_environment(workspace_path)
        except FileNotFoundError as exc:
            messagebox.showwarning("Workspace setup", str(exc))
            return

        self.append_output(f"Launching: {' '.join(cmd)}")

        # Validate extra args
        valid, invalid = validate_extra_args(extra)
        if not valid:
            messagebox.showerror("Invalid args", f"Invalid argument tokens: {invalid}")
            return

        # Confirmation
        if not messagebox.askyesno("Confirm Launch", f"Run command:\n\n{' '.join(cmd)}\n\nProceed?"):
            self.append_output("Launch cancelled by user")
            return

        def run():
            try:
                proc = subprocess.Popen(cmd, env=launch_env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                proc_info = {'name': f"{pkg}:{launch_name}", 'proc': proc}
                self.processes.append(proc_info)
                self._refresh_processes()
                self._monitor_process(proc_info)
            except Exception as e:
                self.append_output(f"Launch failed: {e}")

        t = threading.Thread(target=run, daemon=True)
        t.start()

    def _start(self, name: str, cmd: list, env: dict):
        self.append_output(f"Running: {' '.join(cmd)}")

        def run():
            try:
                proc = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                proc_info = {'name': name, 'proc': proc}
                self.processes.append(proc_info)
                self._refresh_processes()
                self._monitor_process(proc_info)
            except Exception as e:
                self.append_output(f"Start failed: {e}")

        threading.Thread(target=run, daemon=True).start()

    def run_selected_node(self):
        sel = self.node_list.curselection()
        if not sel:
            messagebox.showwarning("No node", "Select a node (executable) to run.")
            return
        pkg, exe = self.pkg_var.get(), self.executables[sel[0]]
        extra = self.args_entry.get().strip()
        valid, invalid = validate_extra_args(extra)
        if not valid:
            messagebox.showerror("Invalid args", f"Invalid argument tokens: {invalid}")
            return
        workspace_path = self.workspace_entry.get().strip() or self._default_workspace_path()
        try:
            cmd = build_ros2_run_command(pkg, exe, extra, workspace_path)
            env = self._get_workspace_environment(workspace_path)
        except FileNotFoundError as exc:
            messagebox.showwarning("Workspace setup", str(exc))
            return
        self._start(f"{pkg}/{exe}", cmd, env)

    def open_rviz(self):
        workspace_path = self.workspace_entry.get().strip() or self._default_workspace_path()
        try:
            env = self._get_workspace_environment(workspace_path)
        except FileNotFoundError:
            env = prepare_rviz_environment()
        self._start("rviz2", ["rviz2"], env)

    def kill_selected_process(self):
        selection = self.process_listbox.curselection()
        if not selection:
            messagebox.showinfo("No process", "Select a process to kill.")
            return
        idx = selection[0]
        if idx >= len(self.processes):
            return
        proc_info = self.processes[idx]
        proc = proc_info['proc']
        if proc.poll() is None:
            self.append_output(f"Killing {proc_info['name']}...")
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
            self._refresh_processes()
        else:
            messagebox.showinfo("Process ended", "The selected process has already exited.")

    def _go_back(self):
        self.destroy()

    def save_current_preset(self):
        pkg = self.pkg_var.get()
        sel = self.launch_list.curselection()
        if not pkg or not sel:
            messagebox.showwarning("Cannot save", "Select a package and launch file first.")
            return
        idx = sel[0]
        launch_name, _path = self.launch_files[idx]
        args = self.args_entry.get().strip()

        name = simpledialog.askstring("Preset name", "Enter a name for this preset:")
        if not name:
            return

        workspace_path = self.workspace_entry.get().strip() or self._default_workspace_path()
        presets = load_presets()
        presets[name] = {
            "package": pkg,
            "launch": launch_name,
            "args": args,
            "workspace_path": workspace_path,
        }
        save_presets(presets)
        self._refresh_preset_combo()
        messagebox.showinfo("Saved", f"Preset '{name}' saved.")

    def load_selected_preset(self):
        name = self.preset_var.get()
        if not name:
            return
        presets = load_presets()
        preset = presets.get(name)
        if not preset:
            return
        # set UI
        try:
            pkg = preset.get('package','')
            if pkg in self.packages:
                self.pkg_combo.set(pkg)
                self.pkg_selected()
            # select launch
            launch = preset.get('launch','')
            for i, (n, p) in enumerate(self.launch_files):
                if n == launch:
                    self.launch_list.selection_clear(0, tk.END)
                    self.launch_list.selection_set(i)
                    self.launch_list.see(i)
                    break
            self.args_entry.delete(0, tk.END)
            self.args_entry.insert(0, preset.get('args',''))
            self.workspace_entry.delete(0, tk.END)
            self.workspace_entry.insert(0, preset.get('workspace_path',''))
            self.append_output(f"Loaded preset '{name}'")
        except Exception as e:
            messagebox.showerror("Load failed", str(e))

    def delete_selected_preset(self):
        name = self.preset_var.get()
        if not name:
            messagebox.showinfo("No preset", "No preset selected.")
            return
        if not messagebox.askyesno("Delete preset", f"Delete preset '{name}'?"):
            return
        presets = load_presets()
        if name in presets:
            presets.pop(name)
            save_presets(presets)
            self._refresh_preset_combo()
            messagebox.showinfo("Deleted", f"Preset '{name}' deleted.")


def main():
    app = RvizLaunchGUI()
    app.mainloop()


if __name__ == '__main__':
    main()
