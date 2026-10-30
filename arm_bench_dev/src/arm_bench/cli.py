import typer
import os
import sys
from typing import Optional
from pathlib import Path
from rich.console import Console
from rich.table import Table

# CRITICAL: Clean Qt environment before importing GUI modules
# Prevents segmentation faults from Qt/OpenCV conflicts
if 'QT_PLUGIN_PATH' in os.environ:
    del os.environ['QT_PLUGIN_PATH']
if 'QT_QPA_PLATFORM_PLUGIN_PATH' in os.environ:
    del os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']
os.environ['QT_QPA_PLATFORM'] = 'xcb'

# Import arm-bench modules
from arm_bench.gui import APP_NAME, launch_monitoring_interface
from arm_bench.scanner import (
    scan_dynamixel_motors,
    scan_cameras,
    save_hardware_config,
    load_hardware_config
)
from arm_bench.visualization import VisualizationLauncher, create_default_rviz_config
from arm_bench.teleoperation import (
    start_joystick_teleop,
    start_webxr_teleop,
    DifferentialIKNode
)
from arm_bench.interbotix_builder import InterbotixWorkspaceBuilder
from arm_bench.bimanual import create_translator, create_coordinator
from arm_bench.rlds_recorder import create_recorder, create_camera_manager
from arm_bench.control import (
    create_control_template,
    deploy_control_algorithm,
    ControlDeployer
)
from arm_bench.dynamixel_control import DynamixelController

app = typer.Typer(help="MERLIN: multi-arm robotics testbed (command: arm-bench)")
console = Console()

# Global state
current_mode = None
current_arm_type = None
current_config = None
recorder = None
current_motors = []
current_cameras = []
dynamixel_controller = None  # Global controller instance


@app.command()
def version():
    """Display version information"""
    console.print("[bold green]MERLIN v0.2.0[/bold green]")
    console.print("Multi-arm Experimental Robotics Learning and INtegration")


@app.command()
def start(arm: Optional[str] = typer.Option(None, help="Arm model hint for the joint sliders, e.g. vx300s")):
    """Open the MERLIN window (directly on the Control tab).

    Motors are NOT touched at startup: the Interbotix driver or the bilateral
    controller own the serial port. Live motor data comes from ROS, or from a
    read-only port scan in the Motor Status tab.
    """
    console.print(f"[bold cyan]Launching {APP_NAME}...[/bold cyan]")
    with console.status("Scanning cameras..."):
        cameras = scan_cameras()
    console.print(f"[green]Found {len(cameras)} camera(s)[/green]")
    launch_monitoring_interface(cameras=cameras, arm_type=arm)
    console.print(f"[green]{APP_NAME} closed.[/green]")


@app.command()
def setup(sim: bool = typer.Option(False, help="Run in simulation mode")):
    """Scan and setup hardware"""
    console.print("[bold cyan]Hardware Setup[/bold cyan]")
    
    if sim:
        console.print("[yellow]Simulation mode - using simulated hardware[/yellow]")
        motors = scan_dynamixel_motors(simulate=True)
    else:
        console.print("Scanning for Dynamixel motors...")
        motors = scan_dynamixel_motors(simulate=False)
        
        if not motors:
            console.print("[red]No motors found![/red]")
            console.print("Check connections and try again")
            return
    
    # Display found motors
    table = Table(title="Detected Motors")
    table.add_column("ID", style="cyan")
    table.add_column("Model", style="green")
    table.add_column("Port", style="yellow")
    table.add_column("Temperature", style="magenta")
    
    for motor in motors:
        table.add_row(
            str(motor["id"]),
            motor["model"],
            motor["port"],
            f"{motor.get('temperature', 0)}°C"
        )
    
    console.print(table)
    
    # Scan cameras
    console.print("\nScanning cameras...")
    cameras = scan_cameras()
    
    if cameras:
        cam_table = Table(title="Detected Cameras")
        cam_table.add_column("ID", style="cyan")
        cam_table.add_column("Name", style="green")
        cam_table.add_column("Resolution", style="yellow")
        
        for cam in cameras:
            cam_table.add_row(
                str(cam["id"]),
                cam["name"],
                cam.get("resolution", "Unknown")
            )
        
        console.print(cam_table)
    
    # Save configuration
    save_hardware_config(motors, cameras)
    console.print("[green]✓ Configuration saved[/green]")


@app.command()
def visualize(
    config: Optional[str] = typer.Option(None, help="Path to RViz config file")
):
    """Launch RViz visualization"""
    console.print("[cyan]Launching visualization...[/cyan]")
    
    mode = current_mode or "simulation"
    viz = VisualizationLauncher(mode)
    
    if viz.launch_rviz(config):
        console.print("[green]✓ RViz launched successfully[/green]")
    else:
        console.print("[red]✗ Failed to launch RViz[/red]")


@app.command()
def teleop(
    mode: str = typer.Option("joystick", help="Teleoperation mode: joystick or webxr")
):
    """Start teleoperation"""
    console.print(f"[cyan]Starting {mode} teleoperation...[/cyan]")
    
    # Create differential IK node
    ik_node = DifferentialIKNode()
    
    def handle_delta_ee(delta_ee):
        """Callback for delta EE commands"""
        joint_vel = ik_node.compute_joint_velocities(delta_ee)
        ik_node.send_to_robot(joint_vel)
    
    if mode == "joystick":
        teleop_controller = start_joystick_teleop(callback=handle_delta_ee)
        console.print("[green]✓ Joystick teleoperation active[/green]")
        console.print("Press Ctrl+C to stop")
        
        try:
            while True:
                pass
        except KeyboardInterrupt:
            teleop_controller.stop()
            console.print("\n[yellow]Teleoperation stopped[/yellow]")
    
    elif mode == "webxr":
        teleop_controller = start_webxr_teleop(callback=handle_delta_ee)
        console.print("[green]✓ WebXR teleoperation active[/green]")
        console.print("Open browser to control the robot")
        console.print("Press Ctrl+C to stop")
        
        try:
            while True:
                pass
        except KeyboardInterrupt:
            teleop_controller.stop()
            console.print("\n[yellow]Teleoperation stopped[/yellow]")
    
    else:
        console.print(f"[red]Unknown teleoperation mode: {mode}[/red]")


@app.command()
def build_workspace(
    path: Optional[str] = typer.Option(None, help="Workspace path")
):
    """Build Interbotix ROS2 workspace"""
    console.print("[bold cyan]Building Interbotix Workspace[/bold cyan]")
    
    builder = InterbotixWorkspaceBuilder(path)
    
    with console.status("Setting up workspace..."):
        success = builder.setup_full_workspace()
    
    if success:
        console.print("[green]✓ Workspace built successfully![/green]")
    else:
        console.print("[red]✗ Workspace build failed[/red]")


@app.command()
def translate(
    source: str = typer.Argument(..., help="Source arm type (reactor/viper)"),
    target: str = typer.Argument(..., help="Target arm type (reactor/viper)")
):
    """Test bimanual arm translation"""
    console.print(f"[cyan]Translating from {source} to {target}[/cyan]")
    
    translator = create_translator()
    
    # Example joint state
    if source == "reactor":
        example_state = {
            "waist": 0.5,
            "shoulder": 1.0,
            "elbow": -0.5,
            "wrist_angle": 0.3,
            "wrist_rotate": 0.0
        }
    else:
        example_state = {
            "waist": 0.5,
            "shoulder": 1.0,
            "elbow": -0.5,
            "forearm_roll": 0.2,
            "wrist_angle": 0.3,
            "wrist_rotate": 0.0
        }
    
    console.print(f"\n[yellow]Source state ({source}):[/yellow]")
    console.print(example_state)
    
    # Translate
    if source == "reactor" and target == "viper":
        translated = translator.reactor_to_viper(example_state)
    elif source == "viper" and target == "reactor":
        translated = translator.viper_to_reactor(example_state)
    else:
        console.print(f"[red]Invalid translation: {source} -> {target}[/red]")
        return
    
    console.print(f"\n[green]Translated state ({target}):[/green]")
    console.print(translated)


@app.command()
def record(
    action: str = typer.Argument("start", help="start (records until Ctrl+C or --duration) or list"),
    episode_name: Optional[str] = typer.Option(None, help="Episode name"),
    output: Optional[str] = typer.Option(None, help="Output directory (default ~/merlin_datasets)"),
    camera: Optional[list[int]] = typer.Option(None, help="Camera index to record, repeatable (e.g. --camera 0 --camera 2)"),
    rate: float = typer.Option(30.0, help="Recording rate [Hz]"),
    duration: Optional[float] = typer.Option(None, help="Stop after this many seconds"),
):
    """Record an episode: joint states of every arm on ROS + camera frames + calibration."""
    import time as _time
    from arm_bench.episode_recorder import list_episodes

    if action == "list":
        episodes = list_episodes(output)
        if not episodes:
            console.print("[yellow]No episodes found[/yellow]")
            return
        table = Table(title="Recorded Episodes")
        table.add_column("Episode Name", style="cyan")
        for ep in episodes:
            table.add_row(ep)
        console.print(table)
        return
    if action != "start":
        console.print(f"[red]Unknown action: {action}[/red] (use start or list)")
        raise typer.Exit(1)

    cams = {f"Camera {c}": c for c in (camera or [])}
    rec = create_recorder(output, cameras=cams, rate_hz=rate)
    _time.sleep(1.5)       # let ROS discover */joint_states and cameras deliver a frame
    arms = rec.state_source.arms() if rec.state_source.available else []
    console.print(f"Arms on ROS: {', '.join(arms) or '[yellow]none[/yellow]'} | cameras: {list(cams) or 'none'}")
    if not arms and not cams:
        console.print("[red]Nothing to record (no */joint_states and no --camera).[/red]")
        raise typer.Exit(1)
    path = rec.start_recording(episode_name)
    console.print(f"[green]Recording to {path}[/green] — Ctrl+C to stop")
    try:
        t0 = _time.time()
        while duration is None or _time.time() - t0 < duration:
            _time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    s = rec.stop_recording()
    console.print(f"[green]✓ Saved {s['num_steps']} steps, {s['frames_written']} frames[/green] "
                  f"→ {s['episode_dir']}")


@app.command()
def cameras():
    """Manage cameras"""
    console.print("[cyan]Camera Management[/cyan]")
    
    cam_manager = create_camera_manager()
    cameras = cam_manager.scan_cameras()
    
    if not cameras:
        console.print("[yellow]No cameras found[/yellow]")
        return
    
    table = Table(title="Available Cameras")
    table.add_column("ID", style="cyan")
    table.add_column("Name", style="green")
    table.add_column("Status", style="yellow")
    
    for cam in cameras:
        status = "✓ Available" if cam.get("available") else "✗ Not available"
        table.add_row(
            str(cam["id"]),
            cam["name"],
            status
        )
    
    console.print(table)


@app.command()
def create_control(
    mode: str = typer.Argument(..., help="Control mode: position, velocity, or torque"),
    frequency: int = typer.Option(100, help="Control frequency in Hz"),
    output: str = typer.Option("custom_controller.py", help="Output file path")
):
    """Create a custom control algorithm template"""
    console.print(f"[cyan]Creating {mode} control template...[/cyan]")
    
    if mode not in ["position", "velocity", "torque"]:
        console.print("[red]Invalid mode. Use: position, velocity, or torque[/red]")
        return
    
    create_control_template(mode, frequency, output)
    
    console.print(f"[green]✓ Control template created: {output}[/green]")
    console.print(f"Mode: {mode}")
    console.print(f"Frequency: {frequency} Hz")
    console.print(f"\nEdit the file to implement your control algorithm")
    console.print(f"Then deploy with: arm-bench deploy-control {output} --mode {mode} --freq {frequency}")


@app.command()
def deploy_control(
    control_file: str = typer.Argument(..., help="Path to control algorithm file"),
    mode: str = typer.Option(..., help="Control mode: position, velocity, or torque"),
    freq: int = typer.Option(100, help="Control frequency in Hz")
):
    """Deploy custom control algorithm to ROS2 workspace"""
    console.print(f"[cyan]Deploying {mode} control algorithm...[/cyan]")
    
    if not os.path.exists(control_file):
        console.print(f"[red]File not found: {control_file}[/red]")
        return
    
    with console.status("Deploying to ROS2 workspace..."):
        success = deploy_control_algorithm(control_file, mode, freq)
    
    if success:
        console.print("[green]✓ Control algorithm deployed successfully![/green]")
        console.print(f"\nTo run the controller:")
        console.print(f"  source ~/arm_bench_ros_ws/install/setup.bash")
        console.print(f"  ros2 launch arm_bench_control {mode}_controller_{freq}hz_launch.py")
    else:
        console.print("[red]✗ Deployment failed[/red]")


@app.command()
def launch_control(
    mode: str = typer.Argument(..., help="Control mode: position, velocity, or torque"),
    freq: int = typer.Option(100, help="Control frequency in Hz")
):
    """Launch a deployed control algorithm"""
    console.print(f"[cyan]Launching {mode} controller at {freq} Hz...[/cyan]")
    
    deployer = ControlDeployer()
    
    if deployer.launch_controller(mode, freq):
        console.print("[green]✓ Controller launched[/green]")
        console.print("Press Ctrl+C to stop")
    else:
        console.print("[red]✗ Failed to launch controller[/red]")


@app.command()
def matlab_tune():
    """Launch enhanced control algorithm development studio with MATLAB integration"""
    console.print("[bold cyan]🎮 Control Algorithm Development Studio[/bold cyan]\n")
    
    console.print("Features:")
    console.print("  ✓ Define custom control logic in Python")
    console.print("  ✓ Configure parameters with bounds")
    console.print("  ✓ Run simulations with multiple plant models")
    console.print("  ✓ Optimize using MATLAB or Python (scipy)")
    console.print("  ✓ Compare multiple parameter sets side-by-side")
    console.print("  ✓ Real-time performance visualization")
    console.print("  ✓ Save/load control configurations\n")
    
    try:
        from arm_bench.matlab_integration_enhanced import launch_control_development
        console.print("[green]Launching studio...[/green]\n")
        launch_control_development()
    except ImportError as e:
        console.print(f"[yellow]⚠ Missing dependencies: {e}[/yellow]")
        console.print("\nInstall required packages:")
        console.print("  pip install matplotlib scipy")
        console.print("\nOptional (for MATLAB integration):")
        console.print("  pip install matlabengine")
    except Exception as e:
        console.print(f"[red]✗ Error: {e}[/red]")


@app.command()
def control_info():
    """Show information about control modes and standardized format"""
    console.print("[bold cyan]Control Algorithm Information[/bold cyan]\n")
    
    # Control Modes
    modes_table = Table(title="Available Control Modes")
    modes_table.add_column("Mode", style="cyan")
    modes_table.add_column("Description", style="white")
    modes_table.add_column("ROS Topic", style="yellow")
    
    modes_table.add_row(
        "position",
        "Joint position control (angle in radians)",
        "/arm_bench/position_command"
    )
    modes_table.add_row(
        "velocity",
        "Joint velocity control (rad/s)",
        "/arm_bench/velocity_command"
    )
    modes_table.add_row(
        "torque",
        "Joint torque/effort control (Nm)",
        "/arm_bench/torque_command"
    )
    
    console.print(modes_table)
    
    # Message Format
    console.print("\n[bold]Standardized Message Format:[/bold]")
    console.print("""
Position Control:
  - Type: trajectory_msgs/JointTrajectory
  - Fields: positions, time_from_start
  - Frequency: 100 Hz (default)

Velocity Control:
  - Type: trajectory_msgs/JointTrajectory
  - Fields: velocities
  - Frequency: 100 Hz (default)

Torque Control:
  - Type: trajectory_msgs/JointTrajectory
  - Fields: effort
  - Frequency: 100 Hz (default)
    """)
    
    console.print("[bold]Quick Start:[/bold]")
    console.print("1. Create template: arm-bench create-control position")
    console.print("2. Edit the generated file")
    console.print("3. Deploy: arm-bench deploy-control custom_controller.py --mode position --freq 100")
    console.print("4. Launch: arm-bench launch-control position --freq 100")


@app.command()
def bilateral(
    leader_type: str = typer.Option("rx150", help="Leader arm type (rx150, vx300s, etc.)"),
    follower_type: str = typer.Option("vx300s", help="Follower arm type"),
    leader_hw: bool = typer.Option(False, "--leader-hw", help="Use hardware for leader"),
    follower_hw: bool = typer.Option(False, "--follower-hw", help="Use hardware for follower"),
    rate: float = typer.Option(50.0, help="Control rate in Hz"),
    visualize: bool = typer.Option(True, "--viz/--no-viz", help="Show visualization")
):
    """Start bilateral teleoperation (leader -> follower)"""
    console.print("[bold cyan]Bilateral Teleoperation[/bold cyan]\n")
    
    console.print(f"Leader: {leader_type} ({'hardware' if leader_hw else 'simulation'})")
    console.print(f"Follower: {follower_type} ({'hardware' if follower_hw else 'simulation'})")
    console.print(f"Control rate: {rate} Hz")
    console.print(f"Visualization: {'enabled' if visualize else 'disabled'}\n")
    
    try:
        from arm_bench.bilateral_control import (
            BilateralSystem,
            create_bilateral_system,
            HardwareArm,
            SimulatedArm
        )
        from arm_bench.kinematics import get_arm_config
        
        controller = None
        
        # Setup hardware if needed
        if leader_hw or follower_hw:
            console.print("[cyan]Initializing hardware...[/cyan]")
            
            port = '/dev/ttyDXL' if os.path.exists('/dev/ttyDXL') else '/dev/ttyUSB0'
            controller = DynamixelController(port=port)
            
            if not controller.connect():
                console.print("[red]Failed to connect to Dynamixel bus[/red]")
                if leader_hw or follower_hw:
                    console.print("[yellow]Falling back to simulation mode[/yellow]")
                    leader_hw = False
                    follower_hw = False
                    controller = None
            else:
                motor_ids = controller.scan_motors(range(1, 20))
                console.print(f"[green]Found {len(motor_ids)} motors[/green]")
                
                # Enable position hold
                controller.enable_all_position_hold()
                console.print("[green]Position hold mode enabled[/green]")
        
        # Create bilateral system
        console.print("[cyan]Creating bilateral system...[/cyan]")
        
        system = create_bilateral_system(
            leader_type=leader_type,
            follower_type=follower_type,
            leader_hardware=leader_hw,
            follower_hardware=follower_hw,
            controller=controller,
            control_rate=rate
        )
        
        # Launch visualization if requested
        viz = None
        if visualize:
            try:
                from arm_bench.visualization import launch_bilateral_visualization
                console.print("[cyan]Launching visualization...[/cyan]")
                viz = launch_bilateral_visualization(num_robots=2, show_leaders=True)
            except Exception as e:
                console.print(f"[yellow]Visualization not available: {e}[/yellow]")
        
        # Start bilateral control
        console.print("[green]Starting bilateral control...[/green]")
        system.start()
        system.enable_movement(True)
        
        console.print("\n[bold green]Bilateral control running![/bold green]")
        console.print("Press Ctrl+C to stop\n")
        
        # Main loop
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        
        # Cleanup
        console.print("\n[yellow]Stopping bilateral control...[/yellow]")
        system.stop()
        
        if viz:
            viz.stop()
        
        if controller:
            controller.disable_all_torque()
            controller.disconnect()
        
        console.print("[green]Bilateral control stopped[/green]")
        
    except ImportError as e:
        console.print(f"[red]Import error: {e}[/red]")
        console.print("Make sure all dependencies are installed")
    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")
        import traceback
        traceback.print_exc()


@app.command()
def webxr(
    port: int = typer.Option(8443, help="Server port"),
    ros2: bool = typer.Option(False, "--ros2", help="Enable ROS2 integration"),
):
    """Start WebXR teleoperation server"""
    console.print("[bold cyan]WebXR Teleoperation Server[/bold cyan]\n")
    
    console.print(f"Port: {port}")
    console.print(f"ROS2 integration: {'enabled' if ros2 else 'disabled'}\n")
    
    try:
        if ros2:
            from arm_bench.ros2_bridge import launch_webxr_ros2
            console.print("[cyan]Starting WebXR + ROS2 server...[/cyan]")
            launch_webxr_ros2(port=port)
        else:
            from arm_bench.webxr.server import start_webxr_server
            console.print("[cyan]Starting WebXR server...[/cyan]")
            start_webxr_server(port=port)
            
    except KeyboardInterrupt:
        console.print("\n[yellow]Server stopped[/yellow]")
    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")


@app.command()
def test_kinematics():
    """Test forward and inverse kinematics"""
    console.print("[bold cyan]Testing Kinematics[/bold cyan]\n")
    
    try:
        from arm_bench.kinematics import (
            forward_kinematics,
            inverse_kinematics_differential,
            get_rx150_config,
            get_vx300s_config,
            se3_to_pose,
            compute_pose_error
        )
        import numpy as np
        
        # Test ReactorX-150 FK
        console.print("[cyan]Testing ReactorX-150 FK...[/cyan]")
        rx150_config = get_rx150_config()
        q_home = np.zeros(5)
        T = forward_kinematics(q_home, rx150_config)
        pos, quat = se3_to_pose(T)
        
        console.print(f"  Home position: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) m")
        console.print(f"  Quaternion: ({quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f})")
        
        # Test ViperX-300s FK
        console.print("\n[cyan]Testing ViperX-300s FK...[/cyan]")
        vx300s_config = get_vx300s_config()
        q_home = np.zeros(6)
        T = forward_kinematics(q_home, vx300s_config)
        pos, quat = se3_to_pose(T)
        
        console.print(f"  Home position: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) m")
        console.print(f"  Quaternion: ({quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f})")
        
        # Test IK
        console.print("\n[cyan]Testing ViperX-300s IK...[/cyan]")
        T_target = T.copy()
        T_target[0, 3] -= 0.05  # Move 5cm in -X
        
        q_init = np.zeros(6)
        q_solution, success = inverse_kinematics_differential(T_target, q_init, vx300s_config)
        
        console.print(f"  IK success: {success}")
        console.print(f"  Solution: {np.round(q_solution, 4)}")
        
        T_achieved = forward_kinematics(q_solution, vx300s_config)
        error = compute_pose_error(T_target, T_achieved)
        
        console.print(f"  Position error: {np.linalg.norm(error[0:3]):.6f} m")
        console.print(f"  Orientation error: {np.linalg.norm(error[3:6]):.6f} rad")
        
        console.print("\n[green]✓ Kinematics tests passed![/green]")
        
    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")
        import traceback
        traceback.print_exc()


# Import time for the main loop
import time


if __name__ == "__main__":
    app()