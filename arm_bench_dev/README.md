# arm-bench: Robot Arm Benchmarking and Control Toolkit

A comprehensive toolkit for controlling, monitoring, and collecting data from robot arms, with a focus on Interbotix arms using Dynamixel Protocol 2.0.

## Features

### 🎮 Multiple Operation Modes
- **Simulation**: Test without hardware
- **Hardware**: Direct robot control
- **Hybrid**: Mixed simulation and hardware

### 🔍 Hardware Scanning
- Automatic Dynamixel motor detection (Protocol 2.0)
- Camera scanning and configuration
- Hardware state monitoring (torque, velocity, position, temperature)

### 🖥️ GUI Interface
- Mode selection dialog
- Real-time monitoring interface
- Motor telemetry display
- Camera management

### 🎯 Teleoperation
- **Joystick Control**: Use game controllers for robot control
- **WebXR Control**: Camera-based control with hand tracking
- Differential IK for end-effector control

### 🔧 ROS2 Integration
- RViz visualization support
- Interbotix workspace builder
- ROS2 control integration

### 🤖 Bimanual Manipulation
- Translation between different arm configurations
- Reactor (5DOF) ↔ Viper (6DOF) conversion
- Synchronized dual-arm control

### 📊 Data Collection
- RLDS (Reinforcement Learning Dataset) format
- Multi-camera video recording
- Joint states, velocities, and torques
- Episode-based data organization

## Installation

### Basic Installation

```bash
# Clone the repository
cd arm_bench_dev

# Create virtual environment
python3 -m venv arm-test
source arm-test/bin/activate  # On Windows: arm-test\Scripts\activate

# Install package
pip install -e .
```

### Full Installation (with optional dependencies)

```bash
pip install -e ".[full]"
```

Optional dependencies include:
- `opencv-python`: Camera support
- `pygame`: Joystick teleoperation

### ROS2 Requirements

For full functionality, install ROS2:

```bash
# Example for Ubuntu 22.04 (Humble)
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

## Quick Start

### 1. Launch the Application

```bash
arm-bench start
```

This will:
1. Open a GUI to select operation mode
2. Scan for hardware (if applicable)
3. Launch the monitoring interface

### 2. Setup Hardware

```bash
# Scan and setup hardware
arm-bench setup

# Or in simulation mode
arm-bench setup --sim
```

### 3. Visualize Robot

```bash
# Launch RViz
arm-bench visualize
```

### 4. Teleoperation

```bash
# Joystick control
arm-bench teleop --mode joystick

# WebXR camera control
arm-bench teleop --mode webxr
```

### 5. Record Data

```bash
# Start recording
arm-bench record start --episode-name my_demo

# Stop recording
arm-bench record stop

# List recorded episodes
arm-bench record list
```

## Command Reference

### Core Commands

- `arm-bench version` - Show version information
- `arm-bench start` - Launch main application
- `arm-bench setup` - Scan and setup hardware

### Visualization

- `arm-bench visualize` - Launch RViz
- `arm-bench visualize --config <path>` - Use custom RViz config

### Teleoperation

- `arm-bench teleop --mode joystick` - Joystick control
- `arm-bench teleop --mode webxr` - WebXR control

### Workspace Management

- `arm-bench build-workspace` - Build Interbotix ROS2 workspace
- `arm-bench build-workspace --path <path>` - Custom workspace path

### Arm Translation

- `arm-bench translate reactor viper` - Test reactor → viper translation
- `arm-bench translate viper reactor` - Test viper → reactor translation

### Data Recording

- `arm-bench record start` - Start recording episode
- `arm-bench record stop` - Stop recording
- `arm-bench record list` - List episodes

### Camera Management

- `arm-bench cameras` - List available cameras

## Architecture

### Module Overview

```
arm_bench/
├── cli.py                 # Command-line interface
├── gui.py                 # GUI components
├── scanner.py             # Hardware scanning
├── visualization.py       # RViz launcher
├── teleoperation.py       # Teleoperation controllers
├── interbotix_builder.py  # Workspace builder
├── bimanual.py            # Bimanual translation
└── rlds_recorder.py       # Data recording
```

### Key Classes

- `ModeSelectionGUI`: Mode selection dialog
- `MonitoringInterface`: Real-time monitoring dashboard
- `JoystickTeleoperation`: Joystick control
- `WebXRTeleoperation`: Camera-based control
- `DifferentialIKNode`: IK computation
- `BimanualTranslator`: Arm configuration translation
- `RLDSRecorder`: Dataset recording
- `InterbotixWorkspaceBuilder`: ROS2 workspace setup

## Development

### Project Structure

```
arm_bench_dev/
├── pyproject.toml          # Package configuration
├── src/
│   └── arm_bench/
│       ├── __init__.py
│       ├── cli.py
│       ├── gui.py
│       ├── scanner.py
│       ├── visualization.py
│       ├── teleoperation.py
│       ├── interbotix_builder.py
│       ├── bimanual.py
│       └── rlds_recorder.py
└── README.md
```

### Adding New Features

1. Create new module in `src/arm_bench/`
2. Import in `cli.py`
3. Add command using `@app.command()` decorator
4. Update dependencies in `pyproject.toml`

### Testing

```bash
# Test version command
arm-bench version

# Test help
arm-bench --help

# Test specific command
arm-bench setup --sim
```

## Hardware Support

### Supported Arms
- Interbotix PX, RX, VX, WX series
- Any Dynamixel Protocol 2.0 compatible arm

### Supported Controllers
- USB game controllers (via pygame)
- VR/AR devices (via WebXR)

### Supported Cameras
- Any V4L2 compatible camera on Linux
- USB webcams
- Built-in laptop cameras

## Data Format

### RLDS Dataset Structure

```
~/arm_bench_datasets/
└── episode_0001_20231201_120000/
    ├── metadata.json
    ├── joint_states/
    │   └── data.json
    └── images/
        ├── camera_0/
        ├── camera_1/
        └── ...
```

### Joint State Format

```json
{
  "timestamp": 1.234,
  "positions": [0.1, 0.2, ...],
  "velocities": [0.01, 0.02, ...],
  "efforts": [0.5, 0.6, ...],
  "temperatures": [32.0, 33.0, ...],
  "joint_names": ["joint_1", "joint_2", ...]
}
```

## Troubleshooting

### ROS2 not found
```bash
# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash
```

### No motors detected
- Check USB connections
- Verify motor power supply
- Check motor IDs and baud rate
- Try simulation mode: `arm-bench setup --sim`

### Camera not working
- Install OpenCV: `pip install opencv-python`
- Check camera permissions
- Test with: `arm-bench cameras`

### Joystick not detected
- Install pygame: `pip install pygame`
- Check controller connection
- Test in system settings

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

MIT License - See LICENSE file for details

## Support

For issues and questions:
- GitHub Issues: [Create an issue]
- Documentation: See README.md

## Roadmap

- [ ] Full Dynamixel SDK integration
- [ ] Complete WebXR implementation
- [ ] RLDS TFRecord export
- [ ] Real-time visualization in GUI
- [ ] Multi-robot coordination
- [ ] Cloud dataset storage
- [ ] Advanced IK solvers
- [ ] Force/torque control modes

## Acknowledgments

- Interbotix for robot arm drivers
- ROS2 community
- OpenAI for development assistance
