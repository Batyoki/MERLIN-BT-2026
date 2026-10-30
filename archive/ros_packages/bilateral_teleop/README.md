# Bilateral Teleoperation: ReactorX-150 → ViperX-300s

This package implements bilateral teleoperation between a **ReactorX-150 (5DOF leader)** and a **ViperX-300s (6DOF follower)** using **MoveIt2** for FK/IK computation and **Gazebo** for simulation.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      BILATERAL TELEOPERATION PIPELINE                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────┐         ┌─────────────────┐        ┌────────────────┐ │
│  │  ReactorX-150   │         │   MoveIt2 FK    │        │    MoveIt2     │ │
│  │   (5DOF Leader) │ ──────► │   compute_fk()  │ ─────► │    IK Service  │ │
│  │                 │ joints  │                 │  pose  │   compute_ik() │ │
│  └─────────────────┘         └─────────────────┘        └────────────────┘ │
│          ▲                                                      │          │
│          │                                                      │ joints   │
│          │                                                      ▼          │
│          │                                              ┌────────────────┐ │
│   /rx150/joint_states                                   │  ViperX-300s   │ │
│                                                         │ (6DOF Follower)│ │
│                                                         │                │ │
│                                                         └────────────────┘ │
│                                                                 ▲          │
│                                                                 │          │
│                                              /vx300s/commands/joint_group  │
│                                                                            │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Features

- **MoveIt2 FK/IK**: Uses PyMoveIt2's `compute_fk()` and `compute_ik()` services
- **5DOF to 6DOF Mapping**: Leader (5 joints) to Follower (6 joints)
- **Gazebo Simulation**: Full simulation support with both robots
- **Hardware Support**: Works with real Interbotix arms
- **Real-time Control**: Configurable control rate (default 30Hz)

## Prerequisites

1. **ROS2 Humble** (or later)
2. **Interbotix ROS2 packages**:
   - `interbotix_xsarm_control`
   - `interbotix_xsarm_sim`
   - `interbotix_xsarm_moveit`
3. **PyMoveIt2**: Already in your workspace
4. **Gazebo Classic** (for simulation)

## Build Instructions

```bash
# Navigate to your ROS2 workspace
cd /home/yash_sai/Yash/Arm/ros_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install --packages-select bilateral_teleop pymoveit2

# Source the workspace
source install/setup.bash
```

## Running the System

### Option 1: Gazebo Simulation (Recommended for Testing)

**Terminal 1 - Launch Leader (ReactorX-150) in Gazebo + MoveIt2:**
```bash
cd /home/yash_sai/Yash/Arm/ros_ws
source install/setup.bash

# Launch RX150 with Gazebo and MoveIt2
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py \
    robot_model:=rx150 \
    robot_name:=rx150 \
    hardware_type:=gz_classic \
    use_rviz:=true
```

**Terminal 2 - Launch Follower (ViperX-300s) in Gazebo + MoveIt2:**
```bash
cd /home/yash_sai/Yash/Arm/ros_ws
source install/setup.bash

# Launch VX300s with Gazebo and MoveIt2
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py \
    robot_model:=vx300s \
    robot_name:=vx300s \
    hardware_type:=gz_classic \
    use_rviz:=false
```

**Terminal 3 - Launch Bilateral Teleoperation:**
```bash
cd /home/yash_sai/Yash/Arm/ros_ws
source install/setup.bash

# Launch the bilateral teleop node
ros2 launch bilateral_teleop bilateral_teleop.launch.py
```

### Option 2: Hardware (Real Robots)

**Terminal 1 - Launch Leader (ReactorX-150) Hardware + MoveIt2:**
```bash
cd /home/yash_sai/Yash/Arm/ros_ws
source install/setup.bash

# Launch RX150 with real hardware and MoveIt2
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py \
    robot_model:=rx150 \
    robot_name:=rx150 \
    hardware_type:=actual \
    use_rviz:=true
```

**Terminal 2 - Launch Follower (ViperX-300s) Hardware + MoveIt2:**
```bash
cd /home/yash_sai/Yash/Arm/ros_ws
source install/setup.bash

# Launch VX300s with real hardware and MoveIt2
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py \
    robot_model:=vx300s \
    robot_name:=vx300s \
    hardware_type:=actual \
    use_rviz:=false
```

**Terminal 3 - Launch Bilateral Teleoperation:**
```bash
cd /home/yash_sai/Yash/Arm/ros_ws
source install/setup.bash

ros2 launch bilateral_teleop bilateral_teleop.launch.py
```

### Option 3: All-in-One Launch (Experimental)

```bash
cd /home/yash_sai/Yash/Arm/ros_ws
source install/setup.bash

# Launch everything with Gazebo simulation
ros2 launch bilateral_teleop bilateral_gazebo.launch.py use_sim:=true

# Or with hardware
ros2 launch bilateral_teleop bilateral_gazebo.launch.py use_sim:=false
```

## Testing the Setup

### 1. Check Topics
```bash
# List all topics
ros2 topic list

# You should see:
# /rx150/joint_states
# /rx150/ee_pose
# /vx300s/joint_states
# /vx300s/commands/joint_group
```

### 2. Monitor Joint States
```bash
# Watch leader joint states
ros2 topic echo /rx150/joint_states

# Watch follower commands
ros2 topic echo /vx300s/commands/joint_group
```

### 3. Monitor End-Effector Pose
```bash
# Watch the computed FK pose
ros2 topic echo /rx150/ee_pose
```

### 4. Move the Leader
In Gazebo or with real hardware, move the ReactorX-150 joints. The ViperX-300s should follow the end-effector position.

```bash
# Publish test joint positions to leader (simulation)
ros2 topic pub /rx150/commands/joint_group interbotix_xs_msgs/msg/JointGroupCommand \
    "name: 'arm'
     cmd: [0.0, -0.5, 0.5, 0.0, 0.0]"
```

## Configuration

Edit `config/bilateral_params.yaml` to adjust:

```yaml
bilateral_teleop:
  ros__parameters:
    control_rate: 30.0      # Hz - increase for faster response
    ik_timeout: 0.1         # seconds - IK solver timeout
    position_scale: 1.0     # Scale factor for workspace
```

## Troubleshooting

### MoveIt2 Services Not Available
```bash
# Check if MoveIt2 services are running
ros2 service list | grep compute

# You should see:
# /rx150/compute_fk
# /rx150/compute_ik
# /vx300s/compute_fk
# /vx300s/compute_ik
```

### IK Failures
- The 6DOF ViperX-300s may not always find IK solutions for all 5DOF leader poses
- Try moving the leader to more reachable positions
- Check that the `forearm_roll` joint (extra DOF) is being utilized

### Robot Not Moving
```bash
# Check if commands are being published
ros2 topic echo /vx300s/commands/joint_group

# Check node status
ros2 node info /moveit2_bilateral_teleop
```

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/rx150/joint_states` | `sensor_msgs/JointState` | Leader joint positions |
| `/rx150/ee_pose` | `geometry_msgs/PoseStamped` | Leader EE pose (FK result) |
| `/vx300s/joint_states` | `sensor_msgs/JointState` | Follower joint positions |
| `/vx300s/commands/joint_group` | `interbotix_xs_msgs/JointGroupCommand` | Follower commands |

## Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `leader.robot_name` | `rx150` | Leader namespace |
| `follower.robot_name` | `vx300s` | Follower namespace |
| `control_rate` | `30.0` | Control loop frequency (Hz) |
| `ik_timeout` | `0.1` | IK solver timeout (seconds) |
| `position_scale` | `1.0` | Workspace scale factor |

## Files

```
bilateral_teleop/
├── bilateral_teleop/
│   ├── __init__.py
│   ├── reactor_fk.py         # Standalone FK node
│   ├── viper_ik.py           # Standalone IK node
│   └── moveit2_bilateral.py  # Main bilateral node (uses MoveIt2)
├── config/
│   └── bilateral_params.yaml # Configuration
├── launch/
│   ├── bilateral_teleop.launch.py   # Simple launch
│   └── bilateral_gazebo.launch.py   # Full Gazebo launch
├── package.xml
├── setup.py
└── README.md
```

## License

MIT License
