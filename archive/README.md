# Archive

Code kept for reference but no longer part of MERLIN. `COLCON_IGNORE` keeps
colcon from building anything in here.

| Folder | What it was | Why archived |
|---|---|---|
| `ros_packages/bilateral_node` | Early leader→follower kinematic mapping (mink + torch) | Superseded by `ros_ws/src/controller`; its `bilateral_node` entry point referred to a file that does not exist |
| `ros_packages/teleoperation_hopefully` | IK / MoveIt experiments | Superseded by `ros_ws/src/controller` |
| `ros_packages/bilateral_teleop` | MoveIt FK/IK leader→follower + PyBullet sim | Superseded by `ros_ws/src/controller`; hardcoded `/home/sp/...` URDF paths |
| `ros_packages/arm_launch_bilateral_node` | `arm_launch` launch files that started `bilateral_node` | Depended on the archived package (and its missing entry point) |
| `arm_bench_tk_tuning_studio` | Tkinter "Control Algorithm Development Studio" | Not reachable from the GUI or CLI; the tuning window in use is `arm_bench_dev/src/control_simulator/control_simulator_gui.py` |

The bilateral teleoperation used by MERLIN is `ros_ws/src/controller`
(`ros2 launch controller controller_misa.launch.py`).
