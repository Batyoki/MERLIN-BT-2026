"""
Bilateral teleoperation launch.

  ros2 launch controller controller_misa.launch.py                     # controller only
  ros2 launch controller controller_misa.launch.py launch_drivers:=true  # + both arms
  ros2 launch controller controller_misa.launch.py control_mode:=pp log_dir:=/tmp/merlin
  ros2 launch controller controller_misa.launch.py params_file:=/path/my_gains.yaml use_rviz:=true

params_file  : controller YAML (default: the installed config/controller.yaml)
loop_rate_hz : overrides the YAML loop rate when > 0
use_rviz     : with launch_drivers:=true, opens RViz showing the slave arm

With launch_drivers:=true the RX200 master is started in current mode and the
VX300s slave in position mode (config/modes_master.yaml, modes_slave.yaml).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("controller")
    config = os.path.join(share, "config", "controller.yaml")

    args = [
        DeclareLaunchArgument("params_file", default_value=config),
        DeclareLaunchArgument("loop_rate_hz", default_value="0.0"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("launch_drivers", default_value="false"),
        DeclareLaunchArgument("control_mode", default_value="misa"),
        DeclareLaunchArgument("log_dir", default_value=""),
        DeclareLaunchArgument("master_name", default_value="masterR"),
        DeclareLaunchArgument("slave_name", default_value="slaveR"),
    ]

    drivers = []
    try:
        xsarm = os.path.join(get_package_share_directory("interbotix_xsarm_control"),
                             "launch", "xsarm_control.launch.py")
        for model, name, modes in (("rx200", "master_name", "modes_master.yaml"),
                                   ("vx300s", "slave_name", "modes_slave.yaml")):
            drivers.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(xsarm),
                launch_arguments={
                    "robot_model": model,
                    "robot_name": LaunchConfiguration(name),
                    "mode_configs": os.path.join(share, "config", modes),
                    "use_rviz": LaunchConfiguration("use_rviz") if model == "vx300s" else "false",
                }.items(),
                condition=IfCondition(LaunchConfiguration("launch_drivers"))))
    except Exception:
        pass   # interbotix_xsarm_control not installed: controller-only launch still works

    controller = Node(
        package="controller",
        executable="controller",
        name="controller",                 # must match the YAML root key
        parameters=[LaunchConfiguration("params_file"),
                    {"control_mode": LaunchConfiguration("control_mode"),
                     "log_dir": LaunchConfiguration("log_dir"),
                     "loop_rate_override_hz": LaunchConfiguration("loop_rate_hz")}],
        output="screen",
    )
    return LaunchDescription(args + drivers + [controller])
