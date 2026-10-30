import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import (
    PathJoinSubstitution,)

from launch_ros.substitutions import FindPackageShare

ports = ['/dev/ttyRX150L','/dev/ttyVX300SR','/dev/ttyRX150','/dev/ttyVX300S']
namespaces = ['master_L','slave_L','master_R','slave_R']
model = ['rx200','vx300s']
config_files = ['rx200.yaml','vx300s.yaml']

def _build_robot_launches(context, *args, **kwargs):
    launches = []
    package_name = context.perform_substitution(LaunchConfiguration('launch_package', default='interbotix_xsarm_control'))
    launch_file = context.perform_substitution(LaunchConfiguration('launch_file', default='xsarm_control.launch.py'))

    for arm_index in range(1, 5):
        robot_model = context.perform_substitution(LaunchConfiguration(f'robot_{arm_index}_model', default=''))
        if not robot_model:
            continue

        namespace = context.perform_substitution(LaunchConfiguration(f'robot_{arm_index}_ns', default=f'robot_{arm_index}'))
        use_sim = context.perform_substitution(LaunchConfiguration(f'robot_{arm_index}_use_sim', default='true'))

        cmd = [
            'ros2',
            'launch',
            package_name,
            launch_file,
            f'robot_model:={model[(arm_index-1)%2]}',
            f'robot_name:={namespaces[arm_index-1]}',
            f'use_sim:={use_sim}',
            f'write_eeprom_on_startup:=false',
            f'''motor_configs:={
                PathJoinSubstitution(
                    [FindPackageShare('interbotix_xsarm_control'),'config',config_files[int(not(arm_index%2))],])}''',
            f'port_name:={ports[arm_index-1]}'
        ]
        launches.append(
            ExecuteProcess(
                cmd=cmd,
                output='screen',
                name=f'arm_{arm_index}',
            )
        )

    return launches


def generate_launch_description():
    declared_arguments = []
    for arm_index in range(1, 5):
        declared_arguments.extend([
            DeclareLaunchArgument(f'robot_{arm_index}_model', default_value=''),
            DeclareLaunchArgument(f'robot_{arm_index}_ns', default_value=f'robot_{arm_index}'),
            DeclareLaunchArgument(f'robot_{arm_index}_use_sim', default_value='true'),
        ])

    declared_arguments.extend([
        DeclareLaunchArgument('launch_package', default_value='interbotix_xsarm_control'),
        DeclareLaunchArgument('launch_file', default_value='xsarm_control.launch.py'),
    ])

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=_build_robot_launches),
    ])

