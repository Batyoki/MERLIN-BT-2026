ros2 service call /masterR/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{'cmd_type': 'group', 'name':'arm', 'enable':false}"

ros2 service call /masterR/set_operating_modes interbotix_xs_msgs/srv/OperatingModes "{cmd_type: 'group', name: 'arm', mode: 'current', profile_type: 'time', profile_velocity: 0, profile_acceleration: 0}"

# ros2 service call /masterR/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{'cmd_type': 'group', 'name':'arm', 'enable':true}"
ros2 service call /slaveR/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{'cmd_type': 'group', 'name':'arm', 'enable':true}"
# ros2 run controller controller --ros-args --params-file config/controller.yaml

