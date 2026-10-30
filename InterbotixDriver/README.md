# Interbotix driver

This folder used to contain committed colcon `build/`, `install/` and `log/`
output (with symlinks into one developer's home directory), which cannot run on
any other machine. It is no longer tracked.

Install the Interbotix X-Series arm driver (ROS 2 Humble) with the official script:

```bash
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d humble
```

Then source it before building MERLIN's `ros_ws`:

```bash
source ~/interbotix_ws/install/setup.bash
cd ros_ws && colcon build && source install/setup.bash
```
