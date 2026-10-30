import unittest

from arm_bench.controller_launcher import build_mpc_launch_command


class MPCLauncherTests(unittest.TestCase):
    def test_build_mpc_launch_command_includes_workspace_setup(self):
        cmd = build_mpc_launch_command('/tmp/ros_ws')
        self.assertEqual(cmd[0], 'bash')
        self.assertEqual(cmd[1], '-lc')
        self.assertIn('/tmp/ros_ws/install/setup.bash', cmd[2])
        self.assertIn('ros2 launch controller controller_misa.launch.py', cmd[2])


if __name__ == '__main__':
    unittest.main()
