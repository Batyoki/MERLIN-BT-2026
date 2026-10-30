import os
import tempfile
import unittest

from arm_bench.launch_manager import build_ros2_launch_command, build_ros2_run_command


class LaunchManagerTests(unittest.TestCase):
    def test_build_command_without_workspace_keeps_ros2_launch_form(self):
        cmd = build_ros2_launch_command('demo_pkg', 'demo.launch.py', 'robot_model:=rx200')
        self.assertEqual(cmd[0], 'ros2')
        self.assertEqual(cmd[1], 'launch')
        self.assertEqual(cmd[2], 'demo_pkg')
        self.assertEqual(cmd[3], 'demo.launch.py')
        self.assertIn('robot_model:=rx200', cmd)

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.ws = self._tmp.name
        os.makedirs(os.path.join(self.ws, 'install'))
        open(os.path.join(self.ws, 'install', 'setup.bash'), 'w').close()

    def tearDown(self):
        self._tmp.cleanup()

    def test_build_command_with_workspace_sources_setup_bash(self):
        cmd = build_ros2_launch_command('demo_pkg', 'demo.launch.py', 'robot_model:=rx200', self.ws)
        self.assertEqual(cmd[0], 'bash')
        self.assertEqual(cmd[1], '-lc')
        self.assertIn(os.path.join(self.ws, 'install', 'setup.bash'), cmd[2])
        self.assertIn('ros2 launch demo_pkg demo.launch.py robot_model:=rx200', cmd[2])

    def test_missing_workspace_raises(self):
        with self.assertRaises(FileNotFoundError):
            build_ros2_launch_command('demo_pkg', 'demo.launch.py', '', os.path.join(self.ws, 'nope'))

    def test_run_node_command(self):
        cmd = build_ros2_run_command('controller', 'controller', '', self.ws)
        self.assertIn('ros2 run controller controller', cmd[2])


if __name__ == '__main__':
    unittest.main()
