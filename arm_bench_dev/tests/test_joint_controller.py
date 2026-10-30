import unittest

from arm_bench.joint_controller import build_joint_group_payload


class JointControllerPayloadTests(unittest.TestCase):
    def test_build_joint_group_payload_returns_expected_shape(self):
        payload = build_joint_group_payload(["joint_1", "joint_2"], [0.1, -0.2], group_name="arm")

        self.assertEqual(payload["name"], "arm")
        self.assertEqual(payload["cmd"], [0.1, -0.2])

    def test_build_joint_group_payload_rejects_mismatched_lengths(self):
        with self.assertRaises(ValueError):
            build_joint_group_payload(["joint_1"], [0.1, 0.2])


if __name__ == "__main__":
    unittest.main()
