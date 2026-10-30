import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import mink
import modern_robotics as mr
from tf_transformations import quaternion_from_euler,euler_from_quaternion


class rx150_fkNode(Node):
    def __init__(self):
        super().__init__('rx150_fk_node')

        # Subscriber to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/rx150/joint_states',
            self.joint_state_callback,
            10)

        # Publisher for end-effector pose
        self.ee_pose_pub = self.create_publisher(
            PoseStamped,
            '/rx150/ee_pose',
            10)

        # Robot-specific parameters
        self.M = np.array([[1, 0, 0, 0.358575],
                           [0, 1, 0, 0.0],
                           [0, 0, 1, 0.25457],
                           [0, 0, 0, 1]])  # Home configuration

        self.s_list = np.array([
            [0.0, 0.0, 1.0,    0.0,      0.0,    0.0],
            [0.0, 1.0, 0.0, -0.10457,      0.0,    0.0],
            [0.0, 1.0, 0.0, -0.25457,      0.0,   0.05],
            [0.0, 1.0, 0.0, -0.25457,      0.0,    0.2],
            [1.0, 0.0, 0.0,    0.0,    0.25457,    0.0]
        ]).transpose()


        self.joint_order = [
            "waist",
            "shoulder",
            "elbow",
            # "forearm_roll",
            "wrist_angle",
            "wrist_rotate"
        ]

        # parameter list from rx150.yaml [waist, shoulder, elbow, wrist_angle, wrist_rotate]

        self.get_logger().info("ReactorX150 FK node started (modern_robotics).")

    def joint_state_callback(self, msg):
        # Map joint states by name
        q_map = dict(zip(msg.name, msg.position))

        try:
            q = np.array([q_map[j] for j in self.joint_order])
        except KeyError:
            return

        # Forward kinematics: T = M * e^[S1*q1] * ... * e^[Sn*qn]
        T = mr.FKinSpace(self.M, self.s_list, q)

        p = T[:3, 3]

        # Convert rotation matrix → quaternion
        quat = mr.so3ToVec(T[:3, :3])
        quat = quaternion_from_euler(quat[0],quat[1],quat[2])

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"

        pose.pose.position.x = float(p[0])
        pose.pose.position.y = float(p[1])
        pose.pose.position.z = float(p[2])
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])

        self.ee_pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = rx150_fkNode()

    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()