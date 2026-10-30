#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class BilateralNode(Node):
    def __init__(self):
        super().__init__('bilateral_node')
        self.declare_parameter('leader_ns', '/rx150')
        self.declare_parameter('follower_ns', '/vx300s')

        leader_ns = self.get_parameter('leader_ns').value
        follower_ns = self.get_parameter('follower_ns').value

        self.get_logger().info(f'Leader: {leader_ns}, Follower: {follower_ns}')

        # Publishers and Subscribers
        self.leader_sub = self.create_subscription(
            JointState,
            f'{leader_ns}/joint_states',
            self.leader_callback,
            10
        )

        self.follower_pub = self.create_publisher(
            JointState,
            f'{follower_ns}/joint_states',
            10
        )

        self.last_leader_state = None

    def leader_callback(self, msg: JointState):
        """Mirror the leader joint positions to the follower."""
        self.last_leader_state = msg

        traj = JointState()
        # traj.joint_names = msg.name
        traj.position=msg.position

        self.get_logger().info(f'{traj.name} are the joint names')
        self.get_logger().info(f'{traj}')

        point = JointTrajectoryPoint()
        point.positions = msg.position
        point.velocities = [0.0] * len(msg.position)
        point.time_from_start.sec = 1

        # traj.points.append(point)
        self.follower_pub.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = BilateralNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
