#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import modern_robotics as mr

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

import threading


def pose_to_transform(p):
    """Convert Pose msg → 4x4 transform."""
    T = np.eye(4)
    # position
    T[0, 3] = p.position.x
    T[1, 3] = p.position.y
    T[2, 3] = p.position.z

    # quaternion → rotation matrix
    qw, qx, qy, qz = p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z
    R = np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])
    T[:3, :3] = R
    return T


class DifferentialIK(Node):
    def __init__(
        self,
        # Slist,
        # M,
        # joint_names,
        # joint_limits,
        # joint_vel_limits,
        rate_hz=60,
        damping=0.05,
        traj_time=0.5,
        nullspace_gain=0.0
    ):
        super().__init__("diff_ik_controller")

        # self.Slist = np.array(Slist)
        self.Slist = np.array([
            [0, 0, 1, 0, 0, 0],
            [0, 1, 0, -0.12705, 0, 0],
            [0, 1, 0, -0.42705, 0, 0.05955],
            [1, 0, 0, 0, 0.42705, 0],
            [0, 1, 0, -0.42705, 0, 0.35955],
            [1, 0, 0, 0, 0.42705, 0]
        ], dtype=float).transpose()


        # self.M = np.array(M)
        self.M = np.array([
            [1, 0, 0, 0.536494],
            [0, 1, 0, 0],
            [0, 0, 1, 0.42705],
            [0, 0, 0, 1]
        ], dtype=float)

        self.joint_vel_limits = np.array([131,131,131,131,131,131])


        self.joint_order =  ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        self.joint_limits = np.array([(-3.1416, 3.1416),(1.292, 4.398),(1.376, 4.746),(-3.1416, 3.1416),(1.273, 5.375),(-3.1416, 3.1416)])
        # self.joint_vel_limits = np.array(joint_vel_limits)

        # self.N = len(joint_names)
        self.N = 6

        # Internal joint state
        self.theta = np.zeros(self.N)
        self.lock = threading.Lock()

        # settings
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.damping = damping
        self.traj_time = traj_time
        self.nullspace_gain = nullspace_gain

        # ROS interfaces
        self.target_sub = self.create_subscription(
            PoseStamped, "/rx150/ee_pose", self.target_cb, 10
        )
        self.joint_pub = self.create_publisher(JointState, "/vx300s/joint_states", 10)

        # timer for background updates
        self.timer = self.create_timer(self.dt, self.control_step)

        # trajectory buffer
        self.traj = []
        self.traj_idx = 0

        self.get_logger().info("Differential IK controller initialized.")

    # --------------------------------------------------------------------
    # UTILS
    # --------------------------------------------------------------------
    def damped_pinv(self, J):
        """Compute damped least squares pseudo-inverse."""
        JJt = J @ J.T
        lam2 = self.damping**2
        inv = np.linalg.inv(JJt + lam2 * np.eye(JJt.shape[0]))
        return J.T @ inv

    def target_cb(self, msg: PoseStamped):
        """Receive a new target pose → build a smooth trajectory."""
        T_goal = pose_to_transform(msg.pose)

        with self.lock:
            theta_now = self.theta.copy()

        T_start = mr.FKinSpace(self.M, self.Slist, theta_now)

        steps = max(2, int(self.traj_time / self.dt))
        self.traj = mr.ScrewTrajectory(T_start, T_goal, self.traj_time, steps, 3)
        self.traj_idx = 0

        self.get_logger().info("New smoothed trajectory generated.")

    # --------------------------------------------------------------------
    # MAIN CONTROL LOOP
    # --------------------------------------------------------------------
    def control_step(self):
        # no trajectory → do nothing
        if self.traj_idx >= len(self.traj):
            return

        # get desired transform at this step + next step
        Tb = np.array(self.traj[self.traj_idx])
        if self.traj_idx < len(self.traj) - 1:
            Tb_next = np.array(self.traj[self.traj_idx + 1])
        else:
            Tb_next = Tb

        # compute twist in body frame
        Vb = mr.se3ToVec(mr.MatrixLog6(np.linalg.inv(Tb) @ Tb_next))

        # compute jacobian
        with self.lock:
            theta = self.theta.copy()

        Jb = mr.JacobianBody(mr.Adjoint(np.linalg.inv(mr.FKinSpace(self.M, self.Slist, theta))) @ self.Slist,theta)

        # v = J⁺ Vb  (DLS)
        J_dls = self.damped_pinv(Jb)
        dtheta = J_dls @ Vb

        # clip velocities
        dtheta = np.clip(dtheta, -self.joint_vel_limits, self.joint_vel_limits)

        # optional null-space posture term
        if self.nullspace_gain > 0:
            midpoints = (self.joint_limits[:, 0] + self.joint_limits[:, 1]) / 2
            z = midpoints - theta
            N = np.eye(self.N) - (J_dls @ Jb)
            dtheta += self.nullspace_gain * (N @ z)

        # integrate
        theta_new = theta + dtheta * self.dt

        # enforce joint limits
        for i in range(self.N):
            theta_new[i] = np.clip(theta_new[i], self.joint_limits[i][0], self.joint_limits[i][1])

        # commit
        with self.lock:
            self.theta = theta_new

        # publish joint state
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_order
        js.position = theta_new.tolist()
        js.velocity = dtheta.tolist()
        self.joint_pub.publish(js)

        self.traj_idx += 1


# ------------------------------------------------------------------------
# MAIN
# ------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    # # -----------------------------
    # # PROVIDE YOUR ROBOT PARAMETERS
    # # -----------------------------
    # # Example 6-DOF arm (replace with your own)
    # Slist = [
    #     [0, 0, 1, 0, 0, 0],
    #     [0, 1, 0, -0.3, 0, 0],
    #     [0, 1, 0, -0.6, 0, 0],
    #     [1, 0, 0, 0, -0.6, 0],
    #     [1, 0, 0, 0, -0.8, 0],
    #     [0, 1, 0, -1.0, 0, 0],
    # ]
    # Slist = np.array(Slist).T  # 6xN

    # M = np.eye(4)
    # M[0, 3] = 1.0  # example end-effector home pose

    # joint_names = [f"joint_{i+1}" for i in range(6)]
    # joint_limits = [
    #     (-3.14, 3.14),
    #     (-2.0, 2.0),
    #     (-3.14, 3.14),
    #     (-3.14, 3.14),
    #     (-2.0, 2.0),
    #     (-3.14, 3.14),
    # ]
    # joint_vel_limits = [1.0] * 6  # rad/s

    node = DifferentialIK(
        # Slist=Slist,
        # M=M,
        # joint_names=joint_names,
        # joint_limits=joint_limits,
        # joint_vel_limits=joint_vel_limits,
        rate_hz=100,
        damping=0.1,
        traj_time=0.4,
        nullspace_gain=0.1,
    )

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
