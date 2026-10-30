# import rclpy
# from rclpy.node import Node

# class bilateralKinematics(Node):
#     def __init__(self):
#         super().__init__('bilateralKinematics')
#         self.get_logger().info('Bilateral Node started')
    


# def main(args=None):
#     try:
#         rclpy.init(args=args)
#         node=bilateralKinematics()
#         rclpy.spin(node=node)
#         rclpy.shutdown()
#     except:
#         print(f'Exception in bilateralKinematics')


# if __name__=='__main__':
#     main()


"""
mink_differential_mapper.py

Leader:  /rx150/joint_states  (5 DOF)
Follower: publish to /vx300s/commands/joint_group (6 DOF)

Uses a Mink-like differentiable kinematics library (torch-based) for:
 - fk: forward kinematics -> 4x4 transform
 - jacobian: 6xN Jacobian (linear+angular)

Differential IK:
 dq = J_pinv * v_ee + (I - J_pinv*J) * q_null_grad * alpha_null
 where q_null_grad is a gradient that minimizes deviation from a preferred
 posture (handles extra DOF).
"""

## !/home/sp/Desktop/5th\ Semester/RnD/rnd python3

# import sys
# print(sys.version)
# print(sys.path)
# import platform
# print(platform.python_version())

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointGroupCommand
import numpy as np
import math
import time
import mink
import torch

# ------------------ CONFIG ------------------
LEADER_TOPIC = '/rx150/joint_states'
FOLLOWER_CMD_TOPIC = '/vx300s/commands/joint_group'

# Names (must match JointState.name)
LEADER_NAMES = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']   # 5 DOF
FOLLOWER_NAMES = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']  # 6 DOF

CONTROL_RATE = 60.0            # Hz
MAX_DQ = 0.06                  # rad per cycle (safety)
DAMPING = 0.05                 # damping for pseudo-inverse (damped least squares)
GAIN = 1.0                     # scale EE velocity -> joint velocity
NULL_SPACE_GAIN = 0.5          # weight for null-space posture correction
EE_VEL_FILTER_ALPHA = 0.6      # low-pass for EE velocity

# URDF identifiers for Mink RobotModel; adapt paths/names to your setup
LEADER_URDF = 'rx150.urdf'     
FOLLOWER_URDF = 'vx300s.urdf'

# Preferred null-space posture for follower (6-dim) — choose a neutral safe posture
PREFERRED_FOLLOWER_Q = np.array([0.0, -0.5, 0.8, 0.0, 0.0, 0.0], dtype=float)
# --------------------------------------------

class MinkDifferentialMapper(Node):
    def __init__(self):
        super().__init__('mink_differential_mapper')

        # if not MINK_AVAILABLE:
        #     self.get_logger().error(
        #         "mink (and/or torch) import failed. Install your Mink library and torch, then retry."
        #     )
        #     raise RuntimeError("mink not available")

        self.get_logger().info('Initializing Mink differential mapper node')

        # Adjust constructor arguments to match your Mink installation (URDF path, base/ee links)
        # Typical alternative: mink.RobotModel.from_urdf_str(urdf_string)
        try:
            self.leader_robot = mink.RobotModel(LEADER_URDF)      # 5-DOF model
            self.follower_robot = mink.RobotModel(FOLLOWER_URDF)  # 6-DOF model
        except Exception as e:
            self.get_logger().error(f'Failed to instantiate RobotModel: {e}')
            raise

        # Verify DOFs
        n_leader = self.leader_robot.num_joints
        n_follower = self.follower_robot.num_joints
        self.get_logger().info(f'Leader DOF: {n_leader}, Follower DOF: {n_follower}')

        # Subscribers & Publishers
        self.sub = self.create_subscription(JointState, LEADER_TOPIC, self.leader_cb, 10)
        self.pub = self.create_publisher(JointGroupCommand, FOLLOWER_CMD_TOPIC, 10)

        # State
        self.last_leader_q = None
        self.last_time = None
        self.last_ee_pose = None
        self.ee_vel_filtered = np.zeros(6, dtype=float)
        self.follower_q = PREFERRED_FOLLOWER_Q.copy()
        self.timer = self.create_timer(1.0/CONTROL_RATE, self.control_loop)

        # Latest computed velocity
        self.latest_v_ee = None

        self.get_logger().info('Mink differential mapper ready')

    # ---------------- helper math ----------------
    @staticmethod
    def matrix_to_pos_quat(T):
        """T (4x4 numpy) => pos (3,), quat (x,y,z,w)"""
        pos = T[0:3, 3]
        R = T[0:3, 0:3]
        # convert to quaternion (x,y,z,w)
        tr = R[0,0] + R[1,1] + R[2,2]
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2,1] - R[1,2]) / S
            qy = (R[0,2] - R[2,0]) / S
            qz = (R[1,0] - R[0,1]) / S
        else:
            if (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
                S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
                qw = (R[2,1] - R[1,2]) / S
                qx = 0.25 * S
                qy = (R[0,1] + R[1,0]) / S
                qz = (R[0,2] + R[2,0]) / S
            elif R[1,1] > R[2,2]:
                S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
                qw = (R[0,2] - R[2,0]) / S
                qx = (R[0,1] + R[1,0]) / S
                qy = 0.25 * S
                qz = (R[1,2] + R[2,1]) / S
            else:
                S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
                qw = (R[1,0] - R[0,1]) / S
                qx = (R[0,2] + R[2,0]) / S
                qy = (R[1,2] + R[2,1]) / S
                qz = 0.25 * S
        return pos, np.array([qx, qy, qz, qw], dtype=float)

    @staticmethod
    def rotmat_to_rotvec(R):
        angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
        if abs(angle) < 1e-8:
            return np.zeros(3, dtype=float)
        rx = (R[2,1] - R[1,2]) / (2.0 * math.sin(angle))
        ry = (R[0,2] - R[2,0]) / (2.0 * math.sin(angle))
        rz = (R[1,0] - R[0,1]) / (2.0 * math.sin(angle))
        return angle * np.array([rx, ry, rz], dtype=float)

    # ---------------- callbacks ----------------
    def leader_cb(self, msg: JointState):
        now = self.get_clock().now().nanoseconds / 1e9
        # reorder leader q to expected order
        name_to_pos = dict(zip(msg.name, msg.position))
        q_leader = np.array([name_to_pos.get(n, 0.0) for n in LEADER_NAMES], dtype=float)

        # store initial follower guess if not set (map common names)
        if self.follower_q is None or self.follower_q.shape[0] != len(FOLLOWER_NAMES):
            self.follower_q = PREFERRED_FOLLOWER_Q.copy()

        # compute FK using mink (assumed API: forward_kinematics returns 4x4 numpy array)
        try:
            T_leader = self.leader_robot.forward_kinematics(q_leader)   # 4x4 numpy
        except Exception as e:
            self.get_logger().warn(f'leader fk failed: {e}')
            return

        # compute EE velocity by finite difference
        if self.last_ee_pose is None or self.last_time is None:
            self.last_ee_pose = T_leader
            self.last_time = now
            self.last_leader_q = q_leader
            return

        dt = now - self.last_time
        if dt <= 1e-6:
            return

        p_prev = self.last_ee_pose[0:3, 3]
        p_now = T_leader[0:3, 3]
        v_lin = (p_now - p_prev) / dt

        R_prev = self.last_ee_pose[0:3, 0:3]
        R_now = T_leader[0:3, 0:3]
        r_prev = self.rotmat_to_rotvec(R_prev)
        r_now = self.rotmat_to_rotvec(R_now)
        v_ang = (r_now - r_prev) / dt

        v_ee = np.concatenate([v_lin, v_ang], axis=0)

        # low-pass filter
        self.ee_vel_filtered = EE_VEL_FILTER_ALPHA * self.ee_vel_filtered + (1.0 - EE_VEL_FILTER_ALPHA) * v_ee
        self.latest_v_ee = self.ee_vel_filtered.copy()

        # update last
        self.last_ee_pose = T_leader
        self.last_time = now
        self.last_leader_q = q_leader

    # ---------------- main control loop ----------------
    def control_loop(self):
        if self.latest_v_ee is None:
            return

        # Get follower jacobian at current follower_q
        qf = self.follower_q.copy()
        try:
            # assumed API: jacobian(q) -> 6xN numpy (linear;angular)
            J = self.follower_robot.jacobian(qf)   # numpy array shape (6, n_follower)
        except Exception as e:
            # fallback: try to compute numeric jacobian via small perturbations (not shown)
            self.get_logger().warn(f'jacobian eval failed: {e}')
            return

        # Damped least squares pseudoinverse: J_pinv = J^T (J J^T + lambda^2 I)^-1
        JJt = J.dot(J.T)
        lambda2 = (DAMPING ** 2)
        try:
            inv = np.linalg.inv(JJt + lambda2 * np.eye(6))
        except np.linalg.LinAlgError:
            inv = np.linalg.pinv(JJt + lambda2 * np.eye(6))
        J_pinv = J.T.dot(inv)   # shape: (n_follower, 6)

        # compute joint velocity delta from EE velocity
        v = self.latest_v_ee.reshape((6,))  # ensure shape
        dq_part = GAIN * (J_pinv.dot(v))    # (n_follower,)

        # null-space to minimize deviation from preferred posture:
        # q_null_grad = (preferred - qf)
        q_null_grad = (PREFERRED_FOLLOWER_Q - qf)
        # project into null-space: (I - J_pinv*J) * q_null_grad
        I = np.eye(J_pinv.shape[0])
        null_projection = I - J_pinv.dot(J)
        dq_null = NULL_SPACE_GAIN * (null_projection.dot(q_null_grad))

        # combined dq
        dq = dq_part + dq_null

        # clamp step for safety
        dq = np.clip(dq, -MAX_DQ, MAX_DQ)

        new_q = qf + dq

        # publish to follower
        cmd = JointGroupCommand()
        cmd.name = 'arm'   # the group defined in interbotix driver
        # ensure length matches follower DOF (some interbotix drivers accept floats in radians)
        cmd.cmd = [float(x) for x in new_q.tolist()]
        self.pub.publish(cmd)

        # update stored follower q
        self.follower_q = new_q

    # ---------------- shutdown ----------------
    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    print('Starting to run')
    try:
        node = MinkDifferentialMapper()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
