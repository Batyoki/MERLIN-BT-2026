# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import numpy as np

# from geometry_msgs.msg import Pose
# from interbotix_xs_msgs.msg import JointGroupCommand

# # ------------------ Math Helpers ------------------ #

# def skew(v):
#     return np.array([
#         [0, -v[2], v[1]],
#         [v[2], 0, -v[0]],
#         [-v[1], v[0], 0]
#     ])

# def adjoint(T):
#     R = T[:3, :3]
#     p = T[:3, 3]
#     p_hat = skew(p)
#     Ad = np.zeros((6, 6))
#     Ad[:3, :3] = R
#     Ad[3:, 3:] = R
#     Ad[3:, :3] = p_hat @ R
#     return Ad

# def matrix_exp6(w, v, theta):
#     w = np.array(w)
#     v = np.array(v)

#     wn = np.linalg.norm(w)
#     if wn < 1e-9:
#         T = np.eye(4)
#         T[:3, 3] = v * theta
#         return T

#     w_unit = w / wn
#     wx = skew(w_unit)
#     th = theta * wn

#     R = np.eye(3) + np.sin(th)*wx + (1 - np.cos(th))*(wx @ wx)
#     G = np.eye(3)*th + (1 - np.cos(th))*wx + (th - np.sin(th))*(wx @ wx)
#     p = G @ (v / wn)

#     T = np.eye(4)
#     T[:3, :3] = R
#     T[:3, 3]  = p
#     return T

# def fk_poe(Slist, q, M):
#     T = np.eye(4)
#     for i in range(len(q)):
#         w = Slist[i, :3]
#         v = Slist[i, 3:]
#         T = T @ matrix_exp6(w, v, q[i])
#     return T @ M

# def jacobian_space(Slist, q):
#     T = np.eye(4)
#     J = np.zeros((6, len(q)))
#     J[:, 0] = Slist[0]

#     for i in range(1, len(q)):
#         w = Slist[i-1, :3]
#         v = Slist[i-1, 3:]
#         T = T @ matrix_exp6(w, v, q[i-1])
#         J[:, i] = adjoint(T) @ Slist[i]

#     return J

# # ------------------ ROS2 Node ------------------ #

# class ViperX300sDiffIK(Node):

#     def __init__(self):
#         super().__init__("viperx300s_diff_ik_controller")

#         # Subscribe to RX150 end effector pose
#         self.pose_sub = self.create_subscription(
#             Pose,
#             "/rx150/end_effector_pose",
#             self.target_pose_cb,
#             10
#         )

#         # Publisher for Interbotix motor controller
#         self.cmd_pub = self.create_publisher(
#             JointGroupCommand,
#             "/vx300s/commands/joint_group",
#             10
#         )

#         self.get_logger().info("ViperX-300S Diff IK Controller Node Started")

#         # ===============================
#         #  ViperX-300S PoE parameters
#         # ===============================
#         # (Replace these with the real screw axes for your arm)
#         # self.Slist = np.array([
#         #     [0,0,1,   0,0,0],
#         #     [0,1,0,   0,0,0.10],
#         #     [0,1,0,   0,0,0.25],
#         #     [1,0,0,   0.20,0,0],
#         #     [0,1,0,   0.30,0,0],
#         #     [1,0,0,   0,0.40,0]
#         # ], dtype=float)


#         self.Slist = np.array([
#             [0,0,1,0,0,0],
#             [0,1,0,-0.12705,0,0],
#             [0,1,0,-0.42705,0,0.05955],
#             [1,0,0,0,0.42705,0],
#             [0,1,0,-0.42705,0,0.35955],
#             [1,0,0,0,0.42705,0]
#         ],
#         dtype=float)

#         # self.M = np.eye(4)
#         # self.M[0,3] = 0.40
#         # self.M[2,3] = 0.30

#         self.M = np.array(
#             [
#                 [1,0,0,0.536494],
#                 [0,1,0,0],
#                 [0,0,1,0.42705],
#                 [0,0,0,1]
#             ],
#         dtype=float
#         )

#         # internal joint angles
#         self.q = np.zeros(6)

#         # IK tuning
#         self.dt = 0.01
#         self.damping = 0.01

#         # Timer for control loop
#         self.timer = self.create_timer(self.dt, self.control_loop)

#         # target position only (orientation optional)
#         self.xd = np.zeros(3)

#     def target_pose_cb(self, pose):
#         self.xd = np.array([pose.position.x, pose.position.y, pose.position.z])

#     def control_loop(self):
#         # FK to get current end effector position
#         T = fk_poe(self.Slist, self.q, self.M)
#         xc = T[:3, 3]

#         # position error
#         v = (self.xd - xc) * 3.0   # proportional gain

#         # desired spatial twist
#         V = np.hstack([v, np.zeros(3)])

#         # compute Jacobian
#         J = jacobian_space(self.Slist, self.q)

#         # damped least squares
#         JT = J.T
#         dampingI = self.damping * np.eye(6)
#         dq = JT @ np.linalg.inv(J @ JT + dampingI) @ V

#         # integrate
#         self.q = self.q + dq * self.dt

#         # publish to Interbotix arm
#         msg = JointGroupCommand()
#         msg.name = "arm"
#         msg.cmd = self.q.tolist()
#         self.get_logger().info(f"Publishing joint commands: {msg.cmd}")
#         self.cmd_pub.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ViperX300sDiffIK()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Pose
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
from sensor_msgs.msg import JointState

# ------------------ Math Helpers ------------------ #

def skew(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def adjoint(T):
    R = T[:3, :3]
    p = T[:3, 3]
    p_hat = skew(p)
    Ad = np.zeros((6, 6))
    Ad[:3, :3] = R
    Ad[3:, 3:] = R
    Ad[3:, :3] = p_hat @ R
    return Ad

def matrix_exp6(w, v, theta):
    w = np.array(w)
    v = np.array(v)

    wn = np.linalg.norm(w)
    if wn < 1e-9:
        T = np.eye(4)
        T[:3, 3] = v * theta
        return T

    w_unit = w / wn
    wx = skew(w_unit)
    th = theta * wn

    R = np.eye(3) + np.sin(th)*wx + (1 - np.cos(th))*(wx @ wx)
    G = np.eye(3)*th + (1 - np.cos(th))*wx + (th - np.sin(th))*(wx @ wx)
    p = G @ (v / wn)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = p
    return T

def fk_poe(Slist, q, M):
    T = np.eye(4)
    for i in range(len(q)):
        w = Slist[i, :3]
        v = Slist[i, 3:]
        T = T @ matrix_exp6(w, v, q[i])
    return T @ M

def jacobian_space(Slist, q):
    T = np.eye(4)
    J = np.zeros((6, len(q)))
    J[:, 0] = Slist[0]

    for i in range(1, len(q)):
        w = Slist[i-1, :3]
        v = Slist[i-1, 3:]
        T = T @ matrix_exp6(w, v, q[i-1])
        J[:, i] = adjoint(T) @ Slist[i]

    return J

def rotation_to_quaternion(R):
    """Convert rotation matrix to quaternion"""
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    return np.array([x, y, z, w])

def quaternion_to_rotation(q):
    """Convert quaternion [x, y, z, w] to rotation matrix"""
    x, y, z, w = q
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])
    return R

def rotation_error(Rd, Rc):
    """Compute orientation error as angular velocity"""
    R_err = Rd @ Rc.T
    # Extract axis-angle
    angle = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))
    if angle < 1e-6:
        return np.zeros(3)
    axis = 1/(2*np.sin(angle)) * np.array([
        R_err[2,1] - R_err[1,2],
        R_err[0,2] - R_err[2,0],
        R_err[1,0] - R_err[0,1]
    ])
    return angle * axis

# ------------------ ROS2 Node ------------------ #

class ViperX300sDiffIK(Node):

    def __init__(self):
        super().__init__("viperx300s_diff_ik_controller")

        # Subscribe to RX150 end effector pose
        self.pose_sub = self.create_subscription(
            Pose,
            "/rx150/end_effector_pose",
            self.target_pose_cb,
            10
        )

        # Subscribe to ViperX-300S joint states to get current configuration
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/vx300s/joint_states",
            self.joint_state_cb,
            10
        )

        # Publisher for Interbotix motor controller
        self.cmd_pub = self.create_publisher(
            JointGroupCommand,
            "/vx300s/commands/joint_group",
            10
        )

        # self.dummy_pub = self.create_publisher(
        #     JointState,
        #     "/vx300s/joint_states",
        #     10
        # )

        self.get_logger().info("ViperX-300S Diff IK Controller Node Started")

        # ViperX-300S PoE parameters (6-DOF)
        self.Slist = np.array([
            [0, 0, 1, 0, 0, 0],
            [0, 1, 0, -0.12705, 0, 0],
            [0, 1, 0, -0.42705, 0, 0.05955],
            [1, 0, 0, 0, 0.42705, 0],
            [0, 1, 0, -0.42705, 0, 0.35955],
            [1, 0, 0, 0, 0.42705, 0]
        ], dtype=float)

        self.M = np.array([
            [1, 0, 0, 0.536494],
            [0, 1, 0, 0],
            [0, 0, 1, 0.42705],
            [0, 0, 0, 1]
        ], dtype=float)

        # Internal joint angles - will be updated from joint_states
        self.q = np.zeros(6)
        self.joint_state_received = False

        # IK tuning
        self.dt = 0.05
        self.damping = 0.05
        self.position_gain = 5.0
        self.orientation_gain = 2.0

        # Timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Target pose (position + orientation)
        self.target_position = np.zeros(3)
        self.target_orientation = np.array([0, 0, 0, 1])  # [x, y, z, w]
        self.target_received = False

    def joint_state_cb(self, msg):
        """Update current joint angles from robot feedback"""
        # try:
        #     # Extract arm joint positions (adjust indices based on your robot)
        #     # Typically joints are named like: waist, shoulder, elbow, wrist_angle, wrist_rotate
        #     arm_joints = ['waist', 'shoulder', 'elbow','forearm_roll', 'wrist_angle', 'wrist_rotate']

        #     # arm_joints=['elbow','shoulder','forearm_roll','wrist_angle','wrist_rotate','waist']
            
        #     for i, joint_name in enumerate(arm_joints[:6]):
        #         if joint_name in msg.name:
        #             idx = msg.name.index(joint_name)
        #             self.q[i] = msg.position[idx]
            
        #     self.joint_state_received = True
        # except Exception as e:
        #     self.get_logger().warn(f"Error parsing joint states: {e}")
        self.joint_state_received=True
        pass

    def target_pose_cb(self, pose):
        """Receive target pose from RX150"""
        self.target_position = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z
        ])
        self.target_orientation = np.array([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        self.target_received = True

    def control_loop(self):
        """Main differential IK control loop"""
        if not self.joint_state_received:
            self.get_logger().warn("Waiting for joint states...", throttle_duration_sec=2.0)
            return

        if not self.target_received:
            self.get_logger().warn("Waiting for target pose...", throttle_duration_sec=2.0)
            return

        # FK to get current end effector pose
        T_current = fk_poe(self.Slist, self.q, self.M)
        current_position = T_current[:3, 3]
        current_rotation = T_current[:3, :3]

        # Position error
        position_error = self.target_position - current_position

        # Orientation error
        target_rotation = quaternion_to_rotation(self.target_orientation)
        orientation_error = rotation_error(target_rotation, current_rotation)

        # Check if we're close enough
        pos_err_norm = np.linalg.norm(position_error)
        ori_err_norm = np.linalg.norm(orientation_error)
        
        if pos_err_norm < 0.001 and ori_err_norm < 0.01:
            # Close enough, don't send commands
            return

        # Construct spatial velocity (twist)
        v_linear = position_error * self.position_gain
        v_angular = orientation_error * self.orientation_gain
        V = np.hstack([v_linear, v_angular])

        # Compute Jacobian
        J = jacobian_space(self.Slist, self.q)

        # Damped least squares inverse
        JT = J.T
        damping_matrix = self.damping * np.eye(6)
        dq = JT @ np.linalg.inv(J @ JT + damping_matrix) @ V

        # Limit joint velocities
        max_joint_vel = 1.0  # rad/s
        dq_norm = np.linalg.norm(dq)
        if dq_norm > max_joint_vel:
            dq = dq * (max_joint_vel / dq_norm)

        # Integrate to get new joint positions
        q_new = self.q + dq * self.dt

        # Apply joint limits (adjust these for your robot)
        joint_limits = np.array([
            [-3.14, 3.14],   # waist
            [-1.88, 1.99],   # shoulder
            [-2.15, 1.60],   # elbow
            [-1.74, 2.15],   # wrist_angle
            [-3.14, 3.14],   # wrist_rotate
            [-3.14, 3.14]    # extra DOF
        ])
        
        for i in range(6):
            q_new[i] = np.clip(q_new[i], joint_limits[i, 0], joint_limits[i, 1])

        # Update internal state
        self.q = q_new

        # Publish to Interbotix arm
        msg = JointGroupCommand()
        msg.name = "arm"
        msg.cmd = self.q.tolist()
        self.get_logger().info(f'Publishing joint commands: {msg}')
        self.get_logger().info(
            f"Pos err: {pos_err_norm:.4f}, Ori err: {ori_err_norm:.4f}"
            # throttle_duration_sec=0.5
        )
        
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ViperX300sDiffIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()