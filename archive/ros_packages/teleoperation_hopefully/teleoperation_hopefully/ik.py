import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import mink
import modern_robotics as mr
from tf_transformations import quaternion_from_euler,euler_from_quaternion
from interbotix_xs_modules.xs_robot import arm
from interbotix_xs_msgs.msg import JointGroupCommand

# arm.InterbotixArmXSInterface()


def IKinDLS(Slist, M, T_target, thetalist0, joint_limits,
            max_iters=300, tol=1e-4, damping=1e-2, random_restarts=3):
    """
    Returns: (theta_solution (6,), success(bool))
    thetalist0: initial guess (6,)
    joint_limits: list of (lo, hi) for each joint
    """
    def clip_limits(theta):
        for i, (lo, hi) in enumerate(joint_limits):
            theta[i] = np.clip(theta[i], lo, hi)
        return theta

    # Build candidate initial guesses (use provided + random restarts)
    candidates = [np.array(thetalist0, dtype=float)]
    for _ in range(random_restarts):
        candidates.append(thetalist0 + 0.02 * np.random.randn(len(thetalist0)))

    for init in candidates:
        theta = clip_limits(init.copy())

        for it in range(max_iters):
            T_curr = mr.FKinSpace(M, Slist, theta)
            se3_err = mr.MatrixLog6(np.dot(mr.TransInv(T_curr), T_target))
            Vb6 = mr.se3ToVec(se3_err)
            err = np.linalg.norm(Vb6)
            if err < tol:
                return theta, True

            J = mr.JacobianSpace(Slist, theta)   # 6xN
            JJt = J.dot(J.T)
            # Damped least squares pseudo-inverse via J^T (J J^T + λ^2 I)^-1
            try:
                inv_term = np.linalg.inv(JJt + (damping**2) * np.eye(6))
            except np.linalg.LinAlgError:
                # If matrix is singular, increase damping temporarily
                inv_term = np.linalg.inv(JJt + (max(damping, 1e-1)**2) * np.eye(6))

            delta = J.T.dot(inv_term).dot(Vb6)

            theta = theta + delta
            theta = clip_limits(theta)

            # Abort early if things blow up
            if np.any(np.isnan(theta)) or np.linalg.norm(theta) > 1e3:
                break

        # Try next random restart

    # Failure
    return theta, False

class vx300s_ik_node(Node):
    def __init__(self):
        super().__init__('viperx_300s_inverse_kinematics_node')
        self.get_logger().info("Starting the inverse kinematics node")


        self.ee_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/rx150/ee_pose',
            self.ee_pose_subscriber_callback,
            10
        )

        self.viper_joint_states = self.create_subscription(
            JointState,
            '/vx300s/joint_states',
            self.viper_joint_states_callback,
            10
        )

        self.viper_joint_states = None
        self.joint_order =  ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        self.joint_limits = [(-3.1416, 3.1416),(1.292, 4.398),(1.376, 4.746),(-3.1416, 3.1416),(1.273, 5.375),(-3.1416, 3.1416)]


        self.s_list = np.array([
            [0, 0, 1, 0, 0, 0],
            [0, 1, 0, -0.12705, 0, 0],
            [0, 1, 0, -0.42705, 0, 0.05955],
            [1, 0, 0, 0, 0.42705, 0],
            [0, 1, 0, -0.42705, 0, 0.35955],
            [1, 0, 0, 0, 0.42705, 0]
        ], dtype=float).transpose()

        self.M = np.array([
            [1, 0, 0, 0.536494],
            [0, 1, 0, 0],
            [0, 0, 1, 0.42705],
            [0, 0, 0, 1]
        ], dtype=float)

        self.home_position = mr.se3ToVec(self.M)
        self.get_logger().info(f"Home position : {self.home_position}")


        # self.arm = arm.InterbotixArmXSInterface(
        #     robot_model="vx300s",
        #     group_name="arm",
        #     core="ros2",
        #     # gripper_name = "gripper"
        # )

        self.joint_command_setting = self.create_publisher(
            JointGroupCommand,
            "/vx300s/commands/joint_group",
            10
        )

        home_command = JointGroupCommand()
        home_command.name="arm"
        home_command.cmd = np.array([0,0,0,0,0,0],dtype=float).tolist()
        self.joint_command_setting.publish(home_command)


        # self.cmd_cli = self.create_client(
        #     JointGroupCommand,
        #     "/viperx300s/commands/joint_group"
        # )
        # while not self.cmd_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("Waiting for ViperX command service...")
    

    def send_to_viperx(self, joint_positions):
        req = JointGroupCommand.Request()
        req.name = "arm"
        req.cmd = joint_positions.tolist()

        self.cmd_cli.call_async(req)


    
    def IKinDLS(self, Slist, M, T_target, steps=200, damping=0.01):
        thetalist = self.viper_joint_states if self.viper_joint_states is not None else np.zeros(6)
        for i in range(steps):
            T_current = mr.FKinSpace(M, Slist, thetalist)
            Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(T_current), T_target)))
            if np.linalg.norm(Vb) < 1e-4:
                return thetalist, True
            J = mr.JacobianSpace(Slist, thetalist)
            thetalist += np.dot(J.T, np.linalg.inv(J @ J.T + damping * np.eye(6))) @ Vb
        return thetalist, False


    def viper_joint_states_callback(self,msg):
        q_map = dict(zip(msg.name, msg.position))

        try:
            q = np.array([q_map[j] for j in self.joint_order])
        except KeyError:
            return
        
        self.viper_joint_states = q

    
    def ee_pose_subscriber_callback(self,msg):

        # Extracted pose parameters
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        wx,wy,wz = euler_from_quaternion([qx,qy,qz,qw])

        T_target = mr.VecTose3([wx,wy,wz,px, py, pz])
        self.get_logger().info(f"Target EE Pose:{T_target}")

        R = T_target[:3, :3]

        # T_target = np.vstack([
        #     np.hstack([R, np.array([[px], [py], [pz]])]),
        #     np.array([0, 0, 0, 1])
        # ])

        # theta_list = np.array([0,0,0,0,0,0])

        # theta_sol , success = mr.IKinSpace(self.s_list, self.M, T_target, eomg=0.2, ev= 0.3)
        # print(theta_sol)

        # if not success:
        #     self.get_logger().info("IK solution not found")
        #     return


        # theta_sol = self.IKinDLS(self.s_list, self.M, T_target)
        theta_sol ,success = IKinDLS(
            self.s_list,
            self.M,
            T_target,
            self.viper_joint_states if self.viper_joint_states is not None else np.zeros(6),
            self.joint_limits
        )

        T_check = mr.FKinSpace(self.M, self.s_list, theta_sol)

        # p_check = T_check[:3, 3]
        # R_check = T_check[:3, :3]


        p_check = T_check[:3, 3]
        # p_check = T_check[:3]


        R_check = T_check[:3, :3]
        # R_check = mr.RpToTrans

        # Errors
        pos_err = np.linalg.norm(np.array([px, py, pz]) - p_check)
        R_err_mat = R.T @ R_check
        rot_err = np.linalg.norm(mr.so3ToVec(mr.MatrixLog3(R_err_mat)))

        # Log results
        self.get_logger().info("\n")
        self.get_logger().info("====== ViperX-300 IK Solution ======")
        self.get_logger().info(f"Joint solution (radians): {theta_sol}")
        self.get_logger().info(f"Position error: {pos_err:.6f} m")
        self.get_logger().info(f"Rotation error: {rot_err:.6f} rad")
        self.get_logger().info("====================================\n")

        # set = self.arm.set_joint_positions(
        #     theta_sol.tolist(),
        #     moving_time=0.5,
        #     accel_time=0.2,
        # )
        # self.send_to_viperx(theta_sol)
        joint_cmd_msg = JointGroupCommand()
        joint_cmd_msg.name="arm"
        joint_cmd_msg.cmd = theta_sol.tolist()

        self.joint_command_setting.publish(joint_cmd_msg)
        

    



def main(args=None):
    rclpy.init(args=args)
    node = vx300s_ik_node()
    # rclpy.spin(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()





# self.joint_limits = {
#             "waist": {
#                 "mid": 3.1415926,
#                 "delta": 3.1415926,
#                 "min": -3.1415926,
#                 "max":  3.1415926,
#             },

#             "shoulder": {
#                 "mid": 2.845,
#                 "delta": 1.553,
#                 "min": 1.292,
#                 "max": 4.398,
#             },

#             "shoulder_shadow": {
#                 "mid": 2.845,
#                 "delta": 1.553,
#                 "min": 1.292,
#                 "max": 4.398,
#             },

#             "elbow": {
#                 "mid": 3.061,
#                 "delta": 1.685,
#                 "min": 1.376,
#                 "max": 4.746,
#             },

#             "elbow_shadow": {
#                 "mid": 3.061,
#                 "delta": 1.685,
#                 "min": 1.376,
#                 "max": 4.746,
#             },

#             "forearm_roll": {
#                 "mid": 3.1415926,
#                 "delta": 3.1415926,
#                 "min": -3.1415926,
#                 "max":  3.1415926,
#             },

#             "wrist_angle": {
#                 "mid": 3.324,
#                 "delta": 2.051,
#                 "min": 1.273,
#                 "max": 5.375,
#             },

#             "wrist_rotate": {
#                 "mid": 3.1415926,
#                 "delta": 3.1415926,
#                 "min": -3.1415926,
#                 "max":  3.1415926,
#             },

#             "gripper": {
#                 "mid": 3.1415926,
#                 "delta": 3.1415926,
#                 "min": -3.1415926,
#                 "max":  3.1415926,
#             },
#         }