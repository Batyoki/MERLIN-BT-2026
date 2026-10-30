#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# ----------------- Utilities ----------------- #
def skew(vec):
    """Return the 3x3 skew-symmetric matrix of a 3-vector."""
    x, y, z = vec
    return np.array([[0.0, -z,  y],
                     [ z,  0.0, -x],
                     [-y,  x,  0.0]])

def matrix_exp6_from_screw(w, v, theta):
    """
    Exponential map of a twist (w, v) * theta.
    w: 3-vector (rotation axis)
    v: 3-vector
    theta: scalar
    Returns 4x4 transformation matrix.
    """
    eps = 1e-8
    w = np.asarray(w, dtype=float)
    v = np.asarray(v, dtype=float)
    w_norm = np.linalg.norm(w)

    if w_norm < eps:
        # Pure translation
        T = np.eye(4)
        T[0:3, 3] = v * theta
        return T

    # ensure w is a proper axis (can be unit or not)
    w_hat = skew(w / w_norm)
    theta_eff = theta * w_norm

    # Rodrigues' formula for rotation
    R = (np.eye(3) +
         np.sin(theta_eff) * w_hat +
         (1 - np.cos(theta_eff)) * (w_hat @ w_hat))

    # compute G (for p)
    G = (np.eye(3) * theta_eff +
         (1 - np.cos(theta_eff)) * w_hat +
         (theta_eff - np.sin(theta_eff)) * (w_hat @ w_hat))

    # note: if original w was not unit, we scaled theta to theta_eff, but v must be scaled accordingly:
    # For twist representation using w (not necessarily unit), formula works by using (w, v) and theta:
    # the formula above used theta_eff = theta * ||w||, so we need to pass v_scaled = v / ||w||
    p = (G @ (v / w_norm))

    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = p
    return T

def rot_to_quaternion(R):
    """Convert 3x3 rotation matrix to quaternion (x, y, z, w)."""
    # stable algorithm
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2  # S = 4*qw
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S
    else:
        # find the largest diagonal element
        if (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            qw = (R[2,1] - R[1,2]) / S
            qx = 0.25 * S
            qy = (R[0,1] + R[1,0]) / S
            qz = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            qw = (R[0,2] - R[2,0]) / S
            qx = (R[0,1] + R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            qw = (R[1,0] - R[0,1]) / S
            qx = (R[0,2] + R[2,0]) / S
            qy = (R[1,2] + R[2,1]) / S
            qz = 0.25 * S
    return (float(qx), float(qy), float(qz), float(qw))

# ----------------- Node ----------------- #
class RX150FK_POE_Params(Node):
    def __init__(self):
        super().__init__("rx150_fk")

        # subscriber: change topic name if your joint_states topic differs
        self.sub = self.create_subscription(
            JointState,
            "/rx150/joint_states",
            self.joint_callback,
            10
        )

        # publisher for end effector pose
        self.pub = self.create_publisher(Pose, "/rx150/end_effector_pose", 10)

        # ------- Use the M and Slist you provided ------- #
        # Home configuration M (from image)
        self.M = np.array([
            [1.0, 0.0, 0.0, 0.358575],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.25457],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=float)

        # The image showed a 5x6 matrix that is transposed (Slist^T). So each ROW in the image
        # corresponds to one 6-vector screw axis (i.e. columns of Slist after transposition).
        # Transcribing the rows (five rows, six columns) as in the image:
        rows = [
            [0.0, 0.0, 1.0,    0.0,      0.0,    0.0],
            [0.0, 1.0, 0.0, -0.10457,      0.0,    0.0],
            [0.0, 1.0, 0.0, -0.25457,      0.0,   0.05],
            [0.0, 1.0, 0.0, -0.25457,      0.0,    0.2],
            [1.0, 0.0, 0.0,    0.0,    0.25457,    0.0]
        ]
        # Slist is the transpose of the above (6 x 5)
        self.Slist = np.array(rows, dtype=float).T  # shape (6,5)

        self.get_logger().info("RX150 FK (PoE) node initialized using provided M and Slist.")

    def joint_callback(self, msg: JointState):
        # Expect joint order — don't ask again; use whatever names user publishes:
        # We will try to map five joint names commonly used. Change names if yours differ.
        expected_names = ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]
        # If the incoming message contains the exact names, map them; otherwise assume first 5 positions.
        try:
            indices = [msg.name.index(n) for n in expected_names]
            q = np.array([msg.position[i] for i in indices], dtype=float)
        except ValueError:
            # fallback: if names differ, take first five positions (best-effort)
            if len(msg.position) >= 5:
                q = np.array(msg.position[:5], dtype=float)
            else:
                self.get_logger().error("JointState does not contain at least 5 positions.")
                return

        # compute PoE: T = e^{[S1]q1} ... e^{[S5]q5} * M
        T = np.eye(4)
        for i in range(self.Slist.shape[1]):  # 5 joints
            Si = self.Slist[:, i]
            w = Si[0:3]
            v = Si[3:6]
            theta = float(q[i])
            expT = matrix_exp6_from_screw(w, v, theta)
            T = T @ expT

        T = T @ self.M

        # fill Pose
        pose = Pose()
        pose.position.x = float(T[0, 3])
        pose.position.y = float(T[1, 3])
        pose.position.z = float(T[2, 3])

        R = T[0:3, 0:3]
        qx, qy, qz, qw = rot_to_quaternion(R)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        self.pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = RX150FK_POE_Params()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
