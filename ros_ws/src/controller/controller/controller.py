"""
controller.py
-------------
ROS 2 node: bilateral teleoperation, RX200 master(s) -> VX300s slave(s).

For every side in the `sides` parameter (e.g. ["R"] or ["L", "R"]):

  subscribes   /<master_ns><side>/joint_states   (sensor_msgs/JointState)
               /<slave_ns><side>/joint_states
               /teleop/enable                    (std_msgs/Bool)
  publishes    /<master_ns><side>/commands/joint_group   current [mA], arm group
                                                         MUST be in 'current' mode
               /<slave_ns><side>/commands/joint_group    position [rad], arm group
                                                         in 'position' mode
               /<slave_ns><side>/commands/joint_single   gripper passthrough
               controller/teleop/diagnostics             (std_msgs/String, JSON, 10 Hz)

Every control tick is optionally appended to a CSV (log_dir parameter) with
the columns tools/evaluate_tracking.py expects.

Changes vs. the previous node (see docs/CODE_FIXES.md for the full list):
  * joints extracted BY NAME straight from the driver topics (the
    master_state_node / slave_state_node relays and their pop(3) are gone);
  * effort mA -> A, shadow motors, correct sign convention (arm_interface.py);
  * raw->URDF joint offsets (`*_joint_offset`) are separate from the
    master->slave reference mapping (`map_offset`), so gravity compensation is
    evaluated at the true master pose;
  * both sides run (the left pair was commented out);
  * no hard-coded /home/... paths; URDFs are installed with the package;
  * on stale data the master gets gravity-only torque and the slave holds,
    instead of silently skipping the cycle with the last current still applied.
"""

import csv
import json
import os
import time

import numpy as np
import rclpy
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from .arm_interface import RX200, VX300S, ArmInterface, extract_joints
from .dynamics import ARM_JOINTS, ArmModel, resolve_urdf
from .misa_mpc_controller import BilateralController, BilateralParams

RX200_GRIPPER_CLOSED, RX200_GRIPPER_OPEN = -0.9, 1.79      # <<USER: measure>>
VX300S_GRIPPER_CLOSED, VX300S_GRIPPER_OPEN = 0.037, -0.037


class Side:
    """Everything that belongs to one master/slave pair."""

    def __init__(self, node: "BilateralNode", side: str, params: BilateralParams):
        self.side = side
        self.node = node
        mns, sns = node.master_ns + side, node.slave_ns + side
        self.master_model = ArmModel(resolve_urdf("rx200"))
        self.slave_model = ArmModel(resolve_urdf("vx300s"))
        self.mi = ArmInterface("rx200", self.master_model, params.dt,
                               node.filter_alpha, estimator="none")
        self.si = ArmInterface("vx300s", self.slave_model, params.dt,
                               node.filter_alpha, estimator=node.slave_estimator,
                               observer_gain=node.observer_gain)
        self.ctrl = BilateralController(params, self.master_model, self.slave_model,
                                        VX300S["q_min"], VX300S["q_max"])
        self.master_msg = None
        self.slave_msg = None
        self.gripper_pos = 0.0

        cb = MutuallyExclusiveCallbackGroup()
        node.create_subscription(JointState, f"/{mns}/joint_states", self._master_cb, 10, callback_group=cb)
        node.create_subscription(JointState, f"/{sns}/joint_states", self._slave_cb, 10, callback_group=cb)
        self.pub_master = node.create_publisher(JointGroupCommand, f"/{mns}/commands/joint_group", 10)
        self.pub_slave = node.create_publisher(JointGroupCommand, f"/{sns}/commands/joint_group", 10)
        self.pub_grip = node.create_publisher(JointSingleCommand, f"/{sns}/commands/joint_single", 10)

        self.csv = None
        if node.log_dir:
            os.makedirs(node.log_dir, exist_ok=True)
            path = os.path.join(node.log_dir, f"bilateral_{side}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
            self.csv_file = open(path, "w", newline="")
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(["t"] + [f"q_m_{j}" for j in ARM_JOINTS] + [f"q_ref_{j}" for j in ARM_JOINTS]
                              + [f"q_s_{j}" for j in ARM_JOINTS] + [f"q_cmd_{j}" for j in ARM_JOINTS]
                              + [f"tau_m_{j}" for j in ARM_JOINTS] + [f"tau_ext_s_{j}" for j in ARM_JOINTS]
                              + [f"tau_f_{j}" for j in ARM_JOINTS] + ["tank", "gamma", "mpc_ms"])
            node.get_logger().info(f"[{side}] logging to {path}")

    def _master_cb(self, msg: JointState):
        self.master_msg = msg
        if "gripper" in msg.name:
            self.gripper_pos = msg.position[list(msg.name).index("gripper")]

    def _slave_cb(self, msg: JointState):
        self.slave_msg = msg

    @staticmethod
    def _stamp(msg: JointState) -> float:
        return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec

    def _read(self, msg: JointState, offset: np.ndarray, interface: ArmInterface):
        q = extract_joints(msg.name, msg.position)
        effort = extract_joints(msg.name, msg.effort)
        if q is None or effort is None:
            return None
        dq = extract_joints(msg.name, msg.velocity) if self.node.use_driver_velocity else None
        return interface.update(q + offset, effort, self._stamp(msg), dq)

    def tick(self, now: float):
        node = self.node
        fresh = all(m is not None and now - self._stamp(m) < node.timeout
                    for m in (self.master_msg, self.slave_msg))
        if not fresh or not node.enabled:
            if self.master_msg is not None:      # hold: gravity only on master
                q = extract_joints(self.master_msg.name, self.master_msg.position)
                if q is not None:
                    self._send_master(self.master_model.gravity(q + node.master_offset))
            self.ctrl.reset()
            return

        ms = self._read(self.master_msg, node.master_offset, self.mi)
        ss = self._read(self.slave_msg, node.slave_offset, self.si)
        if ms is None or ss is None:
            node.get_logger().warn(f"[{self.side}] joint_states missing arm joints/effort",
                                   throttle_duration_sec=2.0)
            return
        tau_m, q_cmd, diag = self.ctrl.step(ms, ss)
        self._send_master(tau_m)
        self._send_slave(q_cmd - node.slave_offset)
        if self.csv:
            self.csv.writerow([now, *ms.q, *diag["q_ref"], *ss.q, *q_cmd, *tau_m, *ss.tau_ext,
                               *diag["tau_f"], diag["tank"], diag["gamma"], diag["mpc_ms"]])
        return diag

    def _send_master(self, tau: np.ndarray):
        tau = np.clip(tau, -self.ctrl.p.tau_master_max, self.ctrl.p.tau_master_max)
        current_mA = tau / (RX200["kt"] * RX200["n_motors"]) * 1e3
        self.pub_master.publish(JointGroupCommand(name="arm", cmd=current_mA.tolist()))

    def _send_slave(self, q_cmd: np.ndarray):
        cmd = list(q_cmd)
        if self.node.slave_has_forearm_roll:
            cmd.insert(3, self.node.forearm_roll_angle)
        self.pub_slave.publish(JointGroupCommand(name="arm", cmd=[float(c) for c in cmd]))
        g = float(np.interp(self.gripper_pos, [RX200_GRIPPER_CLOSED, RX200_GRIPPER_OPEN],
                            [VX300S_GRIPPER_CLOSED, VX300S_GRIPPER_OPEN]))
        self.pub_grip.publish(JointSingleCommand(name="gripper", cmd=g))


class BilateralNode(Node):
    def __init__(self):
        super().__init__("controller")
        P = self.declare_parameter
        P("control_mode", "misa")                  # unilateral | pp | pf | misa
        P("sides", ["R"])
        P("master_ns", "master")
        P("slave_ns", "slave")
        P("loop_rate_hz", 50.0)
        P("loop_rate_override_hz", 0.0)            # > 0 overrides loop_rate_hz (GUI field)
        P("state_timeout_s", 0.1)
        P("filter_alpha", 0.3)
        P("use_driver_velocity", False)
        P("slave_estimator", "momentum")           # momentum | static | none
        P("observer_gain", 15.0)
        P("slave_has_forearm_roll", True)
        P("forearm_roll_angle", 0.0)
        P("master_joint_offset", [0.0] * 5)        # raw -> URDF convention
        P("slave_joint_offset", [0.0] * 5)
        P("map_scale", [1.0] * 5)                  # master -> slave reference
        P("map_offset", [0.0] * 5)
        P("log_dir", "")
        defaults = BilateralParams()
        for name in ("Dmd", "Kmd", "tau_master_max", "tau_dead_zone", "pp_Kp", "pp_Kd",
                     "Msd", "Dsd", "Ksd", "q_e_max", "pos_loop_omega", "pos_loop_zeta",
                     "dq_slave_max", "du_slave_max"):
            P(name, getattr(defaults, name).tolist())
        for name in ("kof", "kom", "s_force", "mpc_Q", "mpc_P", "mpc_R", "mpc_rho",
                     "tank_T0", "tank_T_min", "tank_T_max"):
            P(name, float(getattr(defaults, name)))
        P("mpc_horizon", defaults.mpc_horizon)
        P("mpc_iters", defaults.mpc_iters)
        P("use_mpc", True)
        P("use_tank", True)
        P("force_map", "jacobian")

        def g(n):
            return self.get_parameter(n).value

        def arr(n):
            return np.array(g(n), dtype=float)

        self.master_ns, self.slave_ns = g("master_ns"), g("slave_ns")
        self.timeout = g("state_timeout_s")
        self.filter_alpha = g("filter_alpha")
        self.use_driver_velocity = g("use_driver_velocity")
        self.slave_estimator = g("slave_estimator")
        self.observer_gain = g("observer_gain")
        self.slave_has_forearm_roll = g("slave_has_forearm_roll")
        self.forearm_roll_angle = g("forearm_roll_angle")
        self.master_offset, self.slave_offset = arr("master_joint_offset"), arr("slave_joint_offset")
        self.log_dir = g("log_dir")
        self.enabled = True

        rate = g("loop_rate_override_hz") if g("loop_rate_override_hz") > 0 else g("loop_rate_hz")
        dt = 1.0 / rate
        params = BilateralParams(
            mode=g("control_mode"), dt=dt,
            map_scale=arr("map_scale"), map_offset=arr("map_offset"),
            **{n: arr(n) for n in ("Dmd", "Kmd", "tau_master_max", "tau_dead_zone", "pp_Kp", "pp_Kd",
                                   "Msd", "Dsd", "Ksd", "q_e_max", "pos_loop_omega", "pos_loop_zeta",
                                   "dq_slave_max", "du_slave_max")},
            **{n: float(g(n)) for n in ("kof", "kom", "s_force", "mpc_Q", "mpc_P", "mpc_R", "mpc_rho",
                                        "tank_T0", "tank_T_min", "tank_T_max")},
            mpc_horizon=int(g("mpc_horizon")), mpc_iters=int(g("mpc_iters")), use_mpc=g("use_mpc"),
            use_tank=g("use_tank"), force_map=g("force_map"))

        self.sides = [Side(self, s, params) for s in g("sides")]
        self.create_subscription(Bool, "/teleop/enable", self._enable_cb, 10)
        self.diag_pub = self.create_publisher(String, "controller/teleop/diagnostics", 10)
        self._loop_ms = []
        self._last_diag = {}
        self.create_timer(dt, self._loop, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.1, self._publish_diag)
        self.get_logger().info(f"Bilateral controller: mode={params.mode}, sides={g('sides')}, "
                               f"dt={dt * 1e3:.0f} ms, estimator={self.slave_estimator}")

    def _enable_cb(self, msg: Bool):
        self.enabled = msg.data
        self.get_logger().info(f"Teleoperation {'enabled' if msg.data else 'disabled'}")

    def _loop(self):
        t0 = time.perf_counter()
        now = self.get_clock().now().nanoseconds * 1e-9
        for s in self.sides:
            d = s.tick(now)
            if d:
                self._last_diag[s.side] = d
        self._loop_ms.append((time.perf_counter() - t0) * 1e3)
        self._loop_ms = self._loop_ms[-200:]

    def _publish_diag(self):
        if not self._loop_ms:
            return
        info = dict(enabled=self.enabled, loop_ms_mean=float(np.mean(self._loop_ms)),
                    loop_ms_max=float(np.max(self._loop_ms)))
        for side, d in self._last_diag.items():
            info[side] = dict(tank=float(d["tank"]), gamma=float(d["gamma"]), mpc_ms=float(d["mpc_ms"]),
                              tau_f=np.round(d["tau_f"], 3).tolist())
        self.diag_pub.publish(String(data=json.dumps(info)))

    def destroy_node(self):
        for s in self.sides:
            if s.csv:
                s.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BilateralNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
