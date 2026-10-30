import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointGroupCommand
import numpy as np


import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
# from pink.tasks import FrameTask
from pink import FrameTask
from pink import Robot
from pink.utils import delta_pose

class BilateralKinematicsPink(Node):
    def __init__(self):
        super().__init__('bilateral_kinematics_pink')

        self.get_logger().info('Initialised node')

        # self.leader_robot=RobotWrapper.BuildFromURDF('~/Workspace/interbotix_ws/src/interbotix_ros_core/interbotix_xs_ros/urdf/reactorx150.urdf',pin.JointModelFreeFlyer())
        # self.follower_robot=RobotWrapper.BuildFromURDF('~/Workspace/interbotix_ws/src/interbotix_ros_core/interbotix_xs_ros/urdf/viperx300.urdf',pin.JointModelFreeFlyer())




def main(args=None):
    
    rclpy.init(args=args)
    node=BilateralKinematicsPink()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=='__main__':
    main()
