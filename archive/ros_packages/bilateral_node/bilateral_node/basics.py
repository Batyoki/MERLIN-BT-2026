import rclpy
import mink
from rclpy.node import Node
from sensor_msgs.msg import JointState



class bilateral_basics_mapping(Node):
    def __init__(self):
        super().__init__('bilateral_basics_mapping')
        self.get_logger().info('Started Bilateral Node')
        self.declare_parameter('leader','/rx150')
        self.declare_parameter('follower','/vx300s')


        self.leader_ns=self.get_parameter('leader').value
        self.follower_ns = self.get_parameter('follower').value


        # self.get_namespace('vx300s')
        self.leader_subscription=self.create_subscription(
            JointState,
            f'{self.leader_ns}/joint_states',
            self.joint_state_callback,
            10
        )

        self.follower_publisher=self.create_publisher(
            JointState,
            f'{self.follower_ns}/joint_states',
            10
        )



    def joint_state_callback(self,msg:JointState):
        self.get_logger().info(f'{msg.name}')
        self.get_logger().info(f'Positions : {msg.position}')

        self.follower_publisher.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = bilateral_basics_mapping()
    rclpy.spin(node=node)
    rclpy.shutdown()

        
if __name__=="__main__":
    main()