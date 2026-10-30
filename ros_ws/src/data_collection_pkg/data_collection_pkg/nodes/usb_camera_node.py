#!/usr/bin/env python3
"""Publishes a USB camera as sensor_msgs/Image on <name>/image_raw at a fixed rate."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from data_collection_pkg.nodes.image_conv import to_msg
from data_collection_pkg.usb import CameraStream


class USBCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        self.declare_parameter('device_index', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('frame_id', 'camera')
        g = lambda n: self.get_parameter(n).value

        self.frame_id = g('frame_id')
        self.stream = CameraStream(index=g('device_index'), width=g('width'), height=g('height'))
        self.pub = self.create_publisher(Image, 'image_raw', 10)
        self.create_timer(1.0 / g('fps'), self._tick)
        self.get_logger().info(f"Camera {g('device_index')} -> {self.pub.topic_name} at {g('fps')} Hz")

    def _tick(self):
        ok, frame = self.stream.read()
        if not ok:
            self.get_logger().warn("Camera read failed", throttle_duration_sec=2.0)
            return
        self.pub.publish(to_msg(frame, self.get_clock().now().to_msg(), self.frame_id))

    def destroy_node(self):
        self.stream.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = USBCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
