#!/usr/bin/env python3
"""
Writes every image received on `image` to <output_dir>/<frame_id>/<seq>.jpg
(async, FrameWriter) and one JSON line per frame (stamp, file) to frames.jsonl.
"""
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from data_collection_pkg.nodes.image_conv import from_msg
from data_collection_pkg.writer import FrameWriter, JsonlStore


class WriterNode(Node):
    def __init__(self):
        super().__init__('writer_node')
        self.declare_parameter('max_queue_size', 512)
        self.declare_parameter('output_dir', '/tmp/merlin_frames')
        self.out = Path(self.get_parameter('output_dir').value)
        self.writer = FrameWriter(max_queue=self.get_parameter('max_queue_size').value)
        self.index = JsonlStore(self.out / "frames.jsonl")
        self.count = 0
        self.create_subscription(Image, 'image', self._on_image, 30)
        self.get_logger().info(f"Writing {self.resolve_topic_name('image')} to {self.out}")

    def _on_image(self, msg: Image):
        cam = msg.header.frame_id or "camera"
        path = self.out / cam / f"{self.count:06d}.jpg"
        self.writer.write(path, from_msg(msg))
        self.index.append({"seq": self.count, "frame_id": cam, "file": str(path.relative_to(self.out)),
                           "stamp": msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec})
        self.count += 1

    def destroy_node(self):
        self.get_logger().info(f"Flushing {self.count} frames to disk...")
        self.writer.close()
        self.index.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WriterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
