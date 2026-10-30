#!/usr/bin/env python3
"""
Runs a TimestampedRecorder session (USB cameras + AprilTag calibration) until
the node is shut down. Recording runs in a background thread so the node keeps
spinning and Ctrl+C stops it cleanly (previously record() blocked __init__).
"""
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node

from data_collection_pkg.session import TimestampedRecorder


class SessionNode(Node):
    def __init__(self):
        super().__init__('session_node')
        self.declare_parameter('output_root', '/tmp/sessions')
        self.declare_parameter('marker_length_m', 0.05)
        self.declare_parameter('apriltag_family', 'tag36h11')
        self.declare_parameter('detect_tags', False)
        g = lambda n: self.get_parameter(n).value

        self.recorder = TimestampedRecorder(
            output_root=Path(g('output_root')),
            marker_length_m=g('marker_length_m'),
            apriltag_family=g('apriltag_family'),
            detect_tags=g('detect_tags'),
        )
        self.get_logger().info(f"Session recorder writing to {g('output_root')}")
        self._thread = threading.Thread(target=self._record, daemon=True)
        self._thread.start()

    def _record(self):
        try:
            self.recorder.record()
        except Exception as e:
            self.get_logger().error(f"Recording error: {e}")

    def destroy_node(self):
        self.recorder.stop()
        self._thread.join(timeout=10.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SessionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
