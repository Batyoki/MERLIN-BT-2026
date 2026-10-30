#!/usr/bin/env python3
"""
AprilTag extrinsic calibration from an image topic.

Subscribes to `image` (sensor_msgs/Image), detects AprilTags with the given
intrinsics, publishes the camera pose in the world frame on `camera_pose`
(geometry_msgs/PoseStamped) and, after `samples` detections, saves the averaged
calibration (intrinsics + extrinsics) as JSON to `output_file`.

Intrinsics come from `intrinsics_file` (a JSON with "K"/"intrinsics" and
"dist"/"dist_coeffs", e.g. written by the MERLIN GUI calibration) or, if empty,
from a field-of-view guess (flagged in the saved file).
"""
import json
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

from data_collection_pkg.apriltag import AprilTagCalibrator, default_intrinsics
from data_collection_pkg.nodes.image_conv import from_msg
from data_collection_pkg.store import CameraCalibration


def _quat(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.declare_parameter('marker_length_m', 0.05)
        self.declare_parameter('apriltag_family', 'tag36h11')
        self.declare_parameter('intrinsics_file', '')
        self.declare_parameter('camera_id', 'camera')
        self.declare_parameter('device_index', -1)
        self.declare_parameter('samples', 20)
        self.declare_parameter('output_file', '/tmp/merlin_calibration/camera.json')
        g = lambda n: self.get_parameter(n).value

        self.K, self.dist, self.intrinsics_source = None, [0.0] * 5, 'fov_guess'
        if g('intrinsics_file'):
            data = json.loads(Path(g('intrinsics_file')).read_text())
            self.K = data.get('K') or data.get('intrinsics')
            self.dist = data.get('dist') or data.get('dist_coeffs') or self.dist
            self.intrinsics_source = g('intrinsics_file')
        self.marker, self.family = g('marker_length_m'), g('apriltag_family')
        self.camera_id, self.samples, self.output = g('camera_id'), g('samples'), Path(g('output_file'))
        self.device_index = g('device_index')
        self.poses = []
        self.last_detections = []
        self.pub = self.create_publisher(PoseStamped, 'camera_pose', 10)
        self.create_subscription(Image, 'image', self._on_image, 5)
        self.get_logger().info(f"Waiting for images on {self.resolve_topic_name('image')} "
                               f"(intrinsics: {self.intrinsics_source})")

    def _on_image(self, msg: Image):
        frame = from_msg(msg)
        if self.K is None:
            self.K = default_intrinsics(frame.shape[1], frame.shape[0])
        cal = AprilTagCalibrator(self.marker, self.family, self.K, self.dist)
        result = cal.detect(frame)
        if not result.detections:
            self.get_logger().info("No AprilTag in view", throttle_duration_sec=2.0)
            return
        T = AprilTagCalibrator.estimate_camera_to_world(result.detections)
        self.last_detections = result.detections
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'world'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, T[:3, 3])
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = map(float, _quat(T[:3, :3]))
        self.pub.publish(pose)
        self.poses.append(T)
        if len(self.poses) == self.samples:
            self._save(frame.shape[1], frame.shape[0])

    def _save(self, w, h):
        T = np.mean(self.poses, axis=0)
        U, _, Vt = np.linalg.svd(T[:3, :3])
        T[:3, :3] = U @ Vt
        calib = CameraCalibration(camera_id=self.camera_id, device_index=self.device_index,
                                  intrinsics=[list(map(float, r)) for r in self.K],
                                  dist_coeffs=list(map(float, self.dist)), extrinsics=T.tolist(),
                                  marker_length_m=self.marker, apriltag_family=self.family,
                                  tag_ids=sorted({d.tag_id for d in self.last_detections}))
        self.output.parent.mkdir(parents=True, exist_ok=True)
        calib.save(self.output)
        self.get_logger().info(f"Saved calibration from {self.samples} detections to {self.output} "
                               f"(camera at {np.round(T[:3, 3], 3).tolist()} m, intrinsics: {self.intrinsics_source})")


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
