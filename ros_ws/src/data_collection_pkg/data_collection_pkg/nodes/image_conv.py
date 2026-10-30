"""sensor_msgs/Image <-> numpy (bgr8 / rgb8 / mono8) without cv_bridge."""

import numpy as np
from sensor_msgs.msg import Image


def to_msg(frame: np.ndarray, stamp, frame_id: str) -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = frame.shape[:2]
    if frame.ndim == 2:
        msg.encoding, msg.step = "mono8", msg.width
    else:
        msg.encoding, msg.step = "bgr8", msg.width * 3
    msg.data = np.ascontiguousarray(frame).tobytes()
    return msg


def from_msg(msg: Image) -> np.ndarray:
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == "mono8":
        return buf.reshape(msg.height, msg.step)[:, :msg.width].copy()
    if msg.encoding in ("bgr8", "rgb8"):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img[..., ::-1].copy() if msg.encoding == "rgb8" else img.copy()
    raise ValueError(f"Unsupported encoding {msg.encoding}")
