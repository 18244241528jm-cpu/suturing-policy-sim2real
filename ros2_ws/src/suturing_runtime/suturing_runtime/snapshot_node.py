"""Latch one synchronized stereo initialization snapshot on operator request."""

from __future__ import annotations

import json
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .message_contract import clone_message, stamp_ns


class SnapshotNode(Node):
    def __init__(self) -> None:
        super().__init__("initialization_snapshot")
        defaults = {
            "left_image_topic": "/suturing/camera/left/image",
            "right_image_topic": "/suturing/camera/right/image",
            "left_info_topic": "/suturing/camera/left/camera_info",
            "right_info_topic": "/suturing/camera/right/camera_info",
            "max_pair_skew_s": 0.030,
            "max_image_age_s": 0.75,
            "buffer_size": 30,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.left = deque(maxlen=int(p("buffer_size")))
        self.right = deque(maxlen=int(p("buffer_size")))
        self.left_info = None
        self.right_info = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.left_pub = self.create_publisher(Image, "/suturing/initialization/left/image", latched)
        self.right_pub = self.create_publisher(Image, "/suturing/initialization/right/image", latched)
        self.left_info_pub = self.create_publisher(CameraInfo, "/suturing/initialization/left/camera_info", latched)
        self.right_info_pub = self.create_publisher(CameraInfo, "/suturing/initialization/right/camera_info", latched)
        self.meta_pub = self.create_publisher(String, "/suturing/initialization/snapshot", latched)
        self.create_subscription(Image, p("left_image_topic"), self.left.append, qos_profile_sensor_data)
        self.create_subscription(Image, p("right_image_topic"), self.right.append, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p("left_info_topic"), self._left_info, 5)
        self.create_subscription(CameraInfo, p("right_info_topic"), self._right_info, 5)
        self.create_service(Trigger, "/suturing/initialization/capture", self._capture)
        self.sequence = 0
        self.get_logger().info("D12_R1_READY call /suturing/initialization/capture")

    def _left_info(self, msg): self.left_info = clone_message(msg)
    def _right_info(self, msg): self.right_info = clone_message(msg)

    def _capture(self, request, response):
        del request
        if not self.left or not self.right or self.left_info is None or self.right_info is None:
            response.success = False
            response.message = "D12-E101-SNAPSHOT_INPUT_MISSING"
            return response
        newest_left = self.left[-1]
        left_ns = stamp_ns(newest_left.header)
        newest_right = min(self.right, key=lambda msg: abs(stamp_ns(msg.header) - left_ns))
        right_ns = stamp_ns(newest_right.header)
        skew_s = abs(left_ns - right_ns) * 1.0e-9
        if skew_s > float(self.get_parameter("max_pair_skew_s").value):
            response.success = False
            response.message = f"D12-E102-STEREO_SKEW skew_s={skew_s:.6f}"
            return response
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        age_s = abs(now_s - left_ns * 1.0e-9)
        if age_s > float(self.get_parameter("max_image_age_s").value):
            response.success = False
            response.message = f"D12-E103-SNAPSHOT_STALE age_s={age_s:.6f}"
            return response
        if not newest_left.header.frame_id or not newest_right.header.frame_id:
            response.success = False
            response.message = "D12-E104-EMPTY_CAMERA_FRAME"
            return response
        if int(self.left_info.width) != int(newest_left.width) or int(self.left_info.height) != int(newest_left.height):
            response.success = False
            response.message = "D12-E105-LEFT_INTRINSICS_SHAPE"
            return response
        self.sequence += 1
        init_id = f"init-{left_ns}-{self.sequence:04d}"
        self.left_pub.publish(clone_message(newest_left))
        self.right_pub.publish(clone_message(newest_right))
        self.left_info_pub.publish(clone_message(self.left_info))
        self.right_info_pub.publish(clone_message(self.right_info))
        metadata = {
            "schema": "suturing.snapshot.v1", "init_id": init_id,
            "left_stamp_ns": left_ns, "right_stamp_ns": right_ns,
            "left_frame": newest_left.header.frame_id, "right_frame": newest_right.header.frame_id,
            "width": int(newest_left.width), "height": int(newest_left.height),
            "stereo_skew_ms": skew_s * 1000.0, "capture_wall_time": time.time(),
        }
        out = String(); out.data = json.dumps(metadata, sort_keys=True); self.meta_pub.publish(out)
        response.success = True; response.message = json.dumps(metadata, sort_keys=True)
        return response


def main() -> None:
    rclpy.init(); node = SnapshotNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
