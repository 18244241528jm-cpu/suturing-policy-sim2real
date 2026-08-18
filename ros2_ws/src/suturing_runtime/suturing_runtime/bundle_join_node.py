"""Join RGB, metric depth, mask and intrinsics by exact source-frame contract."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .contract import frame_contract_issues
from .message_contract import clone_message, image_contract, stamp_ns


class BundleJoinNode(Node):
    def __init__(self) -> None:
        super().__init__("fp_bundle_join")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.items = {}; self.emitted_stamp = None
        self.rgb_pub = self.create_publisher(Image, "/suturing/fp_input/rgb", latched)
        self.depth_pub = self.create_publisher(Image, "/suturing/fp_input/depth", latched)
        self.mask_pub = self.create_publisher(Image, "/suturing/fp_input/mask", latched)
        self.info_pub = self.create_publisher(CameraInfo, "/suturing/fp_input/camera_info", latched)
        self.ready_pub = self.create_publisher(String, "/suturing/fp_input/ready", latched)
        self.create_subscription(Image, "/suturing/initialization/left/image", lambda m:self._put("rgb",m), latched)
        self.create_subscription(CameraInfo, "/suturing/initialization/left/camera_info", lambda m:self._put("info",m), latched)
        self.create_subscription(Image, "/suturing/depth/metric", lambda m:self._put("depth",m), latched)
        self.create_subscription(Image, "/suturing/needle/mask", lambda m:self._put("mask",m), latched)
        self.get_logger().info("D12_R4_READY exact-stamp join enabled")

    def _put(self, key, msg):
        self.items[key] = clone_message(msg); self._try_emit()

    def _try_emit(self):
        if any(key not in self.items for key in ("rgb","info","depth","mask")): return
        rgb, info, depth, mask = (self.items[k] for k in ("rgb","info","depth","mask"))
        reference = image_contract(rgb)
        issues = frame_contract_issues(reference, image_contract(depth), ("32FC1",))
        issues += frame_contract_issues(reference, image_contract(mask), ("mono8","8UC1"))
        if stamp_ns(info.header) != reference.stamp_ns:
            issues.append("camera_info_stamp_mismatch")
        if int(info.width) != reference.width or int(info.height) != reference.height:
            issues.append("camera_info_shape_mismatch")
        if issues:
            self.get_logger().error(f"D12-E301-BUNDLE_CONTRACT {issues}"); return
        if self.emitted_stamp == reference.stamp_ns: return
        self.rgb_pub.publish(rgb); self.depth_pub.publish(depth); self.mask_pub.publish(mask); self.info_pub.publish(info)
        out = String(); out.data = json.dumps({"schema":"suturing.fp_input.v1",
            "state":"READY", "stamp_ns":reference.stamp_ns, "frame_id":reference.frame_id,
            "width":reference.width, "height":reference.height}, sort_keys=True)
        self.ready_pub.publish(out); self.emitted_stamp = reference.stamp_ns
        self.get_logger().info(f"D12_R4_BUNDLE_READY stamp_ns={reference.stamp_ns}")


def main() -> None:
    rclpy.init(); node = BundleJoinNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
