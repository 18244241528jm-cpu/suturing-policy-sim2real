"""Validate external DA/mask outputs before admitting them to the real pipeline."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .contract import frame_contract_issues
from .message_contract import clone_message, image_contract


class PerceptionInputAdapter(Node):
    def __init__(self) -> None:
        super().__init__("perception_input_adapter")
        self.declare_parameter("external_depth_topic", "/suturing/external/depth")
        self.declare_parameter("external_mask_topic", "/suturing/external/needle_mask")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.depth_pub = self.create_publisher(Image, "/suturing/depth/metric", latched)
        self.mask_pub = self.create_publisher(Image, "/suturing/needle/mask", latched)
        self.status_pub = self.create_publisher(String, "/suturing/perception_input/status", latched)
        self.reference = None
        self.create_subscription(Image, "/suturing/initialization/left/image", self._rgb, latched)
        self.create_subscription(Image, self.get_parameter("external_depth_topic").value, self._depth, latched)
        self.create_subscription(Image, self.get_parameter("external_mask_topic").value, self._mask, latched)
        self.get_logger().info("D12_R2_R3_READY waiting for external metric depth and mask")

    def _status(self, state: str, **fields) -> None:
        out = String(); out.data = json.dumps({"schema": "suturing.perception_input.v1",
            "state": state, **fields}, sort_keys=True); self.status_pub.publish(out)

    def _rgb(self, msg: Image) -> None:
        self.reference = image_contract(msg)
        self._status("WAITING_EXTERNAL_PRODUCTS", stamp_ns=self.reference.stamp_ns)

    def _validate(self, msg: Image, allowed: tuple[str, ...], label: str) -> bool:
        if self.reference is None:
            self._status("REJECTED", code="D12-E201-NO_SNAPSHOT", product=label); return False
        issues = frame_contract_issues(self.reference, image_contract(msg), allowed)
        if issues:
            self._status("REJECTED", code="D12-E202-CONTRACT_MISMATCH", product=label, issues=issues)
            self.get_logger().error(f"D12-E202-CONTRACT_MISMATCH {label}: {issues}")
            return False
        return True

    def _depth(self, msg: Image) -> None:
        if self._validate(msg, ("32FC1",), "depth"):
            self.depth_pub.publish(clone_message(msg)); self._status("DEPTH_ACCEPTED", stamp_ns=self.reference.stamp_ns)

    def _mask(self, msg: Image) -> None:
        if self._validate(msg, ("mono8", "8UC1"), "mask"):
            self.mask_pub.publish(clone_message(msg)); self._status("MASK_ACCEPTED", stamp_ns=self.reference.stamp_ns)


def main() -> None:
    rclpy.init(); node = PerceptionInputAdapter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
