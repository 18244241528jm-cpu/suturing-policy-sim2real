"""Read-only runtime readiness and freshness supervisor."""

from __future__ import annotations

import json
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


class PipelineSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("pipeline_supervisor")
        topics = {"left_image": "/suturing/camera/left/image",
                  "right_image": "/suturing/camera/right/image",
                  "left_info": "/suturing/camera/left/camera_info",
                  "metric_depth": "/suturing/depth/metric",
                  "needle_mask": "/suturing/needle/mask",
                  "needle_pose": "/suturing/needle/pose_gated",
                  "psm_pose": "/suturing/psm1/measured_pose",
                  "approach_goal": "/suturing/approach/goal"}
        for name, value in topics.items(): self.declare_parameter(name, value)
        self.declare_parameter("max_sensor_age_s", 0.75)
        self.declare_parameter("require_depth", True)
        self.declare_parameter("require_mask", True)
        self.declare_parameter("require_needle_pose", True)
        self.declare_parameter("require_goal", True)
        get = lambda name: self.get_parameter(name).value
        self.last_receipt = {}; self.frames = {}
        self.status_pub = self.create_publisher(String, "/suturing/runtime/status", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/suturing/runtime/diagnostics", 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Image, get("left_image"), lambda m: self._seen("left_image", m), qos_profile_sensor_data)
        self.create_subscription(Image, get("right_image"), lambda m: self._seen("right_image", m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, get("left_info"), lambda m: self._seen("left_info", m), 5)
        self.create_subscription(Image, get("metric_depth"), lambda m: self._seen("metric_depth", m), qos_profile_sensor_data)
        self.create_subscription(Image, get("needle_mask"), lambda m: self._seen("needle_mask", m), qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, get("needle_pose"), lambda m: self._seen("needle_pose", m), 5)
        self.create_subscription(PoseStamped, get("psm_pose"), lambda m: self._seen("psm_pose", m), 10)
        self.create_subscription(PoseStamped, get("approach_goal"), lambda m: self._seen("approach_goal", m), latched)
        self.create_service(Trigger, "/suturing/runtime/check", self._check_service)
        self.create_timer(1.0, self._publish)
        self.get_logger().info("D10_SUPERVISOR_READY read-only")

    def _seen(self, key: str, msg) -> None:
        self.last_receipt[key] = time.monotonic()
        self.frames[key] = getattr(getattr(msg, "header", None), "frame_id", "")

    def _snapshot(self) -> dict:
        now = time.monotonic(); required = ["left_image", "right_image", "left_info", "psm_pose"]
        for parameter, key in (("require_depth", "metric_depth"), ("require_mask", "needle_mask"),
                               ("require_needle_pose", "needle_pose"), ("require_goal", "approach_goal")):
            if bool(self.get_parameter(parameter).value): required.append(key)
        max_age = float(self.get_parameter("max_sensor_age_s").value)
        ages = {key: (None if key not in self.last_receipt else now-self.last_receipt[key]) for key in required}
        missing = [key for key, age in ages.items() if age is None]
        # Frozen-goal deployment needs RGB and PSM feedback continuously. CameraInfo,
        # depth, mask, gated needle pose and goal are initialization/latch inputs.
        continuous = {"left_image", "right_image", "psm_pose"}
        stale = [key for key, age in ages.items()
                 if key in continuous and age is not None and age > max_age]
        ready = not missing and not stale
        return {"schema": "suturing_runtime.status.v1", "state": "READY_READ_ONLY" if ready else "WAITING_INPUTS",
                "ready": ready, "command_enabled": False, "max_sensor_age_s": max_age,
                "ages_s": ages, "missing": missing, "stale": stale, "frames": self.frames}

    def _publish(self) -> None:
        snapshot = self._snapshot(); text = String(); text.data = json.dumps(snapshot, sort_keys=True)
        self.status_pub.publish(text)
        diag = DiagnosticArray(); diag.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(); status.name = "suturing_runtime/readiness"; status.hardware_id = "topic_contract"
        status.level = DiagnosticStatus.OK if snapshot["ready"] else DiagnosticStatus.WARN
        status.message = snapshot["state"]
        status.values = [KeyValue(key="missing", value=",".join(snapshot["missing"])),
                         KeyValue(key="stale", value=",".join(snapshot["stale"]))]
        diag.status = [status]; self.diag_pub.publish(diag)

    def _check_service(self, request, response):
        del request; snapshot = self._snapshot(); response.success = bool(snapshot["ready"])
        response.message = json.dumps(snapshot, sort_keys=True); return response


def main() -> None:
    rclpy.init(); node = PipelineSupervisor()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
