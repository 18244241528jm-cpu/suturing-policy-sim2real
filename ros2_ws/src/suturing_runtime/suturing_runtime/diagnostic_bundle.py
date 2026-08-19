"""Read-only, stage-aware evidence bundle for real dVRK/ECM troubleshooting."""

from __future__ import annotations

import hashlib
import json
import os
import platform
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from .message_contract import clone_message, stamp_ns


LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

IMAGE_TOPICS = {
    "/suturing/camera/left/image": False,
    "/suturing/camera/right/image": False,
    "/suturing/initialization/left/image": True,
    "/suturing/initialization/right/image": True,
    "/suturing/external/depth": True,
    "/suturing/depth/metric": True,
    "/suturing/external/needle_mask": True,
    "/suturing/needle/mask": True,
}

STRING_TOPICS = [
    "/suturing/adapter/status",
    "/suturing/initialization/snapshot",
    "/suturing/da/status",
    "/suturing/operator_mask/status",
    "/suturing/perception_input/status",
    "/suturing/fp_input/ready",
    "/suturing/fp_candidate/status",
    "/suturing/external/needle_candidates",
    "/suturing/needle/candidates",
    "/suturing/needle/gate_status",
    "/suturing/runtime/status",
    "/suturing/execution/status",
]

RAW_SAFETY_TOPICS = [
    "/PSM1/operating_state",
    "/PSM1/goal_reached",
    "/PSM1/error",
    "/PSM1/warning",
    "/ECM/operating_state",
    "/ECM/error",
    "/ECM/warning",
]


def safe_name(topic: str) -> str:
    return topic.strip("/").replace("/", "__") or "root"


def header_dict(message) -> dict:
    header = getattr(message, "header", None)
    if header is None:
        return {}
    return {"stamp_ns": stamp_ns(header), "frame_id": str(header.frame_id)}


class DiagnosticBundle(Node):
    def __init__(self) -> None:
        super().__init__("suturing_diagnostic_bundle")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("duration_s", 20.0)
        self.declare_parameter("trigger_capture", True)
        self.declare_parameter("capture_delay_s", 2.0)
        requested = str(self.get_parameter("output_dir").value)
        if requested:
            self.output = Path(requested).expanduser()
        else:
            self.output = Path.home() / "surgicai_diagnostics" / time.strftime("%Y%m%d_%H%M%S")
        self.output.mkdir(parents=True, exist_ok=True)
        self.started = time.monotonic()
        self.done = False
        self.capture_requested = False
        self.capture_result: dict = {"requested": False}
        self.records: dict[str, dict] = {}
        self.latest: dict[str, object] = {}
        self.string_values: dict[str, object] = {}
        self.capture_client = self.create_client(Trigger, "/suturing/initialization/capture")

        for topic, latched in IMAGE_TOPICS.items():
            self.create_subscription(Image, topic, lambda msg, t=topic: self._message(t, msg),
                                     LATCHED if latched else qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/suturing/camera/left/camera_info",
                                 lambda msg: self._message("/suturing/camera/left/camera_info", msg),
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/suturing/psm1/measured_pose",
                                 lambda msg: self._message("/suturing/psm1/measured_pose", msg), 10)
        self.create_subscription(TwistStamped, "/suturing/psm1/measured_twist",
                                 lambda msg: self._message("/suturing/psm1/measured_twist", msg), 10)
        self.create_subscription(JointState, "/suturing/psm1/jaw/measured_js",
                                 lambda msg: self._message("/suturing/psm1/jaw/measured_js", msg), 10)
        for topic in ["/suturing/approach/goal_camera", "/suturing/approach/goal"]:
            self.create_subscription(PoseStamped, topic,
                                     lambda msg, t=topic: self._message(t, msg), LATCHED)
        self.create_subscription(PoseStamped, "/suturing/execution/preview",
                                 lambda msg: self._message("/suturing/execution/preview", msg), 10)
        for topic in ["/suturing/needle/pose_pending", "/suturing/needle/pose_gated"]:
            self.create_subscription(PoseWithCovarianceStamped, topic,
                                     lambda msg, t=topic: self._message(t, msg), LATCHED)
        volatile_strings = {"/suturing/adapter/status", "/suturing/runtime/status",
                            "/suturing/execution/status"}
        for topic in STRING_TOPICS:
            self.create_subscription(String, topic,
                                     lambda msg, t=topic: self._string(t, msg),
                                     10 if topic in volatile_strings else LATCHED)
        self.create_subscription(TFMessage, "/tf", lambda msg: self._tf("/tf", msg), 50)
        self.create_subscription(TFMessage, "/tf_static", lambda msg: self._tf("/tf_static", msg), LATCHED)
        self.create_timer(0.2, self._tick)
        self.get_logger().info(f"D15_DIAGNOSTICS_READ_ONLY output={self.output}")

    def _record(self, topic: str, message) -> None:
        now = time.monotonic()
        record = self.records.setdefault(topic, {
            "count": 0, "first_receipt_s": now - self.started, "last_receipt_s": 0.0,
            "message_type": f"{message.__class__.__module__}.{message.__class__.__name__}",
        })
        record["count"] += 1
        record["last_receipt_s"] = now - self.started
        record.update(header_dict(message))

    def _message(self, topic: str, message) -> None:
        self._record(topic, message)
        self.latest[topic] = clone_message(message)

    def _string(self, topic: str, message: String) -> None:
        self._record(topic, message)
        try:
            value = json.loads(message.data)
        except Exception:
            value = {"raw": message.data}
        self.string_values[topic] = value
        self.latest[topic] = clone_message(message)

    def _tf(self, topic: str, message: TFMessage) -> None:
        self._record(topic, message)
        transforms = self.latest.setdefault(topic, {})
        for transform in message.transforms:
            key = f"{transform.header.frame_id}->{transform.child_frame_id}"
            transforms[key] = {
                **header_dict(transform),
                "child_frame_id": str(transform.child_frame_id),
                "translation": [float(transform.transform.translation.x),
                                float(transform.transform.translation.y),
                                float(transform.transform.translation.z)],
                "quaternion_xyzw": [float(transform.transform.rotation.x),
                                    float(transform.transform.rotation.y),
                                    float(transform.transform.rotation.z),
                                    float(transform.transform.rotation.w)],
            }

    def _tick(self) -> None:
        elapsed = time.monotonic() - self.started
        if (bool(self.get_parameter("trigger_capture").value) and not self.capture_requested
                and elapsed >= float(self.get_parameter("capture_delay_s").value)):
            self.capture_requested = True
            self.capture_result = {"requested": True, "service_available": False}
            if self.capture_client.service_is_ready():
                self.capture_result["service_available"] = True
                future = self.capture_client.call_async(Trigger.Request())
                future.add_done_callback(self._capture_done)
        if elapsed >= float(self.get_parameter("duration_s").value) and not self.done:
            self._finalize()
            self.done = True

    def _capture_done(self, future) -> None:
        try:
            response = future.result()
            self.capture_result.update({"success": bool(response.success), "message": response.message})
        except Exception as exc:
            self.capture_result.update({"success": False, "exception": str(exc)})

    @staticmethod
    def _run(command: list[str], timeout_s: float = 8.0) -> dict:
        try:
            result = subprocess.run(
                command, capture_output=True, text=True, timeout=timeout_s, check=False
            )
            return {"command": command, "returncode": result.returncode,
                    "stdout": result.stdout[-20000:], "stderr": result.stderr[-10000:]}
        except Exception as exc:
            return {"command": command, "exception": str(exc)}

    def _save_image(self, topic: str, message: Image) -> dict:
        stem = safe_name(topic)
        metadata = {**header_dict(message), "height": int(message.height), "width": int(message.width),
                    "encoding": str(message.encoding), "step": int(message.step)}
        if message.encoding in ("rgb8", "bgr8"):
            row = np.frombuffer(message.data, np.uint8).reshape(int(message.height), int(message.step))
            image = row[:, : int(message.width) * 3].reshape(int(message.height), int(message.width), 3)
            bgr = image[:, :, ::-1] if message.encoding == "rgb8" else image
            path = self.output / f"{stem}.png"
            cv2.imwrite(str(path), bgr)
            metadata["saved"] = path.name
        elif message.encoding == "32FC1":
            row = np.frombuffer(message.data, dtype="<f4").reshape(int(message.height), int(message.step) // 4)
            depth = row[:, : int(message.width)].copy()
            npy = self.output / f"{stem}.npy"
            np.save(npy, depth)
            finite = depth[np.isfinite(depth)]
            metadata.update({"saved": npy.name, "finite_fraction": float(np.isfinite(depth).mean()),
                             "minimum_m": float(finite.min()) if finite.size else None,
                             "maximum_m": float(finite.max()) if finite.size else None})
            if finite.size:
                low, high = np.percentile(finite, [1, 99])
                scaled = np.clip((depth - low) / max(high - low, 1e-9), 0, 1)
                color = cv2.applyColorMap((scaled * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
                preview = self.output / f"{stem}_preview.png"
                cv2.imwrite(str(preview), color)
                metadata["preview"] = preview.name
        elif message.encoding in ("mono8", "8UC1"):
            row = np.frombuffer(message.data, np.uint8).reshape(int(message.height), int(message.step))
            mask = row[:, : int(message.width)].copy()
            path = self.output / f"{stem}.png"
            cv2.imwrite(str(path), mask)
            metadata.update({"saved": path.name,
                             "nonzero_fraction": float(np.count_nonzero(mask)) / mask.size})
        else:
            metadata["save_error"] = f"unsupported encoding {message.encoding!r}"
        return metadata

    @staticmethod
    def _pose(message) -> dict:
        pose = message.pose.pose if isinstance(message, PoseWithCovarianceStamped) else message.pose
        return {**header_dict(message),
                "position_m": [float(pose.position.x), float(pose.position.y),
                               float(pose.position.z)],
                "quaternion_xyzw": [float(pose.orientation.x), float(pose.orientation.y),
                                    float(pose.orientation.z), float(pose.orientation.w)],
                "covariance": list(message.pose.covariance)
                if isinstance(message, PoseWithCovarianceStamped) else None}

    def _stages(self) -> list[dict]:
        checks = [
            ("R0_INPUT_ADAPTER", ["/suturing/camera/left/image", "/suturing/camera/right/image",
                                  "/suturing/camera/left/camera_info", "/suturing/psm1/measured_pose"],
             "Run topic_preflight; verify raw topic types, messages, and frame IDs."),
            ("R1_SNAPSHOT", ["/suturing/initialization/left/image"],
             "Call /suturing/initialization/capture and inspect D12-E101..E105."),
            ("R2_METRIC_DA", ["/suturing/external/depth", "/suturing/depth/metric"],
             "Enable metric_da_depth; verify checkpoint/repo/CUDA and D13-E201..E210."),
            ("R3_NEEDLE_MASK", ["/suturing/external/needle_mask", "/suturing/needle/mask"],
             "Export/draw/import the exact source-frame mask, then review mask_overlay.png."),
            ("R4_FP_BUNDLE", ["/suturing/fp_input/ready"],
             "Compare RGB/K/depth/mask source identities; inspect D12-E301."),
            ("R5_FP_CANDIDATES", ["/suturing/needle/candidates"],
             "Start the FoundationPose bridge and inspect D12-E402/E403."),
            ("R6_NEEDLE_GATE", ["/suturing/needle/pose_gated"],
             "Configure real plane/rest normals, review overlay, then confirm pending pose."),
            ("R7_CAMERA_BASE_GOAL", ["/suturing/approach/goal_camera", "/suturing/approach/goal"],
             "Publish verified hand-eye TF and confirm required frames; inspect D12-E701..E705."),
            ("R8_READINESS", ["/suturing/runtime/status"],
             "Read runtime missing/stale fields; READY_READ_ONLY is not motion authority."),
            ("R9_EXECUTION", ["/suturing/execution/status"],
             "Do not enable motion from diagnostics; validate guarded preview under supervision."),
        ]
        result = []
        blocked = False
        for name, topics, action in checks:
            missing = [topic for topic in topics if self.records.get(topic, {}).get("count", 0) == 0]
            if blocked:
                state = "BLOCKED_BY_EARLIER_STAGE"
            elif not missing:
                state = "OBSERVED"
            else:
                state = "FIRST_UNRESOLVED_STAGE"
                blocked = True
            result.append({"stage": name, "state": state, "required_topics": topics,
                           "missing_topics": missing, "next_action": action})
        return result

    def _finalize(self) -> None:
        graph = {name: types for name, types in self.get_topic_names_and_types()}
        with ThreadPoolExecutor(max_workers=len(RAW_SAFETY_TOPICS)) as pool:
            futures = {
                topic: pool.submit(
                    self._run, ["ros2", "topic", "echo", topic, "--once"], 4.0
                )
                for topic in RAW_SAFETY_TOPICS
            }
            raw_safety = {topic: future.result() for topic, future in futures.items()}
        saved = {}
        for topic, message in self.latest.items():
            if isinstance(message, Image):
                saved[topic] = self._save_image(topic, message)
            elif isinstance(message, CameraInfo):
                saved[topic] = {**header_dict(message), "height": int(message.height),
                                "width": int(message.width), "distortion_model": message.distortion_model,
                                "d": list(message.d), "k": list(message.k), "r": list(message.r),
                                "p": list(message.p)}
            elif isinstance(message, (PoseStamped, PoseWithCovarianceStamped)):
                saved[topic] = self._pose(message)
            elif isinstance(message, TwistStamped):
                saved[topic] = {**header_dict(message),
                    "linear": [message.twist.linear.x, message.twist.linear.y, message.twist.linear.z],
                    "angular": [message.twist.angular.x, message.twist.angular.y, message.twist.angular.z]}
            elif isinstance(message, JointState):
                saved[topic] = {**header_dict(message), "name": list(message.name),
                                "position": list(message.position), "velocity": list(message.velocity),
                                "effort": list(message.effort)}
            elif topic in ("/tf", "/tf_static"):
                saved[topic] = message
        stages = self._stages()
        first = next((item for item in stages if item["state"] == "FIRST_UNRESOLVED_STAGE"), None)
        summary = {
            "schema": "suturing.real_diagnostic_bundle.v1",
            "read_only": True,
            "motion_commands_published": 0,
            "duration_s": time.monotonic() - self.started,
            "capture": self.capture_result,
            "first_unresolved_stage": first,
            "stages": stages,
            "records": self.records,
            "saved_messages": saved,
            "string_messages": self.string_values,
            "topic_graph": graph,
            "raw_safety_topic_snapshots": raw_safety,
            "motion_safety_gap": (
                "Diagnostic evidence is collected, but guarded_pose_executor does not yet interlock "
                "dVRK operating_state/error/warning/goal_reached. Do not enable real output until "
                "that contract is implemented and supervised on hardware."
            ),
            "environment": {
                "python": platform.python_version(), "platform": platform.platform(),
                "ros_distro": os.environ.get("ROS_DISTRO", ""),
                "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
                "nvidia_smi": self._run(["nvidia-smi", "--query-gpu=name,memory.total,driver_version",
                                         "--format=csv,noheader"]),
                "ros2_doctor": self._run(["ros2", "doctor", "--report"]),
                "ros2_nodes": self._run(["ros2", "node", "list"]),
            },
            "claim_boundary": "This bundle reports available interfaces and staged evidence. It does not validate hand-eye, mask semantics, FP correctness, or motion safety by itself.",
        }
        (self.output / "SUMMARY.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True, default=float), encoding="utf-8"
        )
        lines = ["# SHARE THIS FILE FIRST", "", "This collection is read-only: 0 motion commands.", "",
                 f"Output: `{self.output}`", "", "## First unresolved stage", ""]
        if first:
            lines.extend([f"- Stage: `{first['stage']}`", f"- Missing: `{first['missing_topics']}`",
                          f"- Next action: {first['next_action']}"])
        else:
            lines.append("- All observable R0-R9 topic gates produced at least one message. This is not a safety acceptance.")
        lines.extend(["", "## Stage table", "", "| Stage | State | Missing |", "|---|---|---|"])
        for item in stages:
            lines.append(f"| {item['stage']} | {item['state']} | {', '.join(item['missing_topics']) or '—'} |")
        lines.extend(["", "## Send for analysis", "",
                      "Send this file plus `SUMMARY.json`. If perception is involved, also send the saved PNG/NPY files and runtime log.",
                      "Do not send only a terminal screenshot; it omits topic identities and upstream blockers."])
        (self.output / "SHARE_THIS_FIRST.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
        hashes = []
        for path in sorted(self.output.iterdir()):
            if path.is_file() and path.name != "MANIFEST.sha256":
                digest = hashlib.sha256(path.read_bytes()).hexdigest()
                hashes.append(f"{digest}  {path.name}")
        (self.output / "MANIFEST.sha256").write_text("\n".join(hashes) + "\n", encoding="utf-8")
        self.get_logger().info(
            f"D15_DIAGNOSTICS_COMPLETE first_unresolved={first['stage'] if first else 'none'} output={self.output}"
        )


def main() -> None:
    rclpy.init()
    node = DiagnosticBundle()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        if not node.done:
            node._finalize()
            node.done = True
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
