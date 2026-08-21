"""Real PSM pose interface: vision-only, FK-only, or uncertainty fusion."""

from __future__ import annotations

from collections import deque
import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .contract import matrix_to_quaternion_xyzw, pose_matrix
from .message_contract import stamp_ns
from .pose_selection import motion_compensate_pose, select_psm_candidate


MODES = {"disabled", "vision_only", "fk_only", "fused"}


class PSMPoseSelector(Node):
    """Publish one camera-frame PSM control-point pose without hidden fallback.

    Vision candidates normally describe a CAD mesh frame, while dVRK
    ``measured_cp`` describes the control point.  Vision and fusion modes are
    blocked until the fixed ``T_mesh_from_control_point`` is measured.  FK and
    fusion modes are blocked until ``psm_camera_bridge`` supplies a pose using
    a real hand-eye TF.
    """

    def __init__(self) -> None:
        super().__init__("psm_pose_selector")
        defaults = {
            "selection_mode": "disabled",
            "vision_candidates_topic": "/suturing/external/psm_candidates",
            "fk_camera_pose_topic": "/suturing/psm1/measured_pose_camera",
            "selected_pose_topic": "/suturing/psm1/pose_selected_camera",
            "vision_mesh_pose_topic": "/suturing/psm1/vision_mesh_pose_camera",
            "status_topic": "/suturing/psm1/pose_selector_status",
            "vision_mesh_frame": "",
            "mesh_to_control_point_configured": False,
            "mesh_from_control_point_translation_m": [0.0, 0.0, 0.0],
            "mesh_from_control_point_quaternion_xyzw": [0.0, 0.0, 0.0, 0.0],
            "fusion_uncertainty_configured": False,
            # These zero vectors are intentionally invalid placeholders.
            "kinematic_sigma6_m_deg": [0.0] * 6,
            "vision_sigma6_m_deg": [0.0] * 6,
            "fp_rank_weight": 0.0,
            "max_sync_skew_s": 0.050,
            "max_visual_age_s": 1.0,
            "motion_compensation_enabled": False,
            "fk_buffer_size": 500,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        mode = str(self.get_parameter("selection_mode").value)
        if mode not in MODES:
            raise ValueError(f"D18-E201-PSM_MODE {mode!r} not in {sorted(MODES)}")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter("selected_pose_topic").value, 10)
        self.mesh_pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter("vision_mesh_pose_topic").value, 10)
        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, latched)
        self.fk_buffer = deque(maxlen=int(self.get_parameter("fk_buffer_size").value))
        self.create_subscription(
            PoseStamped, self.get_parameter("fk_camera_pose_topic").value,
            self._fk_pose, 50)
        self.create_subscription(
            String, self.get_parameter("vision_candidates_topic").value,
            self._vision_candidates, latched)
        self._status("READY" if mode != "disabled" else "DISABLED",
                     warning="no AMBF, identity hand-eye, or mesh/control-point fallback")

    def _status(self, state: str, **fields) -> None:
        msg = String()
        msg.data = json.dumps({
            "schema": "suturing.psm_pose_selector_status.v1",
            "state": state,
            "mode": str(self.get_parameter("selection_mode").value),
            **fields,
        }, sort_keys=True)
        self.status_pub.publish(msg)

    @staticmethod
    def _pose_message(matrix: np.ndarray, stamp, frame_id: str) -> PoseStamped:
        output = PoseStamped()
        output.header.stamp = stamp
        output.header.frame_id = frame_id
        output.pose.position.x, output.pose.position.y, output.pose.position.z = map(
            float, matrix[:3, 3])
        quaternion = matrix_to_quaternion_xyzw(matrix[:3, :3])
        (output.pose.orientation.x, output.pose.orientation.y,
         output.pose.orientation.z, output.pose.orientation.w) = map(float, quaternion)
        return output

    @staticmethod
    def _matrix_from_pose(msg: PoseStamped) -> np.ndarray:
        p, q = msg.pose.position, msg.pose.orientation
        return pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])

    def _fk_pose(self, msg: PoseStamped) -> None:
        if not msg.header.frame_id:
            self._status("REJECTED", code="D18-E202-FK_FRAME_EMPTY")
            return
        try:
            matrix = self._matrix_from_pose(msg)
        except Exception as exc:
            self._status("REJECTED", code="D18-E203-FK_POSE", detail=str(exc))
            return
        self.fk_buffer.append((stamp_ns(msg.header), msg.header.frame_id, matrix, msg.header.stamp))
        if str(self.get_parameter("selection_mode").value) == "fk_only":
            self.pose_pub.publish(msg)
            self._status("ACCEPTED", source="dvrk_measured_cp_plus_handeye",
                         stamp_ns=stamp_ns(msg.header))

    def _mesh_from_control_point(self) -> np.ndarray:
        if not bool(self.get_parameter("mesh_to_control_point_configured").value):
            raise ValueError("D18-E204-MESH_CONTROL_POINT_TF_UNCONFIGURED")
        translation = self.get_parameter("mesh_from_control_point_translation_m").value
        quaternion = self.get_parameter("mesh_from_control_point_quaternion_xyzw").value
        if np.linalg.norm(np.asarray(quaternion, dtype=float)) < 1.0e-9:
            raise ValueError("D18-E205-MESH_CONTROL_POINT_QUATERNION_INVALID")
        return pose_matrix(translation, quaternion)

    def _parse_candidates(self, msg: String) -> tuple[dict, np.ndarray, np.ndarray]:
        data = json.loads(msg.data)
        if data.get("schema") != "suturing.fp_candidates.v1":
            raise ValueError("D18-E206-CANDIDATE_SCHEMA")
        expected_mesh = str(self.get_parameter("vision_mesh_frame").value)
        if not expected_mesh:
            raise ValueError("D18-E207-VISION_MESH_FRAME_UNCONFIGURED")
        if str(data.get("mesh_frame", "")) != expected_mesh:
            raise ValueError(
                f"D18-E208-VISION_MESH_FRAME expected={expected_mesh!r} "
                f"got={data.get('mesh_frame')!r}")
        items = data.get("poses", [])
        if not items:
            raise ValueError("D18-E209-EMPTY_VISION_CANDIDATES")
        poses = np.stack([
            pose_matrix(item["position_m"], item["quaternion_xyzw"])
            for item in items
        ])
        scores = np.asarray([float(item["score"]) for item in items], dtype=float)
        if not np.isfinite(scores).all():
            raise ValueError("D18-E210-NONFINITE_VISION_SCORE")
        return data, poses, scores

    def _nearest_fk(self, target_ns: int):
        if not self.fk_buffer:
            raise ValueError("D18-E211-FK_CAMERA_POSE_MISSING")
        nearest = min(self.fk_buffer, key=lambda item: abs(item[0] - target_ns))
        skew_s = abs(nearest[0] - target_ns) * 1.0e-9
        if skew_s > float(self.get_parameter("max_sync_skew_s").value):
            raise ValueError(f"D18-E212-FK_VISION_SKEW skew_s={skew_s:.6f}")
        return nearest, skew_s

    def _publish_visual_mesh(self, matrix: np.ndarray, stamp, frame: str) -> None:
        self.mesh_pose_pub.publish(self._pose_message(matrix, stamp, frame))

    def _vision_candidates(self, msg: String) -> None:
        mode = str(self.get_parameter("selection_mode").value)
        if mode in {"disabled", "fk_only"}:
            return
        try:
            data, mesh_poses, scores = self._parse_candidates(msg)
            stamp_value = int(data["stamp_ns"])
            frame = str(data["frame_id"])
            stamp_msg = self.get_clock().now().to_msg()
            stamp_msg.sec = stamp_value // 1_000_000_000
            stamp_msg.nanosec = stamp_value % 1_000_000_000
            top = int(np.argmax(scores))
            self._publish_visual_mesh(mesh_poses[top], stamp_msg, frame)
            mesh_from_cp = self._mesh_from_control_point()
            control_candidates = mesh_poses @ mesh_from_cp
            if mode == "vision_only":
                output = control_candidates[top]
                self.pose_pub.publish(self._pose_message(output, stamp_msg, frame))
                self._status(
                    "ACCEPTED", source="foundationpose_control_point",
                    selected_index=top, candidate_count=len(scores),
                    warning="pure visual PSM 6D has not passed the project deployment gate")
                return
            if not bool(self.get_parameter("fusion_uncertainty_configured").value):
                raise ValueError("D18-E213-FUSION_UNCERTAINTY_UNCONFIGURED")
            (capture_ns, fk_frame, fk_capture, _), skew_s = self._nearest_fk(stamp_value)
            if fk_frame != frame:
                raise ValueError(
                    f"D18-E214-FK_VISION_FRAME expected={frame!r} got={fk_frame!r}")
            selection = select_psm_candidate(
                mesh_poses, scores, fk_capture, mesh_from_cp,
                self.get_parameter("kinematic_sigma6_m_deg").value,
                self.get_parameter("vision_sigma6_m_deg").value,
                float(self.get_parameter("fp_rank_weight").value),
            )
            output = selection.selected_camera_from_control_point
            output_stamp = stamp_msg
            motion_age_s = 0.0
            if bool(self.get_parameter("motion_compensation_enabled").value):
                latest_ns, latest_frame, fk_latest, latest_stamp = self.fk_buffer[-1]
                if latest_frame != frame:
                    raise ValueError("D18-E215-LATEST_FK_FRAME_MISMATCH")
                motion_age_s = max(0.0, (latest_ns - capture_ns) * 1.0e-9)
                if motion_age_s > float(self.get_parameter("max_visual_age_s").value):
                    raise ValueError(f"D18-E216-VISUAL_TOO_OLD age_s={motion_age_s:.6f}")
                output = motion_compensate_pose(output, fk_capture, fk_latest)
                output_stamp = latest_stamp
            self.pose_pub.publish(self._pose_message(output, output_stamp, frame))
            self._status(
                "ACCEPTED", source="foundationpose_candidate_selected_by_fk_uncertainty",
                selected_index=selection.selected_index,
                candidate_count=len(scores), sync_skew_s=skew_s,
                motion_compensation_age_s=motion_age_s,
                translation_innovation_m=selection.translation_innovation_m,
                rotation_innovation_deg=selection.rotation_innovation_deg,
                normalized_cost=selection.normalized_cost,
                fp_rank_fraction=selection.fp_rank_fraction,
            )
        except Exception as exc:
            self._status("BLOCKED", code="D18-E217-PSM_SELECTION", detail=str(exc))


def main() -> None:
    rclpy.init()
    node = PSMPoseSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
