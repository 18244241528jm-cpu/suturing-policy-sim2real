"""Selectable real-needle FP candidate gate; no simulator fallback exists."""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .contract import pose_matrix
from .pose_selection import select_needle_candidate


MODES = {"fp_only", "flat", "support", "flat_planar", "support_planar"}


class NeedlePoseSelector(Node):
    """Keep the final pose inside the original FoundationPose candidate set.

    Modes deliberately expose the ablation requested for real deployment:
    ``fp_only`` uses no flat/CAD constraint; ``flat`` checks signed normal;
    ``support`` also checks candidate-origin height; ``flat_planar`` adds an
    external same-image mask+plane+CAD observation without the OBJ-origin
    height test; ``support_planar`` enables both. Missing required inputs block
    output instead of falling back to AMBF or identity transforms.
    """

    def __init__(self) -> None:
        super().__init__("needle_pose_selector")
        defaults = {
            "selection_mode": "fp_only",
            "candidate_topic": "/suturing/needle/candidates",
            "support_surface_topic": "/suturing/external/support_surface",
            "planar_observation_topic": "/suturing/external/needle_planar_observation",
            "pending_topic": "/suturing/needle/pose_pending",
            "gated_topic": "/suturing/needle/pose_gated",
            "status_topic": "/suturing/needle/selector_status",
            "needle_rest_normal_mesh": [0.0, 0.0, 1.0],
            "needle_heading_axis_mesh": [1.0, 0.0, 0.0],
            "max_flat_angle_deg": 15.0,
            "needle_origin_support_offset_configured": False,
            "needle_origin_support_offset_m": 0.0,
            "max_height_error_m": 0.003,
            "max_planar_skew_s": 0.030,
            "planar_chi2_threshold": 11.3449,
            "manual_confirmation_required": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        mode = str(self.get_parameter("selection_mode").value)
        if mode not in MODES:
            raise ValueError(f"D18-E101-NEEDLE_MODE {mode!r} not in {sorted(MODES)}")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pending_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("pending_topic").value, latched)
        self.gated_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("gated_topic").value, latched)
        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, latched)
        self.support_surface = None
        self.planar_observation = None
        self.pending = None
        self.create_subscription(
            String, self.get_parameter("candidate_topic").value, self._candidates, latched)
        self.create_subscription(
            String, self.get_parameter("support_surface_topic").value, self._support, latched)
        self.create_subscription(
            String, self.get_parameter("planar_observation_topic").value, self._planar, latched)
        self.create_service(SetBool, "/suturing/needle/confirm_pending", self._confirm)
        self.get_logger().info(
            f"D18_NEEDLE_SELECTOR_READY mode={mode}; no AMBF/GT fallback")
        self._status(
            "READY",
            required_inputs=(
                ["foundationpose_candidates"] if mode == "fp_only" else
                ["foundationpose_candidates", "support_surface"] if mode in {"flat", "support"} else
                ["foundationpose_candidates", "support_surface", "planar_observation"]
            ),
        )

    def _status(self, state: str, **fields) -> None:
        message = String()
        message.data = json.dumps({
            "schema": "suturing.needle_selector_status.v1",
            "state": state,
            "mode": str(self.get_parameter("selection_mode").value),
            **fields,
        }, sort_keys=True)
        self.status_pub.publish(message)

    def _support(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            if data.get("schema") != "suturing.support_surface.v1":
                raise ValueError("schema")
            if not str(data.get("frame_id", "")):
                raise ValueError("frame_id")
            point = np.asarray(data["point_camera_m"], dtype=float).reshape(3)
            normal = np.asarray(data["normal_camera"], dtype=float).reshape(3)
            axis_x = np.asarray(data["axis_x_camera"], dtype=float).reshape(3)
            axis_y = np.asarray(data["axis_y_camera"], dtype=float).reshape(3)
            if not np.isfinite(np.r_[point, normal, axis_x, axis_y]).all():
                raise ValueError("nonfinite")
            data["point_camera_m"] = point.tolist()
            data["normal_camera"] = normal.tolist()
            data["axis_x_camera"] = axis_x.tolist()
            data["axis_y_camera"] = axis_y.tolist()
            data["stamp_ns"] = int(data.get("stamp_ns", 0))
            self.support_surface = data
            self._status("SUPPORT_READY", source=data.get("source", "unknown"))
        except Exception as exc:
            self.support_surface = None
            self._status("REJECTED", code="D18-E102-SUPPORT_CONTRACT", detail=str(exc))

    def _planar(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            if data.get("schema") != "suturing.needle_planar_observation.v1":
                raise ValueError("schema")
            position = np.asarray(data["position_camera_m"], dtype=float).reshape(3)
            quaternion = np.asarray(data["quaternion_xyzw"], dtype=float).reshape(4)
            sigmas = np.asarray(data["sigma_xyyaw"], dtype=float).reshape(3)
            if not np.isfinite(np.r_[position, quaternion, sigmas]).all() or np.any(sigmas <= 0.0):
                raise ValueError("finite positive pose/sigma required")
            data["stamp_ns"] = int(data["stamp_ns"])
            data["position_camera_m"] = position.tolist()
            data["quaternion_xyzw"] = quaternion.tolist()
            data["sigma_xyyaw"] = sigmas.tolist()
            self.planar_observation = data
            self._status("PLANAR_READY", source=data.get("source", "unknown"))
        except Exception as exc:
            self.planar_observation = None
            self._status("REJECTED", code="D18-E103-PLANAR_CONTRACT", detail=str(exc))

    def _required_geometry(self, data: dict) -> tuple[dict | None, dict | None]:
        mode = str(self.get_parameter("selection_mode").value)
        if mode == "fp_only":
            return None, None
        if self.support_surface is None:
            raise ValueError("D18-E104-SUPPORT_MISSING")
        if str(self.support_surface["frame_id"]) != str(data["frame_id"]):
            raise ValueError("D18-E105-SUPPORT_FRAME_MISMATCH")
        if mode in {"support", "support_planar"} and not bool(
                self.get_parameter("needle_origin_support_offset_configured").value):
            raise ValueError("D18-E106-MESH_SUPPORT_OFFSET_UNCONFIGURED")
        if mode not in {"flat_planar", "support_planar"}:
            return self.support_surface, None
        if self.planar_observation is None:
            raise ValueError("D18-E107-PLANAR_OBSERVATION_MISSING")
        if str(self.planar_observation.get("frame_id", "")) != str(data["frame_id"]):
            raise ValueError("D18-E108-PLANAR_FRAME_MISMATCH")
        skew = abs(int(self.planar_observation["stamp_ns"]) - int(data["stamp_ns"])) * 1.0e-9
        if skew > float(self.get_parameter("max_planar_skew_s").value):
            raise ValueError(f"D18-E109-PLANAR_STALE skew_s={skew:.6f}")
        return self.support_surface, self.planar_observation

    def _candidates(self, msg: String) -> None:
        self.pending = None
        try:
            data = json.loads(msg.data)
            if data.get("schema") != "suturing.fp_candidates.v1":
                raise ValueError("D18-E110-CANDIDATE_SCHEMA")
            items = data.get("poses", [])
            if not items:
                raise ValueError("D18-E111-EMPTY_CANDIDATES")
            poses = np.stack([
                pose_matrix(item["position_m"], item["quaternion_xyzw"])
                for item in items
            ])
            scores = np.asarray([float(item["score"]) for item in items], dtype=float)
            support, planar = self._required_geometry(data)
            mode = str(self.get_parameter("selection_mode").value)
            kwargs = {}
            if support is not None:
                kwargs.update(
                    flat_enabled=True,
                    plane_normal_camera=support["normal_camera"],
                    needle_rest_normal_mesh=self.get_parameter("needle_rest_normal_mesh").value,
                    maximum_flat_angle_deg=float(self.get_parameter("max_flat_angle_deg").value),
                )
            if mode in {"support", "support_planar"}:
                kwargs.update(
                    support_height_enabled=True,
                    plane_point_camera_m=support["point_camera_m"],
                    needle_origin_support_offset_m=float(
                        self.get_parameter("needle_origin_support_offset_m").value),
                    maximum_height_error_m=float(self.get_parameter("max_height_error_m").value),
                )
            if planar is not None:
                kwargs.update(
                    planar_enabled=True,
                    planar_pose_camera=pose_matrix(
                        planar["position_camera_m"], planar["quaternion_xyzw"]),
                    plane_axis_x_camera=support["axis_x_camera"],
                    plane_axis_y_camera=support["axis_y_camera"],
                    needle_heading_axis_mesh=self.get_parameter("needle_heading_axis_mesh").value,
                    planar_sigmas=planar["sigma_xyyaw"],
                    planar_chi2_threshold=float(
                        self.get_parameter("planar_chi2_threshold").value),
                )
            result = select_needle_candidate(poses, scores, **kwargs)
            if result.selected_index is None:
                raise ValueError("D18-E112-NO_SURVIVING_CANDIDATE")
        except Exception as exc:
            self._status("REJECTED", code="D18-E113-SELECTION", detail=str(exc))
            return

        selected = int(result.selected_index)
        item = items[selected]
        output = PoseWithCovarianceStamped()
        stamp_ns = int(data["stamp_ns"])
        output.header.stamp.sec = stamp_ns // 1_000_000_000
        output.header.stamp.nanosec = stamp_ns % 1_000_000_000
        output.header.frame_id = str(data["frame_id"])
        output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = map(
            float, item["position_m"])
        (output.pose.pose.orientation.x, output.pose.pose.orientation.y,
         output.pose.pose.orientation.z, output.pose.pose.orientation.w) = map(
            float, item["quaternion_xyzw"])
        self.pending = output
        self.pending_pub.publish(output)
        needs_manual = bool(self.get_parameter("manual_confirmation_required").value)
        self._status(
            "WAITING_OPERATOR" if needs_manual else "ACCEPTED",
            selected_index=selected,
            candidate_count=len(items),
            survivor_count=int(len(result.survivor_indices)),
            score=float(item["score"]),
            flat_angle_deg=float(result.flat_angle_deg[selected]),
            height_error_m=float(result.height_error_m[selected]),
            planar_cost=float(result.planar_cost[selected]),
            final_pose_source="foundationpose_candidate",
        )
        if not needs_manual:
            self.gated_pub.publish(output)

    def _confirm(self, request, response):
        if self.pending is None:
            response.success = False
            response.message = "D18-E114-NO_PENDING_POSE"
            return response
        if not request.data:
            self.pending = None
            response.success = True
            response.message = "Pending needle pose rejected by operator"
            self._status("OPERATOR_REJECTED")
            return response
        self.gated_pub.publish(self.pending)
        response.success = True
        response.message = "FoundationPose candidate released after configured gates and operator confirmation"
        self._status("OPERATOR_ACCEPTED")
        return response


def main() -> None:
    rclpy.init()
    node = NeedlePoseSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
