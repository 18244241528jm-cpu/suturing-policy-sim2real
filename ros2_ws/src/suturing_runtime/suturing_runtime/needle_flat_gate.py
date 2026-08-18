"""No-GT first-pose gate for a human-placed, flat needle."""

from __future__ import annotations

import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .contract import pose_matrix, select_flat_candidate


class NeedleFlatGate(Node):
    def __init__(self) -> None:
        super().__init__("needle_flat_gate")
        self.declare_parameter("plane_configured", False)
        self.declare_parameter("plane_frame", "")
        self.declare_parameter("plane_normal_camera", [0.0, 0.0, 1.0])
        self.declare_parameter("needle_rest_normal_mesh", [0.0, 0.0, 1.0])
        self.declare_parameter("max_flat_angle_deg", 15.0)
        self.declare_parameter("manual_confirmation_required", True)
        latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pending_pub=self.create_publisher(PoseWithCovarianceStamped,"/suturing/needle/pose_pending",latched)
        self.gated_pub=self.create_publisher(PoseWithCovarianceStamped,"/suturing/needle/pose_gated",latched)
        self.status_pub=self.create_publisher(String,"/suturing/needle/gate_status",latched)
        self.pending=None
        self.create_subscription(String,"/suturing/needle/candidates",self._candidates,latched)
        self.create_service(SetBool,"/suturing/needle/confirm_pending",self._confirm)
        self.get_logger().info("D12_R6_READY plane calibration and operator confirmation are fail-closed")

    def _status(self,state,**fields):
        out=String(); out.data=json.dumps({"schema":"suturing.needle_gate.v1","state":state,**fields},sort_keys=True)
        self.status_pub.publish(out)

    def _candidates(self,msg):
        self.pending=None
        if not bool(self.get_parameter("plane_configured").value):
            self._status("BLOCKED",code="D12-E501-PHANTOM_PLANE_UNCONFIGURED"); return
        try:
            data=json.loads(msg.data); items=data["poses"]
            plane_frame=str(self.get_parameter("plane_frame").value)
            if not plane_frame or plane_frame != str(data["frame_id"]):
                raise ValueError(
                    f"D12-E505-PLANE_FRAME expected={plane_frame!r} candidate={data['frame_id']!r}")
            poses=np.stack([pose_matrix(i["position_m"],i["quaternion_xyzw"]) for i in items])
            scores=np.asarray([i["score"] for i in items],dtype=np.float64)
            selected,angles=select_flat_candidate(poses,scores,
                self.get_parameter("plane_normal_camera").value,
                self.get_parameter("needle_rest_normal_mesh").value,
                float(self.get_parameter("max_flat_angle_deg").value))
        except Exception as exc:
            self._status("REJECTED",code="D12-E502-GATE_INPUT",detail=str(exc)); return
        if selected is None:
            self._status("REJECTED",code="D12-E503-NO_FLAT_CANDIDATE",
                         minimum_angle_deg=float(np.min(angles))); return
        item=items[selected]; out=PoseWithCovarianceStamped()
        ns=int(data["stamp_ns"]); out.header.stamp.sec=ns//1_000_000_000; out.header.stamp.nanosec=ns%1_000_000_000
        out.header.frame_id=str(data["frame_id"])
        out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,item["position_m"])
        out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,item["quaternion_xyzw"])
        # Covariance remains unknown. A zero matrix here means "not estimated", not zero error.
        self.pending=out; self.pending_pub.publish(out)
        needs_manual=bool(self.get_parameter("manual_confirmation_required").value)
        self._status("WAITING_OPERATOR" if needs_manual else "ACCEPTED",
                     selected_index=selected,flat_angle_deg=float(angles[selected]),score=float(item["score"]),
                     warning="orientation-only gate does not prove translation, yaw, or tip/base identity")
        if not needs_manual: self.gated_pub.publish(out)

    def _confirm(self,request,response):
        if self.pending is None:
            response.success=False; response.message="D12-E504-NO_PENDING_POSE"; return response
        if not request.data:
            self.pending=None; response.success=True; response.message="Pending pose rejected by operator"
            self._status("OPERATOR_REJECTED"); return response
        self.gated_pub.publish(self.pending); response.success=True
        response.message="Pose released; operator confirms external overlay/physical plausibility"
        self._status("OPERATOR_ACCEPTED"); return response


def main() -> None:
    rclpy.init(); node=NeedleFlatGate()
    try:rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
