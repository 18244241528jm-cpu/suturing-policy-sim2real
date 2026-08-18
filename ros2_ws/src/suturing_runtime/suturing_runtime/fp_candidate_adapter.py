"""Validate an external FoundationPose candidate dump; never invent a top-1 pose."""

from __future__ import annotations

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .contract import normalize_quaternion_xyzw


class FPCandidateAdapter(Node):
    def __init__(self) -> None:
        super().__init__("fp_candidate_adapter")
        self.declare_parameter("external_candidate_topic", "/suturing/external/needle_candidates")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready = None
        self.output = self.create_publisher(String, "/suturing/needle/candidates", latched)
        self.status = self.create_publisher(String, "/suturing/fp_candidate/status", latched)
        self.create_subscription(String, "/suturing/fp_input/ready", self._ready, latched)
        self.create_subscription(String, self.get_parameter("external_candidate_topic").value,
                                 self._candidates, latched)
        self.get_logger().info("D12_R5_READY external FP must publish all candidates")

    def _publish_status(self, state, **fields):
        msg=String(); msg.data=json.dumps({"schema":"suturing.fp_adapter_status.v1","state":state,**fields},sort_keys=True)
        self.status.publish(msg)

    def _ready(self, msg):
        try: self.ready=json.loads(msg.data)
        except Exception as exc:
            self.ready=None; self._publish_status("REJECTED",code="D12-E401-BAD_READY",detail=str(exc))

    def _candidates(self, msg):
        if self.ready is None:
            self._publish_status("REJECTED",code="D12-E402-NO_FP_BUNDLE"); return
        try:
            data=json.loads(msg.data)
            if data.get("schema") != "suturing.fp_candidates.v1": raise ValueError("schema")
            if int(data["stamp_ns"]) != int(self.ready["stamp_ns"]): raise ValueError("stamp")
            if str(data["frame_id"]) != str(self.ready["frame_id"]): raise ValueError("frame")
            if not str(data.get("mesh_frame", "")): raise ValueError("mesh_frame")
            poses=data.get("poses",[])
            if not poses: raise ValueError("empty poses")
            for item in poses:
                xyz=[float(v) for v in item["position_m"]]
                quat=normalize_quaternion_xyzw(item["quaternion_xyzw"])
                score=float(item["score"])
                if len(xyz)!=3 or not all(math.isfinite(v) for v in xyz+[score]): raise ValueError("nonfinite")
                item["position_m"]=xyz; item["quaternion_xyzw"]=[float(v) for v in quat]; item["score"]=score
        except Exception as exc:
            self._publish_status("REJECTED",code="D12-E403-CANDIDATE_CONTRACT",detail=str(exc)); return
        clean=String(); clean.data=json.dumps(data,sort_keys=True); self.output.publish(clean)
        self._publish_status("ACCEPTED",stamp_ns=data["stamp_ns"],candidate_count=len(data["poses"]))


def main() -> None:
    rclpy.init(); node=FPCandidateAdapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
