"""Publish an Approach goal in ECM from a FoundationPose needle pose in the camera frame.

FoundationPose reports T_camera_from_needle. The executor commands in ECM. This closes
that gap without a TF tree, using the chain documented in frame_chain.py:

    T_ECM_from_needle = T_ECM_from_psmbase @ T_psmbase_from_camera @ T_camera_from_needle

T_ECM_from_psmbase is recovered live from a simultaneous /PSM1/measured_cp and
/PSM1/local/measured_cp pair -- the same tip in two frames -- so the only calibrated
input is the hand-eye T_psmbase_from_camera, supplied as a parameter.

The grasp geometry (grasp_angle_deg, lift_height_m) is then applied by the same
approach_goal_matrix the simulator and approach_goal_builder use, so a goal produced
here is the same kind of object as a touch-off goal.

Output goes to /suturing/approach/goal_final, which feeds goal_servo_node. This node
never publishes to the executor and never to the robot: segmentation, arming, the
workspace box, the speed cap and the timeout all still apply downstream.
"""

from __future__ import annotations

import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .contract import approach_goal_matrix
from .frame_chain import (
    ecm_from_psmbase,
    load_handeye,
    matrix_to_pose_msg,
    pose_msg_to_matrix,
    validate_rigid,
)


class FPGoalBridge(Node):
    def __init__(self) -> None:
        super().__init__("fp_goal_bridge")
        topics = {
            "tip_ecm_topic": "/PSM1/measured_cp",
            "tip_psmbase_topic": "/PSM1/local/measured_cp",
            "needle_pose_topic": "/suturing/needle/pose_camera",
            "goal_topic": "/suturing/approach/goal_final",
            "ecm_frame": "ECM",
            "required_needle_frame": "",
        }
        for key, value in topics.items():
            self.declare_parameter(key, value)
        for key, value in {"grasp_angle_deg": 12.5, "lift_height_m": 0.007,
                           "max_pair_skew_s": 0.050, "max_pose_age_s": 30.0,
                           "freeze_first_goal": True, "enabled": False}.items():
            self.declare_parameter(key, value)
        # Zeros are an intentionally invalid placeholder, never identity: an identity
        # hand-eye would silently mean "camera is exactly at the PSM base", which is
        # wrong everywhere and produces a plausible-looking goal.
        self.declare_parameter("psmbase_from_camera", [0.0] * 16)

        self.handeye = None
        raw = list(self.get_parameter("psmbase_from_camera").value)
        if any(abs(v) > 0.0 for v in raw):
            self.handeye = load_handeye(raw)
            self.get_logger().info("D13_HANDEYE_LOADED translation_mm=%s" %
                                   np.round(self.handeye[:3, 3] * 1000, 2).tolist())
        else:
            self.get_logger().warning(
                "D13-E06-HANDEYE_UNCONFIGURED psmbase_from_camera is all zeros; "
                "no goal will be produced until it is set")

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("goal_topic").value), latched)
        self.status_pub = self.create_publisher(String, "/suturing/fp_goal/status", 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("tip_ecm_topic").value), self._tip_ecm, 20)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("tip_psmbase_topic").value), self._tip_psmbase, 20)
        # FoundationPose bridges publish either message type depending on whether they
        # carry a covariance, so accept both rather than forcing one on the operator.
        self.create_subscription(
            PoseStamped, str(self.get_parameter("needle_pose_topic").value), self._needle_plain, 5)
        self.create_subscription(
            PoseWithCovarianceStamped, str(self.get_parameter("needle_pose_topic").value) + "_cov",
            self._needle_cov, 5)
        self.create_service(Trigger, "/suturing/fp_goal/reset", self._reset)

        self.tip_ecm = None
        self.tip_ecm_t = None
        self.tip_psmbase = None
        self.tip_psmbase_t = None
        self.frozen = False
        self._publish_status("STARTUP", "writes only to the servo, never to the robot")

    # ------------------------------------------------------------------ inputs
    def _tip_ecm(self, msg: PoseStamped) -> None:
        self.tip_ecm = msg
        self.tip_ecm_t = self._stamp_s(msg)

    def _tip_psmbase(self, msg: PoseStamped) -> None:
        self.tip_psmbase = msg
        self.tip_psmbase_t = self._stamp_s(msg)

    def _needle_plain(self, msg: PoseStamped) -> None:
        self._handle_needle(msg.header, pose_msg_to_matrix(msg))

    def _needle_cov(self, msg: PoseWithCovarianceStamped) -> None:
        shim = PoseStamped()
        shim.header = msg.header
        shim.pose = msg.pose.pose
        self._handle_needle(msg.header, pose_msg_to_matrix(shim))

    @staticmethod
    def _stamp_s(msg) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    # -------------------------------------------------------------------- core
    def _handle_needle(self, header, needle_in_camera: np.ndarray) -> None:
        if not bool(self.get_parameter("enabled").value):
            return
        if self.frozen and bool(self.get_parameter("freeze_first_goal").value):
            return
        if self.handeye is None:
            self._publish_status("BLOCKED", "D13-E06-HANDEYE_UNCONFIGURED")
            return

        required = str(self.get_parameter("required_needle_frame").value)
        if required and header.frame_id != required:
            self._publish_status(
                "BLOCKED", f"D13-E07-NEEDLE_FRAME expected={required} got={header.frame_id}")
            return
        if self.tip_ecm is None or self.tip_psmbase is None:
            self._publish_status("BLOCKED", "D13-E08-MISSING_TIP_PAIR")
            return

        # The pair must be simultaneous. Readings taken with the arm moved in between
        # describe different tips, and composing them yields a transform that looks
        # valid and is wrong by however far the arm travelled.
        skew = abs(self.tip_ecm_t - self.tip_psmbase_t)
        if skew > float(self.get_parameter("max_pair_skew_s").value):
            self._publish_status("BLOCKED", f"D13-E09-TIP_PAIR_SKEW {skew:.3f}s")
            return

        age = abs(self.get_clock().now().nanoseconds * 1e-9 - self._stamp_s(header))
        if self._stamp_s(header) > 0 and age > float(self.get_parameter("max_pose_age_s").value):
            self._publish_status("BLOCKED", f"D13-E10-STALE_NEEDLE_POSE {age:.1f}s")
            return

        try:
            base = ecm_from_psmbase(pose_msg_to_matrix(self.tip_ecm),
                                    pose_msg_to_matrix(self.tip_psmbase))
            validate_rigid(base, "T_ECM_from_psmbase")
            needle_ecm = base @ self.handeye @ needle_in_camera
            validate_rigid(needle_ecm, "T_ECM_from_needle")
        except ValueError as exc:
            self._publish_status("BLOCKED", str(exc))
            return

        goal = approach_goal_matrix(needle_ecm,
                                    float(self.get_parameter("grasp_angle_deg").value),
                                    float(self.get_parameter("lift_height_m").value))
        ecm_frame = str(self.get_parameter("ecm_frame").value)
        self.goal_pub.publish(
            matrix_to_pose_msg(goal, ecm_frame, header.stamp, PoseStamped))
        self.frozen = True
        self._publish_status("GOAL_PUBLISHED", json.dumps({
            "needle_ecm_mm": np.round(needle_ecm[:3, 3] * 1000, 2).tolist(),
            "goal_ecm_mm": np.round(goal[:3, 3] * 1000, 2).tolist(),
            "tip_pair_skew_s": round(skew, 4),
        }))

    def _reset(self, request, response):
        del request
        self.frozen = False
        response.success = True
        response.message = "goal latch cleared; the next needle pose will publish again"
        return response

    def _publish_status(self, state: str, detail: str) -> None:
        msg = String()
        msg.data = json.dumps({"schema": "suturing_runtime.fp_goal.v1",
                               "state": state, "detail": detail})
        self.status_pub.publish(msg)
        self.get_logger().info(f"D13_FP_GOAL state={state} detail={detail}")


def main() -> None:
    rclpy.init()
    node = FPGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
