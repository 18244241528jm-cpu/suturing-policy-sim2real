"""Build the same frozen Approach goal in camera and PSM-base coordinates."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .contract import approach_goal_matrix, matrix_to_quaternion_xyzw, pose_matrix, transform_matrix


class ApproachGoalBuilder(Node):
    def __init__(self) -> None:
        super().__init__("approach_goal_builder")
        self.declare_parameter("needle_pose_topic", "/suturing/needle/pose_gated")
        self.declare_parameter("goal_topic", "/suturing/approach/goal")
        self.declare_parameter("camera_goal_topic", "/suturing/approach/goal_camera")
        self.declare_parameter("required_needle_frame", "")
        self.declare_parameter("target_frame", "PSM1_psm_base_link")
        self.declare_parameter("grasp_angle_deg", 12.5)
        self.declare_parameter("lift_height_m", 0.007)
        self.declare_parameter("max_pose_age_s", 30.0)
        self.declare_parameter("freeze_first_goal", True)
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PoseStamped, self.get_parameter("goal_topic").value, latched)
        self.camera_publisher = self.create_publisher(
            PoseStamped, self.get_parameter("camera_goal_topic").value, latched)
        self.frozen = None
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("needle_pose_topic").value,
                                 self._needle, 5)
        self.create_service(Trigger, "/suturing/goal/reset", self._reset)
        self.get_logger().info("D10_GOAL_BUILDER_READY input must already pass the physical gate")

    def _needle(self, msg: PoseWithCovarianceStamped) -> None:
        if self.frozen is not None and bool(self.get_parameter("freeze_first_goal").value): return
        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec)*1e-9
        now_s = self.get_clock().now().nanoseconds*1e-9
        if stamp_s > 0 and abs(now_s-stamp_s) > float(self.get_parameter("max_pose_age_s").value):
            self.get_logger().error("D10-E31-STALE_NEEDLE_POSE"); return
        source_frame = msg.header.frame_id; target_frame = str(self.get_parameter("target_frame").value)
        if not source_frame: self.get_logger().error("D10-E32-EMPTY_NEEDLE_FRAME"); return
        required = str(self.get_parameter("required_needle_frame").value)
        if required and source_frame != required:
            self.get_logger().error(f"D12-E701-NEEDLE_FRAME expected={required} got={source_frame}"); return
        try:
            tf = self.buffer.lookup_transform(target_frame, source_frame, Time.from_msg(msg.header.stamp),
                                              timeout=Duration(seconds=0.2))
        except TransformException as exc:
            self.get_logger().error(f"D10-E33-TF {source_frame}->{target_frame}: {exc}"); return
        t, q = tf.transform.translation, tf.transform.rotation
        target_from_source = transform_matrix([t.x,t.y,t.z], [q.x,q.y,q.z,q.w])
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        source_from_needle = pose_matrix([p.x,p.y,p.z], [o.x,o.y,o.z,o.w])
        camera_goal = approach_goal_matrix(source_from_needle,
            float(self.get_parameter("grasp_angle_deg").value),
            float(self.get_parameter("lift_height_m").value))
        goal = target_from_source @ camera_goal
        camera_out = PoseStamped(); camera_out.header.stamp = msg.header.stamp
        camera_out.header.frame_id = source_frame
        camera_out.pose.position.x, camera_out.pose.position.y, camera_out.pose.position.z = map(float, camera_goal[:3,3])
        camera_quat = matrix_to_quaternion_xyzw(camera_goal[:3,:3])
        camera_out.pose.orientation.x, camera_out.pose.orientation.y, camera_out.pose.orientation.z, camera_out.pose.orientation.w = map(float, camera_quat)
        out = PoseStamped(); out.header.stamp = msg.header.stamp; out.header.frame_id = target_frame
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, goal[:3,3])
        quat = matrix_to_quaternion_xyzw(goal[:3,:3])
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = map(float, quat)
        self.frozen = out; self.camera_publisher.publish(camera_out); self.publisher.publish(out)
        self.get_logger().info(
            f"D12_R7_GOAL_FROZEN source_stamp={stamp_s:.6f} camera={source_frame} base={target_frame}")

    def _reset(self, request, response):
        del request; self.frozen = None; response.success = True
        response.message = "Goal latch cleared; waiting for the next gated needle pose."; return response


def main() -> None:
    rclpy.init(); node = ApproachGoalBuilder()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
