"""SE(3) goal servo that feeds guarded_pose_executor executor-sized waypoints.

Why this node exists
--------------------
Two gaps in the staged runtime meet here.

*The runtime had no controller node at all.*  guarded_pose_executor interpolates a
straight line to whatever goal it is handed; nothing in the package decided where to
aim.  The measured capability sweep puts the geometric goal servo ahead of the learned
policy on every axis (x +/-20 mm, z +/-10 mm, yaw +/-30 deg all pass, at 2.36 mm in 36
steps against the policy's 2.99 mm in 52), and the servo carries no checkpoint, so it
has no sim-to-real weight transfer to go wrong.

*The executor rejects far goals.*  max_initial_translation_m is 0.010 while the
Approach stroke is about 61 mm, so a goal taken from the home pose earns
D10-E56-DISTANCE.  This node segments the approach instead of widening that valve.

The servo is also branch-independent by construction: se3_goal_servo builds its
rotation command from a relative SO(3) rotvec and wraps the resulting increment, so it
cannot be fooled by the +/-180 deg Euler jump that makes the learned policy saturate.

Safety position
---------------
This node never publishes to the robot.  It only writes waypoints onto the executor
goal topic, so every guard in the executor -- arming, frame checks, staleness,
distance, rotation, workspace, speed caps, timeout -- still stands between it and any
motion.  It starts disabled and, by default, emits one segment and then waits for the
operator.
"""

from __future__ import annotations

import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .contract import normalize_quaternion_xyzw
from .subgoal_planner import DEFAULT_STEP_SIZE, clamp_to_budget, plan_subgoal


def pose_to_vector_cm_rad(msg: PoseStamped, jaw: float = 0.0) -> np.ndarray:
    """ROS pose (metres, quaternion) -> controller vector (centimetres, xyz Euler).

    The centimetre unit is not cosmetic: se3_goal_servo divides the translation error
    by 100 internally, so handing it metres understates the error a hundredfold and the
    arm creeps instead of converging.
    """
    position = msg.pose.position
    quaternion = normalize_quaternion_xyzw(
        np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                  msg.pose.orientation.z, msg.pose.orientation.w], dtype=np.float64)
    )
    rpy = Rotation.from_quat(quaternion).as_euler("xyz")
    return np.array([position.x * 100.0, position.y * 100.0, position.z * 100.0,
                     rpy[0], rpy[1], rpy[2], jaw], dtype=np.float64)


def vector_cm_rad_to_pose(vector: np.ndarray, frame_id: str, stamp) -> PoseStamped:
    """Controller vector (centimetres, xyz Euler) -> ROS pose (metres, quaternion)."""
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(vector[0]) / 100.0
    msg.pose.position.y = float(vector[1]) / 100.0
    msg.pose.position.z = float(vector[2]) / 100.0
    quaternion = Rotation.from_euler("xyz", np.asarray(vector[3:6], dtype=np.float64)).as_quat()
    msg.pose.orientation.x = float(quaternion[0])
    msg.pose.orientation.y = float(quaternion[1])
    msg.pose.orientation.z = float(quaternion[2])
    msg.pose.orientation.w = float(quaternion[3])
    return msg


class GoalServoNode(Node):
    def __init__(self) -> None:
        super().__init__("goal_servo_node")
        topics = {
            "measured_pose_topic": "/suturing/psm1/measured_pose",
            "final_goal_topic": "/suturing/approach/goal_final",
            "subgoal_topic": "/suturing/approach/goal",
            "execution_status_topic": "/suturing/execution/status",
            "command_frame": "PSM1_psm_base_link",
        }
        for key, value in topics.items():
            self.declare_parameter(key, value)
        # Segment budgets sit deliberately under the executor limits (0.010 m / 30 deg)
        # so a waypoint is never rejected for being too far; the executor stays the
        # authority on what is safe, this node only stays inside what it will accept.
        for key, value in {"enabled": False, "auto_advance": False, "publish_hz": 2.0,
                           "max_segment_m": 0.008, "max_segment_deg": 25.0,
                           "goal_tolerance_m": 0.001, "goal_tolerance_deg": 2.0,
                           "max_input_age_s": 0.5}.items():
            self.declare_parameter(key, value)
        self.declare_parameter("step_size", list(DEFAULT_STEP_SIZE))

        self.command_frame = str(self.get_parameter("command_frame").value)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subgoal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("subgoal_topic").value), latched)
        self.status_pub = self.create_publisher(String, "/suturing/goal_servo/status", 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("measured_pose_topic").value), self._measured, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("final_goal_topic").value), self._final_goal, latched)
        self.create_subscription(
            String, str(self.get_parameter("execution_status_topic").value), self._execution, 10)
        self.create_service(SetBool, "/suturing/goal_servo/enable", self._enable)
        self.create_service(Trigger, "/suturing/goal_servo/advance", self._advance)

        self.measured = None
        self.measured_at = None
        self.final_goal = None
        self.enabled = bool(self.get_parameter("enabled").value)
        # One segment is published per request.  Without this latch an auto-advancing
        # loop would keep retargeting mid-execution and the arm would chase a moving
        # waypoint instead of finishing the segment the operator approved.
        self.segment_pending = False
        self.last_info = None
        self.create_timer(1.0 / float(self.get_parameter("publish_hz").value), self._tick)
        self._publish_status("STARTUP", "goal servo never commands the robot directly")

    # ------------------------------------------------------------------ inputs
    def _measured(self, msg: PoseStamped) -> None:
        self.measured = msg
        self.measured_at = self.get_clock().now()

    def _final_goal(self, msg: PoseStamped) -> None:
        self.final_goal = msg
        self.segment_pending = True
        self._publish_status("GOAL_RECEIVED", f"frame={msg.header.frame_id}")

    def _execution(self, msg: String) -> None:
        try:
            state = json.loads(msg.data).get("state")
        except (ValueError, TypeError):
            return
        if state == "COMPLETE" and bool(self.get_parameter("auto_advance").value):
            self.segment_pending = True

    # ---------------------------------------------------------------- services
    def _enable(self, request, response):
        self.enabled = bool(request.data)
        if self.enabled:
            self.segment_pending = True
        response.success = True
        response.message = "enabled" if self.enabled else "disabled"
        self._publish_status("ENABLED" if self.enabled else "DISABLED", response.message)
        return response

    def _advance(self, request, response):
        del request
        if self.final_goal is None:
            response.success = False
            response.message = "D11-E01-NO_FINAL_GOAL"
            return response
        self.segment_pending = True
        response.success = True
        response.message = "next segment queued"
        return response

    # -------------------------------------------------------------------- loop
    def _fresh(self):
        if self.measured is None or self.final_goal is None:
            return False, "D11-E02-MISSING_POSE_OR_GOAL"
        age_s = (self.get_clock().now() - self.measured_at).nanoseconds / 1e9
        if age_s > float(self.get_parameter("max_input_age_s").value):
            return False, f"D11-E03-STALE_MEASURED_POSE age={age_s:.3f}s"
        for label, msg in (("measured", self.measured), ("goal", self.final_goal)):
            if msg.header.frame_id and msg.header.frame_id != self.command_frame:
                return False, (f"D11-E04-FRAME expected={self.command_frame} "
                               f"{label}={msg.header.frame_id}")
        return True, "ok"

    def _tick(self) -> None:
        if not self.enabled or not self.segment_pending:
            return
        ok, reason = self._fresh()
        if not ok:
            self._publish_status("BLOCKED", reason)
            return

        current = pose_to_vector_cm_rad(self.measured)
        final = pose_to_vector_cm_rad(self.final_goal, jaw=current[6])
        max_segment_m = float(self.get_parameter("max_segment_m").value)
        max_segment_deg = float(self.get_parameter("max_segment_deg").value)
        subgoal, info = plan_subgoal(
            current, final,
            step_size=np.asarray(self.get_parameter("step_size").value, dtype=np.float64),
            max_segment_m=max_segment_m,
            max_segment_deg=max_segment_deg,
            goal_tolerance_m=float(self.get_parameter("goal_tolerance_m").value),
            goal_tolerance_deg=float(self.get_parameter("goal_tolerance_deg").value),
        )
        subgoal = clamp_to_budget(current, subgoal, max_segment_m, max_segment_deg)

        if info["reached_final"] and info["segment_translation_m"] <= 1e-9:
            self.segment_pending = False
            self._publish_status("AT_GOAL", json.dumps(info))
            return

        self.subgoal_pub.publish(
            vector_cm_rad_to_pose(subgoal, self.command_frame, self.get_clock().now().to_msg())
        )
        self.segment_pending = False
        self.last_info = info
        self._publish_status("SEGMENT_PUBLISHED", json.dumps(info))

    def _publish_status(self, state: str, detail: str) -> None:
        msg = String()
        msg.data = json.dumps({"schema": "suturing_runtime.goal_servo.v1",
                               "state": state, "detail": detail})
        self.status_pub.publish(msg)
        self.get_logger().info(f"D11_GOAL_SERVO state={state} detail={detail}")


def main() -> None:
    rclpy.init()
    node = GoalServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
