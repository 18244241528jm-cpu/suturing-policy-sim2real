"""Drive the Approach with the trained TD3+HER+BC checkpoint instead of the geometry.

This is a drop-in sibling of goal_servo_node: same inputs, same output topic, same
segment budget, same refusal to talk to the robot. Only the thing that decides the next
step differs -- a policy forward pass instead of a gain-scheduled SE(3) servo -- so the
executor's guards apply identically and the two can be swapped by launching one or the
other.

Observation contract, taken from the simulator rather than guessed:

    obs["observation"]   = concat(current[7], goal[7], goal - current[7])   -> 21
    obs["achieved_goal"] = current[7]
    obs["desired_goal"]  = goal[7]

    each 7-vector is (x, y, z, roll, pitch, yaw, jaw), metres and radians
    normalisation is a fixed multiplier, no running statistics: xyz entries -- indices
    0,1,2, 7,8,9, 14,15,16 of the 21, and 0,1,2 of each 7 -- are multiplied by 100, so
    the policy sees centimetres. Everything else passes through.

Two details that silently break this if guessed wrong:

  * The RPY delta is the *raw* difference. The environment wraps or branch-aligns it
    only when align_obs_rpy_branch or wrap_obs_rpy_delta is set, and training set
    neither, so wrapping here would feed the policy something it never saw.
  * The angles are PyKDL GetRPY, which is fixed-axis XYZ -- scipy's lowercase
    as_euler("xyz"), not the uppercase intrinsic form.

The action is a normalised 7-vector; multiplying by the environment's step size gives a
per-step delta in metres and radians. The executor is pose-only, so the jaw component is
reported for audit and otherwise ignored.
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
from .subgoal_planner import DEFAULT_STEP_SIZE, clamp_to_budget, geodesic_angle_deg

# Indices of the xyz triplets inside the 21-vector, per the simulator's normaliser.
XYZ_INDICES_21 = [0, 1, 2, 7, 8, 9, 14, 15, 16]


def pose_to_vector(msg: PoseStamped, jaw: float = 0.0) -> np.ndarray:
    """PoseStamped -> (x, y, z, roll, pitch, yaw, jaw) in metres and radians."""
    p, o = msg.pose.position, msg.pose.orientation
    q = normalize_quaternion_xyzw(np.array([o.x, o.y, o.z, o.w], dtype=np.float64))
    rpy = Rotation.from_quat(q).as_euler("xyz")
    return np.array([p.x, p.y, p.z, rpy[0], rpy[1], rpy[2], jaw], dtype=np.float64)


def vector_to_pose(vector: np.ndarray, frame_id: str, stamp) -> PoseStamped:
    out = PoseStamped()
    out.header.stamp = stamp
    out.header.frame_id = frame_id
    out.pose.position.x = float(vector[0])
    out.pose.position.y = float(vector[1])
    out.pose.position.z = float(vector[2])
    q = Rotation.from_euler("xyz", np.asarray(vector[3:6], dtype=np.float64)).as_quat()
    out.pose.orientation.x, out.pose.orientation.y = float(q[0]), float(q[1])
    out.pose.orientation.z, out.pose.orientation.w = float(q[2]), float(q[3])
    return out


def build_observation(current: np.ndarray, goal: np.ndarray) -> dict:
    """Assemble and scale the dict observation exactly as the simulator does."""
    delta = goal - current  # raw, unwrapped: training set neither rpy option
    observation = np.concatenate((current, goal, delta)).astype(np.float32)
    observation[XYZ_INDICES_21] *= 100.0
    goal_scale = np.array([100, 100, 100, 1, 1, 1, 1], dtype=np.float32)
    return {
        "observation": observation,
        "achieved_goal": (current.astype(np.float32) * goal_scale),
        "desired_goal": (goal.astype(np.float32) * goal_scale),
    }


class PolicyServoNode(Node):
    def __init__(self) -> None:
        super().__init__("policy_servo_node")
        topics = {
            "measured_pose_topic": "/suturing/psm1/measured_pose",
            "final_goal_topic": "/suturing/approach/goal_final",
            "subgoal_topic": "/suturing/approach/goal",
            "execution_status_topic": "/suturing/execution/status",
            "command_frame": "PSM1_psm_base_link",
            "model_path": "",
        }
        for key, value in topics.items():
            self.declare_parameter(key, value)
        for key, value in {"enabled": False, "auto_advance": False, "publish_hz": 2.0,
                           "max_segment_m": 0.008, "max_segment_deg": 25.0,
                           "goal_tolerance_m": 0.001, "goal_tolerance_deg": 2.0,
                           "max_input_age_s": 0.5, "max_policy_steps": 400}.items():
            self.declare_parameter(key, value)
        self.declare_parameter("step_size", list(DEFAULT_STEP_SIZE))

        self.command_frame = str(self.get_parameter("command_frame").value)
        self.model = None
        self.model_path = str(self.get_parameter("model_path").value)
        if self.model_path:
            self._load_model()
        else:
            self.get_logger().warning(
                "D14-E01-NO_MODEL model_path is empty; no waypoint will be produced")

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subgoal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("subgoal_topic").value), latched)
        self.status_pub = self.create_publisher(String, "/suturing/policy_servo/status", 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("measured_pose_topic").value), self._measured, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("final_goal_topic").value), self._final_goal, latched)
        self.create_subscription(
            String, str(self.get_parameter("execution_status_topic").value), self._execution, 10)
        self.create_service(SetBool, "/suturing/policy_servo/enable", self._enable)
        self.create_service(Trigger, "/suturing/policy_servo/advance", self._advance)

        self.measured = None
        self.measured_at = None
        self.final_goal = None
        self.enabled = bool(self.get_parameter("enabled").value)
        self.segment_pending = False
        self.create_timer(1.0 / float(self.get_parameter("publish_hz").value), self._tick)
        self._publish_status("STARTUP", "policy never commands the robot directly")

    def _load_model(self) -> None:
        # Imported lazily: the rest of the runtime must stay usable on a machine with
        # no torch, and the geometric servo needs none of this.
        from stable_baselines3 import TD3

        self.model = TD3.load(self.model_path, device="cpu")
        self.get_logger().info(f"D14_POLICY_LOADED path={self.model_path}")

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
        if request.data and self.model is None:
            response.success = False
            response.message = "D14-E01-NO_MODEL model_path is empty or failed to load"
            return response
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
            response.message = "D14-E02-NO_FINAL_GOAL"
            return response
        self.segment_pending = True
        response.success = True
        response.message = "next segment queued"
        return response

    # -------------------------------------------------------------------- loop
    def _fresh(self):
        if self.measured is None or self.final_goal is None:
            return False, "D14-E03-MISSING_POSE_OR_GOAL"
        age_s = (self.get_clock().now() - self.measured_at).nanoseconds / 1e9
        if age_s > float(self.get_parameter("max_input_age_s").value):
            return False, f"D14-E04-STALE_MEASURED_POSE age={age_s:.3f}s"
        for label, msg in (("measured", self.measured), ("goal", self.final_goal)):
            if msg.header.frame_id and msg.header.frame_id != self.command_frame:
                return False, (f"D14-E05-FRAME expected={self.command_frame} "
                               f"{label}={msg.header.frame_id}")
        return True, "ok"

    def _tick(self) -> None:
        if not self.enabled or not self.segment_pending or self.model is None:
            return
        ok, reason = self._fresh()
        if not ok:
            self._publish_status("BLOCKED", reason)
            return

        start = pose_to_vector(self.measured)
        goal = pose_to_vector(self.final_goal, jaw=start[6])
        steps = np.asarray(self.get_parameter("step_size").value, dtype=np.float64)
        max_segment_m = float(self.get_parameter("max_segment_m").value)
        max_segment_deg = float(self.get_parameter("max_segment_deg").value)

        # Roll the policy forward in software until a segment budget is spent, the same
        # way subgoal_planner rolls the geometric servo. The waypoint then lies on the
        # trajectory the policy would actually have flown, rather than on a straight
        # line to a scaled point.
        current = start.copy()
        reached = False
        for iterations in range(1, int(self.get_parameter("max_policy_steps").value) + 1):
            translation_error = float(np.linalg.norm(goal[:3] - current[:3]))
            rotation_error = geodesic_angle_deg(current[3:6], goal[3:6])
            if (translation_error <= float(self.get_parameter("goal_tolerance_m").value)
                    and rotation_error <= float(self.get_parameter("goal_tolerance_deg").value)):
                reached = True
                break

            action, _ = self.model.predict(build_observation(current, goal), deterministic=True)
            action = np.asarray(action, dtype=np.float64).reshape(-1)
            proposed = current.copy()
            proposed[:3] = current[:3] + action[:3] * steps[:3]
            proposed[3:6] = current[3:6] + action[3:6] * steps[3:6]

            if (float(np.linalg.norm(proposed[:3] - start[:3])) > max_segment_m
                    or geodesic_angle_deg(start[3:6], proposed[3:6]) > max_segment_deg):
                break
            if np.allclose(proposed, current, atol=1e-12):
                break
            current = proposed
        else:
            iterations = int(self.get_parameter("max_policy_steps").value)

        subgoal = goal.copy() if reached else current
        subgoal[6] = start[6]  # the executor cannot move the jaw
        # Belt and braces: the rollout stops inside the budget, but a saturated policy
        # on the first step could still overshoot it.
        subgoal_cm = subgoal.copy(); subgoal_cm[:3] *= 100.0
        start_cm = start.copy(); start_cm[:3] *= 100.0
        subgoal_cm = clamp_to_budget(start_cm, subgoal_cm, max_segment_m, max_segment_deg)
        subgoal = subgoal_cm.copy(); subgoal[:3] /= 100.0

        travelled = float(np.linalg.norm(subgoal[:3] - start[:3]))
        if reached and travelled <= 1e-9:
            self.segment_pending = False
            self._publish_status("AT_GOAL", json.dumps({"remaining_m": 0.0}))
            return

        self.subgoal_pub.publish(
            vector_to_pose(subgoal, self.command_frame, self.get_clock().now().to_msg()))
        self.segment_pending = False
        self._publish_status("SEGMENT_PUBLISHED", json.dumps({
            "reached_final": reached,
            "policy_steps": iterations,
            "segment_translation_m": round(travelled, 6),
            "segment_rotation_deg": round(geodesic_angle_deg(start[3:6], subgoal[3:6]), 3),
            "remaining_translation_m": round(float(np.linalg.norm(goal[:3] - subgoal[:3])), 6),
            "remaining_rotation_deg": round(geodesic_angle_deg(subgoal[3:6], goal[3:6]), 3),
        }))

    def _publish_status(self, state: str, detail: str) -> None:
        msg = String()
        msg.data = json.dumps({"schema": "suturing_runtime.policy_servo.v1",
                               "state": state, "detail": detail})
        self.status_pub.publish(msg)
        self.get_logger().info(f"D14_POLICY_SERVO state={state} detail={detail}")


def main() -> None:
    rclpy.init()
    node = PolicyServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
