"""Inference-only ROS 2 bridge for the historical SurgicAI Approach policy."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import SetBool, Trigger

from .policy_contract import (
    checkpoint_observation,
    integrate_checkpoint_action,
    quaternion_xyzw_to_legacy_rpy,
    rotation_error_rad,
    rpy_to_quaternion_xyzw,
)


ACK = "I_HAVE_OPERATOR_AND_ESTOP"


class ApproachPolicyNode(Node):
    """Load a frozen TD3-BC ZIP and publish one bounded Cartesian target at a time."""

    def __init__(self) -> None:
        super().__init__("approach_policy_node")
        defaults = {
            "model_path": "",
            "model_sha256": "",
            "surgicai_rl_path": "",
            "device": "cpu",
            "measured_pose_topic": "/suturing/psm1/measured_pose",
            "jaw_topic": "/suturing/psm1/jaw/measured_js",
            "goal_topic": "/suturing/approach/goal",
            "next_goal_topic": "/suturing/policy/next_goal",
            "preview_topic": "/suturing/policy/next_goal_preview",
            "jaw_preview_topic": "/suturing/policy/jaw_preview",
            "action_topic": "/suturing/policy/action",
            "status_topic": "/suturing/policy/status",
            "completion_topic": "/suturing/policy/complete",
            "command_frame": "PSM1_psm_base_link",
            "operator_acknowledgement": "NOT_ACKNOWLEDGED",
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        numeric = {
            "control_hz": 10.0,
            "max_input_age_s": 0.5,
            "settle_translation_m": 0.0005,
            "settle_rotation_deg": 1.0,
            "goal_translation_tolerance_m": 0.001,
            "goal_rotation_tolerance_deg": 2.0,
            "max_initial_goal_translation_m": 0.010,
            "max_initial_goal_rotation_deg": 30.0,
            "translation_step_m": 0.0015,
            "rotation_step_deg": 3.0,
            "jaw_step": 0.05,
            "desired_jaw": 0.0,
            "jaw_min": 0.0,
            "jaw_max": 1.0,
            "max_steps": 100,
        }
        for key, value in numeric.items():
            self.declare_parameter(key, value)
        self.declare_parameter("enabled", False)
        self.declare_parameter("enable_policy_output", False)
        self.declare_parameter("auto_step", True)
        self.declare_parameter("workspace_min_m", [-0.20, -0.20, -0.05])
        self.declare_parameter("workspace_max_m", [0.20, 0.20, 0.25])

        get = lambda key: self.get_parameter(key).value
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Executor requests transient-local QoS so the first policy step must also latch.
        self.next_pub = self.create_publisher(PoseStamped, str(get("next_goal_topic")), latched)
        self.preview_pub = self.create_publisher(PoseStamped, str(get("preview_topic")), 10)
        self.jaw_preview_pub = self.create_publisher(JointState, str(get("jaw_preview_topic")), 10)
        self.action_pub = self.create_publisher(Float32MultiArray, str(get("action_topic")), 10)
        self.status_pub = self.create_publisher(String, str(get("status_topic")), latched)
        self.completion_pub = self.create_publisher(Bool, str(get("completion_topic")), 10)
        self.create_subscription(PoseStamped, str(get("measured_pose_topic")), self._measured, 10)
        self.create_subscription(JointState, str(get("jaw_topic")), self._jaw, 10)
        self.create_subscription(PoseStamped, str(get("goal_topic")), self._goal, latched)
        self.create_service(SetBool, "/suturing/policy/arm", self._arm)
        self.create_service(Trigger, "/suturing/policy/step_once", self._step_once)
        self.create_service(Trigger, "/suturing/policy/stop", self._stop)

        self.model = None
        self.measured_msg = self.goal_msg = None
        self.measured_at = self.goal_at = self.jaw_at = None
        self.measured_jaw = None
        self.command_state = self.last_command = None
        self.armed = self.complete = False
        self.step_count = 0
        self._inference_busy = False
        if bool(get("enabled")):
            try:
                self.model = self._load_model()
                self._publish_status("READY", "model loaded; policy is disarmed")
            except Exception as exc:
                self._publish_status("MODEL_ERROR", f"{type(exc).__name__}: {exc}")
                self.get_logger().error(f"D13-E01-MODEL-LOAD {type(exc).__name__}: {exc}")
        else:
            self._publish_status("DISABLED", "set enabled=true and configure model_path")
        self.create_timer(1.0 / float(get("control_hz")), self._tick)

    def _load_model(self):
        model_path = Path(str(self.get_parameter("model_path").value)).expanduser().resolve()
        if not model_path.is_file():
            raise FileNotFoundError(f"model_path is not a file: {model_path}")
        expected_sha = str(self.get_parameter("model_sha256").value).strip().lower()
        if expected_sha:
            digest = hashlib.sha256()
            with model_path.open("rb") as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(block)
            actual_sha = digest.hexdigest()
            if actual_sha != expected_sha:
                raise ValueError(f"model SHA256 mismatch expected={expected_sha} actual={actual_sha}")

        rl_path = Path(str(self.get_parameter("surgicai_rl_path").value)).expanduser().resolve()
        if not (rl_path / "RL_algo" / "td3_BC.py").is_file():
            raise FileNotFoundError(
                "surgicai_rl_path must be the directory containing RL_algo/td3_BC.py: "
                f"{rl_path}"
            )
        if str(rl_path) not in sys.path:
            sys.path.insert(0, str(rl_path))

        from RL_algo.td3_BC import TD3_BC
        from stable_baselines3.common.buffers import DictReplayBuffer

        # Historical archives serialize DemoHerReplayBuffer kwargs. Replacing the
        # unused training buffer lets inference load without constructing AMBF/Gym.
        model = TD3_BC.load(
            str(model_path),
            device=str(self.get_parameter("device").value),
            custom_objects={
                "replay_buffer_class": DictReplayBuffer,
                "replay_buffer_kwargs": {},
            },
        )
        expected = {"observation": (21,), "achieved_goal": (7,), "desired_goal": (7,)}
        spaces = getattr(model.observation_space, "spaces", {})
        actual = {key: tuple(space.shape) for key, space in spaces.items()}
        if actual != expected or tuple(model.action_space.shape) != (7,):
            raise ValueError(f"unsupported checkpoint spaces observation={actual} action={model.action_space}")
        return model

    def _output_contract_ok(self) -> bool:
        return bool(self.get_parameter("enable_policy_output").value) and str(
            self.get_parameter("operator_acknowledgement").value) == ACK

    def _measured(self, msg: PoseStamped) -> None:
        self.measured_msg, self.measured_at = msg, time.monotonic()

    def _jaw(self, msg: JointState) -> None:
        if msg.position and math.isfinite(float(msg.position[0])):
            self.measured_jaw, self.jaw_at = float(msg.position[0]), time.monotonic()

    def _goal(self, msg: PoseStamped) -> None:
        self.goal_msg, self.goal_at = msg, time.monotonic()

    @staticmethod
    def _pose_vector(msg: PoseStamped, jaw: float) -> np.ndarray:
        p, q = msg.pose.position, msg.pose.orientation
        rpy = quaternion_xyzw_to_legacy_rpy([q.x, q.y, q.z, q.w])
        return np.asarray([p.x, p.y, p.z, *rpy, jaw], dtype=np.float32)

    def _fresh(self):
        if self.model is None:
            return False, "D13-E02-MODEL-NOT-READY"
        if self.measured_msg is None or self.goal_msg is None or self.measured_jaw is None:
            return False, "D13-E03-MISSING-MEASURED-GOAL-OR-JAW"
        expected = str(self.get_parameter("command_frame").value)
        if self.measured_msg.header.frame_id != expected or self.goal_msg.header.frame_id != expected:
            return False, (f"D13-E04-FRAME expected={expected} "
                           f"measured={self.measured_msg.header.frame_id} goal={self.goal_msg.header.frame_id}")
        maximum = float(self.get_parameter("max_input_age_s").value)
        now = time.monotonic()
        if now - self.measured_at > maximum or now - self.jaw_at > maximum:
            return False, "D13-E05-STALE-MEASURED-POSE-OR-JAW"
        jaw_low = float(self.get_parameter("jaw_min").value)
        jaw_high = float(self.get_parameter("jaw_max").value)
        if not jaw_low <= self.measured_jaw <= jaw_high:
            return False, f"D13-E13-JAW-RANGE measured={self.measured_jaw} expected=[{jaw_low},{jaw_high}]"
        # The Approach goal is intentionally frozen per episode, so goal age is audited
        # upstream but is not treated as streaming feedback here.
        return True, "ok"

    def _arm(self, request, response):
        if request.data:
            ok, reason = self._fresh()
            if not ok:
                response.success, response.message = False, reason
                return response
            if not self._output_contract_ok():
                response.success = False
                response.message = "D13-E06-OUTPUT-LOCKED: enable_policy_output=true plus exact acknowledgement required"
                return response
            self.command_state = self._pose_vector(self.measured_msg, self.measured_jaw)
            desired = self._pose_vector(self.goal_msg, float(self.get_parameter("desired_jaw").value))
            initial_translation = float(np.linalg.norm(self.command_state[:3] - desired[:3]))
            initial_rotation_deg = math.degrees(rotation_error_rad(self.command_state[3:6], desired[3:6]))
            if initial_translation > float(self.get_parameter("max_initial_goal_translation_m").value):
                response.success = False
                response.message = f"D13-E14-INITIAL-DISTANCE {initial_translation:.6f}m"
                return response
            if initial_rotation_deg > float(self.get_parameter("max_initial_goal_rotation_deg").value):
                response.success = False
                response.message = f"D13-E15-INITIAL-ROTATION {initial_rotation_deg:.3f}deg"
                return response
            low = np.asarray(self.get_parameter("workspace_min_m").value)
            high = np.asarray(self.get_parameter("workspace_max_m").value)
            if np.any(self.command_state[:3] < low) or np.any(self.command_state[:3] > high) or \
                    np.any(desired[:3] < low) or np.any(desired[:3] > high):
                response.success, response.message = False, "D13-E16-INITIAL-WORKSPACE"
                return response
            self.last_command = None
            self.step_count = 0
            self.complete = False
            self.armed = True
            self.completion_pub.publish(Bool(data=False))
            response.success, response.message = True, "policy armed; command state seeded from measured feedback"
            self._publish_status("ARMED", response.message)
        else:
            self.armed = False
            response.success, response.message = True, "policy disarmed"
            self._publish_status("DISARMED", response.message)
        return response

    def _step_once(self, request, response):
        del request
        ok, reason = self._fresh()
        if not ok:
            response.success, response.message = False, reason
            return response
        if self.armed:
            measured = self._pose_vector(self.measured_msg, self.measured_jaw)
            if self.last_command is not None and not self._settled(measured, self.last_command):
                response.success = False
                response.message = "D13-E12-PREVIOUS-STEP-NOT-SETTLED"
                return response
            if self.step_count >= int(self.get_parameter("max_steps").value):
                response.success, response.message = False, "D13-E07-MAX-STEPS"
                return response
            success, detail = self._infer_and_publish(commit=True)
        else:
            self.command_state = self._pose_vector(self.measured_msg, self.measured_jaw)
            success, detail = self._infer_and_publish(commit=False)
        response.success, response.message = success, detail
        return response

    def _stop(self, request, response):
        del request
        self.armed = False
        self.complete = False
        response.success, response.message = True, "policy stopped and disarmed"
        self._publish_status("STOPPED", response.message)
        return response

    def _settled(self, measured: np.ndarray, command: np.ndarray) -> bool:
        translation = float(np.linalg.norm(measured[:3] - command[:3]))
        rotation = rotation_error_rad(measured[3:6], command[3:6])
        return translation <= float(self.get_parameter("settle_translation_m").value) and rotation <= math.radians(
            float(self.get_parameter("settle_rotation_deg").value)
        )

    def _at_goal(self, measured: np.ndarray, desired: np.ndarray) -> bool:
        translation = float(np.linalg.norm(measured[:3] - desired[:3]))
        rotation = rotation_error_rad(measured[3:6], desired[3:6])
        return translation <= float(self.get_parameter("goal_translation_tolerance_m").value) and rotation <= math.radians(
            float(self.get_parameter("goal_rotation_tolerance_deg").value)
        )

    def _tick(self) -> None:
        if not self.armed or not bool(self.get_parameter("auto_step").value) or self._inference_busy:
            return
        ok, reason = self._fresh()
        if not ok:
            self._fault(reason)
            return
        measured = self._pose_vector(self.measured_msg, self.measured_jaw)
        desired = self._pose_vector(self.goal_msg, float(self.get_parameter("desired_jaw").value))
        if self._at_goal(measured, desired):
            self.armed, self.complete = False, True
            self.completion_pub.publish(Bool(data=True))
            self._publish_status("COMPLETE", f"goal reached after {self.step_count} policy steps; disarmed")
            return
        if self.step_count >= int(self.get_parameter("max_steps").value):
            self._fault("D13-E07-MAX-STEPS")
            return
        if self.last_command is not None and not self._settled(measured, self.last_command):
            return
        self._infer_and_publish(commit=True)

    def _infer_and_publish(self, *, commit: bool):
        if self._inference_busy:
            return False, "D13-E08-INFERENCE-BUSY"
        self._inference_busy = True
        try:
            current = self.command_state.copy()
            desired = self._pose_vector(self.goal_msg, float(self.get_parameter("desired_jaw").value))
            observation = checkpoint_observation(current, desired)
            action, _ = self.model.predict(observation, deterministic=True)
            action = np.asarray(action, dtype=np.float32).reshape(-1)
            if action.shape != (7,) or not np.all(np.isfinite(action)):
                raise ValueError(f"model returned invalid action shape/value: {action}")
            step_size = np.asarray([
                *([float(self.get_parameter("translation_step_m").value)] * 3),
                *([math.radians(float(self.get_parameter("rotation_step_deg").value))] * 3),
                float(self.get_parameter("jaw_step").value),
            ], dtype=np.float32)
            proposed = integrate_checkpoint_action(current, action, step_size)
            low = np.asarray(self.get_parameter("workspace_min_m").value, dtype=np.float32)
            high = np.asarray(self.get_parameter("workspace_max_m").value, dtype=np.float32)
            if low.shape != (3,) or high.shape != (3,) or np.any(proposed[:3] < low) or np.any(proposed[:3] > high):
                raise ValueError(f"D13-E09-WORKSPACE proposed_xyz={proposed[:3].tolist()}")
            proposed[6] = float(np.clip(proposed[6],
                                        float(self.get_parameter("jaw_min").value),
                                        float(self.get_parameter("jaw_max").value)))

            pose = self._pose_message(proposed)
            self.preview_pub.publish(pose)
            jaw = JointState()
            jaw.header = pose.header
            jaw.name = ["legacy_policy_jaw_preview"]
            jaw.position = [float(proposed[6])]
            self.jaw_preview_pub.publish(jaw)
            self.action_pub.publish(Float32MultiArray(data=action.astype(float).tolist()))
            if commit:
                if not self.armed or not self._output_contract_ok():
                    return False, "D13-E10-NOT-ARMED-OR-OUTPUT-LOCKED"
                self.command_state = proposed
                self.last_command = proposed.copy()
                self.step_count += 1
                self.completion_pub.publish(Bool(data=False))
                self.next_pub.publish(pose)
                detail = f"published policy step {self.step_count}; jaw is preview-only"
                self._publish_status("RUNNING", detail, action=action)
                return True, detail
            detail = "preview generated; no command goal published because policy is disarmed"
            self._publish_status("PREVIEW", detail, action=action)
            return True, detail
        except Exception as exc:
            detail = f"D13-E11-INFERENCE {type(exc).__name__}: {exc}"
            if commit:
                self._fault(detail)
            else:
                self._publish_status("INFERENCE_ERROR", detail)
            return False, detail
        finally:
            self._inference_busy = False

    def _pose_message(self, state: np.ndarray) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("command_frame").value)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, state[:3])
        quaternion = rpy_to_quaternion_xyzw(state[3:6])
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = map(float, quaternion)
        return msg

    def _fault(self, detail: str) -> None:
        self.armed = False
        self._publish_status("FAULT", detail)
        self.get_logger().error(detail)

    def _publish_status(self, state: str, detail: str, action=None) -> None:
        payload = {
            "schema": "suturing_runtime.approach_policy.v1",
            "state": state,
            "detail": detail,
            "enabled": bool(self.get_parameter("enabled").value),
            "armed": self.armed,
            "complete": self.complete,
            "step_count": self.step_count,
            "output_contract_ok": self._output_contract_ok(),
            "contract": "checkpoint_compat_command_integrated_xyz_cm_rpy_rad_jaw",
            "jaw_output": "preview_only",
        }
        if action is not None:
            payload["action"] = np.asarray(action, dtype=float).tolist()
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ApproachPolicyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
