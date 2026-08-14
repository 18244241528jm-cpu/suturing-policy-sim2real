"""Explicitly armed, bounded PSM Cartesian Reach executor."""

from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .contract import bounded_pose_step, inside_workspace, matrix_to_quaternion_xyzw, pose_matrix


ACK = "I_HAVE_OPERATOR_AND_ESTOP"


class GuardedPoseExecutor(Node):
    def __init__(self) -> None:
        super().__init__("guarded_pose_executor")
        defaults = {"measured_pose_topic": "/suturing/psm1/measured_pose",
                    "goal_topic": "/suturing/approach/goal",
                    "preview_topic": "/suturing/execution/preview",
                    "raw_servo_topic": "/PSM1/servo_cp",
                    "raw_servo_type": "geometry_msgs/msg/PoseStamped",
                    "command_frame": "PSM1_psm_base_link",
                    "operator_acknowledgement": "NOT_ACKNOWLEDGED"}
        for key, value in defaults.items(): self.declare_parameter(key, value)
        for key, value in {"enable_output": False, "control_hz": 20.0, "max_input_age_s": 0.5,
                           "max_initial_translation_m": 0.010, "max_initial_rotation_deg": 30.0,
                           "max_linear_speed_m_s": 0.002, "max_angular_speed_deg_s": 5.0,
                           "translation_tolerance_m": 0.001, "rotation_tolerance_deg": 2.0,
                           "execution_timeout_s": 20.0}.items(): self.declare_parameter(key, value)
        self.declare_parameter("workspace_min_m", [-0.20,-0.20,-0.05])
        self.declare_parameter("workspace_max_m", [0.20,0.20,0.25])
        get = lambda key: self.get_parameter(key).value
        self.preview_pub = self.create_publisher(PoseStamped, get("preview_topic"), 10)
        self.command_pose_pub = None; self.command_transform_pub = None
        if self._output_contract_ok():
            raw_servo_type = str(get("raw_servo_type"))
            if raw_servo_type == "geometry_msgs/msg/PoseStamped":
                self.command_pose_pub = self.create_publisher(PoseStamped, get("raw_servo_topic"), 10)
            elif raw_servo_type == "geometry_msgs/msg/TransformStamped":
                self.command_transform_pub = self.create_publisher(TransformStamped, get("raw_servo_topic"), 10)
            else:
                raise ValueError(f"D10-E12-SERVO-TYPE unsupported raw_servo_type={raw_servo_type}")
        self.status_pub = self.create_publisher(String, "/suturing/execution/status", 10)
        self.create_subscription(PoseStamped, get("measured_pose_topic"), self._measured, 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, get("goal_topic"), self._goal, latched)
        self.create_service(SetBool, "/suturing/execution/arm", self._arm)
        self.create_service(Trigger, "/suturing/execution/execute_once", self._execute)
        self.create_service(Trigger, "/suturing/execution/stop", self._stop_service)
        self.measured = self.goal = None; self.measured_at = self.goal_at = None
        self.armed = self.active = False; self.started_at = None
        self.create_timer(1.0/float(get("control_hz")), self._tick)
        mode = "LIVE_CAPABLE_BUT_DISARMED" if self._output_contract_ok() else "PREVIEW_ONLY"
        self._publish_status(mode, "startup")
        self.get_logger().warning(f"D10_EXECUTOR_READY mode={mode}; startup never moves the robot")

    def _output_contract_ok(self) -> bool:
        return bool(self.get_parameter("enable_output").value) and str(
            self.get_parameter("operator_acknowledgement").value) == ACK

    @staticmethod
    def _matrix(msg: PoseStamped) -> np.ndarray:
        p, q = msg.pose.position, msg.pose.orientation
        return pose_matrix([p.x,p.y,p.z], [q.x,q.y,q.z,q.w])

    def _measured(self, msg): self.measured, self.measured_at = msg, time.monotonic()
    def _goal(self, msg): self.goal, self.goal_at = msg, time.monotonic()

    def _arm(self, request, response):
        if request.data and not self._output_contract_ok():
            response.success = False
            response.message = "D10-E50-OUTPUT_LOCKED: enable_output=true plus exact operator acknowledgement required."
            return response
        self.armed = bool(request.data)
        if not self.armed: self.active = False
        response.success = True; response.message = "armed" if self.armed else "disarmed"
        self._publish_status("ARMED" if self.armed else "DISARMED", response.message); return response

    def _fresh(self):
        if self.measured is None or self.goal is None: return False, "D10-E51-MISSING_POSE_OR_GOAL"
        maximum = float(self.get_parameter("max_input_age_s").value); now = time.monotonic()
        if now-self.measured_at > maximum: return False, "D10-E52-STALE_MEASURED_POSE"
        # The validated deployment contract intentionally freezes one goal while
        # ECM and needle stay fixed. Its source timestamp remains available for
        # audit, but a latched goal is not treated like continuous feedback.
        expected = str(self.get_parameter("command_frame").value)
        if self.measured.header.frame_id != expected or self.goal.header.frame_id != expected:
            return False, f"D10-E54-FRAME expected={expected} measured={self.measured.header.frame_id} goal={self.goal.header.frame_id}"
        return True, "ok"

    def _execute(self, request, response):
        del request
        if not self.armed: response.success=False; response.message="D10-E55-NOT_ARMED"; return response
        ok, reason = self._fresh()
        if not ok: response.success=False; response.message=reason; return response
        current, target = self._matrix(self.measured), self._matrix(self.goal)
        distance = float(np.linalg.norm(target[:3,3]-current[:3,3]))
        cosine = float(np.clip((np.trace(current[:3,:3].T@target[:3,:3])-1)/2, -1, 1))
        angle_deg = math.degrees(math.acos(cosine))
        if distance > float(self.get_parameter("max_initial_translation_m").value):
            response.success=False; response.message=f"D10-E56-DISTANCE {distance:.6f}m"; return response
        if angle_deg > float(self.get_parameter("max_initial_rotation_deg").value):
            response.success=False; response.message=f"D10-E57-ROTATION {angle_deg:.3f}deg"; return response
        low = np.asarray(self.get_parameter("workspace_min_m").value)
        high = np.asarray(self.get_parameter("workspace_max_m").value)
        if not inside_workspace(current[:3,3], low, high) or not inside_workspace(target[:3,3], low, high):
            response.success=False; response.message="D10-E58-WORKSPACE"; return response
        self.active=True; self.started_at=time.monotonic(); response.success=True
        response.message=f"accepted distance={distance:.6f}m rotation={angle_deg:.3f}deg"
        self._publish_status("EXECUTING", response.message); return response

    def _stop_service(self, request, response):
        del request; self.active=False; self.armed=False; response.success=True
        response.message="stopped and disarmed"; self._publish_status("STOPPED", response.message); return response

    def _fault(self, reason):
        self.active=False; self.armed=False; self._publish_status("FAULT", reason); self.get_logger().error(reason)

    def _tick(self):
        if not self.active: return
        ok, reason = self._fresh()
        if not ok: self._fault(reason); return
        if time.monotonic()-self.started_at > float(self.get_parameter("execution_timeout_s").value):
            self._fault("D10-E59-TIMEOUT"); return
        current, target = self._matrix(self.measured), self._matrix(self.goal)
        hz = float(self.get_parameter("control_hz").value)
        step, distance, angle = bounded_pose_step(current, target,
            float(self.get_parameter("max_linear_speed_m_s").value)/hz,
            math.radians(float(self.get_parameter("max_angular_speed_deg_s").value))/hz)
        preview=PoseStamped(); preview.header.stamp=self.get_clock().now().to_msg()
        preview.header.frame_id=str(self.get_parameter("command_frame").value)
        preview.pose.position.x,preview.pose.position.y,preview.pose.position.z=map(float,step[:3,3])
        q=matrix_to_quaternion_xyzw(step[:3,:3])
        preview.pose.orientation.x,preview.pose.orientation.y,preview.pose.orientation.z,preview.pose.orientation.w=map(float,q)
        self.preview_pub.publish(preview)
        if self._output_contract_ok():
            if self.command_pose_pub is not None:
                self.command_pose_pub.publish(preview)
            else:
                command=TransformStamped(); command.header=preview.header; command.child_frame_id="PSM1_tool_tip_command"
                command.transform.translation.x=preview.pose.position.x
                command.transform.translation.y=preview.pose.position.y
                command.transform.translation.z=preview.pose.position.z
                command.transform.rotation=preview.pose.orientation
                self.command_transform_pub.publish(command)
        if distance <= float(self.get_parameter("translation_tolerance_m").value) and \
           math.degrees(angle) <= float(self.get_parameter("rotation_tolerance_deg").value):
            self.active=False; self.armed=False
            self._publish_status("COMPLETE", f"distance={distance:.6f}m rotation={math.degrees(angle):.3f}deg; disarmed")

    def _publish_status(self, state, detail):
        msg=String(); msg.data=json.dumps({"schema":"suturing_runtime.execution.v1","state":state,
            "detail":detail,"armed":self.armed,"active":self.active,
            "output_contract_ok":self._output_contract_ok(),
            "raw_servo_topic":self.get_parameter("raw_servo_topic").value},sort_keys=True)
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init(); node=GuardedPoseExecutor()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
