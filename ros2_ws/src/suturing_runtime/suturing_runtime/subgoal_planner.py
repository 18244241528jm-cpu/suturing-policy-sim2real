"""Segment planner that turns a far Approach goal into executor-sized sub-goals.

``guarded_pose_executor`` refuses any goal further than ``max_initial_translation_m``
(0.010 m on the real cart) with ``D10-E56-DISTANCE``, while the simulated Approach
stroke is about 61 mm.  Rather than widening that safety valve, this module rolls the
SE(3) goal servo forward in software and emits a waypoint at the segment boundary, so
every command the executor sees is already inside its own limits.

Units follow the controller contract, not the ROS one: ``xyz`` in **centimetres**,
``rpy`` in radians, jaw normalised.  ``se3_goal_servo`` divides the translation error
by 100 internally, so feeding it metres understates the error a hundredfold and the
arm creeps.  Convert at the ROS boundary and nowhere else.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation

from .se3_goal_servo import adaptive_se3_goal_servo_action

# Matches the simulated environment's step size: metres for xyz, radians for rpy.
DEFAULT_STEP_SIZE = (0.001, 0.001, 0.001, 0.05235988, 0.05235988, 0.05235988, 0.05)


def geodesic_angle_deg(rpy_a, rpy_b) -> float:
    """Shortest SO(3) angle between two xyz-Euler triples, in degrees."""
    a = Rotation.from_euler("xyz", np.asarray(rpy_a, dtype=np.float64))
    b = Rotation.from_euler("xyz", np.asarray(rpy_b, dtype=np.float64))
    return float(np.degrees(np.linalg.norm((a.inv() * b).as_rotvec())))


def plan_subgoal(
    current_cm_rad,
    final_cm_rad,
    *,
    step_size=DEFAULT_STEP_SIZE,
    max_segment_m: float = 0.008,
    max_segment_deg: float = 25.0,
    goal_tolerance_m: float = 0.001,
    goal_tolerance_deg: float = 2.0,
    max_iterations: int = 2000,
):
    """Roll the servo forward until a segment budget is spent; return that waypoint.

    Returns ``(subgoal_cm_rad, info)``.  ``info["reached_final"]`` is True when the
    rollout arrived inside tolerance of the final goal, in which case the waypoint is
    the final goal itself and the caller can stop segmenting.

    The rollout reproduces the servo's own gain schedule instead of taking a straight
    line to a scaled point, so the emitted waypoint lies on the trajectory the servo
    would actually have followed.
    """
    current = np.asarray(current_cm_rad, dtype=np.float64).copy()
    final = np.asarray(final_cm_rad, dtype=np.float64)
    steps = np.asarray(step_size, dtype=np.float64)
    if current.shape != (7,) or final.shape != (7,):
        raise ValueError("current and final poses must be 7-vectors (xyz cm, rpy rad, jaw)")

    start = current.copy()
    # A pose-only executor cannot move the jaw, so hold it and keep the jaw action at 0.
    final = final.copy()
    final[6] = start[6]

    reached_final = False
    iterations = 0
    for iterations in range(1, max_iterations + 1):
        translation_error_m = float(np.linalg.norm((final[:3] - current[:3]) / 100.0))
        rotation_error_deg = geodesic_angle_deg(current[3:6], final[3:6])
        if translation_error_m <= goal_tolerance_m and rotation_error_deg <= goal_tolerance_deg:
            reached_final = True
            break

        action = adaptive_se3_goal_servo_action(
            {"achieved_goal": current, "desired_goal": final}, steps
        )
        action = np.asarray(action, dtype=np.float64)
        proposed = current.copy()
        # steps[:3] is metres; the pose vector is centimetres.
        proposed[:3] = current[:3] + action[:3] * steps[:3] * 100.0
        proposed[3:6] = current[3:6] + action[3:6] * steps[3:6]

        spent_m = float(np.linalg.norm((proposed[:3] - start[:3]) / 100.0))
        spent_deg = geodesic_angle_deg(start[3:6], proposed[3:6])
        if spent_m > max_segment_m or spent_deg > max_segment_deg:
            # Stop before crossing the budget; `current` is the last in-budget pose.
            break

        if np.allclose(proposed, current, atol=1e-12):
            # The servo has nothing left to command; emitting the same pose forever
            # would look like progress to a caller that only watches the topic.
            break
        current = proposed

    subgoal = final.copy() if reached_final else current
    info = {
        "reached_final": reached_final,
        "iterations": iterations,
        "segment_translation_m": float(np.linalg.norm((subgoal[:3] - start[:3]) / 100.0)),
        "segment_rotation_deg": geodesic_angle_deg(start[3:6], subgoal[3:6]),
        "remaining_translation_m": float(np.linalg.norm((final[:3] - subgoal[:3]) / 100.0)),
        "remaining_rotation_deg": geodesic_angle_deg(subgoal[3:6], final[3:6]),
    }
    return subgoal, info


def clamp_to_budget(current_cm_rad, subgoal_cm_rad, max_segment_m, max_segment_deg):
    """Final backstop: shrink a waypoint that still exceeds the executor's limits.

    ``plan_subgoal`` already stops inside the budget, but a caller may supply a
    waypoint from elsewhere.  Emitting anything larger only earns a D10-E56/E57
    rejection, so clamp rather than let the executor refuse.
    """
    current = np.asarray(current_cm_rad, dtype=np.float64)
    subgoal = np.asarray(subgoal_cm_rad, dtype=np.float64).copy()

    delta_m = (subgoal[:3] - current[:3]) / 100.0
    distance_m = float(np.linalg.norm(delta_m))
    if distance_m > max_segment_m > 0.0:
        subgoal[:3] = current[:3] + (subgoal[:3] - current[:3]) * (max_segment_m / distance_m)

    angle_deg = geodesic_angle_deg(current[3:6], subgoal[3:6])
    if angle_deg > max_segment_deg > 0.0:
        start = Rotation.from_euler("xyz", current[3:6])
        target = Rotation.from_euler("xyz", subgoal[3:6])
        rotvec = (start.inv() * target).as_rotvec()
        scaled = rotvec * (math.radians(max_segment_deg) / math.radians(angle_deg))
        subgoal[3:6] = (start * Rotation.from_rotvec(scaled)).as_euler("xyz")
    return subgoal
