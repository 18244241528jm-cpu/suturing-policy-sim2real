"""Isolated controller candidates for the 2026-07-21 pipeline experiments.

This module intentionally lives outside ``environments/SurgicAI``.  The
experiment wrapper imports it at runtime and monkey-patches the evaluation
module in memory; no production source file is modified.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
from scipy.spatial.transform import Rotation


Array = np.ndarray


def wrap_to_pi(value: Array | float) -> Array:
    value = np.asarray(value, dtype=np.float64)
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def _validated_inputs(obs: dict, step_size: Array) -> tuple[Array, Array, Array]:
    achieved = np.asarray(obs["achieved_goal"], dtype=np.float64)
    desired = np.asarray(obs["desired_goal"], dtype=np.float64)
    steps = np.asarray(step_size, dtype=np.float64)
    if achieved.shape != (7,) or desired.shape != (7,) or steps.shape != (7,):
        raise ValueError("achieved_goal, desired_goal, and step_size must be 7-vectors")
    if np.any(steps <= 0.0):
        raise ValueError("step_size must be positive")
    return achieved, desired, steps


def _rotation_action(
    current_rpy: Array,
    desired_rpy: Array,
    rotation_steps: Array,
    *,
    gain: float,
    cap_fraction: float,
) -> tuple[Array, float]:
    """Return an RPY command increment derived from an SO(3) relative rotation.

    The environment accepts additive RPY actions.  We therefore compute a
    bounded geodesic step on SO(3), convert the resulting next orientation back
    to RPY, and only then form the additive increment expected by the existing
    interface.  This avoids commanding the long Euler branch near +/-pi.
    """

    current = Rotation.from_euler("xyz", current_rpy)
    desired = Rotation.from_euler("xyz", desired_rpy)
    relative = current.inv() * desired
    rotvec = relative.as_rotvec()
    angle = float(np.linalg.norm(rotvec))
    if angle <= 1.0e-12:
        return np.zeros(3, dtype=np.float64), 0.0

    max_geodesic_step = float(np.min(rotation_steps) * cap_fraction)
    commanded_angle = min(max_geodesic_step, gain * angle)
    next_rotation = current * Rotation.from_rotvec(rotvec * (commanded_angle / angle))
    next_rpy = next_rotation.as_euler("xyz")
    rpy_increment = wrap_to_pi(next_rpy - current_rpy)
    action = np.clip(rpy_increment / rotation_steps, -cap_fraction, cap_fraction)
    return action, angle


def _adaptive_se3_action(
    obs: dict,
    step_size: Array,
    *,
    staged: bool,
) -> Array:
    achieved, desired, steps = _validated_inputs(obs, step_size)

    translation_error_m = (desired[:3] - achieved[:3]) / 100.0
    translation_norm_m = float(np.linalg.norm(translation_error_m))

    # Full steps while far away; progressively smaller commands near the
    # 3-mm evaluation boundary to reduce command/measurement overshoot.
    if translation_norm_m > 0.010:
        trans_gain, trans_cap = 0.85, 1.00
    elif translation_norm_m > 0.003:
        trans_gain, trans_cap = 0.65, 0.70
    elif translation_norm_m > 0.001:
        trans_gain, trans_cap = 0.50, 0.40
    else:
        trans_gain, trans_cap = 0.35, 0.20

    current = Rotation.from_euler("xyz", achieved[3:6])
    desired_rotation = Rotation.from_euler("xyz", desired[3:6])
    rotation_error_rad = float(np.linalg.norm((current.inv() * desired_rotation).as_rotvec()))

    if rotation_error_rad > np.deg2rad(30.0):
        rot_gain, rot_cap = 0.80, 1.00
    elif rotation_error_rad > np.deg2rad(10.0):
        rot_gain, rot_cap = 0.65, 0.75
    elif rotation_error_rad > np.deg2rad(3.0):
        rot_gain, rot_cap = 0.50, 0.45
    else:
        rot_gain, rot_cap = 0.35, 0.25

    translation_action = np.clip(
        trans_gain * translation_error_m / steps[:3],
        -trans_cap,
        trans_cap,
    )

    # The staged candidate first removes large orientation error.  It still
    # allows 20% translation so the arm does not become completely stationary.
    if staged and rotation_error_rad > np.deg2rad(15.0):
        translation_action *= 0.20
    elif staged and rotation_error_rad > np.deg2rad(8.0):
        translation_action *= 0.55

    rotation_action, _ = _rotation_action(
        achieved[3:6],
        desired[3:6],
        steps[3:6],
        gain=rot_gain,
        cap_fraction=rot_cap,
    )

    jaw_action = float(np.clip((desired[6] - achieved[6]) / steps[6], -1.0, 1.0))
    return np.asarray(
        [*translation_action, *rotation_action, jaw_action],
        dtype=np.float32,
    )


def adaptive_se3_goal_servo_action(
    obs: dict,
    step_size: Array,
    raw_rpy_contract: bool = False,
) -> Array:
    """Gain-scheduled relative SE(3) servo; signature matches Model_evaluation."""

    del raw_rpy_contract  # This controller is intentionally branch-independent.
    return _adaptive_se3_action(obs, step_size, staged=False)


def staged_se3_goal_servo_action(
    obs: dict,
    step_size: Array,
    raw_rpy_contract: bool = False,
) -> Array:
    """Orientation-first version used only if the adaptive servo still oscillates."""

    del raw_rpy_contract
    return _adaptive_se3_action(obs, step_size, staged=True)


def make_policy_residual_action(
    servo_action: Callable[[dict, Array, bool], Array],
) -> Callable[..., Array]:
    """Create a policy-plus-servo adapter with an SE(3)-aware direction guard."""

    def policy_residual_action(
        obs: dict,
        policy_action: Array,
        step_size: Array,
        raw_rpy_contract: bool = False,
        policy_weight: float = 0.50,
        servo_weight: float = 0.75,
        direction_guard: bool = True,
    ) -> Array:
        policy = np.asarray(policy_action, dtype=np.float32).reshape(-1)
        servo = np.asarray(
            servo_action(obs, step_size, raw_rpy_contract),
            dtype=np.float32,
        ).reshape(-1)
        if policy.shape != (7,) or servo.shape != (7,):
            raise ValueError("policy and servo actions must be 7-vectors")

        guarded = policy.copy()
        if direction_guard:
            # When the policy points against a confident servo correction, do
            # not double-count the servo; simply suppress that policy axis.
            opposing = (np.abs(servo) > 0.05) & (guarded * servo < 0.0)
            guarded[opposing] = 0.0

        combined = float(policy_weight) * guarded + float(servo_weight) * servo
        return np.clip(combined, -1.0, 1.0).astype(np.float32)

    return policy_residual_action

