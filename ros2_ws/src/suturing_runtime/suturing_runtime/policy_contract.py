"""Pure helpers for the historical seven-dimensional Approach policy contract."""

from __future__ import annotations

import math

import numpy as np


TRANSLATION_SCALE = np.asarray([100.0, 100.0, 100.0], dtype=np.float32)


def quaternion_xyzw_to_legacy_rpy(quaternion) -> np.ndarray:
    """Return XYZ fixed-axis RPY on the branch used by the old AMBF policy."""
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.linalg.norm([x, y, z, w]))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("quaternion must be finite and non-zero")
    x, y, z, w = (np.asarray([x, y, z, w]) / norm).tolist()

    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = float(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    pitch = math.asin(sin_pitch)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)

    # scene_manager.Frame2Vec(bound=True) subtracts 2*pi from positive roll.
    if roll > 0.0:
        roll -= 2.0 * math.pi
    return np.asarray([roll, pitch, yaw], dtype=np.float32)


def rpy_to_quaternion_xyzw(rpy) -> np.ndarray:
    roll, pitch, yaw = np.asarray(rpy, dtype=np.float64)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    quaternion = np.asarray([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    return quaternion.astype(np.float32)


def checkpoint_observation(current_raw, desired_raw) -> dict[str, np.ndarray]:
    """Build the exact GoalEnv dict: xyz in cm, RPY in rad, legacy jaw scalar."""
    current = np.asarray(current_raw, dtype=np.float32)
    desired = np.asarray(desired_raw, dtype=np.float32)
    if current.shape != (7,) or desired.shape != (7,):
        raise ValueError("current_raw and desired_raw must be seven-vectors")
    if not np.all(np.isfinite(current)) or not np.all(np.isfinite(desired)):
        raise ValueError("policy observation contains non-finite values")

    delta = desired - current  # Historical checkpoint contract does not wrap RPY delta.
    observation = np.concatenate((current, desired, delta)).astype(np.float32)
    observation[[0, 1, 2, 7, 8, 9, 14, 15, 16]] *= 100.0
    multiplier = np.asarray([100.0, 100.0, 100.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float32)
    return {
        "observation": observation,
        "achieved_goal": current * multiplier,
        "desired_goal": desired * multiplier,
    }


def integrate_checkpoint_action(current_raw, action, step_size_raw) -> np.ndarray:
    """Apply the old command-integrated transition after clipping action to [-1, 1]."""
    current = np.asarray(current_raw, dtype=np.float32)
    clipped = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
    steps = np.asarray(step_size_raw, dtype=np.float32)
    if current.shape != (7,) or clipped.shape != (7,) or steps.shape != (7,):
        raise ValueError("current, action, and step_size must be seven-vectors")
    if np.any(steps <= 0.0):
        raise ValueError("step_size must be positive")
    return (current + clipped * steps).astype(np.float32)


def rotation_error_rad(rpy_a, rpy_b) -> float:
    """Geodesic SO(3) error between two RPY triples."""
    qa = rpy_to_quaternion_xyzw(rpy_a).astype(np.float64)
    qb = rpy_to_quaternion_xyzw(rpy_b).astype(np.float64)
    # Relative quaternion vector form is stable near zero; acos(dot) is not.
    ax, ay, az, aw = qa
    bx, by, bz, bw = qb
    relative_vector = np.asarray([
        aw * bx - ax * bw - ay * bz + az * by,
        aw * by + ax * bz - ay * bw - az * bx,
        aw * bz - ax * by + ay * bx - az * bw,
    ])
    relative_w = aw * bw + ax * bx + ay * by + az * bz
    return 2.0 * math.atan2(float(np.linalg.norm(relative_vector)), abs(float(relative_w)))
