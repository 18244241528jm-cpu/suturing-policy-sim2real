#!/usr/bin/env python3
"""Pure numpy geometry for P9a needle-pose -> Approach-goal conversion.

This intentionally mirrors:
  * RL/utils/needle.py::Needle.get_pose_angle
  * RL/utils/scene_manager.py::SceneManager.needle_goal_evaluator
  * RL/utils/scene_manager.py::SceneManager.Frame2Vec

Keeping the conversion independent from AMBF lets the exact same reset snapshot
produce both a GT-derived goal and a FoundationPose-derived goal.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation


NEEDLE_RADIUS_MODEL_UNITS = 0.1018
NEEDLE_MODEL_TO_METERS = 0.1


def transform(rotation: np.ndarray | None = None, translation=None) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    if rotation is not None:
        result[:3, :3] = np.asarray(rotation, dtype=np.float64)
    if translation is not None:
        result[:3, 3] = np.asarray(translation, dtype=np.float64)
    return result


def rz(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def matrix_to_goal_vector(matrix: np.ndarray) -> np.ndarray:
    """Match SceneManager.Frame2Vec's historical Euler branch."""
    matrix = np.asarray(matrix, dtype=np.float64)
    roll, pitch, yaw = Rotation.from_matrix(matrix[:3, :3]).as_euler(
        "xyz", degrees=False
    )
    if roll <= -2.0 * np.pi:
        roll += 2.0 * np.pi
    elif roll > 0.0:
        roll -= 2.0 * np.pi
    return np.array(
        [
            matrix[0, 3],
            matrix[1, 3],
            matrix[2, 3],
            roll,
            pitch,
            yaw,
            0.0,
        ],
        dtype=np.float64,
    )


def needle_pose_to_goal(
    needle_in_world: np.ndarray,
    world_to_psm_base: np.ndarray,
    grasp_angle_deg: float,
    lift_height_m: float = 0.007,
) -> np.ndarray:
    """Convert a needle body pose in world into the raw 7-D Approach goal."""
    angle_rad = np.deg2rad(float(grasp_angle_deg))
    radius_m = NEEDLE_RADIUS_MODEL_UNITS * NEEDLE_MODEL_TO_METERS
    angle_in_needle = transform(
        rz(-angle_rad),
        [-radius_m * np.cos(angle_rad), radius_m * np.sin(angle_rad), 0.0],
    )
    lift_in_grasp = transform(translation=[0.0, 0.0, float(lift_height_m)])
    gripper_in_lift = transform(
        np.array(
            [
                [0.0, -1.0, 0.0],
                [-1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
            ]
        )
    )
    gripper_in_base = (
        np.asarray(world_to_psm_base, dtype=np.float64)
        @ np.asarray(needle_in_world, dtype=np.float64)
        @ angle_in_needle
        @ lift_in_grasp
        @ gripper_in_lift
    )
    return matrix_to_goal_vector(gripper_in_base)


def rotation_distance_deg(first: np.ndarray, second: np.ndarray) -> float:
    relative = np.asarray(first)[:3, :3] @ np.asarray(second)[:3, :3].T
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def goal_pose_error(first: np.ndarray, second: np.ndarray) -> tuple[float, float]:
    first = np.asarray(first, dtype=np.float64)
    second = np.asarray(second, dtype=np.float64)
    first_pose = transform(
        Rotation.from_euler("xyz", first[3:6]).as_matrix(), first[:3]
    )
    second_pose = transform(
        Rotation.from_euler("xyz", second[3:6]).as_matrix(), second[:3]
    )
    return (
        float(np.linalg.norm(first[:3] - second[:3]) * 1000.0),
        rotation_distance_deg(first_pose, second_pose),
    )
