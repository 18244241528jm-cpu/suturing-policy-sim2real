"""Pure geometry and safety helpers; intentionally no ROS import."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


NEEDLE_RADIUS_M = 0.1018 * 0.1


@dataclass(frozen=True)
class TopicContract:
    left_image: str = "/suturing/camera/left/image"
    right_image: str = "/suturing/camera/right/image"
    left_info: str = "/suturing/camera/left/camera_info"
    right_info: str = "/suturing/camera/right/camera_info"
    metric_depth: str = "/suturing/depth/metric"
    needle_mask: str = "/suturing/needle/mask"
    needle_pose_gated: str = "/suturing/needle/pose_gated"
    psm_pose: str = "/suturing/psm1/measured_pose"
    psm_twist: str = "/suturing/psm1/measured_twist"
    psm_jaw: str = "/suturing/psm1/jaw/measured_js"
    approach_goal: str = "/suturing/approach/goal"
    runtime_status: str = "/suturing/runtime/status"
    execution_preview: str = "/suturing/execution/preview"


def normalize_quaternion_xyzw(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(q))
    if norm < 1.0e-9 or not np.isfinite(norm):
        raise ValueError("invalid quaternion")
    return q / norm


def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = normalize_quaternion_xyzw(q)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=np.float64)


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    r = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.array([(r[2,1]-r[1,2])/s, (r[0,2]-r[2,0])/s,
                      (r[1,0]-r[0,1])/s, 0.25*s])
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = math.sqrt(max(1+r[0,0]-r[1,1]-r[2,2], 0.0))*2
            q = np.array([0.25*s, (r[0,1]+r[1,0])/s, (r[0,2]+r[2,0])/s, (r[2,1]-r[1,2])/s])
        elif i == 1:
            s = math.sqrt(max(1+r[1,1]-r[0,0]-r[2,2], 0.0))*2
            q = np.array([(r[0,1]+r[1,0])/s, 0.25*s, (r[1,2]+r[2,1])/s, (r[0,2]-r[2,0])/s])
        else:
            s = math.sqrt(max(1+r[2,2]-r[0,0]-r[1,1], 0.0))*2
            q = np.array([(r[0,2]+r[2,0])/s, (r[1,2]+r[2,1])/s, 0.25*s, (r[1,0]-r[0,1])/s])
    return normalize_quaternion_xyzw(q)


def pose_matrix(position_xyz: np.ndarray, quaternion_xyzw: np.ndarray) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = quaternion_to_matrix(quaternion_xyzw)
    out[:3, 3] = np.asarray(position_xyz, dtype=np.float64).reshape(3)
    return out


def transform_matrix(translation_xyz: np.ndarray, quaternion_xyzw: np.ndarray) -> np.ndarray:
    return pose_matrix(translation_xyz, quaternion_xyzw)


def approach_goal_matrix(needle_in_psm_base: np.ndarray, grasp_angle_deg: float = 12.5,
                         lift_height_m: float = 0.007) -> np.ndarray:
    """Mirror the validated P9a needle-pose to Approach-goal geometry."""
    angle = math.radians(float(grasp_angle_deg))
    c, s = math.cos(-angle), math.sin(-angle)
    angle_in_needle = np.eye(4)
    angle_in_needle[:3, :3] = np.array([[c,-s,0],[s,c,0],[0,0,1]])
    angle_in_needle[:3, 3] = [-NEEDLE_RADIUS_M*math.cos(angle),
                              NEEDLE_RADIUS_M*math.sin(angle), 0.0]
    lift = np.eye(4); lift[:3, 3] = [0.0, 0.0, float(lift_height_m)]
    gripper = np.eye(4)
    gripper[:3, :3] = np.array([[0,-1,0],[-1,0,0],[0,0,-1]], dtype=float)
    return np.asarray(needle_in_psm_base, dtype=np.float64) @ angle_in_needle @ lift @ gripper


def rotation_distance_rad(a: np.ndarray, b: np.ndarray) -> float:
    relative = np.asarray(a)[:3,:3].T @ np.asarray(b)[:3,:3]
    cosine = float(np.clip((np.trace(relative)-1.0)/2.0, -1.0, 1.0))
    return math.acos(cosine)


def quaternion_slerp(a: np.ndarray, b: np.ndarray, fraction: float) -> np.ndarray:
    qa, qb = normalize_quaternion_xyzw(a), normalize_quaternion_xyzw(b)
    dot = float(np.dot(qa, qb))
    if dot < 0: qb, dot = -qb, -dot
    dot = float(np.clip(dot, -1.0, 1.0)); fraction = float(np.clip(fraction, 0.0, 1.0))
    if dot > 0.9995: return normalize_quaternion_xyzw(qa + fraction*(qb-qa))
    theta = math.acos(dot)
    return normalize_quaternion_xyzw(math.sin((1-fraction)*theta)/math.sin(theta)*qa
                                     + math.sin(fraction*theta)/math.sin(theta)*qb)


def bounded_pose_step(current: np.ndarray, target: np.ndarray, max_translation_m: float,
                      max_rotation_rad: float) -> tuple[np.ndarray, float, float]:
    current = np.asarray(current, dtype=np.float64).reshape(4,4)
    target = np.asarray(target, dtype=np.float64).reshape(4,4)
    delta = target[:3,3] - current[:3,3]; distance = float(np.linalg.norm(delta))
    angle = rotation_distance_rad(current, target); out = current.copy()
    if distance > 0: out[:3,3] += delta * min(1.0, float(max_translation_m)/distance)
    qa, qb = matrix_to_quaternion_xyzw(current[:3,:3]), matrix_to_quaternion_xyzw(target[:3,:3])
    fraction = 1.0 if angle <= 1e-12 else min(1.0, float(max_rotation_rad)/angle)
    out[:3,:3] = quaternion_to_matrix(quaternion_slerp(qa, qb, fraction))
    return out, distance, angle


def inside_workspace(position_xyz: np.ndarray, minimum: np.ndarray, maximum: np.ndarray) -> bool:
    p = np.asarray(position_xyz, dtype=np.float64).reshape(3)
    return bool(np.all(p >= np.asarray(minimum)) and np.all(p <= np.asarray(maximum)))
