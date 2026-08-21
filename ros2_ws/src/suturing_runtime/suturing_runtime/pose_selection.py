"""Pure, ROS-free candidate selection helpers for real perception interfaces.

All transforms follow ``T_A_from_B``.  These functions never supply missing
calibration values: callers must provide every frame transform and uncertainty.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .contract import matrix_to_quaternion_xyzw


@dataclass(frozen=True)
class NeedleSelection:
    selected_index: int | None
    survivor_indices: np.ndarray
    flat_angle_deg: np.ndarray
    height_error_m: np.ndarray
    planar_cost: np.ndarray


@dataclass(frozen=True)
class PSMSelection:
    selected_index: int
    selected_camera_from_control_point: np.ndarray
    translation_innovation_m: float
    rotation_innovation_deg: float
    normalized_cost: float
    fp_rank_fraction: float


def _unit(vector: np.ndarray, name: str) -> np.ndarray:
    value = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(value))
    if not np.isfinite(value).all() or norm < 1.0e-12:
        raise ValueError(f"{name} must be a finite non-zero 3-vector")
    return value / norm


def _validate_candidates(poses: np.ndarray, scores: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    matrices = np.asarray(poses, dtype=np.float64)
    values = np.asarray(scores, dtype=np.float64).reshape(-1)
    if matrices.ndim != 3 or matrices.shape[1:] != (4, 4):
        raise ValueError("poses must have shape [N,4,4]")
    if len(matrices) == 0 or len(matrices) != len(values):
        raise ValueError("poses and scores must be non-empty and equal length")
    if not np.isfinite(matrices).all() or not np.isfinite(values).all():
        raise ValueError("candidate poses and scores must be finite")
    return matrices, values


def score_rank_fraction(scores: np.ndarray) -> np.ndarray:
    """Return 0 for the best FP score and 1 for the worst; ties are stable."""
    values = np.asarray(scores, dtype=np.float64).reshape(-1)
    order = np.argsort(-values, kind="stable")
    ranks = np.empty(len(values), dtype=np.float64)
    ranks[order] = np.arange(len(values), dtype=np.float64)
    return ranks / max(len(values) - 1, 1)


def rotation_vector_deg(candidate_rotation: np.ndarray, reference_rotation: np.ndarray) -> np.ndarray:
    """Rotation vector taking reference orientation to candidate orientation."""
    relative = np.asarray(reference_rotation).reshape(3, 3).T @ np.asarray(candidate_rotation).reshape(3, 3)
    quaternion = matrix_to_quaternion_xyzw(relative)
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    vector = quaternion[:3]
    length = float(np.linalg.norm(vector))
    if length < 1.0e-12:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * math.atan2(length, float(quaternion[3]))
    return vector / length * math.degrees(angle)


def pose_innovation6(candidate: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Return camera-axis translation metres + reference-axis rotation degrees."""
    candidate = np.asarray(candidate, dtype=np.float64).reshape(4, 4)
    reference = np.asarray(reference, dtype=np.float64).reshape(4, 4)
    return np.r_[candidate[:3, 3] - reference[:3, 3],
                 rotation_vector_deg(candidate[:3, :3], reference[:3, :3])]


def select_needle_candidate(
    poses: np.ndarray,
    scores: np.ndarray,
    *,
    flat_enabled: bool = False,
    plane_normal_camera: np.ndarray | None = None,
    needle_rest_normal_mesh: np.ndarray | None = None,
    maximum_flat_angle_deg: float = 15.0,
    support_height_enabled: bool = False,
    plane_point_camera_m: np.ndarray | None = None,
    needle_origin_support_offset_m: float = 0.0,
    maximum_height_error_m: float = 0.003,
    planar_enabled: bool = False,
    planar_pose_camera: np.ndarray | None = None,
    plane_axis_x_camera: np.ndarray | None = None,
    plane_axis_y_camera: np.ndarray | None = None,
    needle_heading_axis_mesh: np.ndarray | None = None,
    planar_sigmas: np.ndarray | None = None,
    planar_chi2_threshold: float = 11.3449,
) -> NeedleSelection:
    """Filter candidates, then retain the highest original FP score.

    ``planar_enabled`` uses an external same-frame mask+plane+CAD observation.
    It never emits that observation as the final pose; the selected pose always
    remains one of ``poses``.
    """
    matrices, values = _validate_candidates(poses, scores)
    count = len(values)
    flat_angles = np.zeros(count, dtype=np.float64)
    height_errors = np.zeros(count, dtype=np.float64)
    planar_cost = np.zeros(count, dtype=np.float64)
    valid = np.ones(count, dtype=bool)

    if flat_enabled:
        normal = _unit(plane_normal_camera, "plane_normal_camera")
        rest = _unit(needle_rest_normal_mesh, "needle_rest_normal_mesh")
        candidate_normals = np.einsum("nij,j->ni", matrices[:, :3, :3], rest)
        flat_angles = np.degrees(np.arccos(np.clip(candidate_normals @ normal, -1.0, 1.0)))
        valid &= flat_angles <= float(maximum_flat_angle_deg)

    if support_height_enabled:
        if not flat_enabled:
            raise ValueError("support height requires the flat/support frame")
        point = np.asarray(plane_point_camera_m, dtype=np.float64).reshape(3)
        if not np.isfinite(point).all():
            raise ValueError("plane_point_camera_m must be finite")
        height = (matrices[:, :3, 3] - point) @ normal
        height_errors = height - float(needle_origin_support_offset_m)
        valid &= np.abs(height_errors) <= float(maximum_height_error_m)

    if planar_enabled:
        if not flat_enabled:
            raise ValueError("planar geometry requires the flat/support frame")
        reference = np.asarray(planar_pose_camera, dtype=np.float64).reshape(4, 4)
        axis_x = _unit(plane_axis_x_camera, "plane_axis_x_camera")
        axis_y = _unit(plane_axis_y_camera, "plane_axis_y_camera")
        if abs(float(axis_x @ axis_y)) > 1.0e-3:
            raise ValueError("plane axes must be orthogonal")
        if abs(float(axis_x @ normal)) > 1.0e-3 or abs(float(axis_y @ normal)) > 1.0e-3:
            raise ValueError("plane axes must be tangent to the support plane")
        heading_mesh = _unit(needle_heading_axis_mesh, "needle_heading_axis_mesh")
        sigmas = np.asarray(planar_sigmas, dtype=np.float64).reshape(3)
        if not np.isfinite(sigmas).all() or np.any(sigmas <= 0.0):
            raise ValueError("planar sigmas [x_m,y_m,yaw_deg] must be positive")
        delta = matrices[:, :3, 3] - reference[:3, 3]
        dx = delta @ axis_x
        dy = delta @ axis_y
        reference_heading = reference[:3, :3] @ heading_mesh
        reference_heading -= normal * float(reference_heading @ normal)
        reference_heading = _unit(reference_heading, "planar reference heading")
        yaw = []
        for matrix in matrices:
            candidate_heading = matrix[:3, :3] @ heading_mesh
            candidate_heading -= normal * float(candidate_heading @ normal)
            candidate_heading = _unit(candidate_heading, "candidate heading")
            yaw.append(math.degrees(math.atan2(
                float(normal @ np.cross(reference_heading, candidate_heading)),
                float(reference_heading @ candidate_heading),
            )))
        yaw = np.asarray(yaw, dtype=np.float64)
        planar_cost = (dx / sigmas[0]) ** 2 + (dy / sigmas[1]) ** 2 + (yaw / sigmas[2]) ** 2
        valid &= planar_cost <= float(planar_chi2_threshold)

    survivors = np.flatnonzero(valid)
    selected = None if len(survivors) == 0 else int(survivors[np.argmax(values[survivors])])
    return NeedleSelection(selected, survivors, flat_angles, height_errors, planar_cost)


def select_psm_candidate(
    camera_from_mesh_candidates: np.ndarray,
    scores: np.ndarray,
    camera_from_control_point_fk: np.ndarray,
    mesh_from_control_point: np.ndarray,
    kinematic_sigma6: np.ndarray,
    vision_sigma6: np.ndarray,
    fp_rank_weight: float,
) -> PSMSelection:
    """Choose an FP mesh candidate using FK and explicit uncertainty models."""
    mesh_poses, values = _validate_candidates(camera_from_mesh_candidates, scores)
    mesh_from_cp = np.asarray(mesh_from_control_point, dtype=np.float64).reshape(4, 4)
    fk = np.asarray(camera_from_control_point_fk, dtype=np.float64).reshape(4, 4)
    kin_sigma = np.asarray(kinematic_sigma6, dtype=np.float64).reshape(6)
    vis_sigma = np.asarray(vision_sigma6, dtype=np.float64).reshape(6)
    if np.any(kin_sigma <= 0.0) or np.any(vis_sigma <= 0.0):
        raise ValueError("kinematic and vision sigma6 must be positive")
    if not np.isfinite(kin_sigma).all() or not np.isfinite(vis_sigma).all():
        raise ValueError("kinematic and vision sigma6 must be finite")
    if not math.isfinite(float(fp_rank_weight)) or float(fp_rank_weight) < 0.0:
        raise ValueError("fp_rank_weight must be finite and non-negative")
    control_candidates = mesh_poses @ mesh_from_cp
    residuals = np.stack([pose_innovation6(pose, fk) for pose in control_candidates])
    total_variance = kin_sigma ** 2 + vis_sigma ** 2
    ranks = score_rank_fraction(values)
    costs = np.sum(residuals ** 2 / total_variance, axis=1) + float(fp_rank_weight) * ranks
    selected = int(np.argmin(costs))
    residual = residuals[selected]
    return PSMSelection(
        selected_index=selected,
        selected_camera_from_control_point=control_candidates[selected],
        translation_innovation_m=float(np.linalg.norm(residual[:3])),
        rotation_innovation_deg=float(np.linalg.norm(residual[3:])),
        normalized_cost=float(costs[selected]),
        fp_rank_fraction=float(ranks[selected]),
    )


def motion_compensate_pose(
    camera_from_control_point_visual: np.ndarray,
    camera_from_control_point_fk_at_capture: np.ndarray,
    camera_from_control_point_fk_latest: np.ndarray,
) -> np.ndarray:
    """Advance a delayed visual pose by the measured FK relative motion."""
    visual = np.asarray(camera_from_control_point_visual, dtype=np.float64).reshape(4, 4)
    capture = np.asarray(camera_from_control_point_fk_at_capture, dtype=np.float64).reshape(4, 4)
    latest = np.asarray(camera_from_control_point_fk_latest, dtype=np.float64).reshape(4, 4)
    return visual @ np.linalg.inv(capture) @ latest
