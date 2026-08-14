#!/usr/bin/env python3
"""Shared, frame-explicit utilities for the D3a hand-eye tools."""

from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


SCHEMA_VERSION = "d3a_handeye_v1"


def load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def as_transform(value, name: str = "transform") -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"{name} must be 4x4, got {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} contains non-finite values")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1.0e-7):
        raise ValueError(f"{name} has invalid homogeneous last row")
    if not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=2e-4):
        raise ValueError(f"{name} rotation is not orthonormal")
    if np.linalg.det(matrix[:3, :3]) < 0.999:
        raise ValueError(f"{name} rotation determinant is not +1")
    return matrix


def invert(transform: np.ndarray) -> np.ndarray:
    transform = as_transform(transform)
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def transform_from_vec(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    result = np.eye(4)
    result[:3, 3] = vector[:3]
    result[:3, :3] = Rotation.from_rotvec(vector[3:]).as_matrix()
    return result


def transform_to_vec(transform: np.ndarray) -> np.ndarray:
    transform = as_transform(transform)
    return np.r_[
        transform[:3, 3], Rotation.from_matrix(transform[:3, :3]).as_rotvec()
    ]


def pose_error(estimate: np.ndarray, truth: np.ndarray) -> tuple[float, float]:
    delta = invert(truth) @ estimate
    translation_mm = float(np.linalg.norm(delta[:3, 3]) * 1000.0)
    rotation_deg = float(
        np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude())
    )
    return translation_mm, rotation_deg


def mean_transform(transforms: list[np.ndarray]) -> np.ndarray:
    if not transforms:
        raise ValueError("cannot average an empty transform list")
    result = np.eye(4)
    result[:3, 3] = np.mean([item[:3, 3] for item in transforms], axis=0)
    result[:3, :3] = Rotation.from_matrix(
        np.stack([item[:3, :3] for item in transforms])
    ).mean().as_matrix()
    return result


def metric_summary(values: list[tuple[float, float]]) -> dict:
    array = np.asarray(values, dtype=float)
    return {
        "count": int(len(array)),
        "translation_mm": {
            "p50": float(np.percentile(array[:, 0], 50)),
            "p95": float(np.percentile(array[:, 0], 95)),
            "max": float(np.max(array[:, 0])),
        },
        "rotation_deg": {
            "p50": float(np.percentile(array[:, 1], 50)),
            "p95": float(np.percentile(array[:, 1], 95)),
            "max": float(np.max(array[:, 1])),
        },
    }


def marker_object_points(size_m: float) -> np.ndarray:
    half = float(size_m) / 2.0
    # OpenCV ArUco corner order: top-left, top-right, bottom-right, bottom-left.
    return np.array(
        [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
        dtype=np.float32,
    )


def project_marker(
    camera_from_marker: np.ndarray,
    K: np.ndarray,
    distortion: np.ndarray,
    marker_size_m: float,
) -> np.ndarray:
    rotation = Rotation.from_matrix(camera_from_marker[:3, :3]).as_rotvec()
    points, _ = cv2.projectPoints(
        marker_object_points(marker_size_m),
        rotation,
        camera_from_marker[:3, 3],
        np.asarray(K, dtype=float).reshape(3, 3),
        np.asarray(distortion, dtype=float),
    )
    return points.reshape(-1, 2)


def reprojection_rmse_px(sample: dict, predicted_camera_from_marker: np.ndarray) -> float:
    corners = np.asarray(sample["marker"]["corners_px"], dtype=float).reshape(-1, 2)
    camera = sample["camera_info"]
    predicted = project_marker(
        predicted_camera_from_marker,
        np.asarray(camera["K"], dtype=float),
        np.asarray(camera.get("distortion", []), dtype=float),
        float(sample["marker"]["size_m"]),
    )
    return float(np.sqrt(np.mean(np.sum((predicted - corners) ** 2, axis=1))))


def session_samples(session_dir: Path, split: str | None = None) -> list[dict]:
    paths = sorted((session_dir / "samples").glob("*/sample.json"))
    samples = [load_json(path) for path in paths]
    if split is not None:
        samples = [item for item in samples if item["split"] == split]
    return samples


def robot_transform(sample: dict) -> np.ndarray:
    return as_transform(
        sample["robot_pose"]["T_robot_base_from_control_point"],
        "T_robot_base_from_control_point",
    )


def marker_transform(sample: dict) -> np.ndarray:
    return as_transform(
        sample["marker"]["T_camera_from_marker"], "T_camera_from_marker"
    )


def solve_ecm_eye_in_hand(samples: list[dict]) -> dict:
    """Return T_control_point_from_camera and a static base-from-target pose."""
    solve = [item for item in samples if item["split"] == "solve" and item["valid"]]
    if len(solve) < 4:
        raise ValueError("ECM solve requires at least 4 valid solve samples")
    robot = [robot_transform(item) for item in solve]
    marker = [marker_transform(item) for item in solve]
    R, t = cv2.calibrateHandEye(
        [item[:3, :3] for item in robot],
        [item[:3, 3].reshape(3, 1) for item in robot],
        [item[:3, :3] for item in marker],
        [item[:3, 3].reshape(3, 1) for item in marker],
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    control_from_camera = np.eye(4)
    control_from_camera[:3, :3] = R
    control_from_camera[:3, 3] = np.asarray(t).reshape(3)
    base_from_target_samples = [
        robot_pose @ control_from_camera @ marker_pose
        for robot_pose, marker_pose in zip(robot, marker)
    ]
    base_from_target = mean_transform(base_from_target_samples)
    return {
        "T_control_point_from_camera": control_from_camera,
        "T_camera_from_control_point": invert(control_from_camera),
        "T_robot_base_from_static_marker": base_from_target,
    }


def solve_psm_eye_to_hand(samples: list[dict]) -> dict:
    """Solve B_i = X A_i Y for camera-from-base X and cp-from-marker Y."""
    solve = [item for item in samples if item["split"] == "solve" and item["valid"]]
    if len(solve) < 4:
        raise ValueError("PSM solve requires at least 4 valid solve samples")
    A = [robot_transform(item) for item in solve]
    B = [marker_transform(item) for item in solve]

    # A deterministic initialization from the first pose plus several small
    # restarts makes direction mistakes visible without importing a hidden
    # robot-world convention.
    initial_x = np.eye(4)
    initial_y = invert(A[0]) @ B[0]

    def residual(vector: np.ndarray) -> np.ndarray:
        X = transform_from_vec(vector[:6])
        Y = transform_from_vec(vector[6:])
        result = []
        for robot_pose, observed in zip(A, B):
            delta = invert(observed) @ X @ robot_pose @ Y
            result.extend((delta[:3, 3] / 0.001).tolist())
            result.extend(
                (
                    Rotation.from_matrix(delta[:3, :3]).as_rotvec() / 0.01
                ).tolist()
            )
        return np.asarray(result)

    seeds = [
        np.r_[transform_to_vec(initial_x), transform_to_vec(initial_y)],
        np.zeros(12),
    ]
    rng = np.random.default_rng(34006)
    seeds.extend([seeds[0] + rng.normal(scale=0.05, size=12) for _ in range(4)])
    solutions = [
        least_squares(
            residual,
            seed,
            loss="soft_l1",
            f_scale=1.0,
            max_nfev=10000,
            xtol=1.0e-12,
            ftol=1.0e-12,
            gtol=1.0e-12,
        )
        for seed in seeds
    ]
    best = min(solutions, key=lambda item: float(np.sum(residual(item.x) ** 2)))
    camera_from_base = transform_from_vec(best.x[:6])
    control_from_marker = transform_from_vec(best.x[6:])
    return {
        "T_camera_from_robot_base": camera_from_base,
        "T_robot_base_from_camera": invert(camera_from_base),
        "T_control_point_from_marker": control_from_marker,
        "T_marker_from_control_point": invert(control_from_marker),
        "optimizer_cost": float(best.cost),
        "optimizer_optimality": float(best.optimality),
    }


def validate_solution(samples: list[dict], calibration_type: str, solution: dict) -> dict:
    records = []
    if calibration_type == "ecm_eye_in_hand":
        X = as_transform(solution["T_control_point_from_camera"])
        static_target = as_transform(solution["T_robot_base_from_static_marker"])
        for sample in samples:
            A = robot_transform(sample)
            observed = marker_transform(sample)
            predicted = invert(A @ X) @ static_target
            translation, rotation = pose_error(predicted, observed)
            records.append(
                {
                    "sample_id": sample["sample_id"],
                    "split": sample["split"],
                    "translation_mm": translation,
                    "rotation_deg": rotation,
                    "reprojection_rmse_px": reprojection_rmse_px(sample, predicted),
                    "predicted_T_camera_from_marker": predicted.tolist(),
                }
            )
    elif calibration_type == "psm_eye_to_hand":
        X = as_transform(solution["T_camera_from_robot_base"])
        Y = as_transform(solution["T_control_point_from_marker"])
        for sample in samples:
            observed = marker_transform(sample)
            predicted = X @ robot_transform(sample) @ Y
            translation, rotation = pose_error(predicted, observed)
            records.append(
                {
                    "sample_id": sample["sample_id"],
                    "split": sample["split"],
                    "translation_mm": translation,
                    "rotation_deg": rotation,
                    "reprojection_rmse_px": reprojection_rmse_px(sample, predicted),
                    "predicted_T_camera_from_marker": predicted.tolist(),
                }
            )
    else:
        raise ValueError(f"unknown calibration_type {calibration_type}")

    result = {"records": records}
    for split in ["solve", "heldout"]:
        selected = [row for row in records if row["split"] == split]
        result[split] = metric_summary(
            [(row["translation_mm"], row["rotation_deg"]) for row in selected]
        )
        reprojection = np.asarray(
            [row["reprojection_rmse_px"] for row in selected], dtype=float
        )
        result[split]["reprojection_rmse_px"] = {
            "p50": float(np.percentile(reprojection, 50)),
            "p95": float(np.percentile(reprojection, 95)),
            "max": float(np.max(reprojection)),
        }
    return result


def coverage(samples: list[dict]) -> dict:
    poses = [robot_transform(item) for item in samples]
    translations = np.stack([item[:3, 3] for item in poses])
    translation_span = (np.max(translations, axis=0) - np.min(translations, axis=0)) * 1000
    pairwise_rotation = []
    for index, first in enumerate(poses):
        for second in poses[index + 1 :]:
            pairwise_rotation.append(pose_error(first, second)[1])
    return {
        "translation_axis_span_mm": translation_span.tolist(),
        "rotation_pairwise_p50_deg": float(np.percentile(pairwise_rotation, 50)),
        "rotation_pairwise_max_deg": float(np.max(pairwise_rotation)),
    }


def random_visible_marker_pose(rng: np.random.Generator) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = [
        rng.uniform(-0.025, 0.025),
        rng.uniform(-0.018, 0.018),
        rng.uniform(0.13, 0.20),
    ]
    transform[:3, :3] = Rotation.from_euler(
        "xyz",
        [rng.uniform(-25, 25), rng.uniform(-25, 25), rng.uniform(-50, 50)],
        degrees=True,
    ).as_matrix()
    return transform


def add_pose_noise(
    transform: np.ndarray,
    rng: np.random.Generator,
    translation_sigma_mm: float,
    rotation_sigma_deg: float,
) -> np.ndarray:
    noisy = transform.copy()
    noisy[:3, 3] += rng.normal(scale=translation_sigma_mm / 1000.0, size=3)
    noisy[:3, :3] = noisy[:3, :3] @ Rotation.from_rotvec(
        rng.normal(scale=np.radians(rotation_sigma_deg), size=3)
    ).as_matrix()
    return noisy


def matrix_payload(name: str, transform: np.ndarray) -> dict:
    return {
        "name": name,
        "matrix": as_transform(transform).tolist(),
        "inverse_name": name.replace("_from_", "_from_").split("__never__")[0],
    }

