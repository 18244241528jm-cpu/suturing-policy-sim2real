#!/usr/bin/env python3
"""End-to-end known-truth and noisy stability test for both D3a calibrations."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

from handeye_common import load_json, pose_error, write_json


HERE = Path(__file__).resolve().parent


def run(command: list[str]) -> None:
    print("+", " ".join(command))
    subprocess.run(command, check=True)


def manifest(root: Path) -> None:
    lines = []
    for path in sorted(root.rglob("*")):
        if path.is_file() and path.name != "MANIFEST.sha256":
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            lines.append(f"{digest}  {path.relative_to(root).as_posix()}")
    (root / "MANIFEST.sha256").write_text("\n".join(lines) + "\n", encoding="utf-8")


def matrix(solution: dict, name: str) -> np.ndarray:
    return np.asarray(solution["matrices"][name], dtype=float)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=HERE / "config" / "example_session.yaml")
    args = parser.parse_args()
    if args.output_root.exists():
        shutil.rmtree(args.output_root)
    args.output_root.mkdir(parents=True)
    cases = []
    profiles = {
        "clean": ["--robot-noise-mm", "0", "--robot-noise-deg", "0", "--pnp-noise-mm", "0", "--pnp-noise-deg", "0", "--corner-noise-px", "0"],
        "noisy": ["--robot-noise-mm", "0.2", "--robot-noise-deg", "0.1", "--pnp-noise-mm", "0.3", "--pnp-noise-deg", "0.2", "--corner-noise-px", "0.25"],
    }
    for calibration in ["ecm_eye_in_hand", "psm_eye_to_hand"]:
        for profile, noise_args in profiles.items():
            session = args.output_root / f"{calibration}_{profile}"
            run(
                [
                    sys.executable,
                    str(HERE / "capture_handeye_sample.py"),
                    "--config",
                    str(args.config),
                    "--session-dir",
                    str(session),
                    "--calibration-type",
                    calibration,
                    "--mode",
                    "mock",
                    "--count",
                    "30",
                    "--seed",
                    "34006",
                    *noise_args,
                ]
            )
            run(
                [
                    sys.executable,
                    str(HERE / "validate_handeye_dataset.py"),
                    "--session-dir",
                    str(session),
                    "--config",
                    str(args.config),
                ]
            )
            solver = "solve_ecm_camera_handeye.py" if calibration.startswith("ecm") else "solve_psm_camera_extrinsic.py"
            solution_path = session / "solution.json"
            run(
                [
                    sys.executable,
                    str(HERE / solver),
                    "--session-dir",
                    str(session),
                    "--output",
                    str(solution_path),
                ]
            )
            run(
                [
                    sys.executable,
                    str(HERE / "render_handeye_overlay.py"),
                    "--session-dir",
                    str(session),
                    "--solution",
                    str(solution_path),
                    "--output-dir",
                    str(session / "overlays"),
                ]
            )
            truth = load_json(session / "ground_truth.json")
            solution = load_json(solution_path)
            if calibration == "ecm_eye_in_hand":
                errors = {
                    "T_control_point_from_camera": pose_error(
                        matrix(solution, "T_control_point_from_camera"),
                        np.asarray(truth["T_control_point_from_camera"]),
                    )
                }
                inverse_pairs = [["T_control_point_from_camera", "T_camera_from_control_point"]]
            else:
                errors = {
                    "T_camera_from_robot_base": pose_error(
                        matrix(solution, "T_camera_from_robot_base"),
                        np.asarray(truth["T_camera_from_robot_base"]),
                    ),
                    "T_control_point_from_marker": pose_error(
                        matrix(solution, "T_control_point_from_marker"),
                        np.asarray(truth["T_control_point_from_marker"]),
                    ),
                }
                inverse_pairs = [
                    ["T_camera_from_robot_base", "T_robot_base_from_camera"],
                    ["T_control_point_from_marker", "T_marker_from_control_point"],
                ]
            inverse_residuals = {}
            for forward, reverse in inverse_pairs:
                product = matrix(solution, forward) @ matrix(solution, reverse)
                inverse_residuals[f"{forward}__{reverse}"] = float(np.max(np.abs(product - np.eye(4))))
            heldout = solution["validation"]["heldout"]
            case = {
                "calibration_type": calibration,
                "profile": profile,
                "session": str(session),
                "truth_errors": {
                    name: {"translation_mm": value[0], "rotation_deg": value[1]}
                    for name, value in errors.items()
                },
                "heldout": heldout,
                "inverse_matrix_identity_max_abs": inverse_residuals,
            }
            cases.append(case)

    failures = []
    for case in cases:
        clean = case["profile"] == "clean"
        truth_t_limit = 1.0e-3 if clean else 2.0
        truth_r_limit = 1.0e-3 if clean else 1.0
        for name, error in case["truth_errors"].items():
            if error["translation_mm"] > truth_t_limit or error["rotation_deg"] > truth_r_limit:
                failures.append(f"{case['calibration_type']} {case['profile']} {name} truth error")
        if case["heldout"]["translation_mm"]["p95"] > (1.0e-3 if clean else 2.0):
            failures.append(f"{case['calibration_type']} {case['profile']} heldout translation")
        if case["heldout"]["rotation_deg"]["p95"] > (1.0e-3 if clean else 1.0):
            failures.append(f"{case['calibration_type']} {case['profile']} heldout rotation")
        if max(case["inverse_matrix_identity_max_abs"].values()) > 1.0e-8:
            failures.append(f"{case['calibration_type']} {case['profile']} inverse direction")
    payload = {
        "task": "D3a synthetic self-test",
        "cases": cases,
        "assertions": {
            "clean_truth_translation_mm_max": 1.0e-3,
            "clean_truth_rotation_deg_max": 1.0e-3,
            "noisy_truth_translation_mm_max": 2.0,
            "noisy_truth_rotation_deg_max": 1.0,
            "inverse_identity_max_abs": 1.0e-8,
        },
        "failures": failures,
        "passed": not failures,
    }
    write_json(args.output_root / "self_test_summary.json", payload)
    manifest(args.output_root)
    print(json.dumps(payload, indent=2))
    if failures:
        raise SystemExit(2)


if __name__ == "__main__":
    main()

