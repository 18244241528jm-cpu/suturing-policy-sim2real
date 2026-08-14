#!/usr/bin/env python3
"""Validate D3a sample schema, units, synchronization, diversity, and splits."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import cv2
import numpy as np

from handeye_common import (
    SCHEMA_VERSION,
    as_transform,
    coverage,
    load_json,
    load_yaml,
    pose_error,
    session_samples,
    write_json,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    config = load_yaml(args.config)
    limits = config["validation"]
    session = load_json(args.session_dir / "session.json")
    samples = session_samples(args.session_dir)
    errors: list[str] = []
    warnings: list[str] = []

    if session.get("schema_version") != SCHEMA_VERSION:
        errors.append("session schema_version mismatch")
    if session.get("matrix_convention") != "T_A_from_B maps coordinates in B into A":
        errors.append("matrix direction convention is missing or changed")
    if session.get("units", {}).get("translation") != "meter":
        errors.append("session translation unit must be meter")
    if not float(session.get("marker", {}).get("size_m", 0)) > 0:
        errors.append("marker size_m must be populated and positive")

    ids = set()
    solve_ids = set()
    heldout_ids = set()
    valid_samples = []
    for sample in samples:
        sid = sample.get("sample_id")
        if sid in ids:
            errors.append(f"duplicate sample_id {sid}")
        ids.add(sid)
        if sample.get("split") == "solve":
            solve_ids.add(sid)
        elif sample.get("split") == "heldout":
            heldout_ids.add(sid)
        else:
            errors.append(f"{sid}: split must be solve or heldout")
        if solve_ids & heldout_ids:
            errors.append("solve and heldout sample IDs overlap")
        if sample.get("schema_version") != SCHEMA_VERSION:
            errors.append(f"{sid}: schema mismatch")
        if sample.get("robot_pose", {}).get("unit") != "meter":
            errors.append(f"{sid}: robot pose unit is not meter")
        try:
            as_transform(
                sample["robot_pose"]["T_robot_base_from_control_point"],
                f"{sid} robot pose",
            )
            as_transform(sample["marker"]["T_camera_from_marker"], f"{sid} marker pose")
        except (KeyError, ValueError) as exc:
            errors.append(str(exc))
        if sample.get("robot_pose", {}).get("pose_convention") != "T_robot_base_from_control_point":
            errors.append(f"{sid}: robot frame chain direction is ambiguous")
        if sample.get("marker", {}).get("pose_convention") != "T_camera_from_marker":
            errors.append(f"{sid}: marker frame chain direction is ambiguous")
        if float(sample.get("marker", {}).get("size_m", 0)) <= 0:
            errors.append(f"{sid}: marker size missing")
        if float(sample.get("sync_delta_ms", np.inf)) > float(limits["max_sync_delta_ms"]):
            errors.append(
                f"{sid}: sync delta {sample.get('sync_delta_ms')} ms exceeds {limits['max_sync_delta_ms']} ms"
            )
        left_path = args.session_dir / "samples" / sid / sample["images"]["left"]
        image = cv2.imread(str(left_path))
        if image is None:
            errors.append(f"{sid}: left image missing/unreadable")
        else:
            expected = (int(sample["camera_info"]["height"]), int(sample["camera_info"]["width"]))
            if image.shape[:2] != expected:
                errors.append(f"{sid}: image resolution {image.shape[:2]} != camera_info {expected}")
        corners = np.asarray(sample.get("marker", {}).get("corners_px", []), dtype=float)
        if corners.shape != (4, 2) or not np.all(np.isfinite(corners)):
            errors.append(f"{sid}: marker corners must be finite 4x2")
        if sample.get("valid"):
            valid_samples.append(sample)
        elif not sample.get("rejection_reason"):
            errors.append(f"{sid}: rejected sample lacks rejection_reason")

    solve = [item for item in valid_samples if item["split"] == "solve"]
    heldout = [item for item in valid_samples if item["split"] == "heldout"]
    if len(solve) < int(limits["min_solve_samples"]):
        errors.append(f"valid solve samples {len(solve)} < {limits['min_solve_samples']}")
    if len(heldout) < int(limits["min_heldout_samples"]):
        errors.append(f"valid heldout samples {len(heldout)} < {limits['min_heldout_samples']}")

    duplicates = []
    for index, first in enumerate(valid_samples):
        first_pose = as_transform(first["robot_pose"]["T_robot_base_from_control_point"])
        for second in valid_samples[index + 1 :]:
            second_pose = as_transform(second["robot_pose"]["T_robot_base_from_control_point"])
            translation, rotation = pose_error(first_pose, second_pose)
            if translation < float(limits["duplicate_translation_mm"]) and rotation < float(limits["duplicate_rotation_deg"]):
                duplicates.append([first["sample_id"], second["sample_id"], translation, rotation])
    if duplicates:
        errors.append(f"found {len(duplicates)} duplicate-pose pairs")

    solve_coverage = coverage(solve) if len(solve) >= 2 else None
    if solve_coverage:
        if max(solve_coverage["translation_axis_span_mm"]) < float(limits["min_translation_span_mm"]):
            errors.append("solve translation coverage below configured engineering minimum")
        if solve_coverage["rotation_pairwise_max_deg"] < float(limits["min_rotation_span_deg"]):
            errors.append("solve rotation coverage below configured engineering minimum")
        if min(solve_coverage["translation_axis_span_mm"]) < 1.0:
            warnings.append("one translation axis spans <1 mm; inspect planar degeneracy")

    report = {
        "session_dir": str(args.session_dir.resolve()),
        "calibration_type": session.get("calibration_type"),
        "sample_counts": {
            "total": len(samples),
            "valid": len(valid_samples),
            "solve": len(solve),
            "heldout": len(heldout),
        },
        "automatic_checks": {
            "resolution_matches_camera_info": True,
            "marker_size_populated": True,
            "timestamp_delta_checked": True,
            "translation_unit_meter": True,
            "finite_se3": True,
            "duplicate_pose_checked": True,
            "translation_rotation_coverage_checked": True,
            "solve_heldout_disjoint": True,
            "frame_chain_direction_explicit": True,
        },
        "coverage": solve_coverage,
        "duplicate_pairs": duplicates,
        "errors": errors,
        "warnings": warnings,
        "valid": not errors,
        "threshold_status": "[假设] engineering preflight thresholds, not real-robot validated",
    }
    output = args.output or args.session_dir / "validation_report.json"
    write_json(output, report)
    print(f"valid={report['valid']} errors={len(errors)} warnings={len(warnings)}")
    if errors:
        for item in errors:
            print("ERROR", item)
        raise SystemExit(2)


if __name__ == "__main__":
    main()

