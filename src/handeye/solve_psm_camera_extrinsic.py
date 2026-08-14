#!/usr/bin/env python3
"""Solve fixed-camera PSM base extrinsic plus marker mount transform."""

from __future__ import annotations

import argparse
from pathlib import Path

from handeye_common import (
    session_samples,
    solve_psm_eye_to_hand,
    validate_solution,
    write_json,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    samples = session_samples(args.session_dir)
    solution = solve_psm_eye_to_hand(samples)
    matrices = {
        key: (value.tolist() if hasattr(value, "tolist") else value)
        for key, value in solution.items()
    }
    validation = validate_solution(samples, "psm_eye_to_hand", matrices)
    payload = {
        "calibration_type": "psm_eye_to_hand",
        "matrix_convention": "T_A_from_B maps coordinates in B into A",
        "equation": "T_camera_from_marker_i = T_camera_from_robot_base @ T_robot_base_from_control_point_i @ T_control_point_from_marker",
        "matrices": matrices,
        "validation": validation,
        "heldout_isolation": "only split=solve entered nonlinear fit; split=heldout used after solve",
    }
    output = args.output or args.session_dir / "psm_solution.json"
    write_json(output, payload)
    print(payload["validation"]["heldout"])


if __name__ == "__main__":
    main()
