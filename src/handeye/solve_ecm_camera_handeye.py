#!/usr/bin/env python3
"""Solve ECM eye-in-hand calibration and report held-out residuals."""

from __future__ import annotations

import argparse
from pathlib import Path

from handeye_common import (
    session_samples,
    solve_ecm_eye_in_hand,
    validate_solution,
    write_json,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    samples = session_samples(args.session_dir)
    solution = solve_ecm_eye_in_hand(samples)
    matrices = {key: value.tolist() for key, value in solution.items()}
    validation = validate_solution(samples, "ecm_eye_in_hand", matrices)
    payload = {
        "calibration_type": "ecm_eye_in_hand",
        "matrix_convention": "T_A_from_B maps coordinates in B into A",
        "matrices": matrices,
        "validation": validation,
        "heldout_isolation": "only split=solve entered calibrateHandEye; split=heldout used after solve",
    }
    output = args.output or args.session_dir / "ecm_solution.json"
    write_json(output, payload)
    print(payload["validation"]["heldout"])


if __name__ == "__main__":
    main()

