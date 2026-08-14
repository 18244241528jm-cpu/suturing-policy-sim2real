#!/usr/bin/env python3
"""Fail-closed public release preflight without starting ROS or the robot."""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DA_SHA256 = "fc46bead4a5ea0e4122566bb88b93932aa82f110ee98281b5fcb09f499c9ec88"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require(path: Path, label: str) -> None:
    if not path.exists():
        raise SystemExit(f"MISSING {label}: {path}")
    print(f"OK {label}: {path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("code-only", "simulation"), default="code-only")
    args = parser.parse_args()

    required = {
        "Approach environment": ROOT / "src/SurgicAI/RL/Approach_env.py",
        "needle geometry": ROOT / "src/perception/p9a_goal_geometry.py",
        "D2 controller": ROOT / "src/control/controllers.py",
        "hand-eye solver": ROOT / "src/handeye/solve_psm_camera_extrinsic.py",
        "SIM-S4 runner": ROOT / "src/runners/run_sim_s4_deployment_proxy_reach.sh",
    }
    for label, path in required.items():
        require(path, label)

    if args.mode == "simulation":
        variables = ("AMBF_ROOT", "SRC_ROOT", "FOUNDATIONPOSE_ROOT", "DA_ROOT", "DA_CHECKPOINT", "ROS_DOMAIN_ID")
        missing = [name for name in variables if not os.environ.get(name)]
        if missing:
            raise SystemExit("MISSING environment variables: " + ", ".join(missing))
        if os.environ["ROS_DOMAIN_ID"] in {"0", ""}:
            raise SystemExit("ROS_DOMAIN_ID must be an explicit isolated non-zero domain")
        for name in ("AMBF_ROOT", "SRC_ROOT", "FOUNDATIONPOSE_ROOT", "DA_ROOT"):
            require(Path(os.environ[name]).expanduser(), name)
        checkpoint = Path(os.environ["DA_CHECKPOINT"]).expanduser()
        require(checkpoint, "DA_CHECKPOINT")
        actual = sha256(checkpoint)
        if actual != DA_SHA256:
            raise SystemExit(f"DA checkpoint SHA mismatch: {actual}")
        print(f"OK DA checkpoint SHA256: {actual}")

    print(f"PUBLIC_PIPELINE_PREFLIGHT_OK mode={args.mode}")


if __name__ == "__main__":
    main()

