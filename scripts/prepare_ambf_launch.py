#!/usr/bin/env python3
"""Render the archived AMBF stereo launch with host-local SRC paths."""

from __future__ import annotations

import argparse
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OLD_SRC = "/home/jiaming/surgical_robotics_challenge"
OLD_WORLD = "/mnt/c/Users/30518/OneDrive - Johns Hopkins/Desktop/cis2/project34/depth_audit_stage_a/world_depth_audit_stereo.yaml"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--src-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    src_root = args.src_root.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    if not (src_root / "ADF").is_dir():
        raise SystemExit(f"SRC root has no ADF directory: {src_root}")
    output.mkdir(parents=True, exist_ok=True)

    world_source = ROOT / "src/perception/world_depth_audit_stereo.yaml"
    launch_source = ROOT / "src/perception/depth_audit_stereo.launch.yaml"
    world_target = output / world_source.name
    launch_target = output / launch_source.name

    world_target.write_text(
        world_source.read_text(encoding="utf-8").replace(OLD_SRC, src_root.as_posix()),
        encoding="utf-8",
    )
    launch_target.write_text(
        launch_source.read_text(encoding="utf-8")
        .replace(OLD_SRC, src_root.as_posix())
        .replace(OLD_WORLD, world_target.as_posix()),
        encoding="utf-8",
    )
    print(f"SIM_S4_STEREO_LAUNCH={launch_target}")


if __name__ == "__main__":
    main()

