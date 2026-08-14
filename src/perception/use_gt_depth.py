#!/usr/bin/env python3
"""Materialize AMBF GT depth in the DA-output layout for a controlled A/B."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-dir", type=Path, required=True)
    parser.add_argument("--prediction-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    source = args.capture_dir / "depth_gt_m"
    files = sorted(source.glob("*.npy"))
    if not files:
        raise SystemExit(f"No GT depth files under {source}")
    if args.out.exists() or args.prediction_dir.exists():
        raise FileExistsError("Refusing to overwrite an existing GT-depth materialization")
    args.prediction_dir.mkdir(parents=True)
    rows = []
    for path in files:
        array = np.load(path, allow_pickle=False)
        if array.ndim != 2 or not np.isfinite(array).any():
            raise ValueError(f"Invalid GT depth {path}")
        target = args.prediction_dir / path.name
        shutil.copy2(path, target)
        rows.append({"frame_id": path.stem, "source": "AMBF_GT_PRIVILEGED"})
    payload = {
        "schema": "SurgicAI.public_gt_depth_adapter.v1",
        "complete": True,
        "frames": len(rows),
        "source": "AMBF_GT_PRIVILEGED",
        "warning": "This is a simulator privilege and cannot be used on a real camera.",
        "rows": rows,
    }
    args.out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"GT_DEPTH_READY frames={len(rows)} output={args.prediction_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
