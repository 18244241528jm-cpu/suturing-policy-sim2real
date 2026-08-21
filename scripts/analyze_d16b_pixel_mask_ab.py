#!/usr/bin/env python3
"""Compare D16 centreline masks with pixel-refined real-needle masks."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def pose_delta(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    translation = float(np.linalg.norm(a[:3, 3] - b[:3, 3]) * 1000.0)
    relative = a[:3, :3] @ b[:3, :3].T
    angle = float(np.degrees(np.arccos(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))))
    return translation, angle


def bgr(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(path)
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    root = args.root
    rows = []
    for view in (1, 2):
        old_bundle = read_json(root / f"stage_4_bundles/view{view}__rect__w5/bundle.json")
        new_bundle = read_json(root / f"stage_8_pixel_bundles/view{view}__rect__pixel_w5/bundle.json")
        old_pose = np.asarray(read_json(root / f"stage_5_fp/view{view}__rect__w5/result.json")["top_pose_camera"])
        new_pose = np.asarray(read_json(root / f"stage_9_pixel_fp/view{view}__rect__pixel_w5/result.json")["top_pose_camera"])
        dt, dr = pose_delta(old_pose, new_pose)
        rows.append({
            "view": view,
            "old_mask_pixels": old_bundle["mask_pixels"],
            "pixel_refined_mask_pixels": new_bundle["mask_pixels"],
            "old_relief_mm": old_bundle["predicted_needle_relief_mm_positive_means_closer"],
            "pixel_refined_relief_mm": new_bundle["predicted_needle_relief_mm_positive_means_closer"],
            "old_to_refined_top1_translation_mm": dt,
            "old_to_refined_top1_rotation_deg": dr,
        })

    summary = {
        "schema": "SurgicAI.D16b.pixel_mask_ab.v1",
        "complete": True,
        "rows": rows,
        "decision": "MASK_CORRECTION_DOES_NOT_VALIDATE_FP",
        "warning": "The refined masks are manual pixel traces from the real screenshots, not simulator GT. No real 6D GT is available.",
    }
    (root / "d16b_pixel_mask_ab.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    fig, axes = plt.subplots(2, 4, figsize=(20, 10))
    for row, view in enumerate((1, 2)):
        panels = [
            (root / f"inputs/ecm_view_{view}.png", f"View {view}: real RGB"),
            (root / f"stage_3_masks/view{view}/overlay_w5.png", "Old centreline mask\n(not simulator GT)"),
            (root / f"stage_7_pixel_masks_v2/view{view}/overlay_w5.png", "Pixel-refined metal-needle mask"),
            (root / f"stage_9_pixel_fp/view{view}__rect__pixel_w5/top1_overlay.png", "FP top-1 after correction\n(still not validated)"),
        ]
        for col, (path, title) in enumerate(panels):
            axes[row, col].imshow(bgr(path)); axes[row, col].set_title(title, fontsize=12); axes[row, col].axis("off")
    fig.suptitle("D16b real-image mask correction A/B: green mask pixels versus green FP CAD box", fontsize=17)
    fig.tight_layout()
    fig.savefig(root / "figure_4_pixel_mask_correction_ab.png", dpi=160)
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
