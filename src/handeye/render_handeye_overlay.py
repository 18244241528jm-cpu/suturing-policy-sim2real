#!/usr/bin/env python3
"""Render observed-PnP and hand-eye-predicted marker axes on held-out RGB."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from handeye_common import load_json, session_samples, write_json


def draw_axes(image, pose, K, distortion, label, color):
    pose = np.asarray(pose, dtype=float)
    rvec = Rotation.from_matrix(pose[:3, :3]).as_rotvec()
    cv2.drawFrameAxes(
        image,
        np.asarray(K, dtype=float),
        np.asarray(distortion, dtype=float),
        rvec,
        pose[:3, 3],
        0.012,
        2,
    )
    origin, _ = cv2.projectPoints(
        np.zeros((1, 3), dtype=float),
        rvec,
        pose[:3, 3],
        np.asarray(K, dtype=float),
        np.asarray(distortion, dtype=float),
    )
    x, y = np.rint(origin.reshape(2)).astype(int)
    cv2.putText(image, label, (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--solution", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    solution = load_json(args.solution)
    predictions = {
        row["sample_id"]: row
        for row in solution["validation"]["records"]
        if row["split"] == "heldout"
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    records = []
    for sample in session_samples(args.session_dir, "heldout"):
        sid = sample["sample_id"]
        image_path = args.session_dir / "samples" / sid / sample["images"]["left"]
        image = cv2.imread(str(image_path))
        if image is None:
            raise FileNotFoundError(image_path)
        camera = sample["camera_info"]
        observed = sample["marker"]["T_camera_from_marker"]
        predicted = predictions[sid]["predicted_T_camera_from_marker"]
        corners = np.rint(np.asarray(sample["marker"]["corners_px"])).astype(int)
        cv2.polylines(image, [corners], True, (0, 255, 255), 2, cv2.LINE_AA)
        draw_axes(image, observed, camera["K"], camera["distortion"], "PnP", (0, 255, 255))
        draw_axes(image, predicted, camera["K"], camera["distortion"], "pred", (255, 0, 255))
        cv2.putText(
            image,
            f"held-out {sid}: yellow=Pnp corners, axes PnP+prediction",
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (40, 40, 40),
            2,
            cv2.LINE_AA,
        )
        output = args.output_dir / f"{sid}_overlay.png"
        cv2.imwrite(str(output), image)
        records.append(
            {
                "sample_id": sid,
                "overlay": str(output),
                "translation_mm": predictions[sid]["translation_mm"],
                "rotation_deg": predictions[sid]["rotation_deg"],
                "reprojection_rmse_px": predictions[sid]["reprojection_rmse_px"],
            }
        )
    write_json(args.output_dir / "overlay_report.json", {"records": records})
    print(f"rendered {len(records)} held-out overlays")


if __name__ == "__main__":
    main()

