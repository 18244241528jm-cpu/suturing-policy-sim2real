#!/usr/bin/env python3
"""Capture synchronized stereo observations on request from the P9a reset driver."""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy

from capture_stage_c2_stereo import (
    K,
    StereoNode,
    depth_image,
    needle_mask,
    pose_matrix,
    rgb_image,
    stamp_ns,
)
from p9a_goal_geometry import rotation_distance_deg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--request-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--expected-frames", type=int, required=True)
    parser.add_argument("--ready-timeout-s", type=float, default=45.0)
    parser.add_argument("--request-timeout-s", type=float, default=120.0)
    parser.add_argument("--fresh-timeout-s", type=float, default=8.0)
    parser.add_argument("--max-sync-error-ms", type=float, default=5.0)
    return parser.parse_args()


def atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".partial")
    temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    os.replace(temporary, path)


def wait_for(path: Path, timeout_s: float) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.is_file():
            return json.loads(path.read_text(encoding="utf-8"))
        time.sleep(0.02)
    raise TimeoutError(f"Timed out waiting for {path}")


def wait_for_needle_pose(
    node: StereoNode,
    expected_world: np.ndarray,
    timeout_s: float,
    translation_tolerance_mm: float = 0.50,
    rotation_tolerance_deg: float = 1.0,
) -> tuple[float, float]:
    """Drain stale ROS state and wait for the requested settled reset pose."""
    deadline = time.monotonic() + timeout_s
    latest = (float("inf"), float("inf"))
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if "needle" not in node.messages:
            continue
        needle_world = pose_matrix(node.messages["needle"])
        latest = (
            float(
                np.linalg.norm(
                    needle_world[:3, 3] - expected_world[:3, 3]
                )
                * 1000.0
            ),
            rotation_distance_deg(needle_world, expected_world),
        )
        if (
            latest[0] <= translation_tolerance_mm
            and latest[1] <= rotation_tolerance_deg
        ):
            return latest
    raise RuntimeError(
        "Needle state never aligned with requested reset snapshot: "
        f"latest={latest[0]:.4f} mm/{latest[1]:.4f} deg"
    )


def main() -> int:
    args = parse_args()
    if args.out.exists() and any(args.out.iterdir()):
        raise SystemExit(f"Output directory must be empty: {args.out}")
    args.request_dir.mkdir(parents=True, exist_ok=True)
    for side in ("L", "R"):
        for folder in ("rgb", "depth_gt_m", "needle_mask", "segmentation_raw"):
            (args.out / side / folder).mkdir(parents=True, exist_ok=True)
    (args.out / "poses").mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = StereoNode()
    rows: list[dict] = []
    try:
        node.wait_ready(args.ready_timeout_s)
        prior_serials = dict(node.serials)
        names = ("Lrgb", "Lseg", "Ldepth", "Rrgb", "Rseg", "Rdepth")
        prior_stamps = {name: stamp_ns(node.messages[name]) for name in names}

        for index in range(args.expected_frames):
            frame_id = f"frame_{index:06d}"
            request_path = args.request_dir / f"{frame_id}.request.json"
            done_path = args.request_dir / f"{frame_id}.done.json"
            request = wait_for(request_path, args.request_timeout_s)
            expected_world = np.asarray(
                request["expected_T_Wneedle"], dtype=np.float64
            )
            wait_for_needle_pose(node, expected_world, args.fresh_timeout_s)
            stamps = node.fresh_six(
                prior_serials,
                prior_stamps,
                args.fresh_timeout_s,
                args.max_sync_error_ms,
            )
            prior_serials, prior_stamps = dict(node.serials), dict(stamps)

            needle_world = pose_matrix(node.messages["needle"])
            trans_mm = float(
                np.linalg.norm(needle_world[:3, 3] - expected_world[:3, 3])
                * 1000.0
            )
            rot_deg = rotation_distance_deg(needle_world, expected_world)
            if trans_mm > 0.50 or rot_deg > 1.0:
                raise RuntimeError(
                    f"{frame_id}: camera/needle bundle is not paired: "
                    f"{trans_mm:.4f} mm/{rot_deg:.4f} deg"
                )
            world_frame = pose_matrix(node.messages["camera_frame"])
            phantom_world = pose_matrix(node.messages["phantom"])
            side_poses: dict[str, np.ndarray] = {}
            mask_counts: dict[str, int] = {}

            for side in ("L", "R"):
                rgb = rgb_image(node.messages[f"{side}rgb"])
                segmentation = rgb_image(node.messages[f"{side}seg"])
                depth = depth_image(node.messages[f"{side}depth"])
                mask = needle_mask(segmentation)
                if not np.any(mask):
                    raise RuntimeError(f"{frame_id} {side}: empty needle mask")
                side_poses[f"{side}T_Wcamera"] = (
                    world_frame @ pose_matrix(node.messages[f"{side}camera"])
                )
                mask_counts[f"{side}_mask_pixels"] = int((mask > 0).sum())
                cv2.imwrite(
                    str(args.out / side / "rgb" / f"{frame_id}.png"),
                    cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                )
                np.save(
                    args.out / side / "depth_gt_m" / f"{frame_id}.npy",
                    depth.astype(np.float32),
                )
                cv2.imwrite(
                    str(args.out / side / "needle_mask" / f"{frame_id}.png"),
                    mask,
                )
                cv2.imwrite(
                    str(
                        args.out
                        / side
                        / "segmentation_raw"
                        / f"{frame_id}.png"
                    ),
                    cv2.cvtColor(segmentation, cv2.COLOR_RGB2BGR),
                )

            np.savez(
                args.out / "poses" / f"{frame_id}.npz",
                T_Wneedle=needle_world,
                expected_T_Wneedle=expected_world,
                T_Wphantom=phantom_world,
                K=K,
                **side_poses,
            )
            spread_ms = (max(stamps.values()) - min(stamps.values())) / 1e6
            row = {
                "frame_id": frame_id,
                "reset_seed": int(request["reset_seed"]),
                **mask_counts,
                "six_stream_sync_spread_ms": float(spread_ms),
                "capture_vs_reset_translation_mm": trans_mm,
                "capture_vs_reset_rotation_deg": rot_deg,
            }
            rows.append(row)
            atomic_json(done_path, row)
            atomic_json(
                args.out / "capture_progress.json",
                {
                    "complete": False,
                    "frames_completed": len(rows),
                    "frames_expected": args.expected_frames,
                    "rows": rows,
                },
            )
            print(
                f"P9A_CAMERA_CAPTURE {frame_id} "
                f"pair={trans_mm:.4f}mm/{rot_deg:.4f}deg "
                f"mask={mask_counts['L_mask_pixels']}",
                flush=True,
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    atomic_json(
        args.out / "capture_report.json",
        {
            "complete": True,
            "frames_completed": len(rows),
            "frames_expected": args.expected_frames,
            "depth_source": "AMBF ground-truth depth",
            "mask_source": "AMBF segmentation pass",
            "rows": rows,
        },
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
