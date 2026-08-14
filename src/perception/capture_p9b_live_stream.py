#!/usr/bin/env python3
"""Publish a persistent three-slot AMBF camera stream for P9b."""

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
    K, StereoNode, depth_image, needle_mask, pose_matrix, rgb_image
)
from p9a_goal_geometry import rotation_distance_deg


def atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".partial")
    temporary.write_text(json.dumps(payload), encoding="utf-8")
    os.replace(temporary, path)


def write_png(path: Path, image: np.ndarray) -> None:
    temporary = path.with_name(path.stem + ".tmp" + path.suffix)
    cv2.imwrite(str(temporary), image)
    os.replace(temporary, path)


def write_npy(path: Path, array: np.ndarray) -> None:
    temporary = path.with_name(path.stem + ".tmp" + path.suffix)
    with temporary.open("wb") as stream:
        np.save(stream, array)
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--control-file", type=Path, required=True)
    parser.add_argument("--stream-dir", type=Path, required=True)
    parser.add_argument("--rate-hz", type=float, default=3.35)
    parser.add_argument("--stop-file", type=Path, required=True)
    args = parser.parse_args()
    args.stream_dir.mkdir(parents=True, exist_ok=True)
    latest_path = args.stream_dir / "latest.json"

    rclpy.init()
    node = StereoNode()
    node.wait_ready(60.0)
    current_episode = None
    sequence = -1
    next_capture = time.monotonic()
    try:
        while not args.stop_file.exists():
            rclpy.spin_once(node, timeout_sec=0.01)
            try:
                control = json.loads(args.control_file.read_text())
            except (FileNotFoundError, json.JSONDecodeError):
                time.sleep(0.01)
                continue
            episode = int(control["episode"])
            if episode != current_episode:
                current_episode = episode
                sequence = -1
                next_capture = time.monotonic()
            if time.monotonic() < next_capture:
                continue
            expected = control.get("expected_T_Wneedle")
            needle_world = pose_matrix(node.messages["needle"])
            if sequence < 0 and expected is not None:
                expected_array = np.asarray(expected, dtype=np.float64)
                if (
                    np.linalg.norm(
                        needle_world[:3, 3] - expected_array[:3, 3]
                    ) * 1000.0 > 1.0
                    or rotation_distance_deg(
                        needle_world, expected_array
                    ) > 1.0
                ):
                    time.sleep(0.01)
                    continue
            sequence += 1
            slot = sequence % 3
            rgb = rgb_image(node.messages["Lrgb"])
            depth = depth_image(node.messages["Ldepth"]).astype(np.float32)
            mask = needle_mask(rgb_image(node.messages["Lseg"]))
            world_frame = pose_matrix(node.messages["camera_frame"])
            camera_world = (
                world_frame @ pose_matrix(node.messages["Lcamera"])
            )
            rgb_path = args.stream_dir / f"rgb_{slot}.png"
            depth_path = args.stream_dir / f"depth_{slot}.npy"
            mask_path = args.stream_dir / f"mask_{slot}.png"
            write_png(rgb_path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            write_npy(depth_path, depth)
            write_png(mask_path, mask)
            now_ns = time.time_ns()
            atomic_json(
                latest_path,
                {
                    "episode": episode,
                    "sequence": sequence,
                    "capture_time_ns": now_ns,
                    "rgb": str(rgb_path),
                    "depth": str(depth_path),
                    "mask": str(mask_path),
                    "K": K.tolist(),
                    "T_Wcamera": camera_world.tolist(),
                    "T_Wneedle": needle_world.tolist(),
                    "mask_pixels": int((mask > 0).sum()),
                },
            )
            print(
                f"P9B_CAPTURE episode={episode} seq={sequence}", flush=True
            )
            next_capture += 1.0 / args.rate_hz
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

