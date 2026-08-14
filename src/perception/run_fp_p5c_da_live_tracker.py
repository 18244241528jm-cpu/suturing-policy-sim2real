#!/usr/bin/env python3
"""FoundationPose tracker for the P5c paired latest-frame DA stream."""

from __future__ import annotations

import argparse
import json
import os
import random
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from p9a_goal_geometry import rotation_distance_deg


def atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".partial")
    temporary.write_text(json.dumps(payload), encoding="utf-8")
    os.replace(temporary, path)


def append_jsonl(path: Path, payload: dict) -> None:
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload) + "\n")
        stream.flush()
        os.fsync(stream.fileno())


def host_array(value) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--foundationpose-root", type=Path, default=Path("/workspace"))
    parser.add_argument("--stream-dir", type=Path, required=True)
    parser.add_argument("--pose-file", type=Path, required=True)
    parser.add_argument("--jsonl", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path, required=True)
    parser.add_argument("--mesh", type=Path, required=True)
    parser.add_argument("--debug-dir", type=Path, required=True)
    parser.add_argument("--iteration", type=int, default=5)
    parser.add_argument("--seed", type=int, default=2718)
    parser.add_argument("--rotation-jump-reject-deg", type=float, default=90.0)
    parser.add_argument(
        "--ack-file",
        type=Path,
        help="Optional P5d handshake ACK written only after FP pose publication is complete.",
    )
    args = parser.parse_args()
    sys.path.insert(0, str(args.foundationpose_root))
    import nvdiffrast.torch as dr
    import torch
    import trimesh
    from estimater import PoseRefinePredictor, ScorePredictor
    from occlusion_eval import F_INV, make_estimator

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)
    mesh = trimesh.load(args.mesh, force="mesh")
    if hasattr(mesh.visual, "to_color"):
        mesh.visual = mesh.visual.to_color()
    args.debug_dir.mkdir(parents=True, exist_ok=True)
    args.jsonl.parent.mkdir(parents=True, exist_ok=True)
    estimator = make_estimator(
        mesh, ScorePredictor(), PoseRefinePredictor(),
        dr.RasterizeCudaContext(), str(args.debug_dir), 0,
    )
    current_episode = None
    last_sequence = -1
    previous_accepted_world = None
    source = args.stream_dir / "latest_da.json"
    print("P5C_FP_READY pairing=verified iteration=5", flush=True)
    while not args.stop_file.exists():
        try:
            first = json.loads(source.read_text(encoding="utf-8"))
            key = (int(first["episode"]), int(first["sequence"]))
            if key[0] == current_episode and key[1] <= last_sequence:
                time.sleep(0.003)
                continue
            rgb_bgr = cv2.imread(first["rgb"], cv2.IMREAD_COLOR)
            depth = np.load(first["depth"], allow_pickle=False).astype(np.float32)
            mask_u8 = cv2.imread(first["mask"], cv2.IMREAD_GRAYSCALE)
            second = json.loads(source.read_text(encoding="utf-8"))
            second_key = (int(second["episode"]), int(second["sequence"]))
        except (FileNotFoundError, json.JSONDecodeError, KeyError, ValueError, TypeError):
            time.sleep(0.003)
            continue
        if rgb_bgr is None or mask_u8 is None or second_key != key:
            continue
        started_ns = time.time_ns()
        rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
        depth[depth < 0.001] = 0.0
        mask = mask_u8 > 0
        K = np.asarray(first["K"], dtype=np.float64)
        episode, sequence = key
        if episode != current_episode:
            pose_cam = estimator.register(
                K=K, rgb=rgb, depth=depth, ob_mask=mask, iteration=args.iteration
            )
            mode = "register"
            current_episode = episode
            previous_accepted_world = None
            np.savez_compressed(
                args.debug_dir / f"episode_{episode:04d}_register_candidates.npz",
                poses=host_array(getattr(estimator, "poses", [])),
                scores=host_array(getattr(estimator, "scores", [])),
                selected_pose_cam=np.asarray(pose_cam),
            )
        else:
            pose_cam = estimator.track_one(rgb=rgb, depth=depth, K=K, iteration=args.iteration)
            mode = "track"
        candidate_world = np.asarray(first["T_Wcamera"]) @ F_INV @ pose_cam
        gt = np.asarray(first["T_Wneedle"])
        candidate_trans_mm = float(np.linalg.norm(candidate_world[:3, 3] - gt[:3, 3]) * 1000.0)
        candidate_rot_deg = rotation_distance_deg(candidate_world, gt)
        if previous_accepted_world is None:
            jump_mm = jump_deg = 0.0
        else:
            jump_mm = float(np.linalg.norm(candidate_world[:3, 3] - previous_accepted_world[:3, 3]) * 1000.0)
            jump_deg = rotation_distance_deg(candidate_world, previous_accepted_world)
        gate_rejected = bool(
            mode == "track" and previous_accepted_world is not None
            and jump_deg > args.rotation_jump_reject_deg
        )
        if gate_rejected:
            published_world = previous_accepted_world.copy()
            rejection_reason = "rotation_jump_gt_90deg"
        else:
            published_world = candidate_world.copy()
            previous_accepted_world = published_world.copy()
            rejection_reason = None
        processed_ns = time.time_ns()
        published_trans_mm = float(np.linalg.norm(published_world[:3, 3] - gt[:3, 3]) * 1000.0)
        published_rot_deg = rotation_distance_deg(published_world, gt)
        payload = {
            "episode": episode,
            "sequence": sequence,
            "mode": mode,
            "capture_time_ns": int(first["capture_time_ns"]),
            "reader_snapshot_time_ns": int(first["reader_snapshot_time_ns"]),
            "da_processed_time_ns": int(first["da_processed_time_ns"]),
            "fp_started_time_ns": started_ns,
            "processed_time_ns": processed_ns,
            "capture_to_fp_start_ms": (started_ns - int(first["capture_time_ns"])) / 1.0e6,
            "capture_to_fp_publish_ms": (processed_ns - int(first["capture_time_ns"])) / 1.0e6,
            "da_publish_to_fp_publish_ms": (processed_ns - int(first["da_processed_time_ns"])) / 1.0e6,
            "depth_source": first["depth_source"],
            "pairing_contract": first["pairing_contract"],
            "tracker_depth_source": "p5a_new_da",
            "da_checkpoint_sha256": first["da_checkpoint_sha256"],
            "fp_pose_world": published_world.tolist(),
            "gt_pose_world": gt.tolist(),
            "fp_translation_error_mm": published_trans_mm,
            "fp_rotation_error_deg": published_rot_deg,
            "fp_flip_gt_90_deg": bool(published_rot_deg > 90.0),
            "candidate_fp_pose_world": candidate_world.tolist(),
            "candidate_fp_translation_error_mm": candidate_trans_mm,
            "candidate_fp_rotation_error_deg": candidate_rot_deg,
            "candidate_fp_flip_gt_90_deg": bool(candidate_rot_deg > 90.0),
            "pose_jump_translation_mm": jump_mm,
            "pose_jump_rotation_deg": jump_deg,
            "jump_gate_rejected": gate_rejected,
            "jump_gate_rejection_reason": rejection_reason,
            "rotation_jump_reject_deg": args.rotation_jump_reject_deg,
            "mask_pixels": int(first["mask_pixels"]),
            "reader_dropped_total": int(first["reader_dropped_total"]),
        }
        atomic_json(args.pose_file, payload)
        append_jsonl(args.jsonl, payload)
        if args.ack_file is not None:
            atomic_json(
                args.ack_file,
                {
                    "episode": episode,
                    "sequence": sequence,
                    "capture_time_ns": int(first["capture_time_ns"]),
                    "fp_processed_time_ns": processed_ns,
                    "ack_time_ns": time.time_ns(),
                },
            )
        last_sequence = sequence
        print(
            f"P5C_FP episode={episode} seq={sequence} mode={mode} "
            f"published={published_trans_mm:.3f}mm/{published_rot_deg:.2f}deg "
            f"age={payload['capture_to_fp_publish_ms']:.1f}ms rejected={gate_rejected}",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
