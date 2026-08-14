#!/usr/bin/env python3
"""Insert the exact P5a metric-DA inference contract into a P9b stream."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch

from run_da_a1_checkpoint_eval import MODEL_CONFIGS, MetricDepthModel, preprocess, sha256


def atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".partial")
    temporary.write_text(json.dumps(payload), encoding="utf-8")
    os.replace(temporary, path)


def atomic_npy(path: Path, array: np.ndarray) -> None:
    temporary = path.with_name(path.stem + ".tmp" + path.suffix)
    with temporary.open("wb") as stream:
        np.save(stream, array)
    os.replace(temporary, path)


def append_jsonl(path: Path, payload: dict) -> None:
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload) + "\n")
        stream.flush()
        os.fsync(stream.fileno())


def colorize(values: np.ndarray, lower: float, upper: float) -> np.ndarray:
    scale = max(upper - lower, 1.0e-9)
    normalized = np.clip((values - lower) / scale, 0.0, 1.0)
    return cv2.applyColorMap((normalized * 255.0).astype(np.uint8), cv2.COLORMAP_TURBO)


def save_first_frame_debug(
    debug_root: Path,
    episode: int,
    rgb_bgr: np.ndarray,
    gt_depth: np.ndarray,
    da_depth: np.ndarray,
    mask: np.ndarray,
) -> None:
    output = debug_root / f"episode_{episode:04d}"
    if (output / "COMPLETE").exists():
        return
    output.mkdir(parents=True, exist_ok=True)
    difference = da_depth.astype(np.float32) - gt_depth.astype(np.float32)
    needle_difference = np.where(mask, difference, np.nan).astype(np.float32)
    np.save(output / "gt_depth_m.npy", gt_depth.astype(np.float32))
    np.save(output / "da_depth_m.npy", da_depth.astype(np.float32))
    np.save(output / "needle_diff_da_minus_gt_m.npy", needle_difference)
    cv2.imwrite(str(output / "rgb.png"), rgb_bgr)
    cv2.imwrite(str(output / "needle_mask.png"), mask.astype(np.uint8) * 255)

    valid = np.isfinite(gt_depth) & (gt_depth > 0.0)
    lower, upper = np.percentile(gt_depth[valid], [1.0, 99.0])
    gt_color = colorize(gt_depth, float(lower), float(upper))
    da_color = colorize(da_depth, float(lower), float(upper))
    limit = max(float(np.percentile(np.abs(difference[valid]), 99.0)), 1.0e-4)
    signed = np.clip((difference + limit) / (2.0 * limit), 0.0, 1.0)
    diff_color = cv2.applyColorMap((signed * 255.0).astype(np.uint8), cv2.COLORMAP_COOL)
    mask_panel = np.zeros_like(rgb_bgr)
    mask_panel[mask] = (255, 255, 255)
    montage = np.vstack([
        np.hstack([rgb_bgr, mask_panel]),
        np.hstack([gt_color, da_color]),
        np.hstack([diff_color, np.where(mask[..., None], diff_color, 0)]),
    ])
    cv2.imwrite(str(output / "comparison_rgb_mask_gt_da_diff.png"), montage)
    metrics = {
        "episode": episode,
        "needle_pixels": int(mask.sum()),
        "needle_mae_mm": float(np.mean(np.abs(difference[mask])) * 1000.0),
        "needle_signed_bias_mm": float(np.mean(difference[mask]) * 1000.0),
        "full_valid_mae_mm": float(np.mean(np.abs(difference[valid])) * 1000.0),
        "visualization_signed_limit_mm": limit * 1000.0,
    }
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    (output / "COMPLETE").write_text("P5 first-frame DA/GT debug complete\n")


def load_model(repo_root: Path, checkpoint_path: Path, device: str):
    sys.path.insert(0, str(repo_root.resolve()))
    from depth_anything_v2.dpt import DepthAnythingV2

    checkpoint = torch.load(
        checkpoint_path, map_location="cpu", weights_only=False, mmap=True
    )
    saved_args = checkpoint["args"]
    encoder = saved_args["encoder"]
    input_size = int(saved_args["input_size"])
    max_depth = float(saved_args["max_depth"])
    try:
        backbone = DepthAnythingV2(**MODEL_CONFIGS[encoder], max_depth=max_depth)
    except TypeError:
        backbone = DepthAnythingV2(**MODEL_CONFIGS[encoder])
    model = MetricDepthModel(backbone, max_depth=max_depth)
    incompatible = model.load_state_dict(checkpoint["model"], strict=True)
    if incompatible.missing_keys or incompatible.unexpected_keys:
        raise RuntimeError(str(incompatible))
    return model.to(device).eval(), input_size, saved_args


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--stream-dir", type=Path, required=True)
    parser.add_argument("--jsonl", type=Path, required=True)
    parser.add_argument("--debug-dir", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path, required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--register-ack-file", type=Path)
    parser.add_argument(
        "--first-frame-only",
        action="store_true",
        help="Infer DA only for sequence 0; publish GT passthrough metadata afterwards.",
    )
    args = parser.parse_args()

    expected_sha = "fc46bead4a5ea0e4122566bb88b93932aa82f110ee98281b5fcb09f499c9ec88"
    actual_sha = sha256(args.checkpoint)
    if actual_sha != expected_sha:
        raise RuntimeError(f"P5a checkpoint SHA mismatch: {actual_sha}")
    args.debug_dir.mkdir(parents=True, exist_ok=True)
    args.jsonl.parent.mkdir(parents=True, exist_ok=True)
    model, input_size, saved_args = load_model(args.repo_root, args.checkpoint, args.device)
    latest_out = args.stream_dir / "latest_da.json"
    last_key = None
    last_inferred_episode = None
    held_episode = None
    if args.first_frame_only and args.register_ack_file is None:
        raise ValueError("--first-frame-only requires --register-ack-file")
    print(
        f"P5_DA_READY sha={actual_sha} encoder={saved_args['encoder']} "
        f"input={input_size} precision={saved_args['precision']}", flush=True
    )

    with torch.inference_mode():
        while not args.stop_file.exists():
            if held_episode is not None:
                try:
                    ack = json.loads(args.register_ack_file.read_text(encoding="utf-8"))
                    acknowledged = int(ack.get("episode", -1)) == int(held_episode)
                except (FileNotFoundError, json.JSONDecodeError):
                    acknowledged = False
                if not acknowledged:
                    time.sleep(0.005)
                    continue
                held_episode = None
            try:
                source_path = args.stream_dir / "latest.json"
                latest = json.loads(source_path.read_text(encoding="utf-8"))
                key = (int(latest["episode"]), int(latest["sequence"]))
            except (FileNotFoundError, json.JSONDecodeError, KeyError):
                time.sleep(0.01)
                continue
            if key == last_key:
                time.sleep(0.005)
                continue
            try:
                rgb_bgr = cv2.imread(latest["rgb"], cv2.IMREAD_COLOR)
                gt_depth = np.load(latest["depth"], allow_pickle=False).astype(np.float32)
                mask = cv2.imread(latest["mask"], cv2.IMREAD_GRAYSCALE) > 0
            except (FileNotFoundError, ValueError, TypeError):
                continue
            if rgb_bgr is None or mask is None:
                continue
            try:
                verify = json.loads(source_path.read_text(encoding="utf-8"))
                verify_key = (int(verify["episode"]), int(verify["sequence"]))
            except (FileNotFoundError, json.JSONDecodeError, KeyError):
                continue
            if verify_key != key:
                continue

            started_ns = time.time_ns()
            infer_da = not args.first_frame_only or key[0] != last_inferred_episode
            if not infer_da:
                inference_start_ns = inference_end_ns = started_ns
                da_depth = None
                output_depth_path = str(latest["depth"])
                inference_mode = "gt_passthrough_after_da_register"
            else:
                tensor = preprocess(rgb_bgr, input_size).to(args.device)
                if str(args.device).startswith("cuda"):
                    torch.cuda.synchronize()
                inference_start_ns = time.time_ns()
                square = model(tensor)[0, 0].float().cpu().numpy()
                if str(args.device).startswith("cuda"):
                    torch.cuda.synchronize()
                inference_end_ns = time.time_ns()
                da_depth = cv2.resize(
                    square, (rgb_bgr.shape[1], rgb_bgr.shape[0]), interpolation=cv2.INTER_LINEAR
                ).astype(np.float32)
                slot = key[1] % 3
                da_path = args.stream_dir / f"da_depth_{slot}.npy"
                atomic_npy(da_path, da_depth)
                output_depth_path = str(da_path)
                inference_mode = "p5a_new_da"
                save_first_frame_debug(
                    args.debug_dir, key[0], rgb_bgr, gt_depth, da_depth, mask
                )
            processed_ns = time.time_ns()
            payload = dict(latest)
            payload.update({
                "depth": output_depth_path,
                "gt_depth": str(latest["depth"]),
                "depth_source": inference_mode,
                "da_checkpoint_sha256": actual_sha,
                "da_started_time_ns": started_ns,
                "da_inference_start_time_ns": inference_start_ns,
                "da_inference_end_time_ns": inference_end_ns,
                "da_processed_time_ns": processed_ns,
                "da_inference_s": (inference_end_ns - inference_start_ns) / 1.0e9,
                "da_total_s": (processed_ns - started_ns) / 1.0e9,
            })
            atomic_json(latest_out, payload)
            append_jsonl(args.jsonl, payload)
            if args.first_frame_only and infer_da:
                last_inferred_episode = key[0]
                held_episode = key[0]
            last_key = key
            print(
                f"P5_DA episode={key[0]} seq={key[1]} "
                f"mode={inference_mode} "
                f"infer={(inference_end_ns-inference_start_ns)/1e9:.3f}s", flush=True
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
