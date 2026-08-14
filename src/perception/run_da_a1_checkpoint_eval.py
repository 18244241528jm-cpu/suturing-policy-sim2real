#!/usr/bin/env python3
"""Evaluate Xiangrui's metric Depth Anything checkpoint without infer_image()."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
MODEL_CONFIGS = {
    "vits": {"encoder": "vits", "features": 64, "out_channels": [48, 96, 192, 384]},
    "vitb": {"encoder": "vitb", "features": 128, "out_channels": [96, 192, 384, 768]},
    "vitl": {
        "encoder": "vitl",
        "features": 256,
        "out_channels": [256, 512, 1024, 1024],
    },
    "vitg": {
        "encoder": "vitg",
        "features": 384,
        "out_channels": [1536, 1536, 1536, 1536],
    },
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class MetricDepthModel(nn.Module):
    def __init__(self, backbone: nn.Module, max_depth: float):
        super().__init__()
        self.backbone = backbone
        self.log_scale = nn.Parameter(torch.tensor(-2.0))
        self.shift_raw = nn.Parameter(torch.tensor(-2.0))
        self.bias_raw = nn.Parameter(torch.tensor(-10.0))
        self.max_depth = float(max_depth)

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        relative = self.backbone(image)
        if relative.ndim == 3:
            relative = relative.unsqueeze(1)
        relative = F.relu(relative)
        scale = torch.exp(torch.clamp(self.log_scale, -12.0, 12.0))
        shift = F.softplus(self.shift_raw) + 1e-4
        bias = F.softplus(self.bias_raw)
        metric_depth = scale / (relative + shift) + bias
        return torch.clamp(metric_depth, min=1e-6, max=self.max_depth)


def preprocess(image_bgr: np.ndarray, input_size: int) -> torch.Tensor:
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    square = cv2.resize(
        image_rgb,
        (input_size, input_size),
        interpolation=cv2.INTER_LINEAR,
    )
    image = square.astype(np.float32) / 255.0
    image = (image - IMAGENET_MEAN) / IMAGENET_STD
    return torch.from_numpy(image.transpose(2, 0, 1)).unsqueeze(0)


def metrics(prediction: np.ndarray, target: np.ndarray, valid: np.ndarray) -> dict:
    pred = prediction[valid].astype(np.float64)
    gt = target[valid].astype(np.float64)
    diff = pred - gt
    return {
        "valid_pixels": int(valid.sum()),
        "mae_m": float(np.mean(np.abs(diff))),
        "rmse_m": float(np.sqrt(np.mean(diff * diff))),
        "abs_rel": float(np.mean(np.abs(diff) / gt)),
    }


def parse_indices(specification: str) -> list[int]:
    result: list[int] = []
    for item in specification.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" in item:
            start, stop = (int(value) for value in item.split(":", 1))
            result.extend(range(start, stop))
        else:
            result.append(int(item))
    return result


def state_dict_description(state_dict: dict[str, torch.Tensor]) -> list[dict]:
    return [
        {
            "key": key,
            "shape": list(value.shape),
            "dtype": str(value.dtype),
        }
        for key, value in sorted(state_dict.items())
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--data-dir", type=Path, required=True)
    parser.add_argument("--indices", required=True, help="Example: 0:20 or 800:820")
    parser.add_argument("--split-label", required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--prediction-dir", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    sys.path.insert(0, str(args.repo_root.resolve()))
    from depth_anything_v2.dpt import DepthAnythingV2

    checkpoint = torch.load(
        args.checkpoint,
        map_location="cpu",
        weights_only=False,
        mmap=True,
    )
    saved_args = checkpoint["args"]
    state_dict = checkpoint["model"]
    encoder = saved_args["encoder"]
    input_size = int(saved_args["input_size"])
    min_depth = float(saved_args["min_depth"])
    max_depth = float(saved_args["max_depth"])

    try:
        backbone = DepthAnythingV2(**MODEL_CONFIGS[encoder], max_depth=max_depth)
        constructor_mode = "accepted max_depth"
    except TypeError:
        backbone = DepthAnythingV2(**MODEL_CONFIGS[encoder])
        constructor_mode = "relative-depth backbone plus external metric adapter"
    model = MetricDepthModel(backbone, max_depth=max_depth)
    incompatible = model.load_state_dict(state_dict, strict=True)
    if incompatible.missing_keys or incompatible.unexpected_keys:
        raise RuntimeError(str(incompatible))
    model = model.to(args.device).eval()

    indices = parse_indices(args.indices)
    per_frame = []
    all_abs_error = 0.0
    all_sq_error = 0.0
    all_abs_rel = 0.0
    all_valid = 0
    all_clamped = 0
    all_pixels = 0
    pred_global_min = float("inf")
    pred_global_max = float("-inf")
    frame_pred_medians: list[float] = []

    if args.prediction_dir:
        args.prediction_dir.mkdir(parents=True, exist_ok=True)

    with torch.inference_mode():
        for index in indices:
            stem = f"frame_{index:06d}"
            image_path = args.data_dir / "image" / f"{stem}.png"
            depth_path = args.data_dir / "depth" / f"{stem}.npy"
            image_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image_bgr is None:
                raise FileNotFoundError(image_path)
            target = np.load(depth_path, allow_pickle=False).astype(np.float32)
            if target.shape != image_bgr.shape[:2]:
                raise ValueError(f"RGB/depth mismatch for {stem}")

            image_tensor = preprocess(image_bgr, input_size).to(args.device)
            square_pred = model(image_tensor)[0, 0].float().cpu().numpy()
            prediction = cv2.resize(
                square_pred,
                (image_bgr.shape[1], image_bgr.shape[0]),
                interpolation=cv2.INTER_LINEAR,
            ).astype(np.float32)
            valid = (
                np.isfinite(target)
                & (target >= min_depth)
                & (target <= max_depth)
            )
            frame_metrics = metrics(prediction, target, valid)

            square_target = cv2.resize(
                np.where(valid, target, 0.0).astype(np.float32),
                (input_size, input_size),
                interpolation=cv2.INTER_NEAREST,
            )
            square_valid = cv2.resize(
                valid.astype(np.uint8),
                (input_size, input_size),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)
            square_metrics = metrics(square_pred, square_target, square_valid)

            pred_valid = prediction[valid].astype(np.float64)
            gt_valid = target[valid].astype(np.float64)
            diff = pred_valid - gt_valid
            clamp_tolerance = max(1e-6, max_depth * 1e-6)
            clamped = int((prediction >= max_depth - clamp_tolerance).sum())
            all_abs_error += float(np.abs(diff).sum())
            all_sq_error += float((diff * diff).sum())
            all_abs_rel += float((np.abs(diff) / gt_valid).sum())
            all_valid += int(valid.sum())
            all_clamped += clamped
            all_pixels += prediction.size
            pred_global_min = min(pred_global_min, float(prediction.min()))
            pred_global_max = max(pred_global_max, float(prediction.max()))
            frame_pred_medians.append(float(np.median(prediction)))

            if args.prediction_dir:
                np.save(args.prediction_dir / f"{stem}.npy", prediction)

            per_frame.append(
                {
                    "frame": stem,
                    "image_sha256": sha256(image_path),
                    "depth_sha256": sha256(depth_path),
                    "restored_camera_resolution": frame_metrics,
                    "training_square_resolution": square_metrics,
                    "prediction_min_m": float(prediction.min()),
                    "prediction_median_m": float(np.median(prediction)),
                    "prediction_max_m": float(prediction.max()),
                    "max_depth_clamped_pixel_fraction": clamped / prediction.size,
                }
            )

    report = {
        "schema_version": 1,
        "split_label": args.split_label,
        "indices": indices,
        "checkpoint": {
            "path": str(args.checkpoint.resolve()),
            "sha256": sha256(args.checkpoint),
            "top_level_keys": sorted(checkpoint.keys()),
            "epoch_zero_based": int(checkpoint["epoch"]),
            "saved_args": saved_args,
            "saved_metrics": checkpoint["metrics"],
            "state_dict_entry_count": len(state_dict),
            "state_dict": state_dict_description(state_dict),
            "learned_scalars_raw": {
                name: float(state_dict[name].detach().cpu())
                for name in ("log_scale", "shift_raw", "bias_raw")
            },
            "learned_scalars_transformed": {
                "scale_exp_log_scale": math.exp(
                    float(state_dict["log_scale"].detach().cpu())
                ),
                "shift_softplus_plus_1e_4": float(
                    F.softplus(state_dict["shift_raw"]).detach().cpu() + 1e-4
                ),
                "bias_softplus": float(
                    F.softplus(state_dict["bias_raw"]).detach().cpu()
                ),
            },
            "backbone_constructor": constructor_mode,
            "strict_state_dict_load": True,
        },
        "preprocessing": {
            "input_size": [input_size, input_size],
            "resize": "forced square cv2.INTER_LINEAR; no aspect preservation",
            "rgb_scale": "/255",
            "imagenet_mean": IMAGENET_MEAN.tolist(),
            "imagenet_std": IMAGENET_STD.tolist(),
            "output_restore": "cv2.INTER_LINEAR to original camera resolution",
        },
        "aggregate_original_resolution": {
            "frames": len(per_frame),
            "valid_pixels": all_valid,
            "mae_m": all_abs_error / all_valid,
            "rmse_m": math.sqrt(all_sq_error / all_valid),
            "abs_rel": all_abs_rel / all_valid,
            "prediction_global_min_m": pred_global_min,
            "prediction_median_of_frame_medians_m": float(
                np.median(frame_pred_medians)
            ),
            "prediction_global_max_m": pred_global_max,
            "max_depth_clamped_pixel_fraction": all_clamped / all_pixels,
        },
        "per_frame": per_frame,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(
        json.dumps(report, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(json.dumps(report["aggregate_original_resolution"], indent=2))


if __name__ == "__main__":
    main()
