#!/usr/bin/env python3
"""Offline, stage-by-stage real-image checks for the SurgicAI perception chain.

This tool never imports ROS and never publishes a robot command.  Each subcommand
owns one boundary so a DA, mask, camera-contract, or FoundationPose failure can be
diagnosed without launching the complete real-robot runtime.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np


EXPECTED_CHECKPOINT_SHA256 = "fc46bead4a5ea0e4122566bb88b93932aa82f110ee98281b5fcb09f499c9ec88"
MODEL_CONFIGS = {
    "vitl": {"encoder": "vitl", "features": 256,
             "out_channels": [256, 512, 1024, 1024]},
}
CAMERA_PROFILES = {
    # Values transcribed from the JHU camera calibration record supplied with
    # the project.  Select rect only for /image_rect; select raw only for raw.
    "jhu-left-rect-1300x1024": np.asarray(
        [[1860.724932, 0.0, 690.538345],
         [0.0, 1860.724932, 576.803638],
         [0.0, 0.0, 1.0]], dtype=np.float64),
    "jhu-left-raw-1300x1024": np.asarray(
        [[1631.541251, 0.0, 728.223908],
         [0.0, 1624.847023, 574.776752],
         [0.0, 0.0, 1.0]], dtype=np.float64),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".partial")
    temp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temp.replace(path)


def read_color(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"D16-E101-IMAGE_NOT_READABLE {path}")
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"D16-E102-IMAGE_NOT_BGR8 shape={image.shape}")
    return image


def normalize_u8(values: np.ndarray, low: float = 1.0, high: float = 99.0) -> np.ndarray:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return np.zeros(values.shape, dtype=np.uint8)
    lo, hi = np.percentile(finite, [low, high])
    if hi <= lo:
        hi = lo + 1.0
    return np.clip((values - lo) * 255.0 / (hi - lo), 0, 255).astype(np.uint8)


def depth_color(depth: np.ndarray) -> np.ndarray:
    return cv2.applyColorMap(normalize_u8(depth), cv2.COLORMAP_TURBO)


def command_inspect(args: argparse.Namespace) -> int:
    image = read_color(args.image)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    result = {
        "schema": "SurgicAI.D16.image_inspect.v1",
        "image": str(args.image.resolve()),
        "sha256": sha256(args.image),
        "width": int(image.shape[1]), "height": int(image.shape[0]),
        "channels": 3,
        "luminance_p01_p50_p99": [float(x) for x in np.percentile(gray, [1, 50, 99])],
        "clipped_dark_fraction": float(np.mean(gray <= 2)),
        "clipped_bright_fraction": float(np.mean(gray >= 253)),
        "laplacian_variance": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
        "saturation_p50_p95": [float(x) for x in np.percentile(hsv[..., 1], [50, 95])],
    }
    args.out.mkdir(parents=True, exist_ok=False)
    cv2.imwrite(str(args.out / "rgb.png"), image)
    atomic_json(args.out / "inspect.json", result)
    print(json.dumps(result, indent=2))
    return 0


def load_da_model(repo: Path, checkpoint: Path, device: str):
    actual = sha256(checkpoint)
    if actual != EXPECTED_CHECKPOINT_SHA256:
        raise RuntimeError(
            f"D16-E201-CHECKPOINT_SHA expected={EXPECTED_CHECKPOINT_SHA256} actual={actual}"
        )
    if not (repo / "depth_anything_v2" / "dpt.py").is_file():
        raise FileNotFoundError(f"D16-E202-DA_REPO_INVALID {repo}")
    import torch
    import torch.nn as nn
    import torch.nn.functional as functional

    sys.path.insert(0, str(repo.resolve()))
    from depth_anything_v2.dpt import DepthAnythingV2

    class MetricDepthModel(nn.Module):
        def __init__(self, backbone, max_depth):
            super().__init__()
            self.backbone = backbone
            self.log_scale = nn.Parameter(torch.tensor(-2.0))
            self.shift_raw = nn.Parameter(torch.tensor(-2.0))
            self.bias_raw = nn.Parameter(torch.tensor(-10.0))
            self.max_depth = float(max_depth)

        def forward(self, image):
            relative = self.backbone(image)
            if relative.ndim == 3:
                relative = relative.unsqueeze(1)
            relative = functional.relu(relative)
            scale = torch.exp(torch.clamp(self.log_scale, -12.0, 12.0))
            shift = functional.softplus(self.shift_raw) + 1.0e-4
            bias = functional.softplus(self.bias_raw)
            depth = scale / (relative + shift) + bias
            return torch.clamp(depth, min=1.0e-6, max=self.max_depth)

    payload = torch.load(checkpoint, map_location="cpu", weights_only=False, mmap=True)
    if "args" not in payload or "model" not in payload:
        raise RuntimeError("D16-E203-CHECKPOINT_SCHEMA")
    saved = payload["args"]
    contract = (str(saved.get("encoder")), int(saved.get("input_size", -1)),
                str(saved.get("precision")))
    if contract != ("vitl", 518, "fp32"):
        raise RuntimeError(f"D16-E204-MODEL_CONTRACT got={contract}")
    try:
        backbone = DepthAnythingV2(**MODEL_CONFIGS["vitl"], max_depth=float(saved["max_depth"]))
    except TypeError:
        backbone = DepthAnythingV2(**MODEL_CONFIGS["vitl"])
    model = MetricDepthModel(backbone, float(saved["max_depth"]))
    incompatible = model.load_state_dict(payload["model"], strict=True)
    if incompatible.missing_keys or incompatible.unexpected_keys:
        raise RuntimeError(f"D16-E205-STATE_DICT {incompatible}")
    return model.to(device).eval(), torch, actual


def preprocess_da(image_bgr: np.ndarray, torch_module):
    rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    square = cv2.resize(rgb, (518, 518), interpolation=cv2.INTER_LINEAR)
    image = square.astype(np.float32) / 255.0
    image = (image - np.asarray([0.485, 0.456, 0.406], np.float32)) / np.asarray(
        [0.229, 0.224, 0.225], np.float32
    )
    return torch_module.from_numpy(image.transpose(2, 0, 1)).unsqueeze(0)


def photometric_variants(image: np.ndarray, stress: bool) -> dict[str, np.ndarray]:
    variants = {"original": image}
    if not stress:
        return variants
    unit = image.astype(np.float32) / 255.0
    variants["gamma_0p8"] = np.clip(unit ** 0.8 * 255.0, 0, 255).astype(np.uint8)
    variants["gamma_1p2"] = np.clip(unit ** 1.2 * 255.0, 0, 255).astype(np.uint8)
    variants["brightness_minus10"] = np.clip(image.astype(np.int16) - 10, 0, 255).astype(np.uint8)
    variants["brightness_plus10"] = np.clip(image.astype(np.int16) + 10, 0, 255).astype(np.uint8)
    return variants


def command_da(args: argparse.Namespace) -> int:
    image = read_color(args.image)
    args.out.mkdir(parents=True, exist_ok=False)
    model, torch, checkpoint_sha = load_da_model(args.da_repo, args.checkpoint, args.device)
    rows, predictions = [], {}
    for name, variant in photometric_variants(image, args.stress).items():
        tensor = preprocess_da(variant, torch).to(args.device)
        if args.device.startswith("cuda"):
            torch.cuda.synchronize()
        started = time.perf_counter()
        with torch.inference_mode():
            square = model(tensor)[0, 0].float().cpu().numpy()
        if args.device.startswith("cuda"):
            torch.cuda.synchronize()
        elapsed = time.perf_counter() - started
        depth = cv2.resize(square, (image.shape[1], image.shape[0]),
                           interpolation=cv2.INTER_LINEAR).astype(np.float32)
        if not np.isfinite(depth).all() or np.any(depth <= 0):
            raise ValueError(f"D16-E206-DEPTH_INVALID variant={name}")
        predictions[name] = depth
        np.save(args.out / f"depth_{name}_m.npy", depth, allow_pickle=False)
        cv2.imwrite(str(args.out / f"depth_{name}_color.png"), depth_color(depth))
        cv2.imwrite(str(args.out / f"rgb_{name}.png"), variant)
        rows.append({"variant": name, "inference_seconds": elapsed,
                     "depth_min_m": float(depth.min()),
                     "depth_p05_m": float(np.percentile(depth, 5)),
                     "depth_p50_m": float(np.percentile(depth, 50)),
                     "depth_p95_m": float(np.percentile(depth, 95)),
                     "depth_max_m": float(depth.max())})
    original = predictions["original"]
    stability = []
    for name, depth in predictions.items():
        delta = np.abs(depth - original) * 1000.0
        stability.append({"variant": name, "mae_vs_original_mm": float(delta.mean()),
                          "p95_vs_original_mm": float(np.percentile(delta, 95)),
                          "max_vs_original_mm": float(delta.max())})
    panel = np.hstack([image, depth_color(original)])
    cv2.putText(panel, "Real RGB", (20, 38), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    cv2.putText(panel, "P5a DA metric depth (relative colors)", (image.shape[1] + 20, 38),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    cv2.imwrite(str(args.out / "rgb_vs_da.png"), panel)
    result = {"schema": "SurgicAI.D16.offline_DA.v1", "complete": True,
              "image_sha256": sha256(args.image), "checkpoint_sha256": checkpoint_sha,
              "contract": {"encoder": "vitl", "input_size": 518, "precision": "fp32"},
              "device": args.device, "stress_enabled": bool(args.stress),
              "rows": rows, "photometric_stability": stability,
              "accuracy_warning": "No real metric-depth ground truth was supplied; these are output and stability checks, not accuracy measurements."}
    atomic_json(args.out / "da_result.json", result)
    print(json.dumps(result, indent=2))
    return 0


def command_mask(args: argparse.Namespace) -> int:
    image = read_color(args.image)
    spec = json.loads(args.annotation.read_text(encoding="utf-8"))
    points = np.asarray(spec["polyline_xy"], dtype=np.int32).reshape(-1, 1, 2)
    if len(points) < 2:
        raise ValueError("D16-E301-MASK_POLYLINE_TOO_SHORT")
    args.out.mkdir(parents=True, exist_ok=False)
    rows = []
    for width in args.widths:
        if width <= 0:
            raise ValueError("D16-E302-MASK_WIDTH_NONPOSITIVE")
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        cv2.polylines(mask, [points], False, 255, thickness=int(width), lineType=cv2.LINE_AA)
        mask = np.where(mask >= 128, 255, 0).astype(np.uint8)
        overlay = image.copy()
        overlay[mask > 0] = (0.35 * overlay[mask > 0] + 0.65 * np.asarray([0, 255, 0])).astype(np.uint8)
        cv2.polylines(overlay, [points], False, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.imwrite(str(args.out / f"mask_w{width}.png"), mask)
        cv2.imwrite(str(args.out / f"overlay_w{width}.png"), overlay)
        rows.append({"width_px": int(width), "mask_pixels": int(np.count_nonzero(mask)),
                     "mask_fraction": float(np.mean(mask > 0))})
    segments = np.diff(points.reshape(-1, 2).astype(np.float64), axis=0)
    polyline_length_px = float(np.linalg.norm(segments, axis=1).sum())
    result = {"schema": "SurgicAI.D16.manual_mask_variants.v1", "complete": True,
              "image_sha256": sha256(args.image), "annotation": spec, "rows": rows,
              "annotated_polyline_length_px": polyline_length_px,
              "semantic_warning": "Polyline was manually specified for this audit. Inspect every overlay; the program cannot prove that the marked pixels are the physical needle rather than suture thread or glare."}
    atomic_json(args.out / "mask_result.json", result)
    return 0


def load_k(profile: str, custom_k: str | None) -> np.ndarray:
    if custom_k:
        values = [float(x) for x in custom_k.split(",")]
        if len(values) != 9:
            raise ValueError("D16-E401-K_REQUIRES_9_VALUES")
        return np.asarray(values, dtype=np.float64).reshape(3, 3)
    if profile not in CAMERA_PROFILES:
        raise ValueError(f"D16-E402-UNKNOWN_CAMERA_PROFILE {profile}")
    return CAMERA_PROFILES[profile].copy()


def command_bundle(args: argparse.Namespace) -> int:
    image = read_color(args.image)
    depth = np.load(args.depth, allow_pickle=False).astype(np.float32)
    mask = cv2.imread(str(args.mask), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise FileNotFoundError(f"D16-E403-MASK_NOT_READABLE {args.mask}")
    if depth.shape != image.shape[:2] or mask.shape != image.shape[:2]:
        raise ValueError(f"D16-E404-SHAPE image={image.shape[:2]} depth={depth.shape} mask={mask.shape}")
    if not math.isfinite(args.depth_scale) or args.depth_scale <= 0:
        raise ValueError("D16-E405-DEPTH_SCALE_INVALID")
    depth = depth * float(args.depth_scale)
    if not np.isfinite(depth).all() or np.any(depth <= 0):
        raise ValueError("D16-E405-DEPTH_INVALID")
    binary = mask > 0
    if not np.any(binary):
        raise ValueError("D16-E406-MASK_EMPTY")
    K = load_k(args.camera_profile, args.k)
    dilation = cv2.dilate(binary.astype(np.uint8), np.ones((args.ring_px * 2 + 1,) * 2, np.uint8)) > 0
    ring = dilation & ~binary
    needle_depth = depth[binary]
    ring_depth = depth[ring]
    relief_mm = (float(np.median(ring_depth)) - float(np.median(needle_depth))) * 1000.0
    args.out.mkdir(parents=True, exist_ok=False)
    cv2.imwrite(str(args.out / "rgb.png"), image)
    cv2.imwrite(str(args.out / "mask.png"), binary.astype(np.uint8) * 255)
    np.save(args.out / "depth_m.npy", depth, allow_pickle=False)
    np.save(args.out / "K.npy", K, allow_pickle=False)
    overlay = image.copy()
    overlay[ring] = (0.5 * overlay[ring] + 0.5 * np.asarray([255, 0, 0])).astype(np.uint8)
    overlay[binary] = (0.3 * overlay[binary] + 0.7 * np.asarray([0, 255, 0])).astype(np.uint8)
    cv2.imwrite(str(args.out / "mask_ring_overlay.png"), overlay)
    result = {"schema": "SurgicAI.D16.fp_input_bundle.v1", "complete": True,
              "image_sha256": sha256(args.image), "depth_sha256": sha256(args.depth),
              "mask_sha256": sha256(args.mask), "camera_profile": args.camera_profile,
              "depth_scale_diagnostic_only": float(args.depth_scale),
              "camera_contract_warning": "The image topic/header was not supplied with these two screenshots. Raw-vs-rectified K is therefore a hypothesis, not measured provenance.",
              "K": K.tolist(), "width": int(image.shape[1]), "height": int(image.shape[0]),
              "mask_pixels": int(binary.sum()), "ring_pixels": int(ring.sum()),
              "needle_depth_median_m": float(np.median(needle_depth)),
              "ring_depth_median_m": float(np.median(ring_depth)),
              "predicted_needle_relief_mm_positive_means_closer": relief_mm,
              "relief_interpretation": "A positive sign is physically plausible for a needle resting above the nearby phantom, but without real depth GT its magnitude is not an accuracy measurement."}
    atomic_json(args.out / "bundle.json", result)
    return 0


def rotation_distance_deg(a: np.ndarray, b: np.ndarray) -> float:
    relative = a[:3, :3] @ b[:3, :3].T
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def command_fp_compare(args: argparse.Namespace) -> int:
    records = []
    for result_path in sorted(args.fp_root.glob("*/result.json")):
        data = json.loads(result_path.read_text(encoding="utf-8"))
        pose = np.asarray(data["top_pose_camera"], dtype=np.float64)
        records.append({"run": result_path.parent.name, "source_bundle": data["source_bundle"],
                        "register_success": data["register_success"],
                        "candidate_count": data["candidate_count"], "pose": pose})
    if not records:
        raise FileNotFoundError(f"D16-E501-NO_FP_RESULTS {args.fp_root}")
    pairs = []
    for i in range(len(records)):
        for j in range(i + 1, len(records)):
            if records[i]["run"].split("__")[0] != records[j]["run"].split("__")[0]:
                continue
            pa, pb = records[i]["pose"], records[j]["pose"]
            pairs.append({"a": records[i]["run"], "b": records[j]["run"],
                          "translation_mm": float(np.linalg.norm(pa[:3, 3] - pb[:3, 3]) * 1000.0),
                          "rotation_deg": rotation_distance_deg(pa, pb)})
    result = {"schema": "SurgicAI.D16.fp_sensitivity.v1", "complete": True,
              "runs": len(records), "within_image_pairs": pairs,
              "warning": "No GT pose was supplied. Pairwise agreement only measures sensitivity; stable 180-degree wrong branches remain possible."}
    atomic_json(args.out, result)
    print(json.dumps(result, indent=2))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    inspect = sub.add_parser("inspect"); inspect.add_argument("--image", type=Path, required=True)
    inspect.add_argument("--out", type=Path, required=True); inspect.set_defaults(func=command_inspect)
    da = sub.add_parser("da"); da.add_argument("--image", type=Path, required=True)
    da.add_argument("--da-repo", type=Path, required=True); da.add_argument("--checkpoint", type=Path, required=True)
    da.add_argument("--device", default="cuda"); da.add_argument("--stress", action="store_true")
    da.add_argument("--out", type=Path, required=True); da.set_defaults(func=command_da)
    mask = sub.add_parser("mask"); mask.add_argument("--image", type=Path, required=True)
    mask.add_argument("--annotation", type=Path, required=True); mask.add_argument("--widths", type=int, nargs="+", default=[5, 9, 15])
    mask.add_argument("--out", type=Path, required=True); mask.set_defaults(func=command_mask)
    bundle = sub.add_parser("bundle"); bundle.add_argument("--image", type=Path, required=True)
    bundle.add_argument("--depth", type=Path, required=True); bundle.add_argument("--mask", type=Path, required=True)
    bundle.add_argument("--camera-profile", choices=sorted(CAMERA_PROFILES), default="jhu-left-rect-1300x1024")
    bundle.add_argument("--k"); bundle.add_argument("--ring-px", type=int, default=12)
    bundle.add_argument("--depth-scale", type=float, default=1.0,
                        help="Diagnostic sensitivity only; never treat a selected factor as calibrated metric depth")
    bundle.add_argument("--out", type=Path, required=True); bundle.set_defaults(func=command_bundle)
    compare = sub.add_parser("fp-compare"); compare.add_argument("--fp-root", type=Path, required=True)
    compare.add_argument("--out", type=Path, required=True); compare.set_defaults(func=command_fp_compare)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        return int(args.func(args))
    except Exception as exc:
        print(f"D16_STAGE_FAILED command={args.command} error={type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
