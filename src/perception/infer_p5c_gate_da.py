#!/usr/bin/env python3
"""Run the exact P5a DA contract on P5c needle-only safe-view captures."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import torch

from run_p5_da_live_depth import load_model, preprocess, sha256


EXPECTED_SHA = "fc46bead4a5ea0e4122566bb88b93932aa82f110ee98281b5fcb09f499c9ec88"


def q(values: list[float]) -> dict:
    a = np.asarray(values, dtype=np.float64)
    return {"count": len(values), "p50": float(np.percentile(a, 50)),
            "p95": float(np.percentile(a, 95)), "max": float(a.max())}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--capture-dir", type=Path, required=True)
    parser.add_argument("--prediction-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--device", default="cuda")
    args = parser.parse_args()
    actual_sha = sha256(args.checkpoint)
    if actual_sha != EXPECTED_SHA:
        raise RuntimeError(f"checkpoint SHA mismatch: {actual_sha}")
    if args.out.exists():
        raise FileExistsError(f"Refusing overwrite: {args.out}")
    args.prediction_dir.mkdir(parents=True, exist_ok=True)
    model, input_size, saved_args = load_model(args.repo_root, args.checkpoint, args.device)
    frame_ids = sorted(path.stem for path in (args.capture_dir / "rgb").glob("*.png"))
    runtimes, full_mae, needle_mae, rows = [], [], [], []
    with torch.inference_mode():
        for frame_id in frame_ids:
            rgb = cv2.imread(str(args.capture_dir / "rgb" / f"{frame_id}.png"))
            gt = np.load(args.capture_dir / "depth_gt_m" / f"{frame_id}.npy", allow_pickle=False).astype(np.float32)
            mask_u8 = cv2.imread(str(args.capture_dir / "needle_mask" / f"{frame_id}.png"), cv2.IMREAD_GRAYSCALE)
            if rgb is None or mask_u8 is None:
                raise FileNotFoundError(frame_id)
            started = time.perf_counter()
            tensor = preprocess(rgb, input_size).to(args.device)
            torch.cuda.synchronize()
            square = model(tensor)[0, 0].float().cpu().numpy()
            torch.cuda.synchronize()
            prediction = cv2.resize(square, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_LINEAR).astype(np.float32)
            elapsed = time.perf_counter() - started
            np.save(args.prediction_dir / f"{frame_id}.npy", prediction)
            valid = np.isfinite(gt) & (gt > 0)
            mask = (mask_u8 > 0) & valid
            fmae = float(np.mean(np.abs(prediction[valid] - gt[valid])) * 1000.0)
            nmae = float(np.mean(np.abs(prediction[mask] - gt[mask])) * 1000.0)
            runtimes.append(elapsed); full_mae.append(fmae); needle_mae.append(nmae)
            rows.append({"frame_id": frame_id, "runtime_s": elapsed,
                         "full_mae_mm": fmae, "needle_mae_mm": nmae,
                         "needle_pixels": int(mask.sum())})
            print(f"P5C_DA_GATE {frame_id} {elapsed:.3f}s needle_mae={nmae:.3f}mm", flush=True)
    result = {"schema": "SurgicAI.P5c.gate_DA.v1", "complete": True,
              "frames": len(frame_ids), "checkpoint_sha256": actual_sha,
              "encoder": saved_args["encoder"], "input_size": input_size,
              "precision": saved_args["precision"], "runtime_s": q(runtimes),
              "full_mae_mm": q(full_mae), "needle_mae_mm": q(needle_mae), "rows": rows}
    args.out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
