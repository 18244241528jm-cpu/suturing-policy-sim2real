#!/usr/bin/env python3
"""Run independent FoundationPose register calls on validated D16 bundles.

Intended to execute inside the pinned FoundationPose Docker image.  It never
tracks across bundles and never sends ROS or robot commands.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def atomic_json(path: Path, payload: dict) -> None:
    temp = path.with_suffix(path.suffix + ".partial")
    temp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temp, path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--foundationpose-root", type=Path, default=Path("/workspace"))
    parser.add_argument("--bundle-root", type=Path, required=True)
    parser.add_argument("--mesh", type=Path, required=True)
    parser.add_argument("--out-root", type=Path, required=True)
    parser.add_argument("--iteration", type=int, default=5)
    parser.add_argument("--seed", type=int, default=2718)
    args = parser.parse_args()
    if args.out_root.exists() and any(args.out_root.iterdir()):
        raise FileExistsError(f"D16-E601-REFUSE_OVERWRITE {args.out_root}")
    args.out_root.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(args.foundationpose_root))
    import nvdiffrast.torch as dr
    import torch
    import trimesh
    from datareader import draw_posed_3d_box, draw_xyz_axis, set_logging_format, set_seed
    from estimater import FoundationPose, PoseRefinePredictor, ScorePredictor

    set_logging_format(); set_seed(args.seed)
    np.random.seed(args.seed); torch.manual_seed(args.seed); torch.cuda.manual_seed_all(args.seed)
    mesh = trimesh.load(args.mesh, force="mesh")
    if hasattr(mesh.visual, "to_color"):
        mesh.visual = mesh.visual.to_color()
    to_origin, extents = trimesh.bounds.oriented_bounds(mesh)
    bbox = np.stack([-extents / 2.0, extents / 2.0], axis=0).reshape(2, 3)
    estimator = FoundationPose(
        model_pts=mesh.vertices, model_normals=mesh.vertex_normals, mesh=mesh,
        scorer=ScorePredictor(), refiner=PoseRefinePredictor(),
        debug_dir="/tmp/d16_foundationpose", debug=0,
        glctx=dr.RasterizeCudaContext(),
    )
    tf_to_centered = estimator.get_tf_to_centered_mesh().detach().cpu().numpy()
    summaries = []
    for bundle_dir in sorted(p for p in args.bundle_root.iterdir() if p.is_dir()):
        required = [bundle_dir / name for name in ("rgb.png", "mask.png", "depth_m.npy", "K.npy", "bundle.json")]
        if not all(path.is_file() for path in required):
            continue
        run_out = args.out_root / bundle_dir.name; run_out.mkdir()
        bgr = cv2.imread(str(bundle_dir / "rgb.png"), cv2.IMREAD_COLOR)
        mask = cv2.imread(str(bundle_dir / "mask.png"), cv2.IMREAD_GRAYSCALE) > 0
        depth = np.load(bundle_dir / "depth_m.npy", allow_pickle=False).astype(np.float32)
        K = np.load(bundle_dir / "K.npy", allow_pickle=False).astype(np.float64)
        depth[(~np.isfinite(depth)) | (depth < 0.001)] = 0.0
        if bgr is None or depth.shape != bgr.shape[:2] or mask.shape != bgr.shape[:2]:
            raise ValueError(f"D16-E602-BUNDLE_SHAPE {bundle_dir}")
        started = time.monotonic()
        pose = estimator.register(K=K, rgb=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB),
                                  depth=depth, ob_mask=mask, iteration=args.iteration)
        elapsed = time.monotonic() - started
        centered = estimator.poses.detach().cpu().numpy()
        scores = estimator.scores.detach().cpu().numpy().reshape(-1)
        poses = centered @ tf_to_centered
        ranks = np.empty(len(scores), dtype=np.int32); ranks[np.argsort(-scores)] = np.arange(1, len(scores) + 1)
        np.savez_compressed(run_out / "candidates.npz", poses_camera=poses, scores=scores, score_ranks=ranks,
                            K=K, mesh_extents_m=extents, top_pose_camera=pose)
        center_pose = pose @ np.linalg.inv(to_origin)
        overlay = draw_posed_3d_box(K, img=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), ob_in_cam=center_pose, bbox=bbox)
        axis_scale = float(max(np.max(extents), 0.005))
        overlay = draw_xyz_axis(overlay, ob_in_cam=center_pose, scale=axis_scale, K=K,
                                thickness=2, transparency=0, is_input_rgb=True)
        cv2.imwrite(str(run_out / "top1_overlay.png"), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))
        result = {"schema": "SurgicAI.D16.offline_FP_register.v1", "complete": True,
                  "source_bundle": bundle_dir.name, "register_success": True,
                  "register_seconds": elapsed, "iteration": args.iteration,
                  "candidate_count": int(len(scores)), "mask_pixels": int(mask.sum()),
                  "mesh": str(args.mesh), "mesh_extents_m": extents.tolist(),
                  "top_score": float(scores[0]), "top_pose_camera": pose.tolist(),
                  "accuracy_warning": "No real GT pose was supplied. Overlay and cross-variant stability cannot exclude a stable 180-degree wrong branch."}
        atomic_json(run_out / "result.json", result); summaries.append(result)
        print(f"D16_FP_REGISTER {bundle_dir.name} candidates={len(scores)} seconds={elapsed:.3f}", flush=True)
    atomic_json(args.out_root / "summary.json", {"schema": "SurgicAI.D16.offline_FP_batch.v1",
                                                 "complete": True, "runs": len(summaries),
                                                 "results": summaries})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
