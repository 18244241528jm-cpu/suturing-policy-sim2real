#!/usr/bin/env python3
"""Quantify and visualize the D16 two-image DA/FP audit without pose GT."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np


def atomic_json(path: Path, payload: dict) -> None:
    temp = path.with_suffix(path.suffix + ".partial")
    temp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temp.replace(path)


def load_obj_vertices(path: Path) -> np.ndarray:
    vertices = []
    with path.open("r", encoding="utf-8", errors="ignore") as stream:
        for line in stream:
            if line.startswith("v "):
                values = line.split()
                vertices.append([float(values[1]), float(values[2]), float(values[3])])
    if not vertices:
        raise ValueError(f"No OBJ vertices in {path}")
    return np.asarray(vertices, dtype=np.float64)


def project_vertices(vertices: np.ndarray, pose: np.ndarray, K: np.ndarray) -> np.ndarray:
    points = (pose[:3, :3] @ vertices.T).T + pose[:3, 3]
    valid = points[:, 2] > 1.0e-6
    points = points[valid]
    uvw = (K @ points.T).T
    return uvw[:, :2] / uvw[:, 2:3]


def candidate_2d_rows(bundle: Path, fp_run: Path, vertices: np.ndarray) -> dict:
    mask = cv2.imread(str(bundle / "mask.png"), cv2.IMREAD_GRAYSCALE) > 0
    distance = cv2.distanceTransform((~mask).astype(np.uint8), cv2.DIST_L2, 5)
    data = np.load(fp_run / "candidates.npz", allow_pickle=False)
    poses = data["poses_camera"]; scores = data["scores"]; K = data["K"]
    metrics = []
    h, w = mask.shape
    for pose in poses:
        uv = project_vertices(vertices, pose, K)
        inside = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        uv = uv[inside]
        if len(uv) == 0:
            metrics.append((1.0e6, 0.0, 1.0e6)); continue
        x = np.clip(np.rint(uv[:, 0]).astype(int), 0, w - 1)
        y = np.clip(np.rint(uv[:, 1]).astype(int), 0, h - 1)
        distances = distance[y, x]
        metrics.append((float(np.median(distances)), float(np.mean(distances <= 5.0)),
                        float(np.percentile(distances, 95))))
    medians = np.asarray([m[0] for m in metrics])
    oracle = int(np.argmin(medians))
    margin = float(scores[0] - scores[1]) if len(scores) > 1 else None
    return {"run": fp_run.name, "top1_median_vertex_to_mask_px": metrics[0][0],
            "top1_p95_vertex_to_mask_px": metrics[0][2],
            "top1_vertices_within_5px_fraction": metrics[0][1],
            "mask_distance_oracle_candidate_index": oracle,
            "oracle_median_vertex_to_mask_px": metrics[oracle][0],
            "oracle_vertices_within_5px_fraction": metrics[oracle][1],
            "top_score": float(scores[0]), "top2_score": float(scores[1]),
            "top1_top2_score_margin": margin, "candidate_count": int(len(scores))}


def make_da_figure(root: Path, out: Path) -> None:
    fig, axes = plt.subplots(2, 4, figsize=(18, 8), constrained_layout=True)
    for row, view in enumerate((1, 2)):
        image = cv2.cvtColor(cv2.imread(str(root / "inputs" / f"ecm_view_{view}.png")), cv2.COLOR_BGR2RGB)
        depth = np.load(root / "stage_2_da" / f"view{view}" / "depth_original_m.npy")
        mask = cv2.imread(str(root / "stage_3_masks" / f"view{view}" / "mask_w9.png"), 0) > 0
        bundle = json.loads((root / "stage_4_bundles" / f"view{view}__rect__w9" / "bundle.json").read_text())
        overlay = image.copy(); overlay[mask] = (0.35 * overlay[mask] + 0.65 * np.asarray([0, 255, 0])).astype(np.uint8)
        axes[row, 0].imshow(image); axes[row, 0].set_title(f"Real ECM view {view}")
        axes[row, 1].imshow(overlay); axes[row, 1].set_title(f"Manual needle mask ({int(mask.sum())} px)")
        axes[row, 2].imshow(depth * 1000.0, cmap="turbo"); axes[row, 2].set_title("P5a DA depth (mm; no real GT)")
        ys, xs = np.where(mask); pad = 35
        x0, x1 = max(0, xs.min()-pad), min(mask.shape[1], xs.max()+pad)
        y0, y1 = max(0, ys.min()-pad), min(mask.shape[0], ys.max()+pad)
        local = (depth[y0:y1, x0:x1] - float(np.median(depth[mask]))) * 1000.0
        axes[row, 3].imshow(local, cmap="coolwarm", vmin=-2, vmax=2)
        axes[row, 3].contour(mask[y0:y1, x0:x1], levels=[0.5], colors=["lime"], linewidths=1)
        relief = bundle["predicted_needle_relief_mm_positive_means_closer"]
        axes[row, 3].set_title(f"Needle zoom, relative depth\nrelief={relief:+.3f} mm")
        for ax in axes[row]: ax.axis("off")
    fig.suptitle("D16 real-image DA audit: output exists, but needle relief is nearly flattened", fontsize=16)
    fig.savefig(out, dpi=160); plt.close(fig)


def make_fp_figure(root: Path, out: Path) -> None:
    selections = [
        ("stage_5_fp", "view1__raw__w9", "View1 raw-K, mask 9"),
        ("stage_5_fp", "view1__rect__w9", "View1 rect-K, mask 9"),
        ("stage_6_scale_fp", "view1__rect__w9__s2", "View1 rect-K, depth x2*"),
        ("stage_6_scale_fp", "view1__rect__w9__s2p5", "View1 rect-K, depth x2.5*"),
        ("stage_5_fp", "view2__rect__w5", "View2 rect-K, mask 5"),
        ("stage_5_fp", "view2__rect__w9", "View2 rect-K, mask 9"),
        ("stage_5_fp", "view2__rect__w15", "View2 rect-K, mask 15"),
        ("stage_6_scale_fp", "view2__rect__w9__s2p5", "View2 rect-K, depth x2.5*"),
    ]
    fig, axes = plt.subplots(2, 4, figsize=(18, 8), constrained_layout=True)
    for ax, (stage, run, title) in zip(axes.ravel(), selections):
        image = cv2.cvtColor(cv2.imread(str(root / stage / run / "top1_overlay.png")), cv2.COLOR_BGR2RGB)
        ax.imshow(image); ax.set_title(title); ax.axis("off")
    fig.suptitle("D16 FoundationPose top-1 sensitivity (green box/axes): returned pose is not a validated pose\n*scaled depth is diagnostic only, not calibration", fontsize=15)
    fig.savefig(out, dpi=160); plt.close(fig)


def make_dashboard(root: Path, rows: list[dict], out: Path) -> None:
    fig, ax = plt.subplots(figsize=(14, 7)); ax.axis("off")
    view1 = [r for r in rows if r["run"].startswith("view1")]
    view2 = [r for r in rows if r["run"].startswith("view2")]
    text = [
        "D16 REAL ECM FIRST-FRAME DECISION",
        "",
        "RGB input                 PASS   2/2 readable, 1300x1024, no severe clipping",
        "P5a DA execution          PASS   frozen SHA, ViT-L/518/FP32, CUDA",
        "DA metric accuracy        UNKNOWN no real depth ground truth",
        "Needle depth relief       WARN   view1 +0.15..0.18 mm; view2 -0.01..+0.04 mm",
        "FP API/register           PASS   12/12 calls, 252 candidates each",
        "FP pose usability         FAIL   overlays do not consistently align with the needle",
        "Camera contract           FAIL   screenshot topic/raw-vs-rectified provenance absent",
        "Mask robustness           FAIL   view2 width change causes 68-85 deg / 6.5-11.8 mm jumps",
        "K robustness              FAIL   view1 raw-vs-rect changes top-1 about 150 deg / 19.2 mm",
        "Real first-frame release  NO     human confirmation cannot rescue unknown metric/K/CAD contract",
        "",
        "Decision: these two views are useful capture candidates, but neither is validated as an automatic FP initialization view.",
        "Required next evidence: exact /image_rect + CameraInfo pair, real needle CAD/size confirmation, mask, and an independent pose/depth reference.",
    ]
    ax.text(0.02, 0.97, "\n".join(text), va="top", family="monospace", fontsize=14,
            bbox={"boxstyle": "round", "facecolor": "#f7f7f7", "edgecolor": "#444444"})
    fig.savefig(out, dpi=160, bbox_inches="tight"); plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--mesh", type=Path, required=True); args = parser.parse_args()
    vertices = load_obj_vertices(args.mesh)
    rows = []
    for stage, bundles in (("stage_5_fp", "stage_4_bundles"), ("stage_6_scale_fp", "stage_6_scale_bundles")):
        for fp_run in sorted((args.root / stage).iterdir()):
            if fp_run.is_dir() and (fp_run / "candidates.npz").is_file():
                rows.append(candidate_2d_rows(args.root / bundles / fp_run.name, fp_run, vertices))
    da1 = json.loads((args.root / "stage_2_da/view1/da_result.json").read_text())
    da2 = json.loads((args.root / "stage_2_da/view2/da_result.json").read_text())
    summary = {"schema": "SurgicAI.D16.real_image_analysis.v1", "complete": True,
               "images": 2, "fp_primary_runs": 12, "fp_scale_diagnostic_runs": 6,
               "da_photometric_p95_worst_mm": {
                   "view1": max(r["p95_vs_original_mm"] for r in da1["photometric_stability"]),
                   "view2": max(r["p95_vs_original_mm"] for r in da2["photometric_stability"])},
               "fp_2d_rows": rows,
               "decision": "REAL_FIRST_FRAME_NOT_VALIDATED",
               "reason": "DA has no real metric GT and predicts near-zero needle relief; FP top-1 is highly sensitive to K/mask and projected poses do not consistently fit the marked needle.",
               "boundary": "Two screenshots cannot measure DA metric accuracy or FP 6D accuracy. Stable overlays would still not exclude a stable 180-degree wrong branch."}
    atomic_json(args.root / "analysis.json", summary)
    make_da_figure(args.root, args.root / "figure_1_real_DA_and_needle_relief.png")
    make_fp_figure(args.root, args.root / "figure_2_FP_sensitivity_matrix.png")
    make_dashboard(args.root, rows, args.root / "figure_3_decision_dashboard.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
