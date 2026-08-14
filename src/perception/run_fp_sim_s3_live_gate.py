#!/usr/bin/env python3
"""SIM-S3: paired one-shot live FoundationPose register and D7 physical gate."""

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


REST_NORMAL = np.asarray(
    [0.09284837263684882, 0.11340011284196544, 0.9892014931782698],
    dtype=np.float64,
)
REST_NORMAL /= np.linalg.norm(REST_NORMAL)
SUPPORT_HEIGHT_M = 0.7429873869860798


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--foundationpose-root", type=Path, default=Path("/workspace"))
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--reset-bank", type=Path, required=True)
    p.add_argument("--prediction-dir", type=Path, required=True)
    p.add_argument("--perturbation-plan", type=Path, required=True)
    p.add_argument("--mesh", type=Path, required=True)
    p.add_argument("--out-dir", type=Path, required=True)
    p.add_argument("--iteration", type=int, default=5)
    p.add_argument("--seed", type=int, default=2718)
    p.add_argument("--normal-gate-deg", type=float, default=20.0)
    p.add_argument("--height-gate-mm", type=float, default=5.0)
    return p.parse_args()


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".partial")
    tmp.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    os.replace(tmp, path)


def rotation_errors_deg(poses: np.ndarray, truth: np.ndarray) -> np.ndarray:
    relative = np.einsum("nij,jk->nik", poses[:, :3, :3], truth[:3, :3].T)
    traces = np.trace(relative, axis1=1, axis2=2)
    return np.degrees(np.arccos(np.clip((traces - 1.0) / 2.0, -1.0, 1.0)))


def perturb_mask(mask: np.ndarray, spec: dict) -> np.ndarray:
    height, width = mask.shape
    moved = cv2.warpAffine(
        mask.astype(np.uint8),
        np.float32([[1, 0, int(spec["dx_px"])], [0, 1, int(spec["dy_px"])]]),
        (width, height), flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT, borderValue=0,
    )
    radius = int(spec["kernel_radius_px"])
    operation = spec["morphology"]
    if radius > 0 and operation != "none":
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1)
        )
        opcode = cv2.MORPH_ERODE if operation == "erosion" else cv2.MORPH_DILATE
        moved = cv2.morphologyEx(moved, opcode, kernel)
    return moved > 0


def q(values: list[float]) -> dict:
    if not values:
        return {"count": 0, "p50": None, "p95": None, "max": None}
    a = np.asarray(values, dtype=np.float64)
    return {"count": len(values), "p50": float(np.percentile(a, 50)),
            "p95": float(np.percentile(a, 95)), "max": float(a.max())}


def summarize(rows: list[dict]) -> dict:
    accepted = [r for r in rows if r["gate_accepted"]]
    trans = [r["gated_translation_error_mm"] for r in accepted]
    rot = [r["gated_rotation_error_deg"] for r in accepted]
    return {
        "frames": len(rows),
        "register_success": sum(r["register_success"] for r in rows),
        "register_dropout": sum(not r["register_success"] for r in rows),
        "raw_top1_flips": sum(bool(r["raw_top1_flip_gt_90_deg"]) for r in rows),
        "raw_top1_pass_5mm_15deg": sum(bool(r["raw_top1_pass_5mm_15deg"]) for r in rows),
        "accepted": len(accepted),
        "rejected": len(rows) - len(accepted),
        "accepted_flips": sum(bool(r["gated_flip_gt_90_deg"]) for r in accepted),
        "accepted_pass_5mm_15deg": sum(bool(r["gated_pass_5mm_15deg"]) for r in accepted),
        "correct_sample_false_rejects": sum(
            (not r["gate_accepted"]) and r["candidate_oracle_5mm_15deg"] for r in rows
        ),
        "translation_error_mm": q(trans),
        "rotation_error_deg": q(rot),
        "mask_pixels": q([float(r["mask_pixels"]) for r in rows]),
        "mask_iou_vs_control": q([float(r["mask_iou_vs_control"]) for r in rows]),
        "candidate_count": q([float(r["candidate_count"]) for r in rows]),
    }


def main() -> int:
    args = parse_args()
    if args.out_dir.exists() and any(args.out_dir.iterdir()):
        raise FileExistsError(f"Refusing overwrite: {args.out_dir}")
    args.out_dir.mkdir(parents=True)
    candidate_dir = args.out_dir / "candidates"
    mask_dir = args.out_dir / "masks"
    candidate_dir.mkdir(); mask_dir.mkdir()

    sys.path.insert(0, str(args.foundationpose_root))
    import nvdiffrast.torch as dr
    import torch
    import trimesh
    from estimater import PoseRefinePredictor, ScorePredictor
    from occlusion_eval import F_INV, make_estimator

    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)
    reset = json.loads(args.reset_bank.read_text(encoding="utf-8"))
    plan = json.loads(args.perturbation_plan.read_text(encoding="utf-8"))
    plan_by_id = {r["frame_id"]: r for r in plan["rows"]}
    entries = {r["frame_id"]: r for r in reset["entries"]}
    frame_ids = sorted(p.stem for p in (args.dataset / "poses").glob("*.npz"))
    if len(frame_ids) != 40 or set(frame_ids) != set(entries) or set(frame_ids) != set(plan_by_id):
        raise ValueError("Expected the same exact 40 frames in capture, reset bank, and frozen plan")

    mesh = trimesh.load(args.mesh, force="mesh")
    if hasattr(mesh.visual, "to_color"):
        mesh.visual = mesh.visual.to_color()
    estimator = make_estimator(mesh, ScorePredictor(), PoseRefinePredictor(),
                               dr.RasterizeCudaContext(), "/tmp/fp_sim_s3", 0)
    to_original = estimator.get_tf_to_centered_mesh().detach().cpu().numpy()
    rows: list[dict] = []
    jsonl = args.out_dir / "per_condition.jsonl"
    started_all = time.time()

    for frame_index, frame_id in enumerate(frame_ids):
        bundle = np.load(args.dataset / "poses" / f"{frame_id}.npz")
        rgb_bgr = cv2.imread(str(args.dataset / "L" / "rgb" / f"{frame_id}.png"))
        perfect_u8 = cv2.imread(
            str(args.dataset / "L" / "needle_mask" / f"{frame_id}.png"),
            cv2.IMREAD_GRAYSCALE,
        )
        if rgb_bgr is None or perfect_u8 is None:
            raise FileNotFoundError(frame_id)
        depth = np.load(args.prediction_dir / f"{frame_id}.npy").astype(np.float32)
        depth[(~np.isfinite(depth)) | (depth < 0.001)] = 0.0
        perfect = perfect_u8 > 0
        proxy = perturb_mask(perfect, plan_by_id[frame_id])
        cv2.imwrite(str(mask_dir / f"{frame_id}_CONTROL.png"), perfect.astype(np.uint8) * 255)
        cv2.imwrite(str(mask_dir / f"{frame_id}_DEPLOYMENT_PROXY.png"), proxy.astype(np.uint8) * 255)
        union = int(np.count_nonzero(perfect | proxy))
        iou = float(np.count_nonzero(perfect & proxy) / union) if union else 1.0

        for condition, mask in (("CONTROL", perfect), ("DEPLOYMENT_PROXY", proxy)):
            record = {
                "frame_index": frame_index, "frame_id": frame_id,
                "reset_seed": int(entries[frame_id]["reset_seed"]),
                "condition": condition, "depth_source": "P5a new-DA metric prediction",
                "mask_source": ("AMBF perfect semantic" if condition == "CONTROL" else
                                "fixed seeded artificial perturbation of AMBF semantic mask"),
                "mask_pixels": int(mask.sum()), "control_mask_pixels": int(perfect.sum()),
                "mask_iou_vs_control": 1.0 if condition == "CONTROL" else iou,
                "perturbation": None if condition == "CONTROL" else plan_by_id[frame_id],
                "register_call_count_for_reset_condition": 1,
                "gt_used_for_selection": False,
            }
            try:
                tick = time.monotonic()
                estimator.register(K=bundle["K"], rgb=cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB),
                                   depth=depth, ob_mask=mask, iteration=args.iteration)
                centered = estimator.poses.detach().cpu().numpy()
                scores = estimator.scores.detach().cpu().numpy().reshape(-1)
                poses_camera = centered @ to_original
                poses_world = np.stack([bundle["LT_Wcamera"] @ F_INV @ p for p in poses_camera])
                truth = bundle["T_Wneedle"]
                trans = np.linalg.norm(poses_world[:, :3, 3] - truth[:3, 3], axis=1) * 1000.0
                rot = rotation_errors_deg(poses_world, truth)
                normals = poses_world[:, :3, 2]
                normal_angle = np.degrees(np.arccos(np.clip(normals @ REST_NORMAL, -1.0, 1.0)))
                height_error = np.abs(poses_world[:, 2, 3] - SUPPORT_HEIGHT_M) * 1000.0
                valid = (normal_angle <= args.normal_gate_deg) & (height_error <= args.height_gate_mm)
                raw_index = int(np.argmax(scores))
                selected = None
                if np.any(valid):
                    valid_indices = np.flatnonzero(valid)
                    selected = int(valid_indices[np.argmax(scores[valid])])
                ranks = np.empty(len(scores), dtype=np.int32)
                ranks[np.argsort(-scores)] = np.arange(1, len(scores) + 1)
                np.savez_compressed(
                    candidate_dir / f"{frame_id}_{condition}.npz",
                    poses_camera=poses_camera, poses_world=poses_world, scores=scores,
                    score_ranks=ranks, support_valid=valid, normal_deviation_deg=normal_angle,
                    height_error_mm=height_error, translation_error_mm=trans,
                    rotation_error_deg=rot, truth_world=truth,
                )
                record.update({
                    "register_success": True, "register_wall_s": float(time.monotonic() - tick),
                    "candidate_count": int(len(scores)), "valid_candidate_count": int(valid.sum()),
                    "candidate_oracle_5mm_15deg": bool(np.any((trans <= 5) & (rot <= 15))),
                    "raw_top1_index": raw_index, "raw_top1_score": float(scores[raw_index]),
                    "raw_top1_translation_error_mm": float(trans[raw_index]),
                    "raw_top1_rotation_error_deg": float(rot[raw_index]),
                    "raw_top1_flip_gt_90_deg": bool(rot[raw_index] > 90),
                    "raw_top1_pass_5mm_15deg": bool(trans[raw_index] <= 5 and rot[raw_index] <= 15),
                    "gate_accepted": selected is not None, "gated_candidate_index": selected,
                    "gated_score": None if selected is None else float(scores[selected]),
                    "gated_translation_error_mm": None if selected is None else float(trans[selected]),
                    "gated_rotation_error_deg": None if selected is None else float(rot[selected]),
                    "gated_flip_gt_90_deg": None if selected is None else bool(rot[selected] > 90),
                    "gated_pass_5mm_15deg": None if selected is None else bool(trans[selected] <= 5 and rot[selected] <= 15),
                })
            except Exception as exc:
                record.update({"register_success": False, "register_error": repr(exc),
                               "candidate_count": 0, "valid_candidate_count": 0,
                               "candidate_oracle_5mm_15deg": False,
                               "raw_top1_flip_gt_90_deg": False, "raw_top1_pass_5mm_15deg": False,
                               "gate_accepted": False, "gated_flip_gt_90_deg": None,
                               "gated_pass_5mm_15deg": None})
            rows.append(record)
            with jsonl.open("a", encoding="utf-8") as f:
                f.write(json.dumps(record) + "\n")
        print(f"SIM_S3_FP {frame_index + 1}/40 {frame_id}", flush=True)

    groups = {c: summarize([r for r in rows if r["condition"] == c])
              for c in ("CONTROL", "DEPLOYMENT_PROXY")}
    primary = groups["DEPLOYMENT_PROXY"]
    bank_hard = int(reset.get("hard_failure_count", 0))
    gates = {
        "accepted_flip_equals_0": primary["accepted_flips"] == 0,
        "correct_false_reject_lte_4": primary["correct_sample_false_rejects"] <= 4,
        "translation_p95_lte_5mm": primary["translation_error_mm"]["p95"] is not None and primary["translation_error_mm"]["p95"] <= 5,
        "rotation_p95_lte_15deg": primary["rotation_error_deg"]["p95"] is not None and primary["rotation_error_deg"]["p95"] <= 15,
        "reset_hard_failure_equals_0": bank_hard == 0,
    }
    result = {
        "schema": "SurgicAI.SIM-S3.live_needle_initial_gate.v1", "complete": True,
        "decision": "LIVE_INITIAL_GATE_SUPPORTED" if all(gates.values()) else "HUMAN_CONFIRMATION_REQUIRED",
        "frames": 40, "paired_register_calls": 80,
        "register_contract": "one call for each reset x mask condition; no repeat, vote, or temporal selection",
        "depth_source": "P5a/new-DA metric prediction",
        "deployment_proxy_warning": "artificial first-frame mask-error proxy; not a real automatic segmenter",
        "support_gate": {"rest_normal_world": REST_NORMAL.tolist(),
                         "support_height_world_m": SUPPORT_HEIGHT_M,
                         "normal_deg_lte": args.normal_gate_deg,
                         "height_mm_lte": args.height_gate_mm,
                         "fallback": "reject; never raw top-1"},
        "reset_hard_failure_count": bank_hard, "groups": groups,
        "primary_gates": gates, "runtime_seconds": time.time() - started_all,
        "rows": rows,
    }
    atomic_json(args.out_dir / "result.json", result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
