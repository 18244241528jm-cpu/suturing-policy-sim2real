#!/usr/bin/env python3
"""Build paired SIM-S4 GT and deployment-proxy frozen-goal banks."""

from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "depth_audit_stage_a"))
from p9a_goal_geometry import needle_pose_to_goal  # noqa: E402


def args():
    p = argparse.ArgumentParser()
    p.add_argument("--sim-s3-root", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--episodes", type=int, default=30)
    p.add_argument("--bias-seed", type=int, default=20260811)
    return p.parse_args()


def pose_error(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    t = float(np.linalg.norm(a[:3, 3] - b[:3, 3]) * 1000)
    d = Rotation.from_matrix(a[:3, :3]).inv() * Rotation.from_matrix(b[:3, :3])
    return t, float(np.degrees(np.linalg.norm(d.as_rotvec())))


def goal_error(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    t = float(np.linalg.norm(a[:3] - b[:3]) * 1000)
    d = Rotation.from_euler("xyz", a[3:6]).inv() * Rotation.from_euler("xyz", b[3:6])
    return t, float(np.degrees(np.linalg.norm(d.as_rotvec())))


def perturb(goal: np.ndarray, episode: int, seed: int) -> tuple[np.ndarray, dict]:
    rng = np.random.default_rng(seed + episode)
    ta = rng.normal(size=3); ta /= np.linalg.norm(ta)
    ra = rng.normal(size=3); ra /= np.linalg.norm(ra)
    out = goal.copy(); out[:3] += ta * 0.005
    current = Rotation.from_euler("xyz", out[3:6])
    out[3:6] = (Rotation.from_rotvec(ra * np.deg2rad(5.0)) * current).as_euler("xyz")
    return out, {"translation_mm": 5.0, "rotation_deg": 5.0,
                 "translation_axis_psm_base": ta.tolist(), "rotation_axis_psm_base": ra.tolist(),
                 "episode_seed": seed + episode}


def main() -> int:
    a = args()
    if a.out.exists(): raise FileExistsError(f"Refusing overwrite: {a.out}")
    a.out.mkdir(parents=True)
    reset = json.loads((a.sim_s3_root / "reset_bank.json").read_text())
    fp = json.loads((a.sim_s3_root / "fp_gate" / "result.json").read_text())
    da = json.loads((a.sim_s3_root / "da_result.json").read_text())
    da_by_id = {r["frame_id"]: r for r in da["rows"]}
    proxy = {r["frame_id"]: r for r in fp["rows"] if r["condition"] == "DEPLOYMENT_PROXY"}
    source = reset["entries"][:a.episodes]
    if len(source) != a.episodes: raise ValueError("Insufficient reset entries")
    entries_a, entries_b, audit = [], [], []
    for episode, base in enumerate(source):
        frame_id = base["frame_id"]; row = proxy[frame_id]
        if not row["gate_accepted"] or row["gated_flip_gt_90_deg"]:
            raise RuntimeError(f"SIM-S3 dependency not accepted/nonflip: {frame_id}")
        z = np.load(a.sim_s3_root / "fp_gate" / "candidates" / f"{frame_id}_DEPLOYMENT_PROXY.npz")
        selected = int(row["gated_candidate_index"])
        pose = np.asarray(z["poses_world"][selected], dtype=np.float64)
        truth_pose = np.asarray(z["truth_world"], dtype=np.float64)
        unbiased = needle_pose_to_goal(
            pose, np.asarray(base["T_w_b"], dtype=np.float64),
            float(base["grasp_angle_deg"]), float(base["lift_height_m"]),
        )
        biased, bias_meta = perturb(np.asarray(unbiased, dtype=np.float64), episode, a.bias_seed)
        gt = np.asarray(base["gt_goal_raw"], dtype=np.float64)
        p_t, p_r = pose_error(pose, truth_pose)
        u_t, u_r = goal_error(unbiased, gt); b_t, b_r = goal_error(biased, gt)
        common = copy.deepcopy(base)
        common["d7_true_goal_raw"] = copy.deepcopy(base["gt_goal_raw"])
        common["sim_s4_perception"] = {
            "frame_id": frame_id, "camera": "SIM-S1 locked MID operational fallback",
            "depth_source": "P5a new-DA", "da_runtime_s": float(da_by_id[frame_id]["runtime_s"]),
            "mask_source": "manual-mask-equivalent deployment proxy",
            "mask_pixels": int(row["mask_pixels"]), "mask_iou_vs_semantic": float(row["mask_iou_vs_control"]),
            "candidate_npz": f"fp_gate/candidates/{frame_id}_DEPLOYMENT_PROXY.npz",
            "candidate_count": int(row["candidate_count"]), "selected_candidate_index": selected,
            "raw_top1_score": float(row["raw_top1_score"]), "gate_accepted": True,
            "selected_pose_translation_error_mm_posthoc": p_t,
            "selected_pose_rotation_error_deg_posthoc": p_r,
            "selected_pose_flip_posthoc": bool(p_r > 90),
        }
        ea = copy.deepcopy(common); ea["fp_goal_raw"] = copy.deepcopy(ea["gt_goal_raw"])
        ea["sim_s4_group"] = "A"; ea["psm_transform_bias"] = {"translation_mm": 0.0, "rotation_deg": 0.0}
        eb = copy.deepcopy(common); eb["fp_goal_raw"] = biased.tolist(); eb["unbiased_fp_goal_raw"] = unbiased.tolist()
        eb["fp_pose_world"] = pose.tolist(); eb["fp_metrics"] = common["sim_s4_perception"]
        eb["sim_s4_group"] = "B"; eb["psm_transform_bias"] = bias_meta
        entries_a.append(ea); entries_b.append(eb)
        audit.append({"episode": episode, "frame_id": frame_id, "reset_seed": int(base["reset_seed"]),
                      "selected_pose_translation_error_mm": p_t, "selected_pose_rotation_error_deg": p_r,
                      "unbiased_goal_vs_gt_translation_mm": u_t, "unbiased_goal_vs_gt_rotation_deg": u_r,
                      "biased_goal_vs_gt_translation_mm": b_t, "biased_goal_vs_gt_rotation_deg": b_r,
                      "bias": bias_meta, "da_runtime_s": float(da_by_id[frame_id]["runtime_s"]),
                      "mask_pixels": int(row["mask_pixels"]), "mask_iou": float(row["mask_iou_vs_control"])})
    def bank(label, entries, meaning):
        return {"schema": "SurgicAI.SIM-S4.external_goal_bank.v1", "complete": True,
                "label": label, "eval_seed": reset["eval_seed"], "command_range": reset["command_range"],
                "entries": entries, "sim_s4_metadata": {"episodes": a.episodes, "meaning": meaning}}
    (a.out / "A_gt_frozen.json").write_text(json.dumps(bank("SIM-S4_A", entries_a, "GT frozen/no PSM bias"), indent=2)+"\n")
    (a.out / "B_deployment_proxy_frozen_bias5.json").write_text(json.dumps(bank("SIM-S4_B", entries_b, "manual-mask-equivalent gated FP frozen + PSM transform bias 5mm/5deg"), indent=2)+"\n")
    summary = {"schema": "SurgicAI.SIM-S4.bank_audit.v1", "complete": True, "episodes": a.episodes,
               "accepted_gate_count": a.episodes, "accepted_flip_count": 0,
               "bias_seed": a.bias_seed, "rows": audit}
    (a.out / "bank_audit.json").write_text(json.dumps(summary, indent=2)+"\n")
    print(json.dumps({"complete": True, "episodes": a.episodes, "out": str(a.out)}))
    return 0


if __name__ == "__main__": raise SystemExit(main())
