#!/usr/bin/env python3
"""Create deterministic P9a reset snapshots and request same-scene camera frames."""

from __future__ import annotations

import argparse
import json
import os
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from p9a_goal_geometry import goal_pose_error, needle_pose_to_goal


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, required=True)
    parser.add_argument("--eval-seed", type=int, required=True)
    parser.add_argument("--yaw-deg", type=float, required=True)
    parser.add_argument("--x-mm", type=float, default=3.0)
    parser.add_argument("--y-mm", type=float, default=3.0)
    parser.add_argument("--ros-domain-id", type=int, required=True)
    parser.add_argument("--request-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--capture-timeout-s", type=float, default=30.0)
    return parser.parse_args()


def kdl_frame_to_matrix(frame) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    for row in range(3):
        for column in range(3):
            matrix[row, column] = frame.M[row, column]
        matrix[row, 3] = frame.p[row]
    return matrix


def json_ready(value):
    if isinstance(value, dict):
        return {str(key): json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_ready(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.astype(float).tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".partial")
    temporary.write_text(
        json.dumps(json_ready(payload), indent=2), encoding="utf-8"
    )
    os.replace(temporary, path)


def wait_done(path: Path, timeout_s: float, hold_callback=None) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.is_file():
            return json.loads(path.read_text(encoding="utf-8"))
        if hold_callback is not None:
            hold_callback()
        time.sleep(0.02)
    raise TimeoutError(f"Camera daemon did not acknowledge {path}")


def main() -> int:
    args = parse_args()
    if args.episodes < 2:
        raise ValueError("--episodes must be >= 2")
    if os.environ.get("ROS_DOMAIN_ID") != str(args.ros_domain_id):
        raise RuntimeError(
            f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')!r}; "
            f"expected {args.ros_domain_id}"
        )
    if args.output.exists():
        raise FileExistsError(f"Refusing to overwrite {args.output}")
    args.request_dir.mkdir(parents=True, exist_ok=True)

    from RL.Approach_env import SRC_approach
    from RL.utils.seed import seed_everything

    seed_everything(args.eval_seed, deterministic_torch=True)
    env = SRC_approach(
        seed=args.eval_seed,
        reward_type="sparse",
        threshold=np.array([1.0, np.deg2rad(10.0)], dtype=np.float32),
        max_episode_step=200,
        measured_success_reward=True,
        command_state_clamp=True,
        needle_settle_steps=60,
        needle_settle_interval_s=0.1,
        randomize_psm_reset=True,
        randomize_needle_reset=True,
        freeze_live_goal=True,
        needle_random_range=np.array(
            [
                args.x_mm * 1.0e-3,
                args.y_mm * 1.0e-3,
                np.deg2rad(args.yaw_deg),
            ],
            dtype=np.float32,
        ),
    )
    seed_everything(args.eval_seed, deterministic_torch=True, env=env)

    entries: list[dict] = []
    payload = {
        "schema": "SurgicAI.P9a.reset_bank.v1",
        "created": datetime.now().isoformat(),
        "eval_seed": args.eval_seed,
        "ros_domain_id": args.ros_domain_id,
        "command_range": {
            "x_mm": args.x_mm,
            "y_mm": args.y_mm,
            "yaw_deg": args.yaw_deg,
        },
        "episodes_expected": args.episodes,
        "complete": False,
        "entries": entries,
    }
    try:
        for index in range(args.episodes):
            reset_seed = args.eval_seed * 100000 + index
            observation, _ = env.reset(seed=reset_seed)
            needle_world = kdl_frame_to_matrix(env.scene_manager.needle.get_pose())
            world_to_base = kdl_frame_to_matrix(
                env.scene_manager.psm_list[env.psm_idx - 1].get_T_w_b()
            )
            base_to_world = kdl_frame_to_matrix(
                env.scene_manager.psm_list[env.psm_idx - 1].get_T_b_w()
            )
            stored_goal = np.asarray(
                env.fixed_episode_goal, dtype=np.float64
            )
            recomputed_goal = needle_pose_to_goal(
                needle_world,
                world_to_base,
                env.grasp_angle,
                env.lift_height,
            )
            geometry_trans_mm, geometry_rot_deg = goal_pose_error(
                stored_goal, recomputed_goal
            )
            if geometry_trans_mm > 1.0e-2 or geometry_rot_deg > 5.0e-2:
                raise RuntimeError(
                    "P9a pure geometry does not match environment goal: "
                    f"{geometry_trans_mm:.6f} mm/{geometry_rot_deg:.6f} deg"
                )
            frame_id = f"frame_{index:06d}"
            request = {
                "frame_id": frame_id,
                "index": index,
                "reset_seed": reset_seed,
                "expected_T_Wneedle": needle_world.tolist(),
            }
            request_path = args.request_dir / f"{frame_id}.request.json"
            atomic_json(request_path, request)
            capture = wait_done(
                args.request_dir / f"{frame_id}.done.json",
                args.capture_timeout_s,
                hold_callback=lambda frame=env.scene_manager.needle.get_pose(): (
                    env.scene_manager.needle.hold_pose(frame)
                ),
            )
            entry = {
                "index": index,
                "frame_id": frame_id,
                "reset_seed": reset_seed,
                "grasp_angle_deg": float(env.grasp_angle),
                "lift_height_m": float(env.lift_height),
                "expected_T_Wneedle": needle_world.tolist(),
                "T_w_b": world_to_base.tolist(),
                "T_b_w": base_to_world.tolist(),
                "gt_goal_raw": stored_goal.tolist(),
                "geometry_crosscheck_translation_mm": geometry_trans_mm,
                "geometry_crosscheck_rotation_deg": geometry_rot_deg,
                "reset_attempts_used": int(env.reset_attempts_used),
                "reset_settle_audit": json_ready(env.reset_settle_audit),
                "reset_goal_audit": json_ready(env.reset_goal_audit),
                "camera_capture": capture,
                "reset_observation_desired_goal": np.asarray(
                    observation["desired_goal"], dtype=float
                ).tolist(),
            }
            entries.append(entry)
            atomic_json(args.output, payload)
            print(
                f"P9A_RESET_BANK {frame_id} seed={reset_seed} "
                f"yaw_range={args.yaw_deg:g} "
                f"drift_cm={env.reset_settle_audit['translation_drift_cm']:.4f}",
                flush=True,
            )
    finally:
        try:
            env.close()
        except Exception:
            pass
        for psm in getattr(env.scene_manager, "psm_list", []):
            with psm._cmd_lock:
                psm._cmd = None
            with psm._actuator_cmd_lock:
                psm._actuator_cmd = None
        ral_instance = getattr(env, "ral_instance", None)
        if ral_instance is not None and hasattr(ral_instance, "shutdown"):
            ral_instance.shutdown()

    payload["complete"] = True
    payload["episodes_completed"] = len(entries)
    atomic_json(args.output, payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
