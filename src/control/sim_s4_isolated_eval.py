#!/usr/bin/env python3
"""Run D2 with an in-memory, read-only finger-contact instrumentation patch."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from controllers import staged_se3_goal_servo_action


def main() -> None:
    if "--" not in sys.argv: raise SystemExit("Expected -- before evaluator arguments")
    cut = sys.argv.index("--")
    parser = argparse.ArgumentParser(); parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--artifacts-dir", type=Path, required=True)
    cfg = parser.parse_args(sys.argv[1:cut]); evaluator_argv = sys.argv[cut+1:]
    repo = cfg.repo.resolve(); sys.path[:0] = [str(repo / "RL"), str(repo)]
    import RL.Model_evaluation as evaluation
    artifacts = cfg.artifacts_dir.resolve(); artifacts.mkdir(parents=True, exist_ok=True)

    def save_results(eval_args, results, train_seeds, test_env):
        target = artifacts / str(test_env); target.mkdir(parents=True, exist_ok=True)
        (target / "results.txt").write_text(
            f"task={eval_args.task_name}\nsuccess_rate={results['mean_success_rate']}\n", encoding="utf-8")
    evaluation.save_results = save_results
    evaluation.goal_servo_action = staged_se3_goal_servo_action

    target_class = evaluation.resolve_src_env("Approach")
    original_step = target_class.step
    def instrumented_step(self, action):
        output = original_step(self, action)
        try:
            psm = self.scene_manager.psm_list[self.psm_idx - 1]
            self.last_grasp_status = psm.grasp_status()
        except Exception as exc:
            self.last_grasp_status = {"instrumentation_error": repr(exc)}
        return output
    target_class.step = instrumented_step
    print("SIM_S4_RUNTIME_PATCH staged_D2 + read_only_finger_contact_audit", flush=True)
    sys.argv = [str(repo / "RL" / "Model_evaluation.py"), *evaluator_argv]
    evaluation.main()


if __name__ == "__main__": main()
