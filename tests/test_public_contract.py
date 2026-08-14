from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def load_controllers():
    path = ROOT / "src/control/controllers.py"
    spec = importlib.util.spec_from_file_location("public_controllers", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class PublicContractTest(unittest.TestCase):
    def test_staged_controller_is_bounded(self):
        controllers = load_controllers()
        obs = {
            "achieved_goal": np.zeros(7, dtype=np.float64),
            "desired_goal": np.array([1.0, -1.0, 0.5, 0.2, -0.1, 0.3, 0.8]),
        }
        step = np.array([0.0015, 0.0015, 0.0015, 0.05, 0.05, 0.05, 0.1])
        action = controllers.staged_se3_goal_servo_action(obs, step)
        self.assertEqual(action.shape, (7,))
        self.assertTrue(np.all(np.isfinite(action)))
        self.assertTrue(np.all(np.abs(action) <= 1.0))

    def test_required_public_files_exist(self):
        for relative in (
            "docs/ARCHITECTURE.md",
            "docs/SETUP.md",
            "docs/zh/从零复现仿真.md",
            "docs/zh/真机与仿真的区别.md",
            "scripts/doctor.py",
            "scripts/run_simulation.py",
            "src/perception/run_fp_sim_s3_live_gate.py",
            "src/perception/capture_p9a_reset_bank.py",
            "src/perception/infer_p5c_gate_da.py",
            "models/rl/m3_measured_r3_100k.zip",
            "data/reference/sim_s3_20260811/reset_bank.json",
            "src/handeye/config/example_session.yaml",
            "ros2_ws/src/suturing_runtime/package.xml",
            "ros2_ws/src/suturing_runtime/config/jhu_real.yaml",
            "ros2_ws/src/suturing_runtime/launch/real_read_only.launch.py",
            "ros2_ws/src/suturing_runtime/launch/real_guarded.launch.py",
            "ros2_ws/src/suturing_runtime/suturing_runtime/dvrk_topic_adapter.py",
            "ros2_ws/src/suturing_runtime/suturing_runtime/approach_goal_builder.py",
            "ros2_ws/src/suturing_runtime/suturing_runtime/guarded_pose_executor.py",
        ):
            self.assertTrue((ROOT / relative).is_file(), relative)


if __name__ == "__main__":
    unittest.main()
