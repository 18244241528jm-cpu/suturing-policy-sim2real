"""Tests for the segmented SE(3) goal servo that feeds guarded_pose_executor."""

from __future__ import annotations

import hashlib
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "suturing_runtime"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from suturing_runtime.subgoal_planner import (  # noqa: E402
    DEFAULT_STEP_SIZE,
    clamp_to_budget,
    geodesic_angle_deg,
    plan_subgoal,
)

# The executor rejects anything past these, so every waypoint must stay inside them.
EXECUTOR_MAX_TRANSLATION_M = 0.010
EXECUTOR_MAX_ROTATION_DEG = 30.0


class VendoredControllerTest(unittest.TestCase):
    def test_vendored_servo_matches_the_experiment_source(self):
        """The package copy must not drift from src/control/controllers.py.

        The node imports its own copy so a ROS install does not depend on a path
        outside the package, which means the two files can silently diverge and the
        real cart would then run a controller nobody swept.
        """
        original = (ROOT / "src" / "control" / "controllers.py").read_bytes()
        vendored = (PACKAGE_ROOT / "suturing_runtime" / "se3_goal_servo.py").read_bytes()
        self.assertEqual(
            hashlib.sha256(original).hexdigest(),
            hashlib.sha256(vendored).hexdigest(),
            "se3_goal_servo.py drifted from src/control/controllers.py; re-copy it",
        )


class PlanSubgoalTest(unittest.TestCase):
    def setUp(self):
        # Centimetres and radians, matching the controller contract.
        self.start = np.array([-3.2556, 1.1749, -11.8730, -3.7754, 0.4933, 1.3117, 0.0])

    def test_far_goal_is_split_into_segments_inside_the_executor_limit(self):
        # 61 mm is the measured simulated Approach stroke, the case that earns
        # D10-E56-DISTANCE when handed to the executor whole.
        final = self.start.copy()
        final[2] += 6.1  # +61 mm in z
        subgoal, info = plan_subgoal(self.start, final)
        self.assertFalse(info["reached_final"])
        self.assertLessEqual(info["segment_translation_m"], EXECUTOR_MAX_TRANSLATION_M)
        self.assertGreater(info["segment_translation_m"], 0.0)

    def test_repeated_segments_converge_on_a_61mm_goal(self):
        final = self.start.copy()
        final[2] += 6.1
        current = self.start.copy()
        for _ in range(60):
            subgoal, info = plan_subgoal(current, final)
            self.assertLessEqual(info["segment_translation_m"], EXECUTOR_MAX_TRANSLATION_M)
            current = subgoal
            if info["reached_final"]:
                break
        remaining_m = float(np.linalg.norm((final[:3] - current[:3]) / 100.0))
        self.assertLessEqual(remaining_m, 0.001, f"did not converge, {remaining_m*1000:.2f} mm left")

    def test_rotation_budget_is_respected(self):
        final = self.start.copy()
        final[5] += np.deg2rad(80.0)
        subgoal, info = plan_subgoal(self.start, final)
        self.assertLessEqual(info["segment_rotation_deg"], EXECUTOR_MAX_ROTATION_DEG)
        self.assertLessEqual(
            geodesic_angle_deg(self.start[3:6], subgoal[3:6]), EXECUTOR_MAX_ROTATION_DEG
        )

    def test_goal_already_reached_reports_reached_final(self):
        subgoal, info = plan_subgoal(self.start, self.start.copy())
        self.assertTrue(info["reached_final"])
        np.testing.assert_allclose(subgoal[:3], self.start[:3], atol=1e-9)

    def test_jaw_is_held_because_the_executor_cannot_move_it(self):
        start = self.start.copy()
        start[6] = 0.4
        final = start.copy()
        final[2] += 2.0
        final[6] = 1.0  # a jaw request the pose-only executor cannot honour
        subgoal, _ = plan_subgoal(start, final)
        self.assertAlmostEqual(subgoal[6], 0.4, places=9)

    def test_metre_input_would_understate_the_error(self):
        """Guards the unit contract: the planner works in centimetres, not metres.

        se3_goal_servo divides the translation error by 100 internally.  Passing
        metres makes a 61 mm move look like 0.61 mm, so the arm creeps and every
        segment lands far short; this asserts the two are not interchangeable.
        """
        final_cm = self.start.copy()
        final_cm[2] += 6.1
        _, info_cm = plan_subgoal(self.start, final_cm)
        start_m = self.start / np.array([100.0, 100.0, 100.0, 1.0, 1.0, 1.0, 1.0])
        final_m = final_cm / np.array([100.0, 100.0, 100.0, 1.0, 1.0, 1.0, 1.0])
        _, info_m = plan_subgoal(start_m, final_m)
        self.assertGreater(info_cm["segment_translation_m"], 10 * info_m["segment_translation_m"])


class ClampToBudgetTest(unittest.TestCase):
    def test_over_budget_translation_is_shrunk(self):
        current = np.zeros(7)
        subgoal = np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 50 mm away
        clamped = clamp_to_budget(current, subgoal, 0.008, 25.0)
        self.assertAlmostEqual(float(np.linalg.norm(clamped[:3] / 100.0)), 0.008, places=9)

    def test_over_budget_rotation_is_shrunk(self):
        current = np.zeros(7)
        subgoal = np.zeros(7)
        subgoal[5] = np.deg2rad(90.0)
        clamped = clamp_to_budget(current, subgoal, 0.008, 25.0)
        self.assertAlmostEqual(geodesic_angle_deg(current[3:6], clamped[3:6]), 25.0, places=6)

    def test_in_budget_waypoint_is_untouched(self):
        current = np.zeros(7)
        subgoal = np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 3 mm
        clamped = clamp_to_budget(current, subgoal, 0.008, 25.0)
        np.testing.assert_allclose(clamped, subgoal, atol=1e-12)


class StepSizeContractTest(unittest.TestCase):
    def test_default_step_size_matches_the_simulated_environment(self):
        self.assertEqual(len(DEFAULT_STEP_SIZE), 7)
        np.testing.assert_allclose(DEFAULT_STEP_SIZE[:3], [0.001, 0.001, 0.001])
        np.testing.assert_allclose(DEFAULT_STEP_SIZE[3:6], [0.05235988] * 3, atol=1e-8)


if __name__ == "__main__":
    unittest.main()
