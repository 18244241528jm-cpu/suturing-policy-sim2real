"""Distance, speed and timeout are coupled; the executor must say so at startup.

At max_linear_speed_m_s the furthest a single execute_once can travel is
speed * execution_timeout_s -- 40 mm with the shipped 0.002 m/s and 20 s.  Raising
max_initial_translation_m alone does not extend the reach; it turns an up-front
D10-E56 rejection into a D10-E59 timeout that strands the arm partway.

Skipped where rclpy is unavailable; passes under ROS 2 Humble.
"""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

rclpy = None
try:
    import rclpy as _rclpy
    from rclpy.parameter import Parameter

    rclpy = _rclpy
except ImportError:  # pragma: no cover - environment dependent
    pass

ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "suturing_runtime"


def load_executor_module():
    if str(PACKAGE_ROOT) not in sys.path:
        sys.path.insert(0, str(PACKAGE_ROOT))
    spec = importlib.util.find_spec("suturing_runtime.guarded_pose_executor")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@unittest.skipIf(rclpy is None, "rclpy unavailable; run under a sourced ROS 2 install")
class MotionBudgetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.module = load_executor_module()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def test_shipped_defaults_are_self_consistent(self):
        node = self.module.GuardedPoseExecutor()
        self.addCleanup(node.destroy_node)
        # 0.002 m/s * 20 s = 40 mm reachable, against a 10 mm cap.
        self.assertLessEqual(
            float(node.get_parameter("max_initial_translation_m").value),
            float(node.get_parameter("max_linear_speed_m_s").value)
            * float(node.get_parameter("execution_timeout_s").value),
        )

    def test_raising_the_distance_cap_alone_is_refused(self):
        node = self.module.GuardedPoseExecutor()
        self.addCleanup(node.destroy_node)
        node.set_parameters([Parameter("max_initial_translation_m", value=0.061)])
        with self.assertRaises(ValueError) as caught:
            node._validate_motion_budget()
        message = str(caught.exception)
        self.assertIn("D10-E13-MOTION-BUDGET", message)
        # The message must name the timeout the operator has to set, 0.061/0.002 = 30.5 s.
        self.assertIn("30.5", message)

    def test_raising_distance_and_timeout_together_is_accepted(self):
        node = self.module.GuardedPoseExecutor()
        self.addCleanup(node.destroy_node)
        node.set_parameters([
            Parameter("max_initial_translation_m", value=0.061),
            Parameter("execution_timeout_s", value=45.0),
        ])
        node._validate_motion_budget()  # must not raise

    def test_rotation_budget_is_checked_too(self):
        node = self.module.GuardedPoseExecutor()
        self.addCleanup(node.destroy_node)
        node.set_parameters([Parameter("max_initial_rotation_deg", value=400.0)])
        with self.assertRaises(ValueError) as caught:
            node._validate_motion_budget()
        self.assertIn("D10-E13-MOTION-BUDGET", str(caught.exception))

    def test_nonpositive_speed_is_refused(self):
        node = self.module.GuardedPoseExecutor()
        self.addCleanup(node.destroy_node)
        node.set_parameters([Parameter("max_linear_speed_m_s", value=0.0)])
        with self.assertRaises(ValueError):
            node._validate_motion_budget()


if __name__ == "__main__":
    unittest.main()
