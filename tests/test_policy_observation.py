"""The policy observation must match what the simulator fed the checkpoint.

The contract was read out of the simulator rather than guessed, and each piece of it is
a way to be silently wrong: a policy handed metres where it expects centimetres, or a
wrapped angle where training used a raw one, still returns a confident action.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "suturing_runtime"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


def load_builder():
    """Import the observation helpers without importing rclpy."""
    import importlib.util

    path = PACKAGE_ROOT / "suturing_runtime" / "policy_servo_node.py"
    source = path.read_text(encoding="utf-8")
    # The module imports rclpy at top level; exec only the pieces under test.
    namespace = {"__name__": "policy_obs_under_test"}
    exec(compile("import numpy as np\n", "<prelude>", "exec"), namespace)
    start = source.index("XYZ_INDICES_21")
    end = source.index("class PolicyServoNode")
    body = source[start:end]
    body = body.replace("from .contract import normalize_quaternion_xyzw", "")
    exec(compile(body, str(path), "exec"), namespace)
    return namespace


NS = load_builder()
build_observation = NS["build_observation"]
XYZ_INDICES_21 = NS["XYZ_INDICES_21"]


class ObservationShapeTest(unittest.TestCase):
    def setUp(self):
        self.current = np.array([-0.0873, 0.0648, 0.0441, -0.10, 0.20, -0.30, 0.0])
        self.goal = np.array([-0.0896, -0.0016, 0.2252, -0.12, 0.25, -0.28, 0.0])
        self.obs = build_observation(self.current, self.goal)

    def test_keys_and_shapes(self):
        self.assertEqual(set(self.obs), {"observation", "achieved_goal", "desired_goal"})
        self.assertEqual(self.obs["observation"].shape, (21,))
        self.assertEqual(self.obs["achieved_goal"].shape, (7,))
        self.assertEqual(self.obs["desired_goal"].shape, (7,))

    def test_layout_is_current_goal_delta(self):
        o = self.obs["observation"]
        # xyz blocks are scaled, so compare the untouched angle entries.
        np.testing.assert_allclose(o[3:6], self.current[3:6], rtol=1e-6)
        np.testing.assert_allclose(o[10:13], self.goal[3:6], rtol=1e-6)
        np.testing.assert_allclose(o[17:20], (self.goal - self.current)[3:6], rtol=1e-6)

    def test_xyz_entries_are_centimetres(self):
        o = self.obs["observation"]
        for index, metres in zip(XYZ_INDICES_21,
                                 list(self.current[:3]) + list(self.goal[:3])
                                 + list((self.goal - self.current)[:3])):
            self.assertAlmostEqual(float(o[index]), metres * 100.0, places=4)

    def test_angles_are_not_scaled(self):
        o = self.obs["observation"]
        for index in range(21):
            if index not in XYZ_INDICES_21:
                self.assertLess(abs(float(o[index])), 10.0,
                                f"index {index} looks scaled; only xyz entries should be")

    def test_goal_vectors_scale_xyz_only(self):
        np.testing.assert_allclose(
            self.obs["desired_goal"][:3], self.goal[:3] * 100.0, rtol=1e-5)
        np.testing.assert_allclose(
            self.obs["desired_goal"][3:], self.goal[3:], rtol=1e-5)

    def test_rpy_delta_is_raw_not_wrapped(self):
        """Training set neither align_obs_rpy_branch nor wrap_obs_rpy_delta.

        A delta past 180 degrees must stay past 180 degrees. Wrapping it here would be
        an input the checkpoint never saw.
        """
        current = np.array([0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0])
        goal = np.array([0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0])
        delta = build_observation(current, goal)["observation"][17]
        self.assertAlmostEqual(float(delta), -6.0, places=5)
        self.assertNotAlmostEqual(float(delta), 2 * np.pi - 6.0, places=3)

    def test_dtype_is_float32(self):
        for key in ("observation", "achieved_goal", "desired_goal"):
            self.assertEqual(self.obs[key].dtype, np.float32)


if __name__ == "__main__":
    unittest.main()
