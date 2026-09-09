import math
import unittest

import numpy as np

from suturing_runtime.policy_contract import (
    checkpoint_observation,
    integrate_checkpoint_action,
    quaternion_xyzw_to_legacy_rpy,
    rotation_error_rad,
    rpy_to_quaternion_xyzw,
)


class PolicyContractTest(unittest.TestCase):
    def test_checkpoint_observation_units_and_order(self):
        current = np.asarray([0.01, 0.02, 0.03, -3.0, 0.2, 0.3, 0.8], dtype=np.float32)
        desired = np.asarray([0.04, 0.05, 0.06, -2.5, 0.1, -0.2, 0.0], dtype=np.float32)
        obs = checkpoint_observation(current, desired)
        np.testing.assert_allclose(obs["achieved_goal"][:3], [1.0, 2.0, 3.0])
        np.testing.assert_allclose(obs["desired_goal"][:3], [4.0, 5.0, 6.0])
        np.testing.assert_allclose(obs["observation"][:7], obs["achieved_goal"])
        np.testing.assert_allclose(obs["observation"][7:14], obs["desired_goal"])
        np.testing.assert_allclose(obs["observation"][14:21], obs["desired_goal"] - obs["achieved_goal"])

    def test_action_is_clipped_and_scaled(self):
        result = integrate_checkpoint_action(np.zeros(7), np.asarray([2, -2, 0, 0, 0, 0, 1]),
                                             np.asarray([0.0015] * 3 + [0.1] * 3 + [0.05]))
        np.testing.assert_allclose(result, [0.0015, -0.0015, 0, 0, 0, 0, 0.05])

    def test_legacy_positive_roll_moves_to_negative_branch(self):
        quaternion = rpy_to_quaternion_xyzw([0.25, -0.1, 0.2])
        recovered = quaternion_xyzw_to_legacy_rpy(quaternion)
        self.assertAlmostEqual(float(recovered[0]), 0.25 - 2.0 * math.pi, places=5)
        self.assertAlmostEqual(rotation_error_rad(recovered, [0.25, -0.1, 0.2]), 0.0, places=5)


if __name__ == "__main__":
    unittest.main()
