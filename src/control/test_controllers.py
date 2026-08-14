#!/usr/bin/env python3

import unittest

import numpy as np

from controllers import (
    adaptive_se3_goal_servo_action,
    make_policy_residual_action,
    staged_se3_goal_servo_action,
)


STEP = np.array(
    [0.0015, 0.0015, 0.0015, np.deg2rad(3), np.deg2rad(3), np.deg2rad(3), 0.05],
    dtype=np.float64,
)


def observation(achieved, desired):
    return {
        "achieved_goal": np.asarray(achieved, dtype=np.float64),
        "desired_goal": np.asarray(desired, dtype=np.float64),
    }


class IsolatedControllerTests(unittest.TestCase):
    def test_zero_error_outputs_zero(self):
        state = np.zeros(7)
        action = adaptive_se3_goal_servo_action(observation(state, state), STEP)
        np.testing.assert_allclose(action, 0.0, atol=1.0e-7)

    def test_pi_branch_uses_short_rotation(self):
        achieved = np.array([0, 0, 0, 0, 0, np.deg2rad(179), 0.0])
        desired = np.array([0, 0, 0, 0, 0, np.deg2rad(-179), 0.0])
        action = adaptive_se3_goal_servo_action(observation(achieved, desired), STEP)
        self.assertLess(abs(float(action[5])), 0.5)
        self.assertGreater(float(action[5]), 0.0)

    def test_near_goal_is_more_conservative(self):
        origin = np.zeros(7)
        far = np.array([2.0, 0, 0, 0, 0, 0, 0])  # xyz observations use cm
        near = np.array([0.05, 0, 0, 0, 0, 0, 0])
        far_action = adaptive_se3_goal_servo_action(observation(origin, far), STEP)
        near_action = adaptive_se3_goal_servo_action(observation(origin, near), STEP)
        self.assertGreater(abs(float(far_action[0])), abs(float(near_action[0])))

    def test_staged_servo_reduces_translation_during_large_rotation(self):
        achieved = np.zeros(7)
        desired = np.array([2.0, 0, 0, np.deg2rad(60), 0, 0, 0])
        adaptive = adaptive_se3_goal_servo_action(observation(achieved, desired), STEP)
        staged = staged_se3_goal_servo_action(observation(achieved, desired), STEP)
        self.assertLess(abs(float(staged[0])), abs(float(adaptive[0])))
        self.assertAlmostEqual(float(staged[3]), float(adaptive[3]), places=6)

    def test_residual_guard_suppresses_opposing_policy_axis(self):
        achieved = np.zeros(7)
        desired = np.array([2.0, 0, 0, 0, 0, 0, 0])
        residual = make_policy_residual_action(adaptive_se3_goal_servo_action)
        action = residual(
            observation(achieved, desired),
            np.array([-1.0, 0, 0, 0, 0, 0, 0], dtype=np.float32),
            STEP,
            policy_weight=0.5,
            servo_weight=0.5,
            direction_guard=True,
        )
        self.assertGreater(float(action[0]), 0.0)

    def test_all_actions_are_bounded(self):
        achieved = np.array([-100, 100, -100, 2.9, -1.2, 3.0, 1.0])
        desired = np.array([100, -100, 100, -2.9, 1.2, -3.0, 0.0])
        action = adaptive_se3_goal_servo_action(observation(achieved, desired), STEP)
        self.assertTrue(np.all(np.isfinite(action)))
        self.assertTrue(np.all(action <= 1.0))
        self.assertTrue(np.all(action >= -1.0))


if __name__ == "__main__":
    unittest.main()

