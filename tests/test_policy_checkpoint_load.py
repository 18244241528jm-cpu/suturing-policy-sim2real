"""Loading the shipped checkpoint, and the trainer dependency it smuggles in.

The Approach checkpoints were written by a custom TD3_HER_BC whose replay buffer class
lives in the RL_algo package. cloudpickle stores that by name, so a plain
stable_baselines3 TD3.load raises

    ModuleNotFoundError: No module named 'RL_algo'

on any machine without the training tree -- which is exactly the cart. None of those
entries are read to run the actor, so the node replaces them rather than installing the
trainer on the robot.

These tests need stable_baselines3 and a checkpoint, and skip cleanly without either, so
they run wherever the RL environment happens to be installed.
"""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "suturing_runtime"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

# Point this at a checkpoint to exercise the load path; the contract tests below run
# regardless, since they only need the override table.
CHECKPOINT = os.environ.get("SUTURING_TEST_CHECKPOINT", "")

try:
    import stable_baselines3  # noqa: F401

    HAVE_SB3 = True
except ImportError:  # pragma: no cover - environment dependent
    HAVE_SB3 = False


def read_overrides() -> dict:
    """Pull INFERENCE_ONLY_OVERRIDES out without importing rclpy."""
    source = (PACKAGE_ROOT / "suturing_runtime" / "policy_servo_node.py").read_text(
        encoding="utf-8")
    start = source.index("INFERENCE_ONLY_OVERRIDES")
    end = source.index("def _load_model", start)
    namespace: dict = {}
    body = "\n".join(line[4:] if line.startswith("    ") else line
                     for line in source[start:end].splitlines())
    exec(compile(body, "<overrides>", "exec"), namespace)
    return namespace["INFERENCE_ONLY_OVERRIDES"]


class OverrideTableTest(unittest.TestCase):
    def test_names_every_entry_that_reaches_into_rl_algo(self):
        overrides = read_overrides()
        # replay_buffer_class is the one that actually raises; the other two are large
        # training-only blobs that would be deserialised for nothing.
        self.assertIn("replay_buffer_class", overrides)
        self.assertIsNone(overrides["replay_buffer_class"])
        self.assertIn("replay_buffer_kwargs", overrides)
        self.assertIn("demo_data", overrides)

    def test_does_not_override_anything_the_actor_needs(self):
        overrides = read_overrides()
        for key in ("policy_class", "observation_space", "action_space", "lr_schedule"):
            self.assertNotIn(
                key, overrides,
                f"{key} is read when rebuilding the actor; overriding it would change "
                "the network rather than skip unused training state")


@unittest.skipUnless(HAVE_SB3 and CHECKPOINT and Path(CHECKPOINT).is_file(),
                     "set SUTURING_TEST_CHECKPOINT to a checkpoint to run")
class CheckpointLoadTest(unittest.TestCase):
    def test_plain_load_fails_the_way_the_cart_would_see_it(self):
        from stable_baselines3 import TD3

        with self.assertRaises(ModuleNotFoundError) as caught:
            TD3.load(CHECKPOINT, device="cpu")
        self.assertIn("RL_algo", str(caught.exception))

    def test_load_with_the_overrides_succeeds(self):
        from stable_baselines3 import TD3

        model = TD3.load(CHECKPOINT, device="cpu", custom_objects=dict(read_overrides()))
        shapes = {k: tuple(v.shape) for k, v in model.observation_space.spaces.items()}
        self.assertEqual(
            shapes, {"observation": (21,), "achieved_goal": (7,), "desired_goal": (7,)})
        self.assertEqual(tuple(model.action_space.shape), (7,))

    def test_forward_pass_returns_a_bounded_seven_vector(self):
        from stable_baselines3 import TD3

        model = TD3.load(CHECKPOINT, device="cpu", custom_objects=dict(read_overrides()))
        current = np.array([-0.0318, 0.0250, -0.1152, -3.03, 0.92, 2.49, 0.0])
        goal = current.copy()
        goal[2] += 0.005
        scale = np.array([100, 100, 100, 1, 1, 1, 1], dtype=np.float32)
        observation = np.concatenate((current, goal, goal - current)).astype(np.float32)
        observation[[0, 1, 2, 7, 8, 9, 14, 15, 16]] *= 100.0
        action, _ = model.predict({
            "observation": observation,
            "achieved_goal": (current.astype(np.float32) * scale),
            "desired_goal": (goal.astype(np.float32) * scale),
        }, deterministic=True)
        action = np.asarray(action).reshape(-1)
        self.assertEqual(action.shape, (7,))
        self.assertTrue(np.all(np.abs(action) <= 1.0 + 1e-6))
        self.assertTrue(np.isfinite(action).all())


if __name__ == "__main__":
    unittest.main()
