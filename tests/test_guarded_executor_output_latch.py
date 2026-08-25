"""The executor must never arm, or tick, without a command publisher behind it.

The publishers are built once in __init__, gated on _output_contract_ok().  That
predicate re-reads its parameters on every call, so flipping enable_output at runtime
used to make arming succeed while both publishers were still None -- and the very next
tick dereferenced None inside the timer callback, with the robot already armed.

These tests need rclpy and are skipped where it is unavailable (the Windows-side test
run); they pass under ROS 2 Humble in WSL.
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
    from std_srvs.srv import SetBool
    from geometry_msgs.msg import PoseStamped

    rclpy = _rclpy
except ImportError:  # pragma: no cover - depends on the environment, not the code
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
class OutputLatchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.module = load_executor_module()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        self.node = self.module.GuardedPoseExecutor()
        self.addCleanup(self.node.destroy_node)

    def _arm(self, value: bool):
        response = SetBool.Response()
        request = SetBool.Request()
        request.data = value
        return self.node._arm(request, response)

    def test_default_launch_constructs_no_command_publisher(self):
        self.assertFalse(self.node._output_constructed_at_startup)
        self.assertIsNone(self.node.command_pose_pub)
        self.assertIsNone(self.node.command_transform_pub)

    def test_arming_is_locked_while_the_contract_fails(self):
        response = self._arm(True)
        self.assertFalse(response.success)
        self.assertIn("D10-E50-OUTPUT_LOCKED", response.message)
        self.assertFalse(self.node.armed)

    def test_runtime_enable_cannot_arm_without_a_publisher(self):
        # The exact situation that used to crash: parameters flipped after startup.
        self.node.set_parameters([
            Parameter("enable_output", value=True),
            Parameter("operator_acknowledgement", value=self.module.ACK),
        ])
        self.assertTrue(self.node._output_contract_ok(), "precondition: contract now passes")
        response = self._arm(True)
        self.assertFalse(response.success, "arming must refuse when no publisher exists")
        self.assertIn("D10-E60-OUTPUT_NOT_CONSTRUCTED", response.message)
        self.assertFalse(self.node.armed)

    def test_tick_faults_instead_of_dereferencing_a_missing_publisher(self):
        self.node.set_parameters([
            Parameter("enable_output", value=True),
            Parameter("operator_acknowledgement", value=self.module.ACK),
        ])
        frame = str(self.node.get_parameter("command_frame").value)
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.orientation.w = 1.0
        goal = PoseStamped()
        goal.header.frame_id = frame
        goal.pose.position.z = 0.002
        goal.pose.orientation.w = 1.0
        self.node._measured(pose)
        self.node._goal(goal)
        # Bypass the arm service to reach the tick, which is where the crash happened.
        self.node.armed = True
        self.node.active = True
        self.node.started_at = self.module.time.monotonic()

        faults = []
        self.node._fault = faults.append
        self.node._tick()  # must not raise AttributeError

        self.assertTrue(faults, "tick should have faulted")
        self.assertIn("D10-E60-OUTPUT_NOT_CONSTRUCTED", faults[0])


if __name__ == "__main__":
    unittest.main()
