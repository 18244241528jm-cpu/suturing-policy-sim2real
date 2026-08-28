"""The camera-to-ECM chain, and the checks that stop a bad transform reaching the arm.

A wrong frame composition does not announce itself. The servo drives confidently to a
pose that is wrong by however far the composition is off, and every downstream guard
still passes because the goal is inside the workspace and the segment is short. These
tests pin the algebra and the rejection of transforms that are not rigid.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "suturing_runtime"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from suturing_runtime.frame_chain import (  # noqa: E402
    ecm_from_psmbase,
    invert,
    load_handeye,
    needle_in_ecm,
    validate_rigid,
)


def transform(rpy_deg, xyz):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    T[:3, 3] = xyz
    return T


class InvertTest(unittest.TestCase):
    def test_rigid_inverse_matches_the_general_one(self):
        T = transform([13.0, -47.0, 88.0], [0.11, -0.02, 0.34])
        np.testing.assert_allclose(invert(T), np.linalg.inv(T), atol=1e-12)

    def test_round_trip_is_identity(self):
        T = transform([-5.0, 61.0, 170.0], [-0.09, 0.06, 0.04])
        np.testing.assert_allclose(invert(T) @ T, np.eye(4), atol=1e-12)


class EcmFromPsmBaseTest(unittest.TestCase):
    """The free half of the chain: two views of one tip give the frame relationship."""

    def setUp(self):
        # A ground-truth ECM<-PSM_base, and a tip somewhere in the PSM base frame.
        self.truth = transform([12.0, -30.0, 95.0], [-0.06, 0.07, 0.08])
        self.tip_in_psmbase = transform([40.0, 15.0, -70.0], [-0.026, -0.006, -0.034])
        self.tip_in_ecm = self.truth @ self.tip_in_psmbase

    def test_recovers_the_transform_from_one_simultaneous_pair(self):
        np.testing.assert_allclose(
            ecm_from_psmbase(self.tip_in_ecm, self.tip_in_psmbase), self.truth, atol=1e-12)

    def test_independent_of_where_the_tip_happens_to_be(self):
        other = transform([-80.0, 5.0, 120.0], [0.01, 0.02, -0.05])
        np.testing.assert_allclose(
            ecm_from_psmbase(self.truth @ other, other), self.truth, atol=1e-12)

    def test_a_stale_pair_gives_a_wrong_transform(self):
        """Why the node enforces max_pair_skew_s.

        If the arm moves between the two reads, the pair describes different tips and
        the composition is off by the motion -- silently, since the result is still a
        valid rigid transform.
        """
        moved = self.tip_in_psmbase.copy()
        moved[:3, 3] += np.array([0.010, 0.0, 0.0])  # 10 mm between the two reads
        recovered = ecm_from_psmbase(self.tip_in_ecm, moved)
        error_mm = float(np.linalg.norm(recovered[:3, 3] - self.truth[:3, 3])) * 1000
        self.assertGreater(error_mm, 5.0)
        validate_rigid(recovered, "recovered")  # still rigid, hence undetectable here


class FullChainTest(unittest.TestCase):
    def test_composes_camera_to_ecm(self):
        ecm_from_base = transform([12.0, -30.0, 95.0], [-0.06, 0.07, 0.08])
        base_from_camera = transform([-100.0, 20.0, 45.0], [0.02, -0.03, 0.12])
        camera_from_needle = transform([119.6, -25.0, 7.9], [0.0194, 0.0127, 0.0801])
        tip_in_psmbase = transform([40.0, 15.0, -70.0], [-0.026, -0.006, -0.034])

        got = needle_in_ecm(ecm_from_base @ tip_in_psmbase, tip_in_psmbase,
                            base_from_camera, camera_from_needle)
        expected = ecm_from_base @ base_from_camera @ camera_from_needle
        np.testing.assert_allclose(got, expected, atol=1e-12)

    def test_order_matters(self):
        """Swapping two links stays a rigid transform, so only the value reveals it."""
        ecm_from_base = transform([12.0, -30.0, 95.0], [-0.06, 0.07, 0.08])
        base_from_camera = transform([-100.0, 20.0, 45.0], [0.02, -0.03, 0.12])
        camera_from_needle = transform([119.6, -25.0, 7.9], [0.0194, 0.0127, 0.0801])
        right = ecm_from_base @ base_from_camera @ camera_from_needle
        wrong = ecm_from_base @ camera_from_needle @ base_from_camera
        self.assertGreater(float(np.linalg.norm(right[:3, 3] - wrong[:3, 3])) * 1000, 10.0)


class ValidateRigidTest(unittest.TestCase):
    def test_accepts_a_proper_transform(self):
        validate_rigid(transform([10.0, 20.0, 30.0], [0.1, 0.2, 0.3]), "ok")

    def test_rejects_a_scaled_rotation(self):
        T = transform([10.0, 20.0, 30.0], [0.1, 0.2, 0.3])
        T[:3, :3] *= 1.05  # a 5% scale, the shape a bad calibration solve produces
        with self.assertRaises(ValueError) as caught:
            validate_rigid(T, "scaled")
        self.assertIn("D13-E03-NOT_ORTHONORMAL", str(caught.exception))

    def test_rejects_a_mirrored_frame(self):
        T = transform([10.0, 20.0, 30.0], [0.1, 0.2, 0.3])
        T[:3, 0] *= -1.0  # flipped column: det becomes -1
        with self.assertRaises(ValueError) as caught:
            validate_rigid(T, "mirrored")
        self.assertIn("D13-E04-NOT_RIGHT_HANDED", str(caught.exception))

    def test_rejects_a_broken_bottom_row(self):
        T = transform([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        T[3, 0] = 0.5
        with self.assertRaises(ValueError):
            validate_rigid(T, "bottom")


class LoadHandeyeTest(unittest.TestCase):
    def test_accepts_sixteen_row_major(self):
        T = transform([5.0, -12.0, 30.0], [0.01, 0.02, 0.03])
        np.testing.assert_allclose(load_handeye(T.flatten().tolist()), T, atol=1e-12)

    def test_accepts_seven_as_xyz_plus_quaternion(self):
        T = transform([5.0, -12.0, 30.0], [0.01, 0.02, 0.03])
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        got = load_handeye([0.01, 0.02, 0.03, *q.tolist()])
        np.testing.assert_allclose(got, T, atol=1e-9)

    def test_rejects_any_other_length(self):
        with self.assertRaises(ValueError) as caught:
            load_handeye([0.0] * 12)
        self.assertIn("D13-E01-HANDEYE_SHAPE", str(caught.exception))

    def test_rejects_a_non_rigid_matrix(self):
        with self.assertRaises(ValueError):
            load_handeye([2.0, 0, 0, 0, 0, 2.0, 0, 0, 0, 0, 2.0, 0, 0, 0, 0, 1.0])


if __name__ == "__main__":
    unittest.main()
