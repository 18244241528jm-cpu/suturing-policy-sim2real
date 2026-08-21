import math
import unittest

import numpy as np

from suturing_runtime.pose_selection import (
    motion_compensate_pose,
    select_needle_candidate,
    select_psm_candidate,
)


def yaw(degrees: float) -> np.ndarray:
    angle = math.radians(degrees)
    c, s = math.cos(angle), math.sin(angle)
    out = np.eye(4)
    out[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    return out


class PoseSelectionTest(unittest.TestCase):
    def test_needle_modes_are_real_ablation_switches(self):
        poses = np.repeat(np.eye(4)[None, :, :], 3, axis=0)
        poses[0, :3, :3] = np.diag([1.0, -1.0, -1.0])  # highest score, upside down
        poses[2] = yaw(10.0)
        scores = np.array([0.99, 0.90, 0.80])

        raw = select_needle_candidate(poses, scores)
        self.assertEqual(raw.selected_index, 0)

        flat = select_needle_candidate(
            poses, scores, flat_enabled=True,
            plane_normal_camera=[0, 0, 1], needle_rest_normal_mesh=[0, 0, 1],
            maximum_flat_angle_deg=15,
        )
        self.assertEqual(flat.selected_index, 1)
        self.assertNotIn(0, flat.survivor_indices)

        planar = select_needle_candidate(
            poses, scores, flat_enabled=True,
            plane_normal_camera=[0, 0, 1], needle_rest_normal_mesh=[0, 0, 1],
            maximum_flat_angle_deg=15, planar_enabled=True,
            planar_pose_camera=yaw(10), plane_axis_x_camera=[1, 0, 0],
            plane_axis_y_camera=[0, 1, 0], needle_heading_axis_mesh=[1, 0, 0],
            planar_sigmas=[0.001, 0.001, 2.0], planar_chi2_threshold=11.3449,
        )
        self.assertEqual(planar.selected_index, 2)
        self.assertTrue(np.allclose(planar.survivor_indices, [2]))

    def test_support_height_is_optional_and_explicit(self):
        poses = np.repeat(np.eye(4)[None, :, :], 2, axis=0)
        poses[0, 2, 3] = 0.010
        poses[1, 2, 3] = 0.001
        result = select_needle_candidate(
            poses, [1.0, 0.5], flat_enabled=True,
            plane_normal_camera=[0, 0, 1], needle_rest_normal_mesh=[0, 0, 1],
            support_height_enabled=True, plane_point_camera_m=[0, 0, 0],
            needle_origin_support_offset_m=0.001, maximum_height_error_m=0.002,
        )
        self.assertEqual(result.selected_index, 1)

    def test_planar_axes_must_be_tangent_to_support(self):
        with self.assertRaisesRegex(ValueError, "tangent"):
            select_needle_candidate(
                np.eye(4)[None, :, :], [1.0], flat_enabled=True,
                plane_normal_camera=[0, 0, 1], needle_rest_normal_mesh=[0, 0, 1],
                planar_enabled=True, planar_pose_camera=np.eye(4),
                plane_axis_x_camera=[1, 0, 1], plane_axis_y_camera=[0, 1, 0],
                needle_heading_axis_mesh=[1, 0, 0],
                planar_sigmas=[0.001, 0.001, 2.0],
            )

    def test_psm_fusion_converts_mesh_to_control_point_then_uses_uncertainty(self):
        candidates = np.repeat(np.eye(4)[None, :, :], 2, axis=0)
        candidates[0, 0, 3] = 0.050  # FP top score but far from FK
        candidates[1, 0, 3] = 0.010
        mesh_from_cp = np.eye(4)
        mesh_from_cp[0, 3] = -0.010
        fk = np.eye(4)
        selected = select_psm_candidate(
            candidates, [1.0, 0.5], fk, mesh_from_cp,
            kinematic_sigma6=[0.003, 0.003, 0.003, 5, 5, 5],
            vision_sigma6=[0.002, 0.002, 0.002, 3, 3, 3],
            fp_rank_weight=1.0,
        )
        self.assertEqual(selected.selected_index, 1)
        self.assertAlmostEqual(selected.translation_innovation_m, 0.0, places=9)

    def test_psm_fusion_refuses_missing_uncertainty(self):
        with self.assertRaises(ValueError):
            select_psm_candidate(
                np.eye(4)[None, :, :], [1.0], np.eye(4), np.eye(4),
                kinematic_sigma6=np.zeros(6), vision_sigma6=np.ones(6),
                fp_rank_weight=0.0,
            )

    def test_motion_compensation_applies_fk_relative_motion(self):
        visual = np.eye(4)
        capture = np.eye(4)
        latest = np.eye(4)
        latest[1, 3] = 0.004
        compensated = motion_compensate_pose(visual, capture, latest)
        np.testing.assert_allclose(compensated[:3, 3], [0.0, 0.004, 0.0])


if __name__ == "__main__":
    unittest.main()
