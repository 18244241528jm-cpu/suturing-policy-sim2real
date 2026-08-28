"""Turn a needle pose seen by the camera into an Approach goal the executor accepts.

The three frames on this cart, and why each is needed:

    ECM         what /PSM1/measured_cp reports and what /PSM1/servo_cp obeys, because
                set_base_frame put PSM1's base at the ECM. Everything the executor
                touches lives here.
    PSM1_base   what /PSM1/local/measured_cp reports. Hand-eye calibration is solved
                against this frame, since it wants T_robot_base_from_control_point.
    camera      what FoundationPose reports the needle in.

The chain is therefore

    T_ECM_from_needle = T_ECM_from_psmbase @ T_psmbase_from_camera @ T_camera_from_needle

Only the middle term needs calibrating. The left term is free: measured_cp and
local/measured_cp describe the *same physical control point* in two frames, so

    T_ECM_from_psmbase = T_ECM_from_tip @ inverse(T_psmbase_from_tip)

and a single simultaneous reading of the two topics recovers it exactly. It is
constant while the ECM stays parked, which is required anyway.

The two readings must be simultaneous. Samples taken minutes apart with the arm moved
in between describe different tips and compose into a meaningless transform.
"""

from __future__ import annotations

import numpy as np

from .contract import matrix_to_quaternion_xyzw, normalize_quaternion_xyzw, pose_matrix


def invert(T: np.ndarray) -> np.ndarray:
    """Rigid inverse; cheaper and better conditioned than a general inverse."""
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def pose_msg_to_matrix(msg) -> np.ndarray:
    """geometry_msgs PoseStamped -> 4x4, with the quaternion renormalised."""
    p, o = msg.pose.position, msg.pose.orientation
    q = normalize_quaternion_xyzw(np.array([o.x, o.y, o.z, o.w], dtype=np.float64))
    return pose_matrix([p.x, p.y, p.z], q)


def matrix_to_pose_msg(T: np.ndarray, frame_id: str, stamp, msg_type):
    """4x4 -> PoseStamped in `frame_id`."""
    out = msg_type()
    out.header.stamp = stamp
    out.header.frame_id = frame_id
    out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, T[:3, 3])
    q = matrix_to_quaternion_xyzw(T[:3, :3])
    out.pose.orientation.x, out.pose.orientation.y = float(q[0]), float(q[1])
    out.pose.orientation.z, out.pose.orientation.w = float(q[2]), float(q[3])
    return out


def ecm_from_psmbase(tip_in_ecm: np.ndarray, tip_in_psmbase: np.ndarray) -> np.ndarray:
    """Recover the fixed ECM<-PSM_base transform from one simultaneous tip reading.

    Both arguments describe the same control point, so composing one with the inverse
    of the other cancels the tip and leaves the frame relationship.
    """
    return np.asarray(tip_in_ecm).reshape(4, 4) @ invert(tip_in_psmbase)


def load_handeye(values) -> np.ndarray:
    """Accept T_psmbase_from_camera as 16 row-major numbers or 7 as xyz + xyzw quat."""
    values = list(values)
    if len(values) == 16:
        T = np.asarray(values, dtype=np.float64).reshape(4, 4)
    elif len(values) == 7:
        q = normalize_quaternion_xyzw(np.asarray(values[3:7], dtype=np.float64))
        T = pose_matrix(values[0:3], q)
    else:
        raise ValueError(
            f"D13-E01-HANDEYE_SHAPE expected 16 (4x4 row-major) or 7 (xyz + quat xyzw), got {len(values)}"
        )
    validate_rigid(T, "T_psmbase_from_camera")
    return T


def validate_rigid(T: np.ndarray, name: str, tolerance: float = 1e-4) -> None:
    """Reject a transform that is not a rotation plus a translation.

    A hand-eye solve that did not converge, or a matrix pasted with a dropped digit,
    otherwise propagates silently: the servo drives confidently to a wrong pose and
    nothing downstream can tell.
    """
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"D13-E02-SHAPE {name} is {T.shape}, expected (4, 4)")
    R = T[:3, :3]
    orthonormality = float(np.abs(R @ R.T - np.eye(3)).max())
    determinant = float(np.linalg.det(R))
    if orthonormality > tolerance:
        raise ValueError(
            f"D13-E03-NOT_ORTHONORMAL {name} |RR^T - I| = {orthonormality:.2e} > {tolerance:g}"
        )
    if abs(determinant - 1.0) > tolerance:
        raise ValueError(
            f"D13-E04-NOT_RIGHT_HANDED {name} det(R) = {determinant:.6f}; "
            "-1 means the frame is mirrored, which usually is a column-order mistake"
        )
    if not np.allclose(T[3, :], [0, 0, 0, 1], atol=tolerance):
        raise ValueError(f"D13-E05-BOTTOM_ROW {name} last row is {T[3, :].tolist()}")


def needle_in_ecm(tip_in_ecm, tip_in_psmbase, psmbase_from_camera, needle_in_camera):
    """Compose the full chain, in the one order that type-checks by frame name."""
    return (
        ecm_from_psmbase(tip_in_ecm, tip_in_psmbase)
        @ np.asarray(psmbase_from_camera).reshape(4, 4)
        @ np.asarray(needle_in_camera).reshape(4, 4)
    )
