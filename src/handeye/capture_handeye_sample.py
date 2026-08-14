#!/usr/bin/env python3
"""Read-only ECM/PSM hand-eye sample collector with a hardware-free mock mode.

This file intentionally creates subscribers only.  It contains no publisher,
action client, servo command, jaw command, power, home, or motion API.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from handeye_common import (
    SCHEMA_VERSION,
    add_pose_noise,
    invert,
    load_yaml,
    project_marker,
    random_visible_marker_pose,
    write_json,
)


def git_commit() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL
        ).strip()
    except Exception:
        return "unavailable"


def stamp_ns(message) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def transform_from_ros(message) -> np.ndarray:
    value = message.transform if hasattr(message, "transform") else message.pose
    result = np.eye(4)
    result[:3, 3] = [value.translation.x, value.translation.y, value.translation.z] if hasattr(value, "translation") else [value.position.x, value.position.y, value.position.z]
    quaternion = value.rotation if hasattr(value, "rotation") else value.orientation
    result[:3, :3] = Rotation.from_quat(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    ).as_matrix()
    return result


def camera_info_payload(message) -> dict:
    return {
        "width": int(message.width),
        "height": int(message.height),
        "K": np.asarray(message.k, dtype=float).reshape(3, 3).tolist(),
        "distortion": list(message.d),
        "distortion_model": message.distortion_model,
        "frame_id": message.header.frame_id,
        "timestamp_ns": stamp_ns(message),
    }


def aruco_module():
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "OpenCV ArUco is unavailable; install an opencv-contrib build"
        )
    return cv2.aruco


def marker_dictionary(name: str):
    aruco = aruco_module()
    if not hasattr(aruco, name):
        raise ValueError(f"OpenCV has no ArUco dictionary named {name}")
    return aruco.getPredefinedDictionary(getattr(aruco, name))


def detect_aruco_markers(image: np.ndarray, dictionary):
    """Support both the OpenCV 4.7+ and legacy contrib ArUco APIs."""
    aruco = aruco_module()
    if hasattr(aruco, "ArucoDetector"):
        parameters = aruco.DetectorParameters()
        return aruco.ArucoDetector(dictionary, parameters).detectMarkers(image)
    parameters = aruco.DetectorParameters_create()
    return aruco.detectMarkers(image, dictionary, parameters=parameters)


def generate_marker_image(dictionary, marker_id: int, side_pixels: int) -> np.ndarray:
    """Support renamed marker rendering APIs across OpenCV contrib releases."""
    aruco = aruco_module()
    if hasattr(aruco, "generateImageMarker"):
        return aruco.generateImageMarker(dictionary, marker_id, side_pixels)
    if hasattr(aruco, "drawMarker"):
        return aruco.drawMarker(dictionary, marker_id, side_pixels)
    raise RuntimeError("OpenCV ArUco has no supported marker rendering API")


def detect_marker(image: np.ndarray, camera: dict, marker: dict) -> dict:
    dictionary = marker_dictionary(marker["dictionary"])
    corners, ids, _ = detect_aruco_markers(image, dictionary)
    if ids is None:
        raise RuntimeError("no ArUco marker detected")
    wanted = np.flatnonzero(ids.reshape(-1) == int(marker["id"]))
    if len(wanted) != 1:
        raise RuntimeError(
            f"expected one marker id {marker['id']}, found {len(wanted)}"
        )
    selected = np.asarray(corners[int(wanted[0])], dtype=np.float32).reshape(4, 2)
    object_points = np.array(
        [
            [-marker["size_m"] / 2, marker["size_m"] / 2, 0],
            [marker["size_m"] / 2, marker["size_m"] / 2, 0],
            [marker["size_m"] / 2, -marker["size_m"] / 2, 0],
            [-marker["size_m"] / 2, -marker["size_m"] / 2, 0],
        ],
        dtype=np.float32,
    )
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        selected,
        np.asarray(camera["K"], dtype=float),
        np.asarray(camera["distortion"], dtype=float),
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        raise RuntimeError("solvePnP failed")
    pose = np.eye(4)
    pose[:3, :3] = cv2.Rodrigues(rvec)[0]
    pose[:3, 3] = tvec.reshape(3)
    return {"corners_px": selected.tolist(), "T_camera_from_marker": pose.tolist()}


def raw_robot_payload(message) -> dict:
    value = message.transform if hasattr(message, "transform") else message.pose
    translation = value.translation if hasattr(value, "translation") else value.position
    rotation = value.rotation if hasattr(value, "rotation") else value.orientation
    return {
        "message_type": type(message).__module__ + "." + type(message).__name__,
        "frame_id": message.header.frame_id,
        "timestamp_ns": stamp_ns(message),
        "translation_m": [translation.x, translation.y, translation.z],
        "quaternion_xyzw": [rotation.x, rotation.y, rotation.z, rotation.w],
    }


def session_metadata(config: dict, config_path: Path, calibration_type: str) -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "calibration_type": calibration_type,
        "matrix_convention": "T_A_from_B maps coordinates in B into A",
        "units": {"translation": "meter", "rotation": "radian", "pixels": "pixel"},
        "created_unix_ns": time.time_ns(),
        "config_path": str(config_path.resolve()),
        "git_commit": git_commit(),
        "marker": config["marker"],
        "checkerboard": config.get("checkerboard"),
        "robot_config_path": config.get("robot_config_path", "TO_CONFIRM_ON_SITE"),
        "camera_config_path": config.get("camera_config_path", "TO_CONFIRM_ON_SITE"),
        "installation_photo_path": config.get("installation_photo_path", "TO_CAPTURE_ON_SITE"),
        "topics": config.get("topics", {}),
        "sampling_plan": {"solve": 24, "heldout": 6, "status": "[假设] engineering recommendation"},
        "safety": "read-only subscribers only; operator moves robot manually/teleoperates",
    }


def save_sample(
    session_dir: Path,
    sample_id: str,
    split: str,
    calibration_type: str,
    left: np.ndarray,
    right: np.ndarray | None,
    camera: dict,
    marker_config: dict,
    marker_detection: dict,
    robot_pose: np.ndarray,
    robot_stamp_ns: int,
    image_stamp_ns: int,
    robot_frame: str,
    operator_notes: str,
    raw_robot_message: dict,
    valid: bool = True,
    rejection_reason: str = "",
) -> None:
    directory = session_dir / "samples" / sample_id
    directory.mkdir(parents=True, exist_ok=False)
    cv2.imwrite(str(directory / "left.png"), left)
    right_path = None
    if right is not None:
        right_path = "right.png"
        cv2.imwrite(str(directory / right_path), right)
    payload = {
        "schema_version": SCHEMA_VERSION,
        "sample_id": sample_id,
        "split": split,
        "calibration_type": calibration_type,
        "valid": bool(valid),
        "rejection_reason": rejection_reason,
        "operator_notes": operator_notes,
        "images": {
            "left": "left.png",
            "right": right_path,
            "left_timestamp_ns": int(image_stamp_ns),
            "right_timestamp_ns": int(image_stamp_ns) if right is not None else None,
        },
        "camera_info": camera,
        "marker": {
            **marker_config,
            **marker_detection,
            "pose_convention": "T_camera_from_marker",
        },
        "robot_pose": {
            "T_robot_base_from_control_point": robot_pose.tolist(),
            "pose_convention": "T_robot_base_from_control_point",
            "timestamp_ns": int(robot_stamp_ns),
            "frame_id": robot_frame,
            "unit": "meter",
            "raw_message": raw_robot_message,
        },
        "sync_delta_ms": abs(int(image_stamp_ns) - int(robot_stamp_ns)) / 1.0e6,
    }
    write_json(directory / "sample.json", payload)


def render_mock_image(corners: np.ndarray, marker_id: int, dictionary_name: str) -> np.ndarray:
    canvas = np.full((480, 640, 3), 210, dtype=np.uint8)
    dictionary = marker_dictionary(dictionary_name)
    marker = generate_marker_image(dictionary, marker_id, 180)
    source = np.array([[0, 0], [179, 0], [179, 179], [0, 179]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(source, corners.astype(np.float32))
    warped = cv2.warpPerspective(marker, homography, (640, 480), borderValue=255)
    mask = cv2.warpPerspective(
        np.full_like(marker, 255), homography, (640, 480), borderValue=0
    )
    for channel in range(3):
        canvas[:, :, channel] = np.where(mask > 0, warped, canvas[:, :, channel])
    return canvas


def capture_mock(args, config: dict) -> None:
    rng = np.random.default_rng(args.seed)
    camera = {
        "width": 640,
        "height": 480,
        "K": [[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]],
        "distortion": [0, 0, 0, 0, 0],
        "distortion_model": "plumb_bob",
        "frame_id": "mock_left_camera_optical",
        "timestamp_ns": 0,
    }
    marker = config["marker"]
    if args.calibration_type == "ecm_eye_in_hand":
        control_from_camera = np.eye(4)
        control_from_camera[:3, 3] = [0.003, -0.002, 0.012]
        control_from_camera[:3, :3] = Rotation.from_euler(
            "xyz", [2, -4, 3], degrees=True
        ).as_matrix()
        base_from_target = np.eye(4)
        base_from_target[:3, 3] = [0.12, -0.04, 0.32]
        base_from_target[:3, :3] = Rotation.from_euler(
            "xyz", [5, -7, 20], degrees=True
        ).as_matrix()
        truth = {
            "T_control_point_from_camera": control_from_camera.tolist(),
            "T_robot_base_from_static_marker": base_from_target.tolist(),
        }
    else:
        camera_from_base = np.eye(4)
        camera_from_base[:3, 3] = [0.04, -0.03, 0.28]
        camera_from_base[:3, :3] = Rotation.from_euler(
            "xyz", [165, 4, -12], degrees=True
        ).as_matrix()
        control_from_marker = np.eye(4)
        control_from_marker[:3, 3] = [0.0, 0.018, 0.006]
        control_from_marker[:3, :3] = Rotation.from_euler(
            "xyz", [90, 0, 0], degrees=True
        ).as_matrix()
        truth = {
            "T_camera_from_robot_base": camera_from_base.tolist(),
            "T_control_point_from_marker": control_from_marker.tolist(),
        }
    write_json(args.session_dir / "ground_truth.json", truth)

    for index in range(args.count):
        observed = random_visible_marker_pose(rng)
        if args.calibration_type == "ecm_eye_in_hand":
            robot = base_from_target @ invert(observed) @ invert(control_from_camera)
        else:
            robot = invert(camera_from_base) @ observed @ invert(control_from_marker)
        noisy_robot = add_pose_noise(robot, rng, args.robot_noise_mm, args.robot_noise_deg)
        noisy_observed = add_pose_noise(observed, rng, args.pnp_noise_mm, args.pnp_noise_deg)
        corners = project_marker(
            noisy_observed,
            np.asarray(camera["K"]),
            np.asarray(camera["distortion"]),
            float(marker["size_m"]),
        )
        corners += rng.normal(scale=args.corner_noise_px, size=corners.shape)
        left = render_mock_image(corners, int(marker["id"]), marker["dictionary"])
        split = "solve" if index < min(24, args.count) else "heldout"
        stamp = 1_800_000_000_000_000_000 + index * 100_000_000
        save_sample(
            args.session_dir,
            f"sample_{index:03d}",
            split,
            args.calibration_type,
            left,
            left.copy(),
            {**camera, "timestamp_ns": stamp},
            marker,
            {
                "corners_px": corners.tolist(),
                "T_camera_from_marker": noisy_observed.tolist(),
            },
            noisy_robot,
            stamp + int(args.sync_delta_ms * 1e6),
            stamp,
            "mock_robot_base",
            "synthetic mock sample",
            {"message_type": "mock", "matrix": noisy_robot.tolist()},
        )


def capture_ros(args, config: dict) -> None:
    try:
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from rosidl_runtime_py.utilities import get_message
        from sensor_msgs.msg import CameraInfo, Image
    except ImportError as exc:
        raise RuntimeError(
            "ROS mode requires rclpy, cv_bridge, sensor_msgs and rosidl_runtime_py"
        ) from exc

    class Collector(Node):
        def __init__(self):
            super().__init__("d3a_read_only_handeye_collector")
            self.bridge = CvBridge()
            self.left = self.right = self.camera = self.robot = None
            topics = config["topics"]
            self.create_subscription(Image, topics["left_image"], self.on_left, 10)
            if topics.get("right_image"):
                self.create_subscription(Image, topics["right_image"], self.on_right, 10)
            self.create_subscription(CameraInfo, topics["left_camera_info"], self.on_camera, 10)
            robot_type = get_message(topics["robot_pose_msg_type"])
            self.create_subscription(robot_type, topics["robot_measured_cp"], self.on_robot, 50)

        def on_left(self, message):
            self.left = message

        def on_right(self, message):
            self.right = message

        def on_camera(self, message):
            self.camera = message

        def on_robot(self, message):
            self.robot = message

    rclpy.init()
    node = Collector()
    collected = 0
    try:
        while rclpy.ok() and collected < args.count:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.left is None or node.camera is None or node.robot is None:
                continue
            command = input("ENTER=capture, r=reject-note, q=quit: ").strip().lower()
            if command == "q":
                break
            left = node.bridge.imgmsg_to_cv2(node.left, desired_encoding="bgr8")
            right = None if node.right is None else node.bridge.imgmsg_to_cv2(node.right, desired_encoding="bgr8")
            camera = camera_info_payload(node.camera)
            detection = detect_marker(left, camera, config["marker"])
            split = args.split or ("solve" if collected < 24 else "heldout")
            image_stamp = stamp_ns(node.left)
            robot_stamp = stamp_ns(node.robot)
            save_sample(
                args.session_dir,
                f"sample_{collected:03d}",
                split,
                args.calibration_type,
                left,
                right,
                camera,
                config["marker"],
                detection,
                transform_from_ros(node.robot),
                robot_stamp,
                image_stamp,
                node.robot.header.frame_id,
                input("operator notes (optional): ").strip(),
                raw_robot_payload(node.robot),
                valid=command != "r",
                rejection_reason=(input("rejection reason: ").strip() if command == "r" else ""),
            )
            collected += 1
            print(f"saved sample_{collected-1:03d} split={split}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument(
        "--calibration-type",
        choices=["ecm_eye_in_hand", "psm_eye_to_hand"],
        required=True,
    )
    parser.add_argument("--mode", choices=["ros", "mock"], default="ros")
    parser.add_argument("--count", type=int, default=30)
    parser.add_argument("--split", choices=["solve", "heldout"])
    parser.add_argument("--seed", type=int, default=34006)
    parser.add_argument("--robot-noise-mm", type=float, default=0.0)
    parser.add_argument("--robot-noise-deg", type=float, default=0.0)
    parser.add_argument("--pnp-noise-mm", type=float, default=0.0)
    parser.add_argument("--pnp-noise-deg", type=float, default=0.0)
    parser.add_argument("--corner-noise-px", type=float, default=0.0)
    parser.add_argument("--sync-delta-ms", type=float, default=1.0)
    args = parser.parse_args()
    config = load_yaml(args.config)
    args.session_dir.mkdir(parents=True, exist_ok=False)
    write_json(
        args.session_dir / "session.json",
        session_metadata(config, args.config, args.calibration_type),
    )
    if args.mode == "mock":
        capture_mock(args, config)
    else:
        capture_ros(args, config)
    print(f"session written to {args.session_dir}")


if __name__ == "__main__":
    main()
