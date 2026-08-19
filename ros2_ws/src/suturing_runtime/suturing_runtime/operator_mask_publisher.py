"""Export, review, and publish a first-frame needle mask with the exact RGB header."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .mask_utils import mask_overlay, normalize_mask, polygon_mask
from .message_contract import clone_message, stamp_ns


def decode_color(msg: Image) -> np.ndarray:
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"D15-E301-RGB_ENCODING {msg.encoding!r}")
    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(int(msg.height), int(msg.step))
    bgr = row[:, : int(msg.width) * 3].reshape(int(msg.height), int(msg.width), 3).copy()
    return bgr[:, :, ::-1].copy() if msg.encoding == "rgb8" else bgr


class OperatorMaskPublisher(Node):
    def __init__(self) -> None:
        super().__init__("operator_mask_publisher")
        self.declare_parameter("input_topic", "/suturing/initialization/left/image")
        self.declare_parameter("output_topic", "/suturing/external/needle_mask")
        self.declare_parameter("output_root", "~/surgicai_operator_masks")
        self.declare_parameter("mask_filename", "needle_mask.png")
        self.declare_parameter("auto_export", True)
        self.declare_parameter("gui_enabled", False)
        self.declare_parameter("minimum_mask_fraction", 0.00001)
        self.declare_parameter("maximum_mask_fraction", 0.20)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            Image, str(self.get_parameter("output_topic").value), latched
        )
        self.status = self.create_publisher(String, "/suturing/operator_mask/status", latched)
        self.create_subscription(
            Image, str(self.get_parameter("input_topic").value), self._image, latched
        )
        self.create_service(Trigger, "/suturing/operator_mask/export", self._export_service)
        self.create_service(
            Trigger, "/suturing/operator_mask/publish_file", self._publish_file_service
        )
        self.create_service(
            Trigger, "/suturing/operator_mask/publish_polygon", self._publish_polygon_service
        )
        self.create_service(Trigger, "/suturing/operator_mask/clear_points", self._clear_service)
        self.source: Image | None = None
        self.bgr: np.ndarray | None = None
        self.session: Path | None = None
        self.points: list[tuple[int, int]] = []
        self.gui_ready = False
        self.gui_enabled = bool(self.get_parameter("gui_enabled").value)
        if self.gui_enabled:
            self.create_timer(0.05, self._gui_tick)
        self._emit("WAITING_FOR_SNAPSHOT", code="D15-E300-WAITING_FOR_SNAPSHOT")

    def _emit(self, state: str, **fields) -> None:
        message = String()
        message.data = json.dumps(
            {"schema": "suturing.operator_mask_status.v1", "state": state, **fields},
            sort_keys=True,
        )
        self.status.publish(message)

    def _image(self, msg: Image) -> None:
        try:
            self.source = clone_message(msg)
            self.bgr = decode_color(msg)
            root = Path(str(self.get_parameter("output_root").value)).expanduser()
            self.session = root / f"stamp_{stamp_ns(msg.header)}"
            self.points.clear()
            if bool(self.get_parameter("auto_export").value):
                self._export()
            else:
                self._emit("SNAPSHOT_READY", session_dir=str(self.session))
        except Exception as exc:
            self._emit("FAILED", code="D15-E302-SNAPSHOT", detail=str(exc))

    def _export(self) -> Path:
        if self.source is None or self.bgr is None or self.session is None:
            raise RuntimeError("D15-E303-NO_SNAPSHOT")
        self.session.mkdir(parents=True, exist_ok=True)
        rgb_path = self.session / "source_rgb.png"
        if not cv2.imwrite(str(rgb_path), self.bgr):
            raise RuntimeError(f"D15-E304-WRITE_RGB {rgb_path}")
        raw_sha = hashlib.sha256(self.bgr.tobytes(order="C")).hexdigest()
        metadata = {
            "schema": "suturing.operator_mask_session.v1",
            "stamp_ns": stamp_ns(self.source.header),
            "frame_id": str(self.source.header.frame_id),
            "width": int(self.source.width),
            "height": int(self.source.height),
            "encoding": str(self.source.encoding),
            "decoded_bgr_sha256": raw_sha,
            "expected_mask_filename": str(self.get_parameter("mask_filename").value),
            "instructions": "Draw only the needle as white (255) on black (0), keep exact width/height.",
        }
        (self.session / "source.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8"
        )
        self._emit("EXPORTED", session_dir=str(self.session), source_rgb=str(rgb_path), **metadata)
        return rgb_path

    def _publish(self, mask: np.ndarray, source: str) -> tuple[float, Path]:
        if self.source is None or self.bgr is None or self.session is None:
            raise RuntimeError("D15-E303-NO_SNAPSHOT")
        binary, fraction = normalize_mask(
            mask,
            int(self.source.height),
            int(self.source.width),
            float(self.get_parameter("minimum_mask_fraction").value),
            float(self.get_parameter("maximum_mask_fraction").value),
        )
        normalized_path = self.session / "needle_mask_normalized.png"
        overlay_path = self.session / "mask_overlay.png"
        cv2.imwrite(str(normalized_path), binary)
        cv2.imwrite(str(overlay_path), mask_overlay(self.bgr, binary))
        out = Image()
        out.header = self.source.header
        out.height = self.source.height
        out.width = self.source.width
        out.encoding = "mono8"
        out.is_bigendian = 0
        out.step = int(self.source.width)
        out.data = binary.tobytes(order="C")
        self.publisher.publish(out)
        self._emit(
            "PUBLISHED_WAITING_FOR_OPERATOR_OVERLAY_REVIEW",
            source=source,
            stamp_ns=stamp_ns(out.header),
            frame_id=str(out.header.frame_id),
            mask_fraction=fraction,
            normalized_mask=str(normalized_path),
            overlay=str(overlay_path),
            warning="Publication validates the file contract, not semantic correctness. Review overlay before confirming R6.",
        )
        return fraction, overlay_path

    def _export_service(self, _request, response):
        try:
            response.success = True
            response.message = str(self._export())
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _publish_file_service(self, _request, response):
        try:
            if self.session is None:
                raise RuntimeError("D15-E303-NO_SNAPSHOT")
            path = self.session / str(self.get_parameter("mask_filename").value)
            mask = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
            if mask is None:
                raise FileNotFoundError(f"D15-E312-MASK_FILE_NOT_FOUND {path}")
            fraction, overlay = self._publish(mask, source=str(path))
            response.success = True
            response.message = f"published fraction={fraction:.8f} overlay={overlay}"
        except Exception as exc:
            self._emit("FAILED", code="D15-E313-PUBLISH_FILE", detail=str(exc))
            response.success = False
            response.message = str(exc)
        return response

    def _publish_polygon_service(self, _request, response):
        try:
            if self.source is None:
                raise RuntimeError("D15-E303-NO_SNAPSHOT")
            mask = polygon_mask(int(self.source.height), int(self.source.width), self.points)
            fraction, overlay = self._publish(mask, source="interactive_polygon")
            response.success = True
            response.message = f"published fraction={fraction:.8f} overlay={overlay}"
        except Exception as exc:
            self._emit("FAILED", code="D15-E314-PUBLISH_POLYGON", detail=str(exc))
            response.success = False
            response.message = str(exc)
        return response

    def _clear_service(self, _request, response):
        self.points.clear()
        response.success = True
        response.message = "points cleared"
        return response

    def _mouse(self, event, x, y, _flags, _userdata) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((int(x), int(y)))
        elif event == cv2.EVENT_RBUTTONDOWN and self.points:
            self.points.pop()

    def _gui_tick(self) -> None:
        if self.bgr is None:
            return
        try:
            if not self.gui_ready:
                cv2.namedWindow("SurgicAI needle mask", cv2.WINDOW_NORMAL)
                cv2.setMouseCallback("SurgicAI needle mask", self._mouse)
                self.gui_ready = True
            view = self.bgr.copy()
            if self.points:
                points = np.asarray(self.points, dtype=np.int32)
                cv2.polylines(view, [points], len(self.points) >= 3, (0, 255, 255), 2)
                for point in self.points:
                    cv2.circle(view, point, 3, (0, 0, 255), -1)
            cv2.putText(view, "L:add R:undo  p:publish  e:export  c:clear  q:close",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            cv2.imshow("SurgicAI needle mask", view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("p"):
                dummy = Trigger.Response()
                self._publish_polygon_service(None, dummy)
            elif key == ord("e"):
                self._export()
            elif key == ord("c"):
                self.points.clear()
            elif key == ord("u") and self.points:
                self.points.pop()
            elif key == ord("q"):
                cv2.destroyWindow("SurgicAI needle mask")
                self.gui_ready = False
                self.gui_enabled = False
        except Exception as exc:
            self._emit("FAILED", code="D15-E315-GUI", detail=str(exc))
            self.gui_enabled = False


def main() -> None:
    rclpy.init()
    node = OperatorMaskPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.gui_ready:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
