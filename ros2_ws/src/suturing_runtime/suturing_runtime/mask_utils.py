"""Pure image helpers for the operator-approved needle mask boundary."""

from __future__ import annotations

import cv2
import numpy as np


def normalize_mask(mask: np.ndarray, height: int, width: int,
                   minimum_fraction: float, maximum_fraction: float) -> tuple[np.ndarray, float]:
    array = np.asarray(mask)
    if array.ndim == 3:
        if array.shape[2] not in (3, 4):
            raise ValueError(f"D15-E305-MASK_CHANNELS shape={array.shape}")
        array = cv2.cvtColor(array[:, :, :3], cv2.COLOR_BGR2GRAY)
    if array.ndim != 2 or array.shape != (height, width):
        raise ValueError(
            f"D15-E306-MASK_SHAPE expected={(height, width)} actual={array.shape}"
        )
    binary = np.where(array > 127, 255, 0).astype(np.uint8)
    fraction = float(np.count_nonzero(binary)) / float(height * width)
    if fraction < minimum_fraction:
        raise ValueError(
            f"D15-E307-MASK_TOO_SMALL fraction={fraction:.8f} minimum={minimum_fraction:.8f}"
        )
    if fraction > maximum_fraction:
        raise ValueError(
            f"D15-E308-MASK_TOO_LARGE fraction={fraction:.8f} maximum={maximum_fraction:.8f}"
        )
    return binary, fraction


def polygon_mask(height: int, width: int, points: list[tuple[int, int]]) -> np.ndarray:
    if len(points) < 3:
        raise ValueError(f"D15-E309-POLYGON_POINTS count={len(points)}")
    polygon = np.asarray(points, dtype=np.int32)
    if np.any(polygon[:, 0] < 0) or np.any(polygon[:, 0] >= width):
        raise ValueError("D15-E310-POLYGON_X_OUT_OF_RANGE")
    if np.any(polygon[:, 1] < 0) or np.any(polygon[:, 1] >= height):
        raise ValueError("D15-E311-POLYGON_Y_OUT_OF_RANGE")
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [polygon], 255)
    return mask


def mask_overlay(bgr: np.ndarray, mask: np.ndarray) -> np.ndarray:
    overlay = np.asarray(bgr).copy()
    green = np.zeros_like(overlay)
    green[:, :, 1] = 255
    selected = mask > 0
    overlay[selected] = (0.55 * overlay[selected] + 0.45 * green[selected]).astype(np.uint8)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, (0, 255, 255), 2)
    return overlay
