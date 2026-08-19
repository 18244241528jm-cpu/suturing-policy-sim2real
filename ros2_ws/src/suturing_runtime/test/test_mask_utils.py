import numpy as np
import pytest

from suturing_runtime.mask_utils import mask_overlay, normalize_mask, polygon_mask


def test_polygon_and_normalize():
    mask = polygon_mask(100, 120, [(10, 10), (60, 10), (35, 40)])
    normalized, fraction = normalize_mask(mask, 100, 120, 0.001, 0.20)
    assert normalized.dtype == np.uint8
    assert set(np.unique(normalized)) <= {0, 255}
    assert 0.001 < fraction < 0.20


def test_shape_and_coverage_fail_closed():
    with pytest.raises(ValueError, match="D15-E306"):
        normalize_mask(np.zeros((5, 5), np.uint8), 6, 5, 0.0, 1.0)
    with pytest.raises(ValueError, match="D15-E307"):
        normalize_mask(np.zeros((5, 5), np.uint8), 5, 5, 0.1, 1.0)
    with pytest.raises(ValueError, match="D15-E308"):
        normalize_mask(np.full((5, 5), 255, np.uint8), 5, 5, 0.0, 0.9)


def test_overlay_preserves_shape():
    image = np.zeros((20, 30, 3), np.uint8)
    mask = polygon_mask(20, 30, [(2, 2), (10, 2), (5, 8)])
    assert mask_overlay(image, mask).shape == image.shape
