#!/usr/bin/env python

import numpy as np
import pytest

from examples.rebot_b601_smolvla_record.resample_b601_dataset import normalize_feature_value


def test_normalize_video_frame_from_chw_to_metadata_hwc() -> None:
    frame = np.zeros((3, 480, 640), dtype=np.float32)
    feature = {"dtype": "video", "shape": [480, 640, 3], "names": None}

    normalized = normalize_feature_value("observation.images.wrist", frame, feature)

    assert normalized.shape == (480, 640, 3)
    assert normalized.dtype == np.float32
    assert normalized.flags.c_contiguous


def test_normalize_uint16_depth_loaded_as_int64() -> None:
    depth = np.array([[0, 1000], [65000, 65535]], dtype=np.int64)
    feature = {"dtype": "uint16", "shape": [2, 2], "names": ["height", "width"]}

    normalized = normalize_feature_value("observation.depths.top", depth, feature)

    assert normalized.dtype == np.uint16
    np.testing.assert_array_equal(normalized, depth)


def test_normalize_uint16_depth_rejects_overflow() -> None:
    depth = np.array([[65536]], dtype=np.int64)
    feature = {"dtype": "uint16", "shape": [1, 1], "names": ["height", "width"]}

    with pytest.raises(OverflowError, match="do not fit"):
        normalize_feature_value("observation.depths.top", depth, feature)
