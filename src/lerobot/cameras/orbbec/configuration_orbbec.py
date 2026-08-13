# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from dataclasses import dataclass, field
from pathlib import Path

from ..configs import CameraConfig, ColorMode, Cv2Rotation


@CameraConfig.register_subclass("orbbec")
@dataclass
class OrbbecCameraConfig(CameraConfig):
    """Configuration for an Orbbec RGB-D camera read through the C++ SDK bridge.

    The Python camera class expects an external bridge executable built against
    OrbbecSDK v2 C++. The bridge streams synchronized RGB and depth frames using
    the small binary protocol documented in ``cpp/orbbec_rgbd_bridge.cpp``.
    """

    serial_number: str | None = None
    bridge_binary: Path | None = None
    color_mode: ColorMode = ColorMode.RGB
    rotation: Cv2Rotation = Cv2Rotation.NO_ROTATION
    warmup_s: int = 1
    timeout_ms: int = 1000

    # Recording controls. ``record_color=False`` is useful for a top camera that
    # should only add a depth map to the dataset.
    record_color: bool = True
    use_depth: bool = True
    record_depth: bool = True
    depth_key: str | None = None
    depth_dtype: str = "uint16"
    record_depth_viz: bool = False
    depth_viz_key: str | None = None
    depth_viz_min_mm: int = 250
    depth_viz_max_mm: int = 2500

    # Optional switches for the C++ bridge.
    align_depth_to_color: bool = True
    align_depth_to_color_mode: str = "sw"

    # Orbbec LingBot EnhancedDepthFilter. This requires a supported Gemini 330
    # series camera, Jetson Linux ARM64, CUDA/TensorRT runtimes, a valid
    # LingBot-Depth license, and model.sm4/extension libraries from the same
    # OrbbecSDK release.
    use_enhanced_depth_filter: bool = False
    enhanced_depth_filter_name: str = "EnhancedDepthFilter"
    enhanced_depth_model_path: Path | None = None
    enhanced_depth_confidence_key: str = "confidence_threshold"
    enhanced_depth_confidence_threshold: int = 51
    enhanced_depth_license_check_command: list[str] = field(default_factory=list)
    enhanced_depth_license_check_timeout_s: float = 15.0

    # Backward-compatible aliases used by the first Orbbec bridge draft.
    use_lingbo_filter: bool = False
    lingbo_model_path: Path | None = None

    def __post_init__(self) -> None:
        if self.color_mode not in (ColorMode.RGB, ColorMode.BGR):
            raise ValueError(
                f"`color_mode` is expected to be {ColorMode.RGB.value} or {ColorMode.BGR.value}, "
                f"but {self.color_mode} is provided."
            )

        valid_rotations = (
            Cv2Rotation.NO_ROTATION,
            Cv2Rotation.ROTATE_90,
            Cv2Rotation.ROTATE_180,
            Cv2Rotation.ROTATE_270,
        )
        if self.rotation not in valid_rotations:
            raise ValueError(
                f"`rotation` is expected to be in "
                f"{valid_rotations}, "
                f"but {self.rotation} is provided."
            )

        values = (self.fps, self.width, self.height)
        if any(v is not None for v in values) and any(v is None for v in values):
            raise ValueError("For `fps`, `width` and `height`, either all of them need to be set, or none.")

        if self.depth_dtype != "uint16":
            raise ValueError("`depth_dtype` must be 'uint16' for the current Orbbec C++ bridge.")

        if self.record_depth_viz and not self.use_depth:
            raise ValueError("`use_depth=True` is required when `record_depth_viz=True`.")

        if self.depth_viz_min_mm >= self.depth_viz_max_mm:
            raise ValueError("`depth_viz_min_mm` must be smaller than `depth_viz_max_mm`.")

        if self.align_depth_to_color_mode not in {"sw", "software", "hw", "hardware"}:
            raise ValueError("`align_depth_to_color_mode` must be 'sw', 'software', 'hw', or 'hardware'.")

        if self.use_lingbo_filter:
            self.use_enhanced_depth_filter = True
        if self.lingbo_model_path is not None and self.enhanced_depth_model_path is None:
            self.enhanced_depth_model_path = self.lingbo_model_path

        if self.use_enhanced_depth_filter and self.enhanced_depth_model_path is None:
            raise ValueError(
                "`enhanced_depth_model_path` is required when `use_enhanced_depth_filter=True`."
            )

        if self.use_enhanced_depth_filter and not self.enhanced_depth_filter_name:
            raise ValueError("`enhanced_depth_filter_name` cannot be empty.")

        if self.use_enhanced_depth_filter and not self.enhanced_depth_confidence_key:
            raise ValueError("`enhanced_depth_confidence_key` cannot be empty.")

        if not 0 <= self.enhanced_depth_confidence_threshold <= 255:
            raise ValueError("`enhanced_depth_confidence_threshold` must be in [0, 255].")

        if self.enhanced_depth_license_check_timeout_s <= 0:
            raise ValueError("`enhanced_depth_license_check_timeout_s` must be positive.")

        if self.use_lingbo_filter and self.enhanced_depth_model_path is None:
            raise ValueError("`lingbo_model_path` is required when `use_lingbo_filter=True`.")
