# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Orbbec RGB-D camera support through an OrbbecSDK v2 C++ bridge."""

import logging
import struct
import subprocess
import time
from pathlib import Path
from threading import Event, Lock, Thread
from typing import Any

import cv2  # type: ignore  # TODO: add type stubs for OpenCV
import numpy as np
from numpy.typing import NDArray  # type: ignore  # TODO: add type stubs for numpy.typing

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..camera import Camera
from ..configs import ColorMode
from ..utils import get_cv2_rotation
from .configuration_orbbec import OrbbecCameraConfig

logger = logging.getLogger(__name__)

_PACKET_MAGIC = b"OBLR"
_HEADER_STRUCT = struct.Struct("<4sQIIII")


class OrbbecCamera(Camera):
    """Read RGB and depth frames from an Orbbec camera through a C++ bridge process."""

    def __init__(self, config: OrbbecCameraConfig):
        super().__init__(config)
        self.config = config
        self.serial_number = config.serial_number
        self.color_mode = config.color_mode
        self.warmup_s = config.warmup_s
        self.use_depth = config.use_depth
        self.record_color = config.record_color
        self.record_depth = config.record_depth
        self.depth_dtype = config.depth_dtype
        self.record_depth_viz = config.record_depth_viz
        self.rotation: int | None = get_cv2_rotation(config.rotation)

        self.process: subprocess.Popen[bytes] | None = None
        self.thread: Thread | None = None
        self.stop_event: Event | None = None
        self.frame_lock: Lock = Lock()
        self.latest_color_frame: NDArray[Any] | None = None
        self.latest_depth_frame: NDArray[Any] | None = None
        self.latest_timestamp: float | None = None
        self.new_frame_event: Event = Event()

        if self.height and self.width:
            self.capture_width, self.capture_height = self.width, self.height
            if self.rotation in [cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE]:
                self.capture_width, self.capture_height = self.height, self.width

    def __str__(self) -> str:
        ident = self.serial_number or self.config.bridge_binary or "default"
        return f"{self.__class__.__name__}({ident})"

    @property
    def is_connected(self) -> bool:
        return self.process is not None and self.process.poll() is None

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        """Camera discovery is delegated to the Orbbec bridge binary."""
        return []

    def connect(self, warmup: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")

        if self.config.bridge_binary is None:
            raise RuntimeError(
                "OrbbecCamera requires `bridge_binary` pointing to an executable built from "
                "`lerobot/cameras/orbbec/cpp/orbbec_rgbd_bridge.cpp`."
            )

        bridge = Path(self.config.bridge_binary)
        if not bridge.exists():
            raise FileNotFoundError(f"Orbbec bridge binary not found: {bridge}")

        if self.width is None or self.height is None or self.fps is None:
            raise ValueError("OrbbecCamera requires `width`, `height`, and `fps`.")

        self._check_enhanced_depth_license()

        command = [
            str(bridge),
            "--width",
            str(self.capture_width),
            "--height",
            str(self.capture_height),
            "--fps",
            str(self.fps),
        ]
        if self.serial_number:
            command.extend(["--serial", self.serial_number])
        if self.config.align_depth_to_color:
            command.append("--align-depth-to-color")
        if self.config.use_enhanced_depth_filter:
            command.append("--enhanced-depth-filter")
            command.extend(["--enhanced-depth-filter-name", self.config.enhanced_depth_filter_name])
            command.extend(["--enhanced-depth-model", str(self.config.enhanced_depth_model_path)])
            command.extend(["--enhanced-depth-confidence-key", self.config.enhanced_depth_confidence_key])
            command.extend(
                [
                    "--enhanced-depth-confidence-threshold",
                    str(self.config.enhanced_depth_confidence_threshold),
                ]
            )

        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=None, bufsize=0)
        self._start_read_thread()

        if warmup and self.warmup_s > 0:
            start_time = time.time()
            while time.time() - start_time < self.warmup_s:
                self._wait_for_expected_frames(timeout_ms=self.warmup_s * 1000)
                time.sleep(0.1)

        logger.info(f"{self} connected.")

    def _check_enhanced_depth_license(self) -> None:
        if not self.config.use_enhanced_depth_filter:
            return

        if not self.config.enhanced_depth_license_check_command:
            logger.warning(
                "%s is starting with EnhancedDepthFilter enabled but no "
                "`enhanced_depth_license_check_command` was configured. The OrbbecSDK "
                "filter initialization must still validate the device license in the C++ bridge.",
                self,
            )
            return

        command = [
            token.format(
                serial_number=self.serial_number or "",
                model_path=str(self.config.enhanced_depth_model_path or ""),
            )
            for token in self.config.enhanced_depth_license_check_command
        ]
        try:
            subprocess.run(
                command,
                check=True,
                timeout=self.config.enhanced_depth_license_check_timeout_s,
            )
        except subprocess.TimeoutExpired as exc:
            raise TimeoutError(
                f"EnhancedDepthFilter license check timed out after "
                f"{self.config.enhanced_depth_license_check_timeout_s}s: {command}"
            ) from exc
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(f"EnhancedDepthFilter license check failed with exit code {exc.returncode}.") from exc

    def _wait_for_expected_frames(self, timeout_ms: float) -> None:
        if not self.new_frame_event.wait(timeout=timeout_ms / 1000.0):
            raise TimeoutError(f"Timed out waiting for frames from {self}.")
        with self.frame_lock:
            has_color = self.latest_color_frame is not None
            has_depth = self.latest_depth_frame is not None
            self.new_frame_event.clear()
        if self.record_color and not has_color:
            raise ConnectionError(f"{self} did not produce a color frame during warmup.")
        if self.use_depth and self.record_depth and not has_depth:
            raise ConnectionError(f"{self} did not produce a depth frame during warmup.")
        if self.record_depth_viz and not has_depth:
            raise ConnectionError(f"{self} did not produce a depth frame for depth visualization during warmup.")

    def _read_exact(self, size: int) -> bytes:
        if self.process is None or self.process.stdout is None:
            raise DeviceNotConnectedError(f"{self} bridge stdout is not available.")

        chunks = []
        remaining = size
        while remaining > 0:
            chunk = self.process.stdout.read(remaining)
            if not chunk:
                raise RuntimeError(f"{self} bridge ended while reading a frame packet.")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _read_from_hardware(self) -> tuple[NDArray[Any] | None, NDArray[Any] | None]:
        header = self._read_exact(_HEADER_STRUCT.size)
        magic, _timestamp_ns, width, height, color_size, depth_size = _HEADER_STRUCT.unpack(header)
        if magic != _PACKET_MAGIC:
            raise RuntimeError(f"{self} bridge protocol magic mismatch: {magic!r}.")

        color = None
        if color_size > 0:
            color_bytes = self._read_exact(color_size)
            color = np.frombuffer(color_bytes, dtype=np.uint8).reshape(height, width, 3).copy()

        depth = None
        if depth_size > 0:
            depth_bytes = self._read_exact(depth_size)
            depth_dtype = np.uint16 if self.depth_dtype == "uint16" else np.float32
            depth = np.frombuffer(depth_bytes, dtype=depth_dtype).reshape(height, width).copy()

        return color, depth

    def _postprocess_color(self, image: NDArray[Any]) -> NDArray[Any]:
        h, w, c = image.shape
        if h != self.capture_height or w != self.capture_width or c != 3:
            raise RuntimeError(
                f"{self} color frame shape={image.shape} does not match "
                f"({self.capture_height}, {self.capture_width}, 3)."
            )

        processed_image = image
        if self.color_mode == ColorMode.BGR:
            processed_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if self.rotation in [cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE, cv2.ROTATE_180]:
            processed_image = cv2.rotate(processed_image, self.rotation)
        return processed_image

    def _postprocess_depth(self, depth: NDArray[Any]) -> NDArray[Any]:
        h, w = depth.shape
        if h != self.capture_height or w != self.capture_width:
            raise RuntimeError(
                f"{self} depth frame shape={depth.shape} does not match "
                f"({self.capture_height}, {self.capture_width})."
            )

        processed_depth = depth
        if self.rotation in [cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE, cv2.ROTATE_180]:
            processed_depth = cv2.rotate(processed_depth, self.rotation)
        return processed_depth.astype(np.dtype(self.depth_dtype), copy=False)

    def _read_loop(self) -> None:
        if self.stop_event is None:
            raise RuntimeError(f"{self}: stop_event is not initialized before starting read loop.")

        failure_count = 0
        while not self.stop_event.is_set():
            try:
                color_frame, depth_frame = self._read_from_hardware()
                processed_color = self._postprocess_color(color_frame) if color_frame is not None else None
                processed_depth = self._postprocess_depth(depth_frame) if depth_frame is not None else None
                capture_time = time.perf_counter()

                with self.frame_lock:
                    self.latest_color_frame = processed_color
                    self.latest_depth_frame = processed_depth
                    self.latest_timestamp = capture_time
                self.new_frame_event.set()
                failure_count = 0
            except DeviceNotConnectedError:
                break
            except Exception as e:
                if failure_count <= 10:
                    failure_count += 1
                    logger.warning(f"Error reading frame in background thread for {self}: {e}")
                else:
                    raise RuntimeError(f"{self} exceeded maximum consecutive read failures.") from e

    def _start_read_thread(self) -> None:
        self._stop_read_thread()
        self.stop_event = Event()
        self.thread = Thread(target=self._read_loop, args=(), name=f"{self}_read_loop")
        self.thread.daemon = True
        self.thread.start()

    def _stop_read_thread(self) -> None:
        if self.stop_event is not None:
            self.stop_event.set()

        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=2.0)

        self.thread = None
        self.stop_event = None

        with self.frame_lock:
            self.latest_color_frame = None
            self.latest_depth_frame = None
            self.latest_timestamp = None
            self.new_frame_event.clear()

    def read(self, color_mode: ColorMode | None = None) -> NDArray[Any]:
        if color_mode is not None:
            logger.warning(f"{self} read() color_mode parameter is deprecated and will be ignored.")
        self.new_frame_event.clear()
        return self.async_read(timeout_ms=10000)

    def async_read(self, timeout_ms: float = 200) -> NDArray[Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        if self.thread is None or not self.thread.is_alive():
            raise RuntimeError(f"{self} read thread is not running.")
        if not self.new_frame_event.wait(timeout=timeout_ms / 1000.0):
            raise TimeoutError(f"Timed out waiting for color frame from {self} after {timeout_ms} ms.")
        with self.frame_lock:
            frame = self.latest_color_frame
            self.new_frame_event.clear()
        if frame is None:
            raise RuntimeError(
                f"{self} has no color frame. Set `record_color=True` or use async_read_depth()."
            )
        return frame

    def async_read_depth(self, timeout_ms: float = 200) -> NDArray[Any]:
        if not self.use_depth:
            raise RuntimeError(f"Depth stream is not enabled for {self}.")
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        if self.thread is None or not self.thread.is_alive():
            raise RuntimeError(f"{self} read thread is not running.")
        if not self.new_frame_event.wait(timeout=timeout_ms / 1000.0):
            raise TimeoutError(f"Timed out waiting for depth frame from {self} after {timeout_ms} ms.")
        with self.frame_lock:
            frame = self.latest_depth_frame
            self.new_frame_event.clear()
        if frame is None:
            raise RuntimeError(f"{self} has no depth frame.")
        return frame

    def read_depth(self, timeout_ms: int = 200) -> NDArray[Any]:
        return self.async_read_depth(timeout_ms=timeout_ms)

    def read_latest_depth(self, max_age_ms: int = 1000) -> NDArray[Any]:
        if not self.use_depth:
            raise RuntimeError(f"Depth stream is not enabled for {self}.")
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        with self.frame_lock:
            frame = self.latest_depth_frame
            timestamp = self.latest_timestamp
        if frame is None or timestamp is None:
            raise RuntimeError(f"{self} has not captured any depth frames yet.")
        age_ms = (time.perf_counter() - timestamp) * 1e3
        if age_ms > max_age_ms:
            raise TimeoutError(f"{self} latest depth frame is too old: {age_ms:.1f} ms.")
        return frame

    def _depth_to_viz(self, depth: NDArray[Any]) -> NDArray[Any]:
        clipped = np.clip(depth, self.config.depth_viz_min_mm, self.config.depth_viz_max_mm)
        depth_norm = ((clipped - self.config.depth_viz_min_mm) * 255.0) / (
            self.config.depth_viz_max_mm - self.config.depth_viz_min_mm
        )
        depth_u8 = depth_norm.astype(np.uint8)
        invalid = depth == 0
        depth_u8[invalid] = 0
        viz_bgr = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
        viz_bgr[invalid] = 0
        if self.color_mode == ColorMode.RGB:
            return cv2.cvtColor(viz_bgr, cv2.COLOR_BGR2RGB)
        return viz_bgr

    def read_latest_depth_viz(self, max_age_ms: int = 1000) -> NDArray[Any]:
        return self._depth_to_viz(self.read_latest_depth(max_age_ms=max_age_ms))

    def async_read_depth_viz(self, timeout_ms: float = 200) -> NDArray[Any]:
        return self._depth_to_viz(self.async_read_depth(timeout_ms=timeout_ms))

    def read_latest(self, max_age_ms: int = 1000) -> NDArray[Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        with self.frame_lock:
            frame = self.latest_color_frame
            timestamp = self.latest_timestamp
        if frame is None or timestamp is None:
            raise RuntimeError(f"{self} has not captured any color frames yet.")
        age_ms = (time.perf_counter() - timestamp) * 1e3
        if age_ms > max_age_ms:
            raise TimeoutError(f"{self} latest frame is too old: {age_ms:.1f} ms.")
        return frame

    def disconnect(self) -> None:
        if not self.is_connected and self.thread is None:
            raise DeviceNotConnectedError(f"{self} not connected.")

        if self.thread is not None:
            self._stop_read_thread()

        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.0)
            self.process = None

        logger.info(f"{self} disconnected.")
