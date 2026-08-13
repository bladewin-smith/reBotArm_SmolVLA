#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");

import argparse
import time
from pathlib import Path

from lerobot.cameras.orbbec import OrbbecCamera, OrbbecCameraConfig


def make_camera(
    *,
    serial: str,
    bridge: Path,
    width: int,
    height: int,
    fps: int,
    warmup_s: int,
    timeout_ms: int,
    depth: bool,
    enhanced_depth: bool = False,
    enhanced_depth_model: Path | None = None,
    align_depth_to_color: bool = True,
    align_depth_to_color_mode: str = "sw",
) -> OrbbecCamera:
    return OrbbecCamera(
        OrbbecCameraConfig(
            serial_number=serial,
            bridge_binary=bridge,
            width=width,
            height=height,
            fps=fps,
            warmup_s=warmup_s,
            timeout_ms=timeout_ms,
            use_depth=depth,
            record_depth=depth,
            record_depth_viz=False,
            align_depth_to_color=align_depth_to_color,
            align_depth_to_color_mode=align_depth_to_color_mode,
            use_enhanced_depth_filter=enhanced_depth,
            enhanced_depth_model_path=enhanced_depth_model,
        )
    )


def run_single(args: argparse.Namespace) -> None:
    camera = make_camera(
        serial=args.serial,
        bridge=Path(args.bridge),
        width=args.width,
        height=args.height,
        fps=args.fps,
        warmup_s=args.warmup_s,
        timeout_ms=args.timeout_ms,
        depth=args.depth,
        enhanced_depth=args.enhanced_depth,
        enhanced_depth_model=Path(args.enhanced_depth_model) if args.enhanced_depth_model else None,
        align_depth_to_color=args.align_depth_to_color,
        align_depth_to_color_mode=args.align_depth_to_color_mode,
    )

    print(
        "Starting Orbbec debug\n"
        f"  serial: {args.serial}\n"
        f"  bridge: {args.bridge}\n"
        f"  size/fps: {args.width}x{args.height}@{args.fps}\n"
        f"  depth: {args.depth}\n"
        f"  enhanced depth: {args.enhanced_depth}\n"
        f"  warmup/timeout: {args.warmup_s}s / {args.timeout_ms}ms"
    )

    try:
        camera.connect()
        last_t = time.perf_counter()
        for idx in range(args.frames):
            start = time.perf_counter()
            color = camera.async_read()
            depth_text = ""
            if args.depth:
                depth = camera.read_latest_depth()
                depth_text = f", depth_shape={depth.shape}, depth_dtype={depth.dtype}"
            now = time.perf_counter()
            print(
                f"[{idx + 1:03d}] color_shape={color.shape}, color_dtype={color.dtype}"
                f"{depth_text}, read_ms={(now - start) * 1e3:.1f}, dt_ms={(now - last_t) * 1e3:.1f}"
            )
            last_t = now
    finally:
        if camera.is_connected or camera.thread is not None:
            camera.disconnect()


def run_pair(args: argparse.Namespace) -> None:
    bridge = Path(args.bridge)
    wrist = make_camera(
        serial=args.wrist_serial,
        bridge=bridge,
        width=args.width,
        height=args.height,
        fps=args.fps,
        warmup_s=args.wrist_warmup_s,
        timeout_ms=args.wrist_timeout_ms,
        depth=False,
    )
    top = make_camera(
        serial=args.top_serial,
        bridge=bridge,
        width=args.width,
        height=args.height,
        fps=args.fps,
        warmup_s=args.top_warmup_s,
        timeout_ms=args.top_timeout_ms,
        depth=True,
        enhanced_depth=args.enhanced_depth,
        enhanced_depth_model=Path(args.enhanced_depth_model) if args.enhanced_depth_model else None,
        align_depth_to_color=True,
        align_depth_to_color_mode=args.align_depth_to_color_mode,
    )
    cameras = [("wrist", wrist), ("top", top)]

    print(
        "Starting dual Orbbec debug\n"
        f"  wrist serial: {args.wrist_serial}\n"
        f"  top serial: {args.top_serial}\n"
        f"  bridge: {args.bridge}\n"
        f"  size/fps: {args.width}x{args.height}@{args.fps}\n"
        f"  top enhanced depth: {args.enhanced_depth}"
    )

    try:
        for name, camera in cameras:
            print(f"Connecting {name}...")
            camera.connect()

        last_t = time.perf_counter()
        for idx in range(args.frames):
            start = time.perf_counter()
            wrist_color = wrist.async_read()
            top_color = top.async_read()
            top_depth = top.read_latest_depth()
            now = time.perf_counter()
            print(
                f"[{idx + 1:03d}] wrist={wrist_color.shape}/{wrist_color.dtype}, "
                f"top={top_color.shape}/{top_color.dtype}, depth={top_depth.shape}/{top_depth.dtype}, "
                f"read_ms={(now - start) * 1e3:.1f}, dt_ms={(now - last_t) * 1e3:.1f}"
            )
            last_t = now
    finally:
        for _, camera in reversed(cameras):
            if camera.is_connected or camera.thread is not None:
                camera.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser(description="Debug Orbbec cameras through the LeRobot C++ bridge.")
    parser.add_argument("--serial", help="OrbbecSDK serial number for single-camera mode, not a /dev/video* path.")
    parser.add_argument(
        "--bridge",
        default="src/lerobot/cameras/orbbec/cpp/build/orbbec_rgbd_bridge",
        help="Path to the built orbbec_rgbd_bridge binary.",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-s", type=int, default=10)
    parser.add_argument("--timeout-ms", type=int, default=10000)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--depth", action="store_true", help="Enable and read the depth stream too.")
    parser.add_argument("--enhanced-depth", action="store_true", help="Enable EnhancedDepthFilter.")
    parser.add_argument("--enhanced-depth-model", help="Path to model.sm4 when --enhanced-depth is enabled.")
    parser.add_argument("--align-depth-to-color", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--align-depth-to-color-mode", default="sw", choices=["sw", "software", "hw", "hardware"])
    parser.add_argument("--pair", action="store_true", help="Open wrist and top cameras together.")
    parser.add_argument("--wrist-serial", help="Wrist Gemini 305 serial for --pair.")
    parser.add_argument("--top-serial", help="Top Gemini 335L serial for --pair.")
    parser.add_argument("--wrist-warmup-s", type=int, default=15)
    parser.add_argument("--wrist-timeout-ms", type=int, default=15000)
    parser.add_argument("--top-warmup-s", type=int, default=20)
    parser.add_argument("--top-timeout-ms", type=int, default=20000)
    args = parser.parse_args()

    if args.enhanced_depth and not args.enhanced_depth_model:
        parser.error("--enhanced-depth-model is required with --enhanced-depth.")
    if args.pair:
        if not args.wrist_serial or not args.top_serial:
            parser.error("--wrist-serial and --top-serial are required with --pair.")
        run_pair(args)
    else:
        if not args.serial:
            parser.error("--serial is required unless --pair is used.")
        run_single(args)


if __name__ == "__main__":
    main()
