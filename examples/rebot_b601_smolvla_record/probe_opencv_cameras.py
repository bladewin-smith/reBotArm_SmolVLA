#!/usr/bin/env python

import argparse
import time
from pathlib import Path

import cv2


#!/usr/bin/env python

import argparse
import time
from pathlib import Path

import cv2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Probe OpenCV /dev/video* camera streams.")
    parser.add_argument("--devices", nargs="*", default=None, help="Devices or indexes, e.g. /dev/video0 2.")
    parser.add_argument("--max-index", type=int, default=10, help="Scan /dev/video0..N when --devices is omitted.")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--fourcc", default="MJPG", help="FourCC, e.g. MJPG/YUYV. Use '' to skip.")
    parser.add_argument("--preview", action="store_true", help="Show live preview windows.")
    parser.add_argument("--seconds", type=float, default=3.0, help="Preview/read duration per camera.")
    return parser.parse_args()


def device_value(token: str) -> str | int:
    if token.isdigit():
        return int(token)
    return token


def open_capture(device: str | int, args: argparse.Namespace) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    if args.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    return cap


def probe_one(label: str, device: str | int, args: argparse.Namespace) -> None:
    cap = open_capture(device, args)
    if not cap.isOpened():
        print(f"{label}: CLOSED")
        return

    ok, frame = cap.read()
    if not ok or frame is None:
        print(f"{label}: OPENED but no frame")
        cap.release()
        return

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc = "".join(chr((fourcc_int >> (8 * i)) & 0xFF) for i in range(4))
    print(f"{label}: OK shape={frame.shape} actual={actual_w}x{actual_h}@{actual_fps:.1f} fourcc={fourcc!r}")

    if args.preview:
        start = time.perf_counter()
        frames = 0
        while time.perf_counter() - start < args.seconds:
            ok, frame = cap.read()
            if not ok or frame is None:
                break
            frames += 1
            cv2.putText(frame, label, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow(label, frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
        elapsed = max(time.perf_counter() - start, 1e-6)
        print(f"{label}: preview fps={frames / elapsed:.1f}")

    cap.release()
    cv2.destroyWindow(label) if args.preview else None


def main() -> None:
    args = parse_args()
    if args.devices is None:
        devices = [f"/dev/video{i}" for i in range(args.max_index + 1) if Path(f"/dev/video{i}").exists()]
    else:
        devices = args.devices

    if not devices:
        print("No devices found. Try --devices /dev/video0 /dev/video1")
        return

    print(f"Requested: {args.width}x{args.height}@{args.fps}, fourcc={args.fourcc!r}")
    for token in devices:
        probe_one(str(token), device_value(str(token)), args)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
