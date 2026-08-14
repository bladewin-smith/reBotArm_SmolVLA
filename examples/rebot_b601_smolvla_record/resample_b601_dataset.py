#!/usr/bin/env python

"""Downsample a local LeRobot dataset while preserving cross-modal alignment."""

from __future__ import annotations

import argparse
import copy
import os
from pathlib import Path

import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import DEFAULT_FEATURES
from lerobot.datasets.video_utils import decode_video_frames


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create a lower-FPS copy of a local LeRobot dataset. The same source "
            "frame indices are used for videos, depth, robot state, and actions."
        )
    )
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--source-repo-id", required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--output-repo-id", required=True)
    parser.add_argument("--target-fps", type=int, required=True)
    parser.add_argument("--video-backend", default="pyav", choices=("pyav", "video_reader", "torchcodec"))
    parser.add_argument("--vcodec", default="h264", choices=("h264", "hevc", "libsvtav1"))
    parser.add_argument(
        "--decode-batch-size",
        type=int,
        default=16,
        help="Frames decoded per camera at once. Lower this if RAM is limited.",
    )
    parser.add_argument("--image-writer-processes", type=int, default=0)
    parser.add_argument("--image-writer-threads", type=int, default=1)
    parser.add_argument("--video-encoding-threads", type=int, default=1)
    return parser.parse_args()


def nearest_source_offsets(source_length: int, source_fps: int, target_fps: int) -> list[int]:
    """Return unique nearest-frame offsets for regular downsampling."""
    if source_length <= 0:
        raise ValueError("An episode must contain at least one frame.")
    if source_fps <= 0 or target_fps <= 0:
        raise ValueError("FPS values must be positive.")
    if target_fps > source_fps:
        raise ValueError(
            f"Upsampling from {source_fps} FPS to {target_fps} FPS is not supported because it "
            "would duplicate observations and actions."
        )

    target_length = ((source_length - 1) * target_fps) // source_fps + 1
    offsets = [
        (2 * target_index * source_fps + target_fps) // (2 * target_fps)
        for target_index in range(target_length)
    ]
    return [min(offset, source_length - 1) for offset in offsets]


def scalar_int(value) -> int:
    return int(value.item()) if hasattr(value, "item") else int(value)


def to_numpy(value) -> np.ndarray:
    if isinstance(value, np.ndarray):
        return value
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def normalize_feature_value(name: str, value, feature: dict) -> np.ndarray | str:
    """Restore a decoded/HF value to the shape and dtype declared in metadata."""
    feature_dtype = feature["dtype"]
    if feature_dtype == "string":
        if not isinstance(value, str):
            raise TypeError(f"Feature {name!r} must be a string, got {type(value).__name__}.")
        return value

    array = to_numpy(value)
    expected_shape = tuple(feature["shape"])
    if feature_dtype in {"image", "video"}:
        if array.shape != expected_shape and array.ndim == 3:
            channel_first_to_last = np.transpose(array, (1, 2, 0))
            channel_last_to_first = np.transpose(array, (2, 0, 1))
            if channel_first_to_last.shape == expected_shape:
                array = channel_first_to_last
            elif channel_last_to_first.shape == expected_shape:
                array = channel_last_to_first
        if array.shape != expected_shape:
            raise ValueError(
                f"Feature {name!r} decoded with shape {array.shape}, but metadata expects "
                f"{expected_shape}."
            )
        return np.ascontiguousarray(array)

    if array.shape != expected_shape:
        raise ValueError(
            f"Feature {name!r} decoded with shape {array.shape}, but metadata expects "
            f"{expected_shape}."
        )

    expected_dtype = np.dtype(feature_dtype)
    if np.issubdtype(expected_dtype, np.integer) and array.size:
        bounds = np.iinfo(expected_dtype)
        minimum = np.min(array)
        maximum = np.max(array)
        if minimum < bounds.min or maximum > bounds.max:
            raise OverflowError(
                f"Feature {name!r} values [{minimum}, {maximum}] do not fit in {expected_dtype}."
            )
    return array.astype(expected_dtype, copy=False)


def main() -> None:
    args = parse_args()
    if args.decode_batch_size < 1:
        raise ValueError("--decode-batch-size must be at least 1.")
    if args.video_encoding_threads < 1:
        raise ValueError("--video-encoding-threads must be at least 1.")
    if not (args.source_root / "meta" / "info.json").is_file():
        raise FileNotFoundError(f"Source is not a local LeRobot dataset: {args.source_root}")
    if args.output_root.exists():
        raise FileExistsError(
            f"Output root already exists: {args.output_root}. Choose a new path or move/delete it manually."
        )

    os.environ["LEROBOT_VIDEO_ENCODING_THREADS"] = str(args.video_encoding_threads)
    source = LeRobotDataset(
        args.source_repo_id,
        root=args.source_root,
        video_backend=args.video_backend,
    )
    if args.target_fps == source.fps:
        raise ValueError(
            f"Source already uses {source.fps} FPS; use the source dataset directly instead of resampling it."
        )
    if args.target_fps > source.fps:
        raise ValueError(
            f"Target {args.target_fps} FPS is higher than source {source.fps} FPS. "
            "Choose the lowest FPS shared by the batches (normally 10 FPS for this project)."
        )

    features = copy.deepcopy(source.meta.features)
    for feature in features.values():
        if feature["dtype"] == "video":
            # The encoded output has a new FPS, so video metadata must be regenerated.
            feature.pop("info", None)

    output = LeRobotDataset.create(
        repo_id=args.output_repo_id,
        root=args.output_root,
        fps=args.target_fps,
        robot_type=source.meta.robot_type,
        features=features,
        use_videos=bool(source.meta.video_keys),
        image_writer_processes=args.image_writer_processes,
        image_writer_threads=args.image_writer_threads,
        video_backend=args.video_backend,
        vcodec=args.vcodec,
    )

    video_keys = list(source.meta.video_keys)
    frame_feature_keys = [
        key for key in source.meta.features if key not in DEFAULT_FEATURES and key not in video_keys
    ]
    total_output_frames = 0

    print(
        f"Resampling {args.source_root} from {source.fps} FPS to {args.target_fps} FPS "
        f"({source.meta.total_episodes} episodes)"
    )
    try:
        for episode_index in range(source.meta.total_episodes):
            episode = source.meta.episodes[episode_index]
            start = int(episode["dataset_from_index"])
            end = int(episode["dataset_to_index"])
            offsets = nearest_source_offsets(end - start, source.fps, args.target_fps)

            for chunk_start in range(0, len(offsets), args.decode_batch_size):
                offset_batch = offsets[chunk_start : chunk_start + args.decode_batch_size]
                source_items = [source.hf_dataset[start + offset] for offset in offset_batch]
                source_timestamps = [float(item["timestamp"].item()) for item in source_items]

                video_batches = {}
                for video_key in video_keys:
                    from_timestamp = float(episode[f"videos/{video_key}/from_timestamp"])
                    query_timestamps = [from_timestamp + timestamp for timestamp in source_timestamps]
                    video_path = source.root / source.meta.get_video_file_path(episode_index, video_key)
                    video_batches[video_key] = decode_video_frames(
                        video_path,
                        query_timestamps,
                        source.tolerance_s,
                        args.video_backend,
                )

                for batch_index, source_item in enumerate(source_items):
                    frame = {
                        key: normalize_feature_value(key, source_item[key], features[key])
                        for key in frame_feature_keys
                    }
                    for video_key in video_keys:
                        frame[video_key] = normalize_feature_value(
                            video_key,
                            video_batches[video_key][batch_index],
                            features[video_key],
                        )
                    task_index = scalar_int(source_item["task_index"])
                    frame["task"] = source.meta.tasks.iloc[task_index].name
                    output.add_frame(frame)

            output.save_episode(parallel_encoding=False)
            total_output_frames += len(offsets)
            print(
                f"  episode {episode_index}: {end - start} -> {len(offsets)} frames "
                f"({total_output_frames} written)"
            )
    except BaseException:
        print(
            f"Resampling stopped. Partial output was left at {args.output_root}; "
            "move/delete it manually before retrying."
        )
        raise
    finally:
        output.stop_image_writer()
        output.finalize()

    converted = LeRobotDataset(
        args.output_repo_id,
        root=args.output_root,
        video_backend=args.video_backend,
    )
    if converted.fps != args.target_fps:
        raise RuntimeError(f"Verification failed: output reports {converted.fps} FPS.")
    if converted.meta.total_episodes != source.meta.total_episodes:
        raise RuntimeError("Verification failed: output episode count differs from source.")
    if converted.meta.total_frames != total_output_frames:
        raise RuntimeError(
            f"Verification failed: expected {total_output_frames} output frames, "
            f"found {converted.meta.total_frames}."
        )
    print(
        f"Resampled dataset ready: root={args.output_root}, "
        f"episodes={converted.meta.total_episodes}, frames={converted.meta.total_frames}, "
        f"fps={converted.fps}"
    )


if __name__ == "__main__":
    main()
