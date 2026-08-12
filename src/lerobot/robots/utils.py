# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from pprint import pformat
from typing import Any, cast

from lerobot.cameras import Camera, CameraConfig
from lerobot.utils.import_utils import make_device_from_device_class

from .config import RobotConfig
from .robot import Robot


def make_robot_from_config(config: RobotConfig) -> Robot:
    # TODO(Steven): Consider just using the make_device_from_device_class for all types
    if config.type == "koch_follower":
        from .koch_follower import KochFollower

        return KochFollower(config)
    elif config.type == "omx_follower":
        from .omx_follower import OmxFollower

        return OmxFollower(config)
    elif config.type == "so100_follower":
        from .so_follower import SO100Follower

        return SO100Follower(config)
    elif config.type == "so101_follower":
        from .so_follower import SO101Follower

        return SO101Follower(config)
    elif config.type == "lekiwi":
        from .lekiwi import LeKiwi

        return LeKiwi(config)
    elif config.type == "hope_jr_hand":
        from .hope_jr import HopeJrHand

        return HopeJrHand(config)
    elif config.type == "hope_jr_arm":
        from .hope_jr import HopeJrArm

        return HopeJrArm(config)
    elif config.type == "bi_so_follower":
        from .bi_so_follower import BiSOFollower

        return BiSOFollower(config)
    elif config.type == "reachy2":
        from .reachy2 import Reachy2Robot

        return Reachy2Robot(config)
    elif config.type == "openarm_follower":
        from .openarm_follower import OpenArmFollower

        return OpenArmFollower(config)
    elif config.type == "rebot_b601_follower":
        from .rebot_b601_follower import RebotB601Follower

        return RebotB601Follower(config)
    elif config.type == "bi_openarm_follower":
        from .bi_openarm_follower import BiOpenArmFollower

        return BiOpenArmFollower(config)
    elif config.type == "mock_robot":
        from tests.mocks.mock_robot import MockRobot

        return MockRobot(config)
    else:
        try:
            return cast(Robot, make_device_from_device_class(config))
        except Exception as e:
            raise ValueError(f"Error creating robot with config {config}: {e}") from e


def get_camera_observation_features(
    camera_configs: dict[str, CameraConfig], cameras: dict[str, Camera]
) -> dict[str, tuple | dict]:
    """Build robot observation features for RGB cameras and optional depth streams."""
    features: dict[str, tuple | dict] = {}
    for cam_key in cameras:
        cfg = camera_configs[cam_key]
        if getattr(cfg, "record_color", True):
            features[cam_key] = (cfg.height, cfg.width, 3)

        if getattr(cfg, "use_depth", False) and getattr(cfg, "record_depth", False):
            depth_key = getattr(cfg, "depth_key", None) or f"depths.{cam_key}"
            features[depth_key] = {
                "dtype": getattr(cfg, "depth_dtype", "uint16"),
                "shape": (cfg.height, cfg.width),
                "names": ["height", "width"],
            }

        if getattr(cfg, "use_depth", False) and getattr(cfg, "record_depth_viz", False):
            depth_viz_key = getattr(cfg, "depth_viz_key", None) or f"{cam_key}_depth"
            features[depth_viz_key] = (cfg.height, cfg.width, 3)
    return features


def read_camera_observations(
    cameras: dict[str, Camera],
    camera_configs: dict[str, CameraConfig],
    *,
    logger: logging.Logger | None = None,
    robot_name: str = "robot",
) -> dict[str, Any]:
    """Read RGB frames and optional depth maps from configured robot cameras."""
    observations: dict[str, Any] = {}
    for cam_key, cam in cameras.items():
        cfg = camera_configs[cam_key]
        if getattr(cfg, "record_color", True):
            start = time.perf_counter()
            observations[cam_key] = cam.async_read()
            if logger is not None:
                dt_ms = (time.perf_counter() - start) * 1e3
                logger.debug(f"{robot_name} read {cam_key}: {dt_ms:.1f}ms")

        if getattr(cfg, "use_depth", False) and getattr(cfg, "record_depth", False):
            depth_key = getattr(cfg, "depth_key", None) or f"depths.{cam_key}"
            start = time.perf_counter()
            if getattr(cfg, "record_color", True) and hasattr(cam, "read_latest_depth"):
                observations[depth_key] = cam.read_latest_depth()
            elif hasattr(cam, "async_read_depth"):
                observations[depth_key] = cam.async_read_depth()
            elif hasattr(cam, "read_depth"):
                observations[depth_key] = cam.read_depth()
            else:
                raise RuntimeError(
                    f"Camera '{cam_key}' is configured for depth but has no depth read method."
                )
            if logger is not None:
                dt_ms = (time.perf_counter() - start) * 1e3
                logger.debug(f"{robot_name} read {depth_key}: {dt_ms:.1f}ms")

        if getattr(cfg, "use_depth", False) and getattr(cfg, "record_depth_viz", False):
            depth_viz_key = getattr(cfg, "depth_viz_key", None) or f"{cam_key}_depth"
            start = time.perf_counter()
            if hasattr(cam, "read_latest_depth_viz"):
                observations[depth_viz_key] = cam.read_latest_depth_viz()
            elif hasattr(cam, "async_read_depth_viz"):
                observations[depth_viz_key] = cam.async_read_depth_viz()
            else:
                raise RuntimeError(
                    f"Camera '{cam_key}' is configured for depth visualization but has no depth-viz read method."
                )
            if logger is not None:
                dt_ms = (time.perf_counter() - start) * 1e3
                logger.debug(f"{robot_name} read {depth_viz_key}: {dt_ms:.1f}ms")
    return observations


# TODO(pepijn): Move to pipeline step to make sure we don't have to do this in the robot code and send action to robot is clean for use in dataset
def ensure_safe_goal_position(
    goal_present_pos: dict[str, tuple[float, float]], max_relative_target: float | dict[str, float]
) -> dict[str, float]:
    """Caps relative action target magnitude for safety."""

    if isinstance(max_relative_target, float):
        diff_cap = dict.fromkeys(goal_present_pos, max_relative_target)
    elif isinstance(max_relative_target, dict):
        if not set(goal_present_pos) == set(max_relative_target):
            raise ValueError("max_relative_target keys must match those of goal_present_pos.")
        diff_cap = max_relative_target
    else:
        raise TypeError(max_relative_target)

    warnings_dict = {}
    safe_goal_positions = {}
    for key, (goal_pos, present_pos) in goal_present_pos.items():
        diff = goal_pos - present_pos
        max_diff = diff_cap[key]
        safe_diff = min(diff, max_diff)
        safe_diff = max(safe_diff, -max_diff)
        safe_goal_pos = present_pos + safe_diff
        safe_goal_positions[key] = safe_goal_pos
        if abs(safe_goal_pos - goal_pos) > 1e-4:
            warnings_dict[key] = {
                "original goal_pos": goal_pos,
                "safe goal_pos": safe_goal_pos,
            }

    if warnings_dict:
        logging.warning(
            "Relative goal position magnitude had to be clamped to be safe.\n"
            f"{pformat(warnings_dict, indent=4)}"
        )

    return safe_goal_positions
