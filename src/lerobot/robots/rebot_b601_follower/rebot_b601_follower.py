#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");

import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.damiao import DamiaoMotorsBus
from lerobot.motors.rebot_motorbridge import RebotMotorbridgeBus
from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..robot import Robot
from ..utils import get_camera_observation_features, read_camera_observations
from .config_rebot_b601_follower import RebotB601FollowerConfig

logger = logging.getLogger(__name__)


class RebotB601Follower(Robot):
    """reBot Arm B601-DM follower using Damiao CAN MIT control."""

    config_class = RebotB601FollowerConfig
    name = "rebot_b601_follower"

    def __init__(self, config: RebotB601FollowerConfig):
        super().__init__(config)
        self.config = config
        if self.config.transport not in {"motorbridge", "socketcan"}:
            raise ValueError("B601 follower transport must be 'motorbridge' or 'socketcan'.")

        motors: dict[str, Motor] = {}
        for motor_name, (send_id, recv_id, motor_type_str) in config.motor_config.items():
            motor = Motor(send_id, motor_type_str, MotorNormMode.DEGREES)
            motor.recv_id = recv_id
            motor.motor_type_str = motor_type_str
            motors[motor_name] = motor

        if self.config.transport == "motorbridge":
            self.bus = RebotMotorbridgeBus(
                port=self.config.port,
                motors=motors,
                calibration=self.calibration,
                baudrate=self.config.motorbridge_baudrate,
            )
        else:
            self.bus = DamiaoMotorsBus(
                port=self.config.port,
                motors=motors,
                calibration=self.calibration,
                can_interface=self.config.can_interface,
                use_can_fd=self.config.use_can_fd,
                bitrate=self.config.can_bitrate,
                data_bitrate=self.config.can_data_bitrate if self.config.use_can_fd else None,
            )
        self.cameras = make_cameras_from_configs(config.cameras)
        self._last_observation_states: dict[str, dict[str, float]] = {}
        self._last_safety_hold_log_s = 0.0
        self._safety_hold_active = False

    @property
    def _state_ft(self) -> dict[str, type]:
        features: dict[str, type] = {}
        for motor in self.bus.motors:
            features[f"{motor}.pos"] = float
            features[f"{motor}.vel"] = float
            features[f"{motor}.torque"] = float
        return features

    @property
    def _action_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple | dict]:
        return get_camera_observation_features(self.config.cameras, self.cameras)

    @cached_property
    def observation_features(self) -> dict[str, type | tuple | dict]:
        return {**self._state_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._action_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info(f"Connecting {self.name} on {self.config.port}...")
        self.bus.connect()

        if not self.is_calibrated and calibrate:
            logger.info("No B601-DM calibration file found; starting calibration.")
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        self.bus.enable_torque()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            user_input = input(
                f"Press ENTER to use calibration file for id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"Running calibration for {self}")
        self.bus.disable_torque()
        input(
            "\nMove the B601-DM follower to its neutral zero pose with gripper closed, then press ENTER..."
        )
        self.bus.set_zero_position()

        self.calibration = {}
        for motor_name, motor in self.bus.motors.items():
            min_limit, max_limit = self.config.joint_limits.get(motor_name, (-180.0, 180.0))
            self.calibration[motor_name] = MotorCalibration(
                id=motor.id,
                drive_mode=0,
                homing_offset=0,
                range_min=int(min_limit),
                range_max=int(max_limit),
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        self.bus.configure_motors()

    def setup_motors(self) -> None:
        raise NotImplementedError("Use the Damiao/reBot vendor tools to configure B601-DM CAN IDs.")

    def get_observation(self) -> RobotObservation:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        obs_dict: dict[str, Any] = {}
        states = self.bus.sync_read_all_states()
        self._last_observation_states = {motor: state.copy() for motor, state in states.items()}
        for motor in self.bus.motors:
            state = states.get(motor, {})
            obs_dict[f"{motor}.pos"] = state.get("position", 0.0)
            obs_dict[f"{motor}.vel"] = state.get("velocity", 0.0)
            obs_dict[f"{motor}.torque"] = state.get("torque", 0.0)

        obs_dict.update(
            read_camera_observations(self.cameras, self.config.cameras, logger=logger, robot_name=str(self))
        )

        logger.debug(f"{self} get_observation took {(time.perf_counter() - start) * 1e3:.1f}ms")
        return obs_dict

    def _gain_for(self, values: list[float] | float, motor_name: str) -> float:
        if isinstance(values, list):
            names = list(self.bus.motors)
            return float(values[names.index(motor_name)])
        return float(values)

    def _map_gripper_goal(self, leader_position: float) -> float:
        if (
            self.config.gripper_leader_close_pos is None
            or self.config.gripper_leader_open_pos is None
            or self.config.gripper_follower_close_pos is None
            or self.config.gripper_follower_open_pos is None
        ):
            return leader_position * self.config.gripper_action_scale + self.config.gripper_action_offset

        leader_close = self.config.gripper_leader_close_pos
        leader_open = self.config.gripper_leader_open_pos
        follower_close = self.config.gripper_follower_close_pos
        follower_open = self.config.gripper_follower_open_pos
        span = leader_open - leader_close
        if abs(span) < 1e-6:
            raise ValueError("gripper_leader_open_pos and gripper_leader_close_pos must be different.")

        alpha = (leader_position - leader_close) / span
        alpha = max(0.0, min(1.0, alpha))
        return follower_close + alpha * (follower_open - follower_close)

    def _joint_limits_for(self, motor_name: str) -> tuple[float, float] | None:
        if motor_name not in self.config.joint_limits:
            return None

        min_limit, max_limit = self.config.joint_limits[motor_name]
        if motor_name == "gripper":
            if self.config.gripper_min_pos is not None:
                min_limit = self.config.gripper_min_pos
            if self.config.gripper_max_pos is not None:
                max_limit = self.config.gripper_max_pos
        return float(min_limit), float(max_limit)

    def _clamp_to_joint_limits(self, goal_pos: dict[str, float]) -> dict[str, float]:
        clamped: dict[str, float] = {}
        for motor_name, position in goal_pos.items():
            limits = self._joint_limits_for(motor_name)
            if limits is None:
                clamped[motor_name] = float(position)
                continue
            min_limit, max_limit = limits
            clamped[motor_name] = max(min_limit, min(max_limit, float(position)))
        return clamped

    def _relative_target_for(self, motor_name: str) -> float | None:
        max_relative_target = self.config.max_relative_target
        if motor_name == "gripper" and self.config.gripper_max_relative_target is not None:
            return float(self.config.gripper_max_relative_target)
        if max_relative_target is None:
            return None
        if isinstance(max_relative_target, dict):
            if motor_name not in max_relative_target:
                return None
            return float(max_relative_target[motor_name])
        return float(max_relative_target)

    def _apply_relative_target_limits(
        self, goal_pos: dict[str, float], present_pos: dict[str, float]
    ) -> tuple[dict[str, float], dict[str, tuple[float, float, float, float]]]:
        safe_goal_pos: dict[str, float] = {}
        clamp_info: dict[str, tuple[float, float, float, float]] = {}

        for motor_name, target_pos in goal_pos.items():
            current_pos = float(present_pos[motor_name])
            target_pos = float(target_pos)
            max_relative_target = self._relative_target_for(motor_name)
            if max_relative_target is None or max_relative_target <= 0:
                safe_goal_pos[motor_name] = target_pos
                continue

            delta = target_pos - current_pos
            if abs(delta) <= max_relative_target:
                safe_goal_pos[motor_name] = target_pos
                continue

            safe_target = current_pos + max_relative_target * (1.0 if delta > 0 else -1.0)
            safe_goal_pos[motor_name] = safe_target
            clamp_info[motor_name] = (target_pos, safe_target, current_pos, max_relative_target)

        return safe_goal_pos, clamp_info

    def _should_hold_on_clamp(self, clamp_info: dict[str, tuple[float, float, float, float]]) -> bool:
        if not self.config.safety_hold_on_relative_clamp or not clamp_info:
            return False

        monitored_joints = set(self.config.safety_hold_joints)
        large_clamps = 0
        for motor_name, (target_pos, safe_target, _current_pos, max_relative_target) in clamp_info.items():
            if motor_name not in monitored_joints:
                continue
            excess = abs(target_pos - safe_target)
            if excess >= max_relative_target * self.config.safety_hold_single_joint_ratio:
                return True
            if excess >= max_relative_target * self.config.safety_hold_clamp_ratio:
                large_clamps += 1

        return large_clamps >= self.config.safety_hold_clamp_joint_count

    def _log_safety_hold(self, clamp_info: dict[str, tuple[float, float, float, float]]) -> None:
        now_s = time.monotonic()
        if now_s - self._last_safety_hold_log_s < self.config.safety_hold_log_interval_s:
            return

        self._last_safety_hold_log_s = now_s
        details = {
            motor: {
                "requested": target,
                "limited": limited,
                "present": present,
                "max_relative_target": max_relative,
            }
            for motor, (target, limited, present, max_relative) in clamp_info.items()
            if motor in set(self.config.safety_hold_joints)
        }
        logger.error(
            "%s safety hold active: follower target is too far from current pose; holding present pose. %s",
            self.name,
            details,
        )

    def send_action(self, action: RobotAction) -> RobotAction:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}
        if "gripper" in goal_pos:
            goal_pos["gripper"] = self._map_gripper_goal(float(goal_pos["gripper"]))

        goal_pos = self._clamp_to_joint_limits(goal_pos)
        safety_hold = False

        if self.config.max_relative_target is not None:
            present_pos = (
                {motor: state["position"] for motor, state in self._last_observation_states.items()}
                if self._last_observation_states
                else self.bus.sync_read("Present_Position")
            )
            goal_pos, clamp_info = self._apply_relative_target_limits(goal_pos, present_pos)
            safety_hold = self._should_hold_on_clamp(clamp_info)
            if safety_hold:
                self._safety_hold_active = True
                self._log_safety_hold(clamp_info)
                goal_pos = {motor: float(present_pos[motor]) for motor in goal_pos}
            else:
                if self._safety_hold_active:
                    logger.info("%s safety hold cleared; follower target is back within safe range.", self.name)
                self._safety_hold_active = False
                goal_pos = self._clamp_to_joint_limits(goal_pos)
        else:
            self._safety_hold_active = False

        commands = {
            motor_name: (
                (
                    self.config.gripper_position_kp
                    if motor_name == "gripper" and self.config.gripper_position_kp is not None
                    else self._gain_for(self.config.position_kp, motor_name)
                )
                * (self.config.safety_hold_position_kp_scale if safety_hold else 1.0),
                self.config.gripper_position_kd
                if motor_name == "gripper" and self.config.gripper_position_kd is not None
                else self._gain_for(self.config.position_kd, motor_name),
                float(position_degrees),
                0.0,
                0.0,
            )
            for motor_name, position_degrees in goal_pos.items()
        }
        self.bus.sync_write_mit(commands)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()
        logger.info(f"{self} disconnected.")
