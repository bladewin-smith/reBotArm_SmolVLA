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
from ..utils import ensure_safe_goal_position, get_camera_observation_features, read_camera_observations
from .config_rebot_b601_follower import RebotB601FollowerConfig

logger = logging.getLogger(__name__)


class RebotB601Follower(Robot):
    """reBot Arm B601-DM follower using Damiao CAN MIT control."""

    config_class = RebotB601FollowerConfig
    name = "rebot_b601_follower"

    def __init__(self, config: RebotB601FollowerConfig):
        super().__init__(config)
        self.config = config

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

    def send_action(self, action: RobotAction) -> RobotAction:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        for motor_name, position in goal_pos.items():
            if motor_name in self.config.joint_limits:
                min_limit, max_limit = self.config.joint_limits[motor_name]
                goal_pos[motor_name] = max(min_limit, min(max_limit, float(position)))

        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position")
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        commands = {
            motor_name: (
                self._gain_for(self.config.position_kp, motor_name),
                self._gain_for(self.config.position_kd, motor_name),
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
