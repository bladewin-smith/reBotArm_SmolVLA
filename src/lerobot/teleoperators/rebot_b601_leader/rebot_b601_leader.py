#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");

import logging
import time
from typing import Any

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.damiao import DamiaoMotorsBus
from lerobot.motors.rebot_motorbridge import RebotMotorbridgeBus
from lerobot.processor import RobotAction
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..teleoperator import Teleoperator
from .config_rebot_b601_leader import RebotB601LeaderConfig

logger = logging.getLogger(__name__)


class RebotB601Leader(Teleoperator):
    """reBot Arm B601-DM leader arm with optional low-stiffness impedance mode."""

    config_class = RebotB601LeaderConfig
    name = "rebot_b601_leader"

    def __init__(self, config: RebotB601LeaderConfig):
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

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info(f"Connecting {self.name} on {self.config.port}...")
        self.bus.connect(handshake=self.config.handshake)

        if not self.is_calibrated and calibrate:
            logger.info("No B601-DM leader calibration file found; starting calibration.")
            self.calibrate()

        self.configure()
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
        input("\nMove the B601-DM leader to the same neutral zero pose as the follower, then press ENTER...")
        self.bus.set_zero_position()

        self.calibration = {}
        for motor_name, motor in self.bus.motors.items():
            self.calibration[motor_name] = MotorCalibration(
                id=motor.id,
                drive_mode=0,
                homing_offset=0,
                range_min=-180,
                range_max=180,
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        if self.config.manual_control_mode == "disabled":
            self.bus.disable_torque()
        else:
            self.bus.configure_motors()
            states = self.bus.sync_read_all_states()
            self._send_compliance(states)

    def setup_motors(self) -> None:
        raise NotImplementedError("Use the Damiao/reBot vendor tools to configure B601-DM CAN IDs.")

    def _gain_for(self, values: list[float] | float, motor_name: str) -> float:
        if isinstance(values, list):
            names = list(self.bus.motors)
            return float(values[names.index(motor_name)])
        return float(values)

    def _send_compliance(self, states: dict[str, dict[str, Any]]) -> None:
        if self.config.manual_control_mode == "disabled":
            return

        kp_values = self.config.stiff_kp if self.config.manual_control_mode == "stiff" else self.config.impedance_kp
        kd_values = self.config.stiff_kd if self.config.manual_control_mode == "stiff" else self.config.impedance_kd

        commands = {}
        for motor_name in self.bus.motors:
            state = states.get(motor_name, {})
            position = float(state.get("position", 0.0))
            commands[motor_name] = (
                self._gain_for(kp_values, motor_name),
                self._gain_for(kd_values, motor_name),
                position,
                0.0,
                0.0,
            )
        self.bus.sync_write_mit(commands)

    def get_action(self) -> RobotAction:
        start = time.perf_counter()
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        states = self.bus.sync_read_all_states()
        if self.config.manual_control_mode != "disabled":
            self._send_compliance(states)

        action_dict = {
            f"{motor}.pos": float(states.get(motor, {}).get("position", 0.0)) for motor in self.bus.motors
        }

        logger.debug(f"{self} read action: {(time.perf_counter() - start) * 1e3:.1f}ms")
        return action_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError("Feedback is not implemented for the B601-DM leader.")

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.bus.disconnect(disable_torque=True)
        logger.info(f"{self} disconnected.")
