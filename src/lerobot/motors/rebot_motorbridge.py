#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""reBot B601-DM motorbridge backend.

The Seeed/reBotArm reference stack drives the B601-DM through motorbridge's
DM serial transport, usually ``/dev/ttyACM0`` at 921600 baud. This adapter
exposes the small subset of the LeRobot motor bus interface used by the
reBot B601 follower and leader classes while keeping the same units as the
rest of LeRobot: positions/velocities are reported in degrees.
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any

from lerobot.motors import Motor, MotorCalibration
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

logger = logging.getLogger(__name__)


class RebotMotorbridgeBus:
    """Small motorbridge wrapper for the reBot B601-DM arm."""

    def __init__(
        self,
        port: str,
        motors: dict[str, Motor],
        calibration: dict[str, MotorCalibration] | None = None,
        *,
        baudrate: int = 921600,
        feedback_poll_retries: int = 3,
        feedback_poll_interval_s: float = 0.001,
    ) -> None:
        self.port = port
        self.motors = motors
        self.calibration = calibration or {}
        self.baudrate = baudrate
        self.feedback_poll_retries = feedback_poll_retries
        self.feedback_poll_interval_s = feedback_poll_interval_s
        self.controller: Any | None = None
        self._motor_handles: dict[str, Any] = {}
        self._is_connected = False
        self._last_known_states: dict[str, dict[str, float]] = {
            name: {
                "position": 0.0,
                "velocity": 0.0,
                "torque": 0.0,
                "temp_mos": 0.0,
                "temp_rotor": 0.0,
            }
            for name in self.motors
        }

    @property
    def is_connected(self) -> bool:
        return self._is_connected and self.controller is not None

    @property
    def is_calibrated(self) -> bool:
        return bool(self.calibration)

    @staticmethod
    def _motorbridge_model(model: str) -> str:
        normalized = model.lower().replace("-", "_")
        if normalized in ("dm4340", "4340", "4340p"):
            return "4340P"
        if normalized in ("dm4310", "4310"):
            return "4310"
        return model

    def connect(self, handshake: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self.__class__.__name__}('{self.port}') is already connected.")

        try:
            from motorbridge import Controller
        except ImportError as exc:
            raise ImportError(
                "motorbridge is required for reBot B601-DM motorbridge transport. "
                "Install it on the Jetson with `pip install motorbridge>=0.4.9`, "
                "or install the Seeed reBotArm_control_py SDK."
            ) from exc

        try:
            if self.port.startswith("/dev/tty"):
                self.controller = Controller.from_dm_serial(self.port, self.baudrate)
            else:
                self.controller = Controller(self.port)

            self._motor_handles = {}
            for motor_name, motor in self.motors.items():
                if motor.recv_id is None:
                    raise ValueError(f"Motor {motor_name} is missing feedback/recv ID.")
                model = self._motorbridge_model(motor.model)
                self._motor_handles[motor_name] = self.controller.add_damiao_motor(
                    motor.id,
                    motor.recv_id,
                    model,
                )

            self._is_connected = True
            if handshake:
                self.sync_read_all_states()
                missing = [
                    name
                    for name in self.motors
                    if self._motor_handles[name].get_state() is None
                ]
                if missing:
                    raise ConnectionError(f"motorbridge handshake got no feedback from: {missing}")
        except Exception as exc:
            self._shutdown_controller()
            self._is_connected = False
            raise ConnectionError(f"Failed to connect motorbridge bus on {self.port}: {exc}") from exc

    def _shutdown_controller(self) -> None:
        if self.controller is None:
            return
        try:
            self.controller.shutdown()
        except Exception as exc:
            logger.debug("motorbridge shutdown failed: %s", exc)
        try:
            self.controller.close()
        except Exception as exc:
            logger.debug("motorbridge close failed: %s", exc)
        self.controller = None
        self._motor_handles = {}

    def disconnect(self, disable_torque: bool = True) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self.__class__.__name__}('{self.port}') is not connected.")

        if disable_torque:
            try:
                self.disable_torque()
            except Exception as exc:
                logger.warning("Failed to disable motorbridge torque during disconnect: %s", exc)
        self._shutdown_controller()
        self._is_connected = False

    def configure_motors(self) -> None:
        self.ensure_mit_mode()

    def ensure_mit_mode(self, motors: str | list[str] | None = None, timeout_ms: int = 1000) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        try:
            from motorbridge import Mode
        except ImportError as exc:
            raise ImportError("motorbridge is required to switch B601-DM motors to MIT mode.") from exc
        for motor_name in self._get_motors_list(motors):
            self._motor_handles[motor_name].ensure_mode(Mode.MIT, timeout_ms)
            time.sleep(0.02)

    def enable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        target_motors = self._get_motors_list(motors)
        for _ in range(num_retry + 1):
            try:
                if motors is None:
                    self.controller.enable_all()
                else:
                    for motor_name in target_motors:
                        self._motor_handles[motor_name].enable()
                return
            except Exception:
                if _ == num_retry:
                    raise
                time.sleep(0.05)

    def disable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        target_motors = self._get_motors_list(motors)
        for _ in range(num_retry + 1):
            try:
                if motors is None:
                    self.controller.disable_all()
                else:
                    for motor_name in target_motors:
                        self._motor_handles[motor_name].disable()
                return
            except Exception:
                if _ == num_retry:
                    raise
                time.sleep(0.05)

    def set_zero_position(self, motors: str | list[str] | None = None) -> None:
        for motor_name in self._get_motors_list(motors):
            self._motor_handles[motor_name].set_zero_position()
            time.sleep(0.05)

    def write_calibration(self, calibration_dict: dict[str, MotorCalibration], cache: bool = True) -> None:
        if cache:
            self.calibration = calibration_dict

    def _get_motors_list(self, motors: str | list[str] | None) -> list[str]:
        if motors is None:
            return list(self.motors)
        if isinstance(motors, str):
            return [motors]
        return list(motors)

    def _poll_motor_state(self, motor_name: str) -> Any | None:
        handle = self._motor_handles[motor_name]
        state = None
        for _ in range(max(1, self.feedback_poll_retries)):
            handle.request_feedback()
            self.controller.poll_feedback_once()
            time.sleep(self.feedback_poll_interval_s)
            state = handle.get_state()
            if state is not None:
                return state
        return state

    def _update_state_cache(self, motor_name: str, state: Any | None) -> None:
        if state is None:
            logger.warning("Packet drop: %s. Using last known motorbridge state.", motor_name)
            return
        self._last_known_states[motor_name] = {
            "position": math.degrees(float(getattr(state, "pos", 0.0))),
            "velocity": math.degrees(float(getattr(state, "vel", 0.0))),
            "torque": float(getattr(state, "torq", 0.0)),
            "temp_mos": 0.0,
            "temp_rotor": 0.0,
        }

    def sync_read_all_states(
        self,
        motors: str | list[str] | None = None,
        *,
        num_retry: int = 0,
    ) -> dict[str, dict[str, float]]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        target_motors = self._get_motors_list(motors)
        for motor_name in target_motors:
            state = self._poll_motor_state(motor_name)
            self._update_state_cache(motor_name, state)
        return {motor_name: self._last_known_states[motor_name].copy() for motor_name in target_motors}

    def sync_read(self, data_name: str, motors: str | list[str] | None = None) -> dict[str, float]:
        states = self.sync_read_all_states(motors)
        if data_name == "Present_Position":
            return {motor: state["position"] for motor, state in states.items()}
        if data_name == "Present_Velocity":
            return {motor: state["velocity"] for motor, state in states.items()}
        if data_name == "Present_Torque":
            return {motor: state["torque"] for motor, state in states.items()}
        raise ValueError(f"Unknown motorbridge data_name: {data_name}")

    def sync_write_mit(
        self,
        commands: dict[str, tuple[float, float, float, float, float]],
    ) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        for motor_name, (kp, kd, position_degrees, velocity_deg_per_sec, torque) in commands.items():
            handle = self._motor_handles[motor_name]
            handle.send_mit(
                math.radians(float(position_degrees)),
                math.radians(float(velocity_deg_per_sec)),
                float(kp),
                float(kd),
                float(torque),
            )
