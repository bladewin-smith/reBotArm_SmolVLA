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
import threading
import time
from typing import Any

from lerobot.motors import Motor, MotorCalibration
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

logger = logging.getLogger(__name__)


DM_STATUS_NAMES = {
    0x0: "DISABLED",
    0x1: "ENABLED",
    0x8: "OVER_VOLTAGE",
    0x9: "UNDER_VOLTAGE",
    0xA: "OVER_CURRENT",
    0xB: "MOS_OVER_TEMPERATURE",
    0xC: "ROTOR_OVER_TEMPERATURE",
    0xD: "LOST_COMMUNICATION",
    0xE: "OVERLOAD",
}
DM_NORMAL_STATUS_CODES = frozenset({0x0, 0x1})


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
        self._io_lock = threading.RLock()
        self._mit_stream_lock = threading.Lock()
        self._mit_stream_stop = threading.Event()
        self._mit_stream_thread: threading.Thread | None = None
        self._mit_stream_commands: dict[str, tuple[float, float, float, float, float]] = {}
        self._mit_stream_commands_updated_s: float | None = None
        self._mit_stream_hz = 0.0
        self._mit_stream_max_gap_s = 0.05
        self._mit_stream_hard_gap_s = 0.5
        self._mit_stream_max_failures = 5
        self._mit_stream_consecutive_failures = 0
        self._mit_stream_consecutive_gap_violations = 0
        self._mit_stream_error: Exception | None = None
        self._mit_stream_fault: Exception | None = None
        self._mit_stream_last_send_s: float | None = None
        self._mit_stream_started_s: float | None = None
        self._mit_stream_total_sends = 0
        self._mit_stream_last_send_duration_s = 0.0
        self._mit_stream_max_send_duration_s = 0.0
        self._mit_stream_max_send_duration_at_s: float | None = None
        self._mit_stream_max_completed_gap_s = 0.0
        self._mit_stream_max_completed_gap_at_s: float | None = None
        self._mit_stream_last_gap_violation_s: float | None = None
        self._mit_stream_last_gap_violation_duration_s = 0.0
        self._last_motor_status_codes = {name: 0 for name in self.motors}
        self._feedback_miss_counts = {name: 0 for name in self.motors}
        self._last_known_states: dict[str, dict[str, float]] = {
            name: {
                "position": 0.0,
                "velocity": 0.0,
                "torque": 0.0,
                "temp_mos": 0.0,
                "temp_rotor": 0.0,
                "status_code": 0.0,
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
                self.check_motor_status_codes(max_feedback_misses=1)
        except Exception as exc:
            self._shutdown_controller()
            self._is_connected = False
            raise ConnectionError(f"Failed to connect motorbridge bus on {self.port}: {exc}") from exc

    def _shutdown_controller(self) -> None:
        if self.controller is None:
            return
        with self._io_lock:
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

        self.stop_mit_command_stream()
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
        with self._io_lock:
            for motor_name in self._get_motors_list(motors):
                self._motor_handles[motor_name].ensure_mode(Mode.MIT, timeout_ms)
                time.sleep(0.02)

    def enable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        target_motors = self._get_motors_list(motors)
        for _ in range(num_retry + 1):
            try:
                with self._io_lock:
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
                with self._io_lock:
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
        with self._io_lock:
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
            with self._io_lock:
                handle.request_feedback()
                self.controller.poll_feedback_once()
                time.sleep(self.feedback_poll_interval_s)
                state = handle.get_state()
            if state is not None:
                return state
        return state

    def _update_state_cache(self, motor_name: str, state: Any | None) -> None:
        if state is None:
            self._feedback_miss_counts[motor_name] += 1
            logger.warning("Packet drop: %s. Using last known motorbridge state.", motor_name)
            return
        self._feedback_miss_counts[motor_name] = 0
        status_code = int(getattr(state, "status_code", 0))
        previous_status = self._last_motor_status_codes[motor_name]
        self._last_motor_status_codes[motor_name] = status_code
        self._last_known_states[motor_name]["status_code"] = float(status_code)
        feedback_snapshot = {
            "position": math.degrees(float(getattr(state, "pos", 0.0))),
            "velocity": math.degrees(float(getattr(state, "vel", 0.0))),
            "torque": float(getattr(state, "torq", 0.0)),
            "temp_mos": float(getattr(state, "t_mos", getattr(state, "temp_mos", 0.0))),
            "temp_rotor": float(getattr(state, "t_rotor", getattr(state, "temp_rotor", 0.0))),
            "status_code": float(status_code),
        }
        if status_code not in DM_NORMAL_STATUS_CODES:
            if status_code != previous_status:
                logger.error(
                    "Motorbridge feedback fault for %s: status_code=0x%X (%s). "
                    "Fault-frame feedback: pos=%.2f deg, vel=%.2f deg/s, torque=%.2f, "
                    "MOS=%.1f C, rotor=%.1f C. Keeping the last valid state; "
                    "do not automatically re-enable the motor.",
                    motor_name,
                    status_code,
                    DM_STATUS_NAMES.get(status_code, "UNKNOWN"),
                    feedback_snapshot["position"],
                    feedback_snapshot["velocity"],
                    feedback_snapshot["torque"],
                    feedback_snapshot["temp_mos"],
                    feedback_snapshot["temp_rotor"],
                )
            return
        if status_code != previous_status:
            log = logger.warning if status_code == 0x0 and previous_status == 0x1 else logger.info
            log(
                "Motorbridge motor %s changed state: status_code=0x%X (%s); "
                "pos=%.2f deg, vel=%.2f deg/s, torque=%.2f, MOS=%.1f C, rotor=%.1f C; "
                "MIT stream=%s.",
                motor_name,
                status_code,
                DM_STATUS_NAMES[status_code],
                feedback_snapshot["position"],
                feedback_snapshot["velocity"],
                feedback_snapshot["torque"],
                feedback_snapshot["temp_mos"],
                feedback_snapshot["temp_rotor"],
                self.mit_command_stream_diagnostics(),
            )
        self._last_known_states[motor_name] = feedback_snapshot

    def motor_status_codes(self) -> dict[str, int]:
        return self._last_motor_status_codes.copy()

    def check_motor_status_codes(
        self,
        *,
        max_feedback_misses: int = 3,
        required_enabled_motors: str | list[str] | None = None,
    ) -> None:
        if max_feedback_misses < 1:
            raise ValueError("max_feedback_misses must be at least 1.")
        fault_status = {
            motor_name: f"0x{status_code:X} ({DM_STATUS_NAMES.get(status_code, 'UNKNOWN')})"
            for motor_name, status_code in self._last_motor_status_codes.items()
            if status_code not in DM_NORMAL_STATUS_CODES
        }
        if fault_status:
            raise RuntimeError(
                "Motorbridge reported DM drive fault status codes "
                f"{fault_status}. Support the arm and inspect power/drive faults; automatic re-enable is disabled."
            )
        if required_enabled_motors is not None:
            required_motors = self._get_motors_list(required_enabled_motors)
            unknown_motors = [motor_name for motor_name in required_motors if motor_name not in self.motors]
            if unknown_motors:
                raise ValueError(f"Unknown motors required to be enabled: {unknown_motors}")
            disabled_motors = {
                motor_name: (
                    "0x0 (DISABLED), "
                    f"pos={self._last_known_states[motor_name]['position']:.2f} deg, "
                    f"vel={self._last_known_states[motor_name]['velocity']:.2f} deg/s, "
                    f"torque={self._last_known_states[motor_name]['torque']:.2f}, "
                    f"MOS={self._last_known_states[motor_name]['temp_mos']:.1f} C, "
                    f"rotor={self._last_known_states[motor_name]['temp_rotor']:.1f} C"
                )
                for motor_name in required_motors
                if self._last_motor_status_codes[motor_name] == 0x0
            }
            if disabled_motors:
                raise RuntimeError(
                    "Motorbridge motors unexpectedly disabled during active control "
                    f"{disabled_motors}. MIT stream diagnostics: "
                    f"{self.mit_command_stream_diagnostics()}. Support the arm; "
                    "automatic re-enable is disabled."
                )
        missing_feedback = {
            motor_name: miss_count
            for motor_name, miss_count in self._feedback_miss_counts.items()
            if miss_count >= max_feedback_misses
        }
        if missing_feedback:
            raise RuntimeError(
                "Motorbridge feedback was missing for consecutive control samples "
                f"{missing_feedback}. Support the arm and inspect the serial/CAN bridge and power wiring."
            )

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
        normalized_commands = {
            motor_name: tuple(float(value) for value in command)
            for motor_name, command in commands.items()
        }
        with self._mit_stream_lock:
            if self.mit_command_stream_active:
                self._mit_stream_commands.update(normalized_commands)
            else:
                self._mit_stream_commands = normalized_commands.copy()
            self._mit_stream_commands_updated_s = time.monotonic()
        if not self.mit_command_stream_active:
            self._send_mit_commands(normalized_commands)

    def _send_mit_commands(
        self,
        commands: dict[str, tuple[float, float, float, float, float]],
    ) -> None:
        with self._io_lock:
            for motor_name, (kp, kd, position_degrees, velocity_deg_per_sec, torque) in commands.items():
                handle = self._motor_handles[motor_name]
                handle.send_mit(
                    math.radians(position_degrees),
                    math.radians(velocity_deg_per_sec),
                    kp,
                    kd,
                    torque,
                )

    @property
    def mit_command_stream_active(self) -> bool:
        return self._mit_stream_thread is not None and self._mit_stream_thread.is_alive()

    @property
    def mit_command_stream_last_send_s(self) -> float | None:
        return self._mit_stream_last_send_s

    def mit_command_stream_diagnostics(self) -> dict[str, Any]:
        with self._mit_stream_lock:
            now_s = time.monotonic()
            last_send_s = self._mit_stream_last_send_s
            started_s = self._mit_stream_started_s
            commands_updated_s = self._mit_stream_commands_updated_s
            max_send_at_s = self._mit_stream_max_send_duration_at_s
            max_gap_at_s = self._mit_stream_max_completed_gap_at_s
            last_gap_violation_s = self._mit_stream_last_gap_violation_s
            target_positions_deg = {
                motor_name: command[2]
                for motor_name, command in self._mit_stream_commands.items()
            }
            elapsed_s = None if started_s is None else max(0.0, now_s - started_s)
            return {
                "active": self.mit_command_stream_active,
                "configured_hz": self._mit_stream_hz,
                "warning_gap_ms": 1000.0 * self._mit_stream_max_gap_s,
                "hard_gap_ms": 1000.0 * self._mit_stream_hard_gap_s,
                "total_sends": self._mit_stream_total_sends,
                "effective_hz": (
                    None
                    if elapsed_s is None or elapsed_s <= 0
                    else self._mit_stream_total_sends / elapsed_s
                ),
                "age_ms": None if last_send_s is None else 1000.0 * (now_s - last_send_s),
                "last_send_duration_ms": 1000.0 * self._mit_stream_last_send_duration_s,
                "max_send_duration_ms": 1000.0 * self._mit_stream_max_send_duration_s,
                "max_send_duration_ago_ms": (
                    None if max_send_at_s is None else 1000.0 * (now_s - max_send_at_s)
                ),
                "max_completed_gap_ms": 1000.0 * self._mit_stream_max_completed_gap_s,
                "max_completed_gap_ago_ms": (
                    None if max_gap_at_s is None else 1000.0 * (now_s - max_gap_at_s)
                ),
                "last_gap_violation_ms": 1000.0 * self._mit_stream_last_gap_violation_duration_s,
                "last_gap_violation_ago_ms": (
                    None if last_gap_violation_s is None else 1000.0 * (now_s - last_gap_violation_s)
                ),
                "command_update_age_ms": (
                    None if commands_updated_s is None else 1000.0 * (now_s - commands_updated_s)
                ),
                "target_positions_deg": target_positions_deg,
                "consecutive_failures": self._mit_stream_consecutive_failures,
                "consecutive_gap_violations": self._mit_stream_consecutive_gap_violations,
            }

    def start_mit_command_stream(
        self,
        commands: dict[str, tuple[float, float, float, float, float]],
        *,
        hz: float = 500.0,
        max_consecutive_failures: int = 5,
        max_gap_s: float = 0.05,
        hard_gap_s: float = 0.5,
    ) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError("MIT command stream frequency must be positive and finite.")
        if max_consecutive_failures < 1:
            raise ValueError("MIT command stream max_consecutive_failures must be at least 1.")
        if not math.isfinite(max_gap_s) or max_gap_s <= 0:
            raise ValueError("MIT command stream max_gap_s must be positive and finite.")
        if not math.isfinite(hard_gap_s) or hard_gap_s < max_gap_s:
            raise ValueError("MIT command stream hard_gap_s must be finite and at least max_gap_s.")

        self.stop_mit_command_stream()
        normalized_commands = {
            motor_name: tuple(float(value) for value in command)
            for motor_name, command in commands.items()
        }
        with self._mit_stream_lock:
            self._mit_stream_commands = normalized_commands.copy()
            self._mit_stream_commands_updated_s = time.monotonic()
            self._mit_stream_hz = float(hz)
            self._mit_stream_max_gap_s = float(max_gap_s)
            self._mit_stream_hard_gap_s = float(hard_gap_s)
            self._mit_stream_max_failures = int(max_consecutive_failures)
            self._mit_stream_consecutive_failures = 0
            self._mit_stream_consecutive_gap_violations = 0
            self._mit_stream_error = None
            self._mit_stream_fault = None
            self._mit_stream_last_send_s = None
            self._mit_stream_started_s = None
            self._mit_stream_total_sends = 0
            self._mit_stream_last_send_duration_s = 0.0
            self._mit_stream_max_send_duration_s = 0.0
            self._mit_stream_max_send_duration_at_s = None
            self._mit_stream_max_completed_gap_s = 0.0
            self._mit_stream_max_completed_gap_at_s = None
            self._mit_stream_last_gap_violation_s = None
            self._mit_stream_last_gap_violation_duration_s = 0.0

        # Prime the motors before returning so the stream always starts from a
        # valid hold target instead of a default zero position.
        self._send_mit_commands(normalized_commands)
        self._mit_stream_last_send_s = time.monotonic()
        self._mit_stream_started_s = self._mit_stream_last_send_s
        self._mit_stream_stop.clear()
        self._mit_stream_thread = threading.Thread(
            target=self._mit_command_stream_loop,
            name=f"rebot-mit-stream-{self.port}",
            daemon=True,
        )
        self._mit_stream_thread.start()
        logger.info("Started motorbridge MIT command stream at %.1f Hz on %s.", hz, self.port)

    def _mit_command_stream_loop(self) -> None:
        period_s = 1.0 / self._mit_stream_hz
        next_send_s = time.monotonic()
        while not self._mit_stream_stop.is_set():
            with self._mit_stream_lock:
                commands = self._mit_stream_commands.copy()
                last_send_s = self._mit_stream_last_send_s
                max_gap_s = self._mit_stream_max_gap_s
                hard_gap_s = self._mit_stream_hard_gap_s
            try:
                if commands:
                    send_started_s = time.monotonic()
                    self._send_mit_commands(commands)
                    completed_send_s = time.monotonic()
                    send_duration_s = completed_send_s - send_started_s
                    completed_gap_s = None if last_send_s is None else completed_send_s - last_send_s
                    with self._mit_stream_lock:
                        self._mit_stream_last_send_s = completed_send_s
                        self._mit_stream_total_sends += 1
                        self._mit_stream_last_send_duration_s = send_duration_s
                        if send_duration_s > self._mit_stream_max_send_duration_s:
                            self._mit_stream_max_send_duration_s = send_duration_s
                            self._mit_stream_max_send_duration_at_s = completed_send_s
                        if (
                            completed_gap_s is not None
                            and completed_gap_s > self._mit_stream_max_completed_gap_s
                        ):
                            self._mit_stream_max_completed_gap_s = completed_gap_s
                            self._mit_stream_max_completed_gap_at_s = completed_send_s
                        self._mit_stream_consecutive_failures = 0
                        self._mit_stream_error = None
                        if completed_gap_s is not None and completed_gap_s > max_gap_s:
                            self._mit_stream_last_gap_violation_s = completed_send_s
                            self._mit_stream_last_gap_violation_duration_s = completed_gap_s
                            self._mit_stream_consecutive_gap_violations += 1
                            gap_violations = self._mit_stream_consecutive_gap_violations
                            if completed_gap_s >= hard_gap_s and self._mit_stream_fault is None:
                                self._mit_stream_fault = RuntimeError(
                                    f"single completed-send gap was {completed_gap_s:.3f}s, exceeding "
                                    f"the hard recovered-gap limit of {hard_gap_s:.3f}s"
                                )
                                gap_faulted = True
                            elif gap_violations >= self._mit_stream_max_failures and self._mit_stream_fault is None:
                                self._mit_stream_fault = RuntimeError(
                                    f"{gap_violations} consecutive completed-send gap violations; "
                                    f"latest gap was {completed_gap_s:.3f}s, exceeding {max_gap_s:.3f}s"
                                )
                                gap_faulted = True
                            else:
                                gap_faulted = False
                        else:
                            self._mit_stream_consecutive_gap_violations = 0
                            gap_violations = 0
                            gap_faulted = False
                    if (
                        completed_gap_s is not None
                        and completed_gap_s > max_gap_s
                        and (gap_violations == 1 or gap_faulted)
                    ):
                        log = logger.error if gap_faulted else logger.warning
                        log(
                            "Motorbridge MIT command stream on %s recovered after a %.3fs gap, exceeding "
                            "%.3fs (completed-gap violation %d/%d).",
                            self.port,
                            completed_gap_s,
                            max_gap_s,
                            gap_violations,
                            self._mit_stream_max_failures,
                        )
            except Exception as exc:
                with self._mit_stream_lock:
                    self._mit_stream_consecutive_failures += 1
                    failures = self._mit_stream_consecutive_failures
                    self._mit_stream_error = exc
                    if failures >= self._mit_stream_max_failures and self._mit_stream_fault is None:
                        self._mit_stream_fault = RuntimeError(
                            f"{failures} consecutive serial-send failures; latest error: {exc}"
                        )
                if failures == 1 or failures == self._mit_stream_max_failures:
                    logger.error(
                        "Motorbridge MIT command stream failure %d/%d on %s: %s",
                        failures,
                        self._mit_stream_max_failures,
                        self.port,
                        exc,
                    )

            next_send_s += period_s
            now_s = time.monotonic()
            if next_send_s < now_s - period_s:
                next_send_s = now_s
            self._mit_stream_stop.wait(max(0.0, next_send_s - now_s))

    def check_mit_command_stream(self, *, max_gap_s: float | None = None) -> None:
        if self._mit_stream_thread is None:
            return
        if not self._mit_stream_thread.is_alive():
            raise RuntimeError("Motorbridge MIT command stream stopped unexpectedly.")
        with self._mit_stream_lock:
            failures = self._mit_stream_consecutive_failures
            error = self._mit_stream_error
            fault = self._mit_stream_fault
            max_failures = self._mit_stream_max_failures
            last_send_s = self._mit_stream_last_send_s
        if fault is not None:
            raise RuntimeError(
                "Motorbridge MIT command stream latched a safety fault: "
                f"{fault}. Diagnostics: {self.mit_command_stream_diagnostics()}"
            )
        if failures >= max_failures:
            raise RuntimeError(
                f"Motorbridge MIT command stream failed {failures} consecutive times: {error}. "
                f"Diagnostics: {self.mit_command_stream_diagnostics()}"
            )
        if max_gap_s is not None:
            if not math.isfinite(max_gap_s) or max_gap_s <= 0:
                raise ValueError("MIT command stream max_gap_s must be positive and finite.")
            if last_send_s is None:
                raise RuntimeError("Motorbridge MIT command stream has not sent its first command.")
            gap_s = time.monotonic() - last_send_s
            if gap_s > max_gap_s:
                raise RuntimeError(
                    f"Motorbridge MIT command stream gap is {gap_s:.3f}s, exceeding {max_gap_s:.3f}s. "
                    f"Diagnostics: {self.mit_command_stream_diagnostics()}"
                )

    def stop_mit_command_stream(self) -> None:
        self._mit_stream_stop.set()
        thread = self._mit_stream_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._mit_stream_thread = None
        with self._mit_stream_lock:
            self._mit_stream_commands = {}
            self._mit_stream_commands_updated_s = None
            self._mit_stream_hz = 0.0
            self._mit_stream_max_gap_s = 0.05
            self._mit_stream_hard_gap_s = 0.5
            self._mit_stream_consecutive_failures = 0
            self._mit_stream_consecutive_gap_violations = 0
            self._mit_stream_error = None
            self._mit_stream_fault = None
            self._mit_stream_last_send_s = None
            self._mit_stream_started_s = None
            self._mit_stream_total_sends = 0
            self._mit_stream_last_send_duration_s = 0.0
            self._mit_stream_max_send_duration_s = 0.0
            self._mit_stream_max_send_duration_at_s = None
            self._mit_stream_max_completed_gap_s = 0.0
            self._mit_stream_max_completed_gap_at_s = None
            self._mit_stream_last_gap_violation_s = None
            self._mit_stream_last_gap_violation_duration_s = 0.0
