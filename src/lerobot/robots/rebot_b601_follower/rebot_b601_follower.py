#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");

import logging
import math
import time
from functools import cached_property
from typing import Any, Callable

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
        self._safety_fault_active = False
        self._episode_start_pos: dict[str, float] | None = None
        self._runtime_error_hold_done = False
        self._gripper_contact_hold_pos: float | None = None
        self._gripper_closing_started_s: float | None = None
        self._gripper_contact_samples = 0
        self._validate_gripper_control_config()
        self._validate_safety_config()

    def _validate_gripper_control_config(self) -> None:
        if self.config.gripper_control_mode not in {"position", "torque_limited_close"}:
            raise ValueError("gripper_control_mode must be 'position' or 'torque_limited_close'.")

        nonnegative_values = {
            "gripper_close_kd": self.config.gripper_close_kd,
            "gripper_contact_min_closing_error_deg": self.config.gripper_contact_min_closing_error_deg,
            "gripper_contact_max_velocity_deg_s": self.config.gripper_contact_max_velocity_deg_s,
            "gripper_contact_min_torque": self.config.gripper_contact_min_torque,
            "gripper_contact_detection_delay_s": self.config.gripper_contact_detection_delay_s,
            "gripper_contact_hold_kp": self.config.gripper_contact_hold_kp,
            "gripper_contact_hold_kd": self.config.gripper_contact_hold_kd,
            "gripper_contact_hold_torque": self.config.gripper_contact_hold_torque,
            "gripper_contact_release_hysteresis_deg": self.config.gripper_contact_release_hysteresis_deg,
        }
        invalid = {
            name: value
            for name, value in nonnegative_values.items()
            if not math.isfinite(float(value)) or float(value) < 0
        }
        if invalid:
            raise ValueError(f"Gripper control values must be finite and nonnegative: {invalid}")
        if not math.isfinite(float(self.config.gripper_max_torque)) or self.config.gripper_max_torque <= 0:
            raise ValueError("gripper_max_torque must be positive and finite.")
        bounded_torques = {
            "gripper_close_torque": self.config.gripper_close_torque,
            "gripper_contact_hold_torque": self.config.gripper_contact_hold_torque,
        }
        invalid_torques = {
            name: value
            for name, value in bounded_torques.items()
            if not math.isfinite(float(value))
            or float(value) < 0
            or float(value) > float(self.config.gripper_max_torque)
        }
        if invalid_torques:
            raise ValueError(
                "Follower gripper torque values must be between zero and "
                f"gripper_max_torque={self.config.gripper_max_torque}: {invalid_torques}"
            )
        if self.config.gripper_contact_detection_samples < 1:
            raise ValueError("gripper_contact_detection_samples must be at least 1.")

    def _reset_gripper_contact_state(self) -> None:
        self._gripper_contact_hold_pos = None
        self._gripper_closing_started_s = None
        self._gripper_contact_samples = 0

    def _validate_safety_config(self) -> None:
        coupled_thresholds = {
            "safety_coupled_joint_2_min_deg": self.config.safety_coupled_joint_2_min_deg,
            "safety_coupled_joint_3_max_deg": self.config.safety_coupled_joint_3_max_deg,
        }
        invalid = {
            name: value
            for name, value in coupled_thresholds.items()
            if not math.isfinite(float(value))
        }
        if invalid:
            raise ValueError(f"Follower coupled-pose safety thresholds must be finite: {invalid}")

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

        self._runtime_error_hold_done = False
        logger.info(f"Connecting {self.name} on {self.config.port}...")
        self.bus.connect()

        if not self.is_calibrated and calibrate:
            logger.info("No B601-DM calibration file found; starting calibration.")
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        states = self.bus.sync_read_all_states()
        self._check_motorbridge_health(require_enabled=False)
        initial_goal = {
            motor: float(state.get("position", 0.0))
            for motor, state in states.items()
            if "position" in state
        }
        initial_commands = self._build_mit_commands(initial_goal)
        self.bus.enable_torque()
        try:
            if self.config.transport == "motorbridge" and self.config.command_stream_enabled:
                self.bus.start_mit_command_stream(
                    initial_commands,
                    hz=self.config.command_stream_hz,
                    max_consecutive_failures=self.config.command_stream_max_consecutive_failures,
                    max_gap_s=self.config.command_stream_max_gap_s,
                    hard_gap_s=self.config.command_stream_hard_gap_s,
                )
            else:
                self.bus.sync_write_mit(initial_commands)
            if self.config.transport == "motorbridge":
                time.sleep(0.05)
                self.bus.sync_read_all_states()
                self._check_motorbridge_health(require_enabled=True)
        except Exception:
            self.bus.disable_torque()
            raise
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            user_input = input(
                f"Press ENTER to use calibration file for id {self.id}, "
                "or type 'c' and press ENTER to run calibration: "
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
        if self.config.transport == "motorbridge" and self.config.command_stream_enabled:
            self.bus.check_mit_command_stream(max_gap_s=self.config.command_stream_max_gap_s)

        start = time.perf_counter()
        obs_dict: dict[str, Any] = {}
        states = self.bus.sync_read_all_states()
        self._check_motorbridge_health()
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

    @property
    def safety_fault_active(self) -> bool:
        return self._safety_fault_active

    def clear_safety_fault(self) -> None:
        self._safety_fault_active = False
        self._safety_hold_active = False

    def _check_motorbridge_health(self, *, require_enabled: bool = True) -> None:
        if self.config.transport == "motorbridge" and self.config.abort_on_motor_fault_status:
            self.bus.check_motor_status_codes(
                max_feedback_misses=self.config.motor_feedback_max_consecutive_misses,
                required_enabled_motors=list(self.bus.motors) if require_enabled else None,
            )

    def hold_after_runtime_error(self) -> None:
        hold_s = float(self.config.runtime_error_hold_s)
        if self._runtime_error_hold_done or hold_s <= 0 or self.config.transport != "motorbridge":
            return
        self._runtime_error_hold_done = True
        if not self.bus.is_connected or not self.bus.mit_command_stream_active:
            return
        arm_motors = [motor for motor in self.bus.motors if motor != "gripper"]
        try:
            self.bus.check_mit_command_stream(max_gap_s=self.config.command_stream_max_gap_s)
            self.bus.check_motor_status_codes(
                max_feedback_misses=self.config.motor_feedback_max_consecutive_misses,
                required_enabled_motors=arm_motors,
            )
        except Exception as exc:
            logger.error("Cannot hold follower after runtime error because motor control is unhealthy: %s", exc)
            return

        gripper_enabled = self.bus.motor_status_codes().get("gripper") == 0x1
        if gripper_enabled:
            gripper_state = self.bus.sync_read_all_states("gripper")["gripper"]
            self.bus.sync_write_mit(
                {
                    "gripper": (
                        float(self.config.gripper_contact_hold_kp),
                        float(self.config.gripper_contact_hold_kd),
                        float(gripper_state["position"]),
                        0.0,
                        0.0,
                    )
                }
            )
        else:
            logger.critical(
                "The follower gripper is disabled, but all six arm joints are still enabled. "
                "Continuing the arm command stream during the operator hold window."
            )

        logger.critical(
            "Recording failed while follower control is still healthy. Holding the last pose for %.1fs; "
            "support the arm or use the E-stop now.",
            hold_s,
        )
        deadline_s = time.monotonic() + hold_s
        next_health_check_s = 0.0
        while time.monotonic() < deadline_s:
            try:
                self.bus.check_mit_command_stream(max_gap_s=self.config.command_stream_max_gap_s)
                now_s = time.monotonic()
                if now_s >= next_health_check_s:
                    self.bus.sync_read_all_states()
                    self.bus.check_motor_status_codes(
                        max_feedback_misses=self.config.motor_feedback_max_consecutive_misses,
                        required_enabled_motors=arm_motors,
                    )
                    next_health_check_s = now_s + 0.25
            except Exception as exc:
                logger.error("Follower runtime-error hold ended early: %s", exc)
                break
            time.sleep(min(0.1, max(0.0, deadline_s - time.monotonic())))

    def mark_episode_start_pose(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        states = self.bus.sync_read_all_states()
        self._check_motorbridge_health()
        self._last_observation_states = {motor: state.copy() for motor, state in states.items()}
        self._episode_start_pos = {
            motor: float(state.get("position", 0.0)) for motor, state in states.items() if "position" in state
        }
        self.clear_safety_fault()
        logger.info("%s marked episode start pose: %s", self.name, self._episode_start_pos)
        return self._episode_start_pos.copy()

    def episode_start_pose(self) -> dict[str, float] | None:
        return None if self._episode_start_pos is None else self._episode_start_pos.copy()

    def _present_positions(self) -> dict[str, float]:
        states = self.bus.sync_read_all_states()
        self._check_motorbridge_health()
        self._last_observation_states = {motor: state.copy() for motor, state in states.items()}
        return {
            motor: float(state.get("position", 0.0))
            for motor, state in states.items()
            if "position" in state
        }

    def _build_mit_commands(
        self,
        goal_pos: dict[str, float],
        *,
        position_kp_scale: float = 1.0,
    ) -> dict[str, tuple[float, float, float, float, float]]:
        return {
            motor_name: (
                (
                    self.config.gripper_position_kp
                    if motor_name == "gripper" and self.config.gripper_position_kp is not None
                    else self._gain_for(self.config.position_kp, motor_name)
                )
                * position_kp_scale,
                self.config.gripper_position_kd
                if motor_name == "gripper" and self.config.gripper_position_kd is not None
                else self._gain_for(self.config.position_kd, motor_name),
                float(position_degrees),
                0.0,
                0.0,
            )
            for motor_name, position_degrees in goal_pos.items()
        }

    def _gripper_close_direction(self) -> float:
        close_pos = self.config.gripper_follower_close_pos
        open_pos = self.config.gripper_follower_open_pos
        if close_pos is not None and open_pos is not None and close_pos != open_pos:
            return 1.0 if close_pos > open_pos else -1.0
        return 1.0

    def _torque_limited_gripper_command(
        self,
        requested_target: float,
        limited_target: float,
    ) -> tuple[float, float, float, float, float] | None:
        if self.config.gripper_control_mode != "torque_limited_close":
            self._reset_gripper_contact_state()
            return None

        state = self._last_observation_states.get("gripper")
        if state is None:
            return None

        current_pos = float(state.get("position", limited_target))
        velocity = float(state.get("velocity", 0.0))
        torque = float(state.get("torque", 0.0))
        close_direction = self._gripper_close_direction()
        now_s = time.monotonic()

        if self._gripper_contact_hold_pos is not None:
            opening_from_contact = close_direction * (
                requested_target - self._gripper_contact_hold_pos
            ) < -float(self.config.gripper_contact_release_hysteresis_deg)
            if opening_from_contact:
                logger.info("%s follower gripper contact hold released by an opening command.", self.name)
                self._reset_gripper_contact_state()
                return None
            return (
                float(self.config.gripper_contact_hold_kp),
                float(self.config.gripper_contact_hold_kd),
                self._gripper_contact_hold_pos,
                0.0,
                close_direction * float(self.config.gripper_contact_hold_torque),
            )

        closing_error = close_direction * (requested_target - current_pos)
        if closing_error <= float(self.config.gripper_contact_min_closing_error_deg):
            self._gripper_closing_started_s = None
            self._gripper_contact_samples = 0
            return None

        if self._gripper_closing_started_s is None:
            self._gripper_closing_started_s = now_s

        detection_ready = (
            now_s - self._gripper_closing_started_s
            >= float(self.config.gripper_contact_detection_delay_s)
        )
        contact_sample = (
            detection_ready
            and abs(velocity) <= float(self.config.gripper_contact_max_velocity_deg_s)
            and abs(torque) >= float(self.config.gripper_contact_min_torque)
        )
        self._gripper_contact_samples = self._gripper_contact_samples + 1 if contact_sample else 0

        if self._gripper_contact_samples >= self.config.gripper_contact_detection_samples:
            self._gripper_contact_hold_pos = current_pos
            logger.info(
                "%s follower gripper contact detected at %.2f deg "
                "(requested=%.2f, velocity=%.2f deg/s, torque=%.2f); "
                "switching to bounded hold torque %.2f Nm.",
                self.name,
                current_pos,
                requested_target,
                velocity,
                torque,
                self.config.gripper_contact_hold_torque,
            )
            return (
                float(self.config.gripper_contact_hold_kp),
                float(self.config.gripper_contact_hold_kd),
                current_pos,
                0.0,
                close_direction * float(self.config.gripper_contact_hold_torque),
            )

        return (
            0.0,
            float(self.config.gripper_close_kd),
            limited_target,
            0.0,
            close_direction * float(self.config.gripper_close_torque),
        )

    def recover_to_episode_start_pose(
        self,
        should_stop: Callable[[], bool] | None = None,
        update_teleoperator: Callable[[], Any] | None = None,
    ) -> bool:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        if not self.config.safety_auto_recover_to_episode_start:
            logger.warning("%s automatic episode-start recovery is disabled.", self.name)
            return False
        if self._episode_start_pos is None:
            logger.warning("%s cannot recover: episode start pose has not been marked.", self.name)
            return False

        recovery_step_deg = float(self.config.safety_recovery_step_deg)
        recovery_hz = float(self.config.safety_recovery_hz)
        recovery_timeout_s = float(self.config.safety_recovery_timeout_s)
        recovery_tolerance_deg = float(self.config.safety_recovery_tolerance_deg)
        recovery_kp_scale = float(self.config.safety_recovery_position_kp_scale)
        if (
            not math.isfinite(recovery_step_deg)
            or recovery_step_deg <= 0
            or not math.isfinite(recovery_hz)
            or recovery_hz <= 0
            or not math.isfinite(recovery_timeout_s)
            or recovery_timeout_s <= 0
            or not math.isfinite(recovery_tolerance_deg)
            or recovery_tolerance_deg < 0
            or not math.isfinite(recovery_kp_scale)
            or not 0 < recovery_kp_scale <= 1
        ):
            logger.error("%s cannot recover: invalid safety recovery parameters.", self.name)
            return False

        recovery_joints = [
            joint for joint in self.config.safety_recovery_joints if joint in self._episode_start_pos
        ]
        if not recovery_joints:
            logger.warning("%s cannot recover: no valid recovery joints configured.", self.name)
            return False

        invalid_targets = {
            joint: self._episode_start_pos[joint]
            for joint in recovery_joints
            if not math.isfinite(self._episode_start_pos[joint])
        }
        if invalid_targets:
            logger.error("%s cannot recover: invalid episode start positions: %s", self.name, invalid_targets)
            return False

        logger.warning("%s recovering follower arm to episode start pose.", self.name)
        start_s = time.monotonic()
        period_s = 1.0 / recovery_hz
        while time.monotonic() - start_s < recovery_timeout_s:
            if should_stop is not None and should_stop():
                logger.warning("%s episode-start recovery was cancelled.", self.name)
                return False
            if update_teleoperator is not None:
                update_teleoperator()

            present_pos = self._present_positions()
            goal_pos: dict[str, float] = {}
            max_error = 0.0
            for joint in recovery_joints:
                current = present_pos[joint]
                target = self._episode_start_pos[joint]
                if not math.isfinite(current):
                    logger.error(
                        "%s cannot recover: invalid current position for %s: %s",
                        self.name,
                        joint,
                        current,
                    )
                    return False
                error = target - current
                max_error = max(max_error, abs(error))
                step = max(-recovery_step_deg, min(recovery_step_deg, error))
                goal_pos[joint] = current + step

            goal_pos = self._clamp_to_joint_limits(goal_pos)
            commands = self._build_mit_commands(
                goal_pos,
                position_kp_scale=recovery_kp_scale,
            )
            self.bus.sync_write_mit(commands)

            if max_error <= recovery_tolerance_deg:
                logger.info("%s recovered to episode start pose.", self.name)
                self.clear_safety_fault()
                return True
            time.sleep(period_s)

        logger.error("%s failed to recover to episode start pose before timeout.", self.name)
        return False

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

    def _coupled_pose_guard_triggered(self, goal_pos: dict[str, float]) -> bool:
        if not self.config.safety_coupled_pose_guard_enabled:
            return False
        if "joint_2" not in goal_pos or "joint_3" not in goal_pos:
            return False
        return (
            float(goal_pos["joint_2"]) <= float(self.config.safety_coupled_joint_2_min_deg)
            and float(goal_pos["joint_3"]) >= float(self.config.safety_coupled_joint_3_max_deg)
        )

    def _apply_coupled_pose_guard(
        self,
        goal_pos: dict[str, float],
        present_pos: dict[str, float],
    ) -> tuple[dict[str, float], bool]:
        if not self._coupled_pose_guard_triggered(goal_pos):
            return goal_pos, False

        self._safety_hold_active = True
        self._safety_fault_active = True
        logger.error(
            "%s coupled-pose safety hold: refusing target joint_2=%.2f deg, joint_3=%.2f deg "
            "because joint_2<=%.2f and joint_3>=%.2f describes the configured high-load, "
            "near-straight envelope. Holding the current arm pose and rerecording the episode.",
            self.name,
            float(goal_pos["joint_2"]),
            float(goal_pos["joint_3"]),
            float(self.config.safety_coupled_joint_2_min_deg),
            float(self.config.safety_coupled_joint_3_max_deg),
        )
        hold_joints = set(self.config.safety_hold_joints)
        guarded_goal = {
            motor: float(present_pos[motor]) if motor in hold_joints else target_pos
            for motor, target_pos in goal_pos.items()
        }
        return self._clamp_to_joint_limits(guarded_goal), True

    def send_action(self, action: RobotAction) -> RobotAction:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        if self.config.transport == "motorbridge" and self.config.command_stream_enabled:
            self.bus.check_mit_command_stream(max_gap_s=self.config.command_stream_max_gap_s)

        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}
        if "gripper" in goal_pos:
            goal_pos["gripper"] = self._map_gripper_goal(float(goal_pos["gripper"]))

        goal_pos = self._clamp_to_joint_limits(goal_pos)
        requested_goal_pos = goal_pos.copy()
        safety_hold = False
        present_pos: dict[str, float] | None = None

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
                self._safety_fault_active = True
                self._log_safety_hold(clamp_info)
                hold_joints = set(self.config.safety_hold_joints)
                goal_pos = {
                    motor: float(present_pos[motor]) if motor in hold_joints else target_pos
                    for motor, target_pos in goal_pos.items()
                }
                goal_pos = self._clamp_to_joint_limits(goal_pos)
            else:
                if self._safety_hold_active:
                    logger.info(
                        "%s safety hold cleared; follower target is back within safe range.",
                        self.name,
                    )
                self._safety_hold_active = False
                goal_pos = self._clamp_to_joint_limits(goal_pos)
        else:
            self._safety_hold_active = False

        if not safety_hold and self.config.safety_coupled_pose_guard_enabled:
            if present_pos is None:
                present_pos = (
                    {motor: state["position"] for motor, state in self._last_observation_states.items()}
                    if self._last_observation_states
                    else self.bus.sync_read("Present_Position")
                )
            goal_pos, safety_hold = self._apply_coupled_pose_guard(goal_pos, present_pos)

        commands = self._build_mit_commands(
            goal_pos,
            position_kp_scale=self.config.safety_hold_position_kp_scale if safety_hold else 1.0,
        )
        if "gripper" in goal_pos:
            gripper_command = self._torque_limited_gripper_command(
                requested_goal_pos["gripper"],
                goal_pos["gripper"],
            )
            if gripper_command is not None:
                commands["gripper"] = gripper_command
                if self._gripper_contact_hold_pos is not None:
                    goal_pos["gripper"] = self._gripper_contact_hold_pos
        self.bus.sync_write_mit(commands)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def disconnect(self) -> None:
        connected_cameras = [cam for cam in self.cameras.values() if cam.is_connected]
        if not self.bus.is_connected and not connected_cameras:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        for cam in connected_cameras:
            try:
                cam.disconnect()
            except Exception as exc:
                logger.warning("Failed to disconnect %s while keeping follower control active: %s", cam, exc)
        if self.bus.is_connected:
            self.bus.disconnect(self.config.disable_torque_on_disconnect)
        self._reset_gripper_contact_state()
        logger.info(f"{self} disconnected.")
