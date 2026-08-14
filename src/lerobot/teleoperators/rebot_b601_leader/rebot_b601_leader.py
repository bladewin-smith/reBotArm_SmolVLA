#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");

import logging
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

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
        if self.config.transport not in {"motorbridge", "socketcan"}:
            raise ValueError("B601 leader transport must be 'motorbridge' or 'socketcan'.")
        if self.config.manual_control_mode not in {"disabled", "impedance", "stiff", "gravity_comp"}:
            raise ValueError(
                "B601 leader manual_control_mode must be 'disabled', 'impedance', 'stiff', or 'gravity_comp'."
            )
        if self.config.gravity_comp_gripper_mode not in {
            "disabled",
            "zero_torque",
            "low_stiffness",
            "force_assist",
        }:
            raise ValueError(
                "B601 leader gravity_comp_gripper_mode must be 'disabled', 'zero_torque', "
                "'low_stiffness', or 'force_assist'."
            )

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
        self._gravity_model: Any | None = None
        self._gravity_data: Any | None = None
        self._compute_generalized_gravity: Any | None = None

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
                f"Press ENTER to use calibration file for id {self.id}, "
                "or type 'c' and press ENTER to run calibration: "
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
        elif self.config.manual_control_mode == "gravity_comp":
            self._ensure_gravity_comp_ready()
            self.bus.configure_motors()
            self.bus.disable_torque()
            self.bus.enable_torque(self._gravity_comp_enabled_joints())
            time.sleep(0.05)
            states = self.bus.sync_read_all_states()
            self._check_motorbridge_health()
            commands = self._send_gravity_comp(states)
            self._start_command_stream(commands)
        else:
            self.bus.configure_motors()
            states = self.bus.sync_read_all_states()
            self._check_motorbridge_health()
            commands = self._send_compliance(states)
            self._start_command_stream(commands)

    def _start_command_stream(
        self,
        commands: dict[str, tuple[float, float, float, float, float]] | None,
    ) -> None:
        if (
            commands is None
            or self.config.transport != "motorbridge"
            or not self.config.command_stream_enabled
        ):
            return
        try:
            self.bus.start_mit_command_stream(
                commands,
                hz=self.config.command_stream_hz,
                max_consecutive_failures=self.config.command_stream_max_consecutive_failures,
                max_gap_s=self.config.command_stream_max_gap_s,
                hard_gap_s=self.config.command_stream_hard_gap_s,
            )
        except Exception:
            self.bus.disable_torque()
            raise

    def setup_motors(self) -> None:
        raise NotImplementedError("Use the Damiao/reBot vendor tools to configure B601-DM CAN IDs.")

    def _check_motorbridge_health(self) -> None:
        if self.config.transport == "motorbridge" and self.config.abort_on_motor_fault_status:
            required_enabled_motors = None
            if self.config.manual_control_mode == "gravity_comp":
                required_enabled_motors = self._gravity_comp_enabled_joints()
            self.bus.check_motor_status_codes(
                max_feedback_misses=self.config.motor_feedback_max_consecutive_misses,
                required_enabled_motors=required_enabled_motors,
            )

    def _gain_for(self, values: list[float] | float, motor_name: str) -> float:
        if isinstance(values, list):
            names = list(self.bus.motors)
            return float(values[names.index(motor_name)])
        return float(values)

    def _gravity_gain_for(self, values: list[float] | float, motor_name: str) -> float:
        if not isinstance(values, list):
            return float(values)
        motor_names = list(self.bus.motors)
        enabled_names = self._gravity_comp_enabled_joints()
        if len(values) == len(motor_names):
            return float(values[motor_names.index(motor_name)])
        if len(values) == len(enabled_names):
            return float(values[enabled_names.index(motor_name)])
        raise ValueError(
            "Gravity compensation gain length must match either all motors "
            f"({len(motor_names)}) or enabled joints ({len(enabled_names)})."
        )

    def _gravity_kp_for(self, motor_name: str) -> float:
        if motor_name == "gripper" and self.config.gravity_comp_gripper_mode == "zero_torque":
            return 0.0
        if motor_name == "gripper" and self.config.gravity_comp_gripper_mode == "force_assist":
            return 0.0 if self.config.gravity_comp_gripper_kp is None else float(self.config.gravity_comp_gripper_kp)
        if (
            motor_name == "gripper"
            and self._uses_gripper_low_stiffness()
            and self.config.gravity_comp_gripper_kp is not None
        ):
            return float(self.config.gravity_comp_gripper_kp)
        return self._gravity_gain_for(self.config.gravity_comp_kp, motor_name)

    def _gravity_kd_for(self, motor_name: str) -> float:
        if motor_name == "gripper" and self.config.gravity_comp_gripper_mode == "zero_torque":
            return 0.0
        if motor_name == "gripper" and self.config.gravity_comp_gripper_mode == "force_assist":
            return 0.0 if self.config.gravity_comp_gripper_kd is None else float(self.config.gravity_comp_gripper_kd)
        if (
            motor_name == "gripper"
            and self._uses_gripper_low_stiffness()
            and self.config.gravity_comp_gripper_kd is not None
        ):
            return float(self.config.gravity_comp_gripper_kd)
        return self._gravity_gain_for(self.config.gravity_comp_kd, motor_name)

    def _gripper_assist_torque(self, state: dict[str, Any]) -> float:
        if self.config.gravity_comp_gripper_mode != "force_assist" and (
            not self.config.gravity_comp_gripper_assist or not self.config.gravity_comp_gripper_force_assist
        ):
            return 0.0

        position = float(state.get("position", 0.0))
        velocity = float(state.get("velocity", 0.0))
        threshold = float(self.config.gravity_comp_gripper_velocity_threshold)
        torque = 0.0

        if position > self.config.gravity_comp_gripper_open_limit + self.config.gravity_comp_gripper_limit_margin:
            torque -= float(self.config.gravity_comp_gripper_open_bias_torque)

        if velocity < -threshold:
            torque -= float(self.config.gravity_comp_gripper_open_motion_torque)
        elif velocity > threshold:
            torque += float(self.config.gravity_comp_gripper_close_motion_torque)

        return float(
            np.clip(
                torque,
                -self.config.gravity_comp_gripper_torque_limit,
                self.config.gravity_comp_gripper_torque_limit,
            )
        )

    def _uses_gripper_low_stiffness(self) -> bool:
        return self.config.gravity_comp_gripper_mode in {"low_stiffness", "force_assist"} or (
            self.config.gravity_comp_gripper_assist and self.config.gravity_comp_gripper_mode != "disabled"
        )

    def _gravity_comp_enabled_joints(self) -> list[str]:
        enabled_joints = list(self.config.gravity_comp_enabled_joints)
        if self.config.gravity_comp_gripper_mode != "disabled" and "gripper" not in enabled_joints:
            enabled_joints.append("gripper")
        return enabled_joints

    @staticmethod
    def _default_sdk_root() -> Path:
        current = Path(__file__).resolve()
        for parent in current.parents:
            candidate = parent / "rebot_grasp" / "sdk" / "reBotArm_control_py"
            if (candidate / "src" / "reBotArm_control_py").exists():
                return candidate
        return Path.home() / "ws" / "rebot_grasp" / "sdk" / "reBotArm_control_py"

    def _ensure_gravity_comp_ready(self) -> None:
        if self._compute_generalized_gravity is not None:
            return
        if self.config.transport != "motorbridge":
            raise ValueError("gravity_comp mode is supported for the B601-DM motorbridge transport.")

        sdk_root = (
            Path(self.config.gravity_comp_sdk_root)
            if self.config.gravity_comp_sdk_root
            else self._default_sdk_root()
        )
        if sdk_root.exists():
            sys.path.insert(0, str(sdk_root / "src"))

        try:
            from reBotArm_control_py.dynamics import compute_generalized_gravity, load_dynamics_model
        except ImportError as exc:
            raise ImportError(
                "gravity_comp mode requires Seeed reBotArm_control_py dynamics. "
                "Install the SDK or set --teleop.gravity_comp_sdk_root=/path/to/reBotArm_control_py."
            ) from exc

        self._gravity_model = load_dynamics_model()
        self._gravity_data = self._gravity_model.createData()
        self._compute_generalized_gravity = compute_generalized_gravity

        motor_names = list(self.bus.motors)
        enabled_joints = self._gravity_comp_enabled_joints()
        for motor_name in enabled_joints:
            if motor_name not in motor_names:
                raise ValueError(f"Unknown gravity compensation joint {motor_name!r}. Available: {motor_names}")
        for motor_name in self._gravity_comp_torque_joints():
            if motor_name not in enabled_joints:
                raise ValueError(
                    f"Gravity compensation torque joint {motor_name!r} must also be enabled. "
                    f"Enabled joints: {enabled_joints}"
                )

    def _gravity_comp_torque_joints(self) -> list[str]:
        if self.config.gravity_comp_torque_joints is not None:
            return list(self.config.gravity_comp_torque_joints)
        return [motor_name for motor_name in self._gravity_comp_enabled_joints() if motor_name != "gripper"]

    def _send_gravity_comp(
        self,
        states: dict[str, dict[str, Any]],
    ) -> dict[str, tuple[float, float, float, float, float]] | None:
        if self.config.manual_control_mode != "gravity_comp":
            return None
        self._ensure_gravity_comp_ready()

        motor_names = list(self.bus.motors)
        q_rad = np.asarray(
            [np.radians(float(states.get(motor_name, {}).get("position", 0.0))) for motor_name in motor_names],
            dtype=np.float64,
        )
        tau_g = self._compute_generalized_gravity(model=self._gravity_model, q=q_rad, data=self._gravity_data)

        commands = {}
        torque_joints = set(self._gravity_comp_torque_joints())
        for motor_name in self._gravity_comp_enabled_joints():
            index = motor_names.index(motor_name)
            tau = 0.0
            if motor_name in torque_joints:
                tau = float(
                    np.clip(
                        self.config.gravity_comp_torque_scale * tau_g[index],
                        -self.config.gravity_comp_torque_limit,
                        self.config.gravity_comp_torque_limit,
                    )
                )
            elif motor_name == "gripper":
                tau = self._gripper_assist_torque(states.get(motor_name, {}))
            commands[motor_name] = (
                self._gravity_kp_for(motor_name),
                self._gravity_kd_for(motor_name),
                float(states.get(motor_name, {}).get("position", 0.0)),
                0.0,
                tau,
            )
        self.bus.sync_write_mit(commands)
        return commands

    def _send_compliance(
        self,
        states: dict[str, dict[str, Any]],
    ) -> dict[str, tuple[float, float, float, float, float]] | None:
        if self.config.manual_control_mode == "disabled":
            return None

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
        return commands

    def get_action(self) -> RobotAction:
        start = time.perf_counter()
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        if self.config.transport == "motorbridge" and self.config.command_stream_enabled:
            self.bus.check_mit_command_stream(max_gap_s=self.config.command_stream_max_gap_s)

        states = self.bus.sync_read_all_states()
        self._check_motorbridge_health()
        if self.config.manual_control_mode == "gravity_comp":
            self._send_gravity_comp(states)
        elif self.config.manual_control_mode != "disabled":
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
