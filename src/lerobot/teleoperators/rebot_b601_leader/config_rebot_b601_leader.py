#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");

from dataclasses import dataclass, field

from lerobot.robots.rebot_b601_follower.config_rebot_b601_follower import DEFAULT_MOTOR_CONFIG

from ..config import TeleoperatorConfig


@dataclass
class RebotB601LeaderConfigBase:
    """reBot Arm B601-DM leader configuration.

    ``manual_control_mode="gravity_comp"`` keeps the leader arm enabled with
    gravity feedforward, making it easier to guide during teleoperation data
    collection.
    """

    port: str
    transport: str = "motorbridge"
    motorbridge_baudrate: int = 921600
    can_interface: str = "socketcan"
    use_can_fd: bool = False
    can_bitrate: int = 1000000
    can_data_bitrate: int = 5000000
    handshake: bool = True
    command_stream_enabled: bool = True
    command_stream_hz: float = 100.0
    command_stream_max_consecutive_failures: int = 5
    command_stream_max_gap_s: float = 0.25
    abort_on_motor_fault_status: bool = True
    motor_feedback_max_consecutive_misses: int = 3
    motor_config: dict[str, tuple[int, int, str]] = field(default_factory=lambda: DEFAULT_MOTOR_CONFIG.copy())

    manual_control_mode: str = "impedance"

    impedance_kp: list[float] | float = field(default_factory=lambda: [4.0, 4.0, 3.5, 1.5, 1.0, 1.0, 0.6])
    impedance_kd: list[float] | float = field(default_factory=lambda: [0.45, 0.45, 0.35, 0.18, 0.12, 0.12, 0.08])
    stiff_kp: list[float] | float = field(default_factory=lambda: [60.0, 60.0, 50.0, 25.0, 20.0, 18.0, 10.0])
    stiff_kd: list[float] | float = field(default_factory=lambda: [2.0, 2.0, 1.5, 0.6, 0.4, 0.4, 0.2])

    gravity_comp_sdk_root: str | None = None
    gravity_comp_enabled_joints: list[str] = field(
        default_factory=lambda: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    )
    gravity_comp_torque_joints: list[str] | None = None
    gravity_comp_torque_scale: float = 0.95
    gravity_comp_torque_limit: float = 8.0
    gravity_comp_kp: list[float] | float = 1.9
    gravity_comp_kd: list[float] | float = 0.75

    gravity_comp_gripper_mode: str = "force_assist"
    gravity_comp_gripper_assist: bool = False
    gravity_comp_gripper_kp: float | None = 0.0
    gravity_comp_gripper_kd: float | None = 0.0
    gravity_comp_gripper_force_assist: bool = True
    gravity_comp_gripper_open_bias_torque: float = 0.09
    gravity_comp_gripper_open_motion_torque: float = 0.0
    gravity_comp_gripper_close_motion_torque: float = 0.0
    gravity_comp_gripper_velocity_threshold: float = 2.0
    gravity_comp_gripper_torque_limit: float = 0.15
    gravity_comp_gripper_open_limit: float = -65.0
    gravity_comp_gripper_limit_margin: float = 3.0


@TeleoperatorConfig.register_subclass("rebot_b601_leader")
@dataclass
class RebotB601LeaderConfig(TeleoperatorConfig, RebotB601LeaderConfigBase):
    pass
