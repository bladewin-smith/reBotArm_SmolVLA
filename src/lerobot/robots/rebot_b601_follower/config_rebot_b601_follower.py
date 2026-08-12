#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


DEFAULT_MOTOR_CONFIG: dict[str, tuple[int, int, str]] = {
    "joint_1": (0x01, 0x11, "dm4340"),
    "joint_2": (0x02, 0x12, "dm4340"),
    "joint_3": (0x03, 0x13, "dm4340"),
    "joint_4": (0x04, 0x14, "dm4310"),
    "joint_5": (0x05, 0x15, "dm4310"),
    "joint_6": (0x06, 0x16, "dm4310"),
    "gripper": (0x07, 0x17, "dm4310"),
}

DEFAULT_JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "joint_1": (-180.0, 180.0),
    "joint_2": (-120.0, 120.0),
    "joint_3": (-150.0, 150.0),
    "joint_4": (-150.0, 150.0),
    "joint_5": (-180.0, 180.0),
    "joint_6": (-180.0, 180.0),
    "gripper": (-65.0, 0.0),
}


@dataclass
class RebotB601FollowerConfigBase:
    """reBot Arm B601-DM follower configuration.

    The B601-DM arm uses Damiao motors. The default transport follows Seeed's
    working reBotArm_control_py stack: motorbridge DM serial on /dev/ttyACM*.
    Set ``transport="socketcan"`` only when using direct SocketCAN wiring.
    """

    port: str
    transport: str = "motorbridge"
    motorbridge_baudrate: int = 921600
    can_interface: str = "socketcan"
    use_can_fd: bool = False
    can_bitrate: int = 1000000
    can_data_bitrate: int = 5000000
    disable_torque_on_disconnect: bool = True
    max_relative_target: float | dict[str, float] | None = 15.0
    gripper_action_scale: float = 1.0
    gripper_action_offset: float = 0.0
    gripper_leader_close_pos: float | None = 5.0
    gripper_leader_open_pos: float | None = -310.0
    gripper_follower_close_pos: float | None = 10.0
    gripper_follower_open_pos: float | None = -320.0
    gripper_max_relative_target: float | None = 60.0
    gripper_min_pos: float | None = -330.0
    gripper_max_pos: float | None = 10.0
    gripper_position_kp: float | None = 35.0
    gripper_position_kd: float | None = 0.8
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    motor_config: dict[str, tuple[int, int, str]] = field(default_factory=lambda: DEFAULT_MOTOR_CONFIG.copy())
    joint_limits: dict[str, tuple[float, float]] = field(default_factory=lambda: DEFAULT_JOINT_LIMITS.copy())

    position_kp: list[float] | float = field(
        default_factory=lambda: [120.0, 120.0, 100.0, 45.0, 35.0, 30.0, 20.0]
    )
    position_kd: list[float] | float = field(default_factory=lambda: [3.0, 3.0, 2.5, 1.0, 0.6, 0.6, 0.4])


@RobotConfig.register_subclass("rebot_b601_follower")
@dataclass
class RebotB601FollowerConfig(RobotConfig, RebotB601FollowerConfigBase):
    pass
