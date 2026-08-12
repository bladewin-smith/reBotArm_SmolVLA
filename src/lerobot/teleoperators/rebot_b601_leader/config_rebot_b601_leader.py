#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");

from dataclasses import dataclass, field
from typing import Literal

from lerobot.robots.rebot_b601_follower.config_rebot_b601_follower import DEFAULT_MOTOR_CONFIG

from ..config import TeleoperatorConfig


@dataclass
class RebotB601LeaderConfigBase:
    """reBot Arm B601-DM leader configuration.

    ``manual_control_mode="impedance"`` keeps the leader arm enabled with low
    stiffness and damping, making it easier to guide than a fully stiff arm while
    avoiding the completely limp feel of disabled torque.
    """

    port: str
    transport: Literal["motorbridge", "socketcan"] = "motorbridge"
    motorbridge_baudrate: int = 921600
    can_interface: str = "socketcan"
    use_can_fd: bool = False
    can_bitrate: int = 1000000
    can_data_bitrate: int = 5000000
    handshake: bool = True
    motor_config: dict[str, tuple[int, int, str]] = field(default_factory=lambda: DEFAULT_MOTOR_CONFIG.copy())

    manual_control_mode: Literal["disabled", "impedance", "stiff"] = "impedance"

    impedance_kp: list[float] | float = field(default_factory=lambda: [4.0, 4.0, 3.5, 1.5, 1.0, 1.0, 0.6])
    impedance_kd: list[float] | float = field(default_factory=lambda: [0.45, 0.45, 0.35, 0.18, 0.12, 0.12, 0.08])
    stiff_kp: list[float] | float = field(default_factory=lambda: [60.0, 60.0, 50.0, 25.0, 20.0, 18.0, 10.0])
    stiff_kd: list[float] | float = field(default_factory=lambda: [2.0, 2.0, 1.5, 0.6, 0.4, 0.4, 0.2])


@TeleoperatorConfig.register_subclass("rebot_b601_leader")
@dataclass
class RebotB601LeaderConfig(TeleoperatorConfig, RebotB601LeaderConfigBase):
    pass
