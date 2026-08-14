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
    command_stream_enabled: bool = True
    command_stream_hz: float = 500.0
    command_stream_max_consecutive_failures: int = 5
    command_stream_max_gap_s: float = 0.05
    command_stream_hard_gap_s: float = 0.5
    abort_on_motor_fault_status: bool = True
    motor_feedback_max_consecutive_misses: int = 3
    runtime_error_hold_s: float = 15.0
    max_relative_target: float | dict[str, float] | None = 15.0
    safety_hold_on_relative_clamp: bool = True
    safety_hold_clamp_joint_count: int = 2
    safety_hold_clamp_ratio: float = 1.2
    safety_hold_single_joint_ratio: float = 3.0
    safety_hold_log_interval_s: float = 1.0
    safety_hold_position_kp_scale: float = 0.65
    safety_hold_joints: list[str] = field(
        default_factory=lambda: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    )
    # Avoid the high-load, near-straight posture observed immediately before a
    # shared follower disable: shoulder joint_2 low while elbow joint_3 is
    # almost straight. These thresholds are expressed in calibrated degrees.
    safety_coupled_pose_guard_enabled: bool = False
    safety_coupled_joint_2_min_deg: float = -110.0
    safety_coupled_joint_3_max_deg: float = -15.0
    safety_abort_episode_on_hold: bool = True
    safety_auto_recover_to_episode_start: bool = True
    safety_recovery_joints: list[str] = field(
        default_factory=lambda: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    )
    safety_recovery_step_deg: float = 1.5
    safety_recovery_hz: float = 20.0
    safety_recovery_timeout_s: float = 25.0
    safety_recovery_tolerance_deg: float = 3.0
    safety_recovery_position_kp_scale: float = 0.5
    safety_wait_for_leader_start: bool = True
    safety_leader_start_tolerance_deg: float = 8.0
    safety_leader_start_timeout_s: float = 45.0
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
    # Seeed's working grasp driver closes the DM gripper with bounded MIT
    # feedforward torque, then switches to a lower holding torque on contact.
    # Keep ``position`` available for comparison and hardware diagnosis.
    gripper_control_mode: str = "torque_limited_close"
    gripper_max_torque: float = 1.5
    gripper_close_torque: float = 1.0
    gripper_close_kd: float = 0.5
    gripper_contact_min_closing_error_deg: float = 8.0
    gripper_contact_max_velocity_deg_s: float = 3.0
    gripper_contact_min_torque: float = 0.0
    gripper_contact_detection_delay_s: float = 0.25
    gripper_contact_detection_samples: int = 3
    gripper_contact_hold_kp: float = 5.0
    gripper_contact_hold_kd: float = 1.0
    gripper_contact_hold_torque: float = 0.30
    gripper_contact_release_hysteresis_deg: float = 8.0
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
