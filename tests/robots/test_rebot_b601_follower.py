#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");

from unittest.mock import MagicMock, patch

import pytest

from lerobot.robots.rebot_b601_follower import RebotB601Follower, RebotB601FollowerConfig


@pytest.fixture
def follower() -> RebotB601Follower:
    config = RebotB601FollowerConfig(
        port="/dev/null",
        max_relative_target=5.0,
        gripper_max_relative_target=60.0,
        safety_recovery_step_deg=1.0,
        safety_recovery_hz=100.0,
        safety_recovery_timeout_s=1.0,
        safety_recovery_tolerance_deg=0.1,
    )
    robot = RebotB601Follower(config)
    robot.bus = MagicMock()
    robot.bus.is_connected = True
    robot.bus.motors = {motor_name: MagicMock() for motor_name in config.motor_config}
    return robot


def _states(positions: dict[str, float]) -> dict[str, dict[str, float]]:
    return {
        joint: {"position": position, "velocity": 0.0, "torque": 0.0} for joint, position in positions.items()
    }


def test_safety_hold_freezes_arm_but_not_gripper(follower: RebotB601Follower) -> None:
    follower._last_observation_states = _states({joint: 0.0 for joint in follower.bus.motors})

    sent_action = follower.send_action({"joint_1.pos": 30.0, "gripper.pos": -310.0})

    assert follower.safety_fault_active
    assert sent_action["joint_1.pos"] == 0.0
    assert sent_action["gripper.pos"] == -60.0
    commands = follower.bus.sync_write_mit.call_args.args[0]
    assert commands["joint_1"][2] == 0.0
    assert commands["gripper"][2] == -60.0


def test_recover_to_episode_start_pose_uses_bounded_steps(follower: RebotB601Follower) -> None:
    positions = {joint: 0.0 for joint in follower.bus.motors}
    follower._episode_start_pos = positions | {"joint_1": 2.0}
    follower._safety_fault_active = True

    follower.bus.sync_read_all_states.side_effect = lambda: _states(positions)

    def update_positions(commands):
        for joint, command in commands.items():
            positions[joint] = command[2]

    follower.bus.sync_write_mit.side_effect = update_positions

    with patch("lerobot.robots.rebot_b601_follower.rebot_b601_follower.time.sleep"):
        assert follower.recover_to_episode_start_pose()

    joint_1_targets = [call.args[0]["joint_1"][2] for call in follower.bus.sync_write_mit.call_args_list]
    assert joint_1_targets == [1.0, 2.0, 2.0]
    assert not follower.safety_fault_active


def test_recovery_can_be_cancelled_before_motion(follower: RebotB601Follower) -> None:
    follower._episode_start_pos = {joint: 0.0 for joint in follower.bus.motors}

    assert not follower.recover_to_episode_start_pose(should_stop=lambda: True)
    follower.bus.sync_write_mit.assert_not_called()
