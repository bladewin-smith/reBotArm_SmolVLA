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


def test_policy_absolute_delta_threshold_allows_moderate_chunk_jump(
    follower: RebotB601Follower,
) -> None:
    follower.config.safety_hold_multi_joint_delta_deg = 20.0
    follower.config.safety_hold_single_joint_delta_deg = 30.0
    follower._last_observation_states = _states({joint: 0.0 for joint in follower.bus.motors})

    sent_action = follower.send_action({"joint_3.pos": -13.0, "joint_6.pos": 18.0})

    assert not follower.safety_fault_active
    assert sent_action["joint_3.pos"] == -5.0
    assert sent_action["joint_6.pos"] == 5.0


def test_policy_absolute_delta_threshold_holds_multiple_large_jumps(
    follower: RebotB601Follower,
) -> None:
    follower.config.safety_hold_multi_joint_delta_deg = 20.0
    follower.config.safety_hold_single_joint_delta_deg = 30.0
    follower._last_observation_states = _states({joint: 0.0 for joint in follower.bus.motors})

    sent_action = follower.send_action({"joint_3.pos": -21.0, "joint_6.pos": 22.0})

    assert follower.safety_fault_active
    assert sent_action["joint_3.pos"] == 0.0
    assert sent_action["joint_6.pos"] == 0.0


def test_policy_absolute_delta_threshold_holds_one_extreme_jump(
    follower: RebotB601Follower,
) -> None:
    follower.config.safety_hold_multi_joint_delta_deg = 20.0
    follower.config.safety_hold_single_joint_delta_deg = 30.0
    follower._last_observation_states = _states({joint: 0.0 for joint in follower.bus.motors})

    sent_action = follower.send_action({"joint_3.pos": -31.0})

    assert follower.safety_fault_active
    assert sent_action["joint_3.pos"] == 0.0


def test_coupled_pose_guard_holds_before_low_shoulder_straight_elbow_pose(
    follower: RebotB601Follower,
) -> None:
    follower.config.max_relative_target = None
    follower._last_observation_states = _states(
        {joint: 0.0 for joint in follower.bus.motors}
        | {"joint_2": -105.0, "joint_3": -20.0}
    )

    sent_action = follower.send_action(
        {"joint_2.pos": -115.0, "joint_3.pos": -10.0, "gripper.pos": -310.0}
    )

    assert follower.safety_fault_active
    assert sent_action["joint_2.pos"] == -105.0
    assert sent_action["joint_3.pos"] == -20.0
    assert sent_action["gripper.pos"] == -320.0
    commands = follower.bus.sync_write_mit.call_args.args[0]
    assert commands["joint_2"][2] == -105.0
    assert commands["joint_3"][2] == -20.0


def test_coupled_pose_guard_allows_low_shoulder_with_bent_elbow(
    follower: RebotB601Follower,
) -> None:
    follower.config.max_relative_target = None
    follower._last_observation_states = _states({joint: 0.0 for joint in follower.bus.motors})

    sent_action = follower.send_action({"joint_2.pos": -115.0, "joint_3.pos": -30.0})

    assert not follower.safety_fault_active
    assert sent_action == {"joint_2.pos": -115.0, "joint_3.pos": -30.0}


def test_gripper_closes_with_bounded_feedforward_torque(follower: RebotB601Follower) -> None:
    follower._last_observation_states = _states({"gripper": -100.0})

    sent_action = follower.send_action({"gripper.pos": 5.0})

    assert sent_action["gripper.pos"] == -40.0
    command = follower.bus.sync_write_mit.call_args.args[0]["gripper"]
    assert command == (0.0, 0.5, -40.0, 0.0, 1.0)


def test_policy_gripper_target_skips_leader_mapping(follower: RebotB601Follower) -> None:
    follower.config.gripper_map_leader_to_follower = False
    follower.config.max_relative_target = None
    follower._last_observation_states = _states({"gripper": -100.0})

    sent_action = follower.send_action({"gripper.pos": -123.0})

    assert sent_action["gripper.pos"] == -123.0
    command = follower.bus.sync_write_mit.call_args.args[0]["gripper"]
    assert command == (35.0, 0.8, -123.0, 0.0, 0.0)


def test_out_of_range_gripper_slews_toward_limit_without_jump(follower: RebotB601Follower) -> None:
    follower.config.gripper_control_mode = "position"
    follower.config.gripper_max_relative_target = 10.0
    follower._last_observation_states = _states({"gripper": 83.0})

    sent_action = follower.send_action({"gripper.pos": 0.0})

    assert sent_action["gripper.pos"] == 73.0
    command = follower.bus.sync_write_mit.call_args.args[0]["gripper"]
    assert command[2] == 73.0


def test_startup_position_guard_reports_out_of_range_gripper(follower: RebotB601Follower) -> None:
    violations = follower._startup_position_violations(_states({"joint_1": 0.0, "gripper": 83.0}))

    assert set(violations) == {"gripper"}
    assert violations["gripper"] == {
        "position": 83.0,
        "min": -330.0,
        "max": 10.0,
        "tolerance": 5.0,
    }


def test_gripper_contact_switches_to_bounded_hold_and_releases_on_open(
    follower: RebotB601Follower,
) -> None:
    follower.config.gripper_contact_detection_delay_s = 0.0
    follower.config.gripper_contact_detection_samples = 2
    follower._last_observation_states = {
        "gripper": {"position": -100.0, "velocity": 0.0, "torque": 0.5}
    }

    follower.send_action({"gripper.pos": 5.0})
    follower._last_observation_states["gripper"]["position"] = -80.0
    follower.send_action({"gripper.pos": 5.0})
    held_action = follower.send_action({"gripper.pos": 5.0})

    assert held_action["gripper.pos"] == -80.0
    hold_command = follower.bus.sync_write_mit.call_args.args[0]["gripper"]
    assert hold_command == (5.0, 1.0, -80.0, 0.0, 0.3)

    released_action = follower.send_action({"gripper.pos": -310.0})

    assert follower._gripper_contact_hold_pos is None
    assert released_action["gripper.pos"] == -140.0
    release_command = follower.bus.sync_write_mit.call_args.args[0]["gripper"]
    assert release_command == (35.0, 0.8, -140.0, 0.0, 0.0)


def test_gripper_does_not_latch_contact_at_open_limit_without_travel(
    follower: RebotB601Follower,
) -> None:
    follower.config.gripper_contact_detection_delay_s = 0.0
    follower.config.gripper_contact_detection_samples = 1
    follower._last_observation_states = {
        "gripper": {"position": -320.0, "velocity": 0.0, "torque": 0.0}
    }

    for _ in range(3):
        sent_action = follower.send_action({"gripper.pos": 5.0})

    assert follower._gripper_contact_hold_pos is None
    assert follower._gripper_closing_start_pos == -320.0
    assert sent_action["gripper.pos"] == -260.0
    command = follower.bus.sync_write_mit.call_args.args[0]["gripper"]
    assert command == (0.0, 0.5, -260.0, 0.0, 1.0)


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
