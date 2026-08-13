#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");

import logging
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.rebot_motorbridge import RebotMotorbridgeBus


@pytest.fixture
def bus() -> RebotMotorbridgeBus:
    motors = {
        "joint_1": Motor(
            id=0x01,
            recv_id=0x11,
            model="dm4340",
            norm_mode=MotorNormMode.DEGREES,
        )
    }
    motor_bus = RebotMotorbridgeBus("/dev/null", motors)
    motor_bus.controller = MagicMock()
    motor_bus._motor_handles = {"joint_1": MagicMock()}
    motor_bus._is_connected = True
    yield motor_bus
    motor_bus.stop_mit_command_stream()


def test_mit_stream_repeats_latest_target_while_main_thread_waits(bus: RebotMotorbridgeBus) -> None:
    sent_targets: list[float] = []
    bus._send_mit_commands = lambda commands: sent_targets.append(commands["joint_1"][2])

    bus.start_mit_command_stream({"joint_1": (10.0, 1.0, 1.0, 0.0, 0.0)}, hz=200.0)
    time.sleep(0.025)
    bus.sync_write_mit({"joint_1": (10.0, 1.0, 8.0, 0.0, 0.0)})
    time.sleep(0.025)

    assert sent_targets.count(1.0) >= 2
    assert sent_targets.count(8.0) >= 2


def test_mit_stream_latches_consecutive_send_failure(bus: RebotMotorbridgeBus) -> None:
    calls = 0

    def fail_after_prime(commands) -> None:
        nonlocal calls
        calls += 1
        if calls > 1:
            raise OSError("serial write failed")

    bus._send_mit_commands = fail_after_prime
    bus.start_mit_command_stream(
        {"joint_1": (10.0, 1.0, 1.0, 0.0, 0.0)},
        hz=500.0,
        max_consecutive_failures=2,
    )
    deadline = time.monotonic() + 0.2
    while bus._mit_stream_fault is None and time.monotonic() < deadline:
        time.sleep(0.005)

    with pytest.raises(RuntimeError, match="2 consecutive serial-send failures"):
        bus.check_mit_command_stream()


def test_mit_stream_does_not_latch_one_recovered_gap(
    bus: RebotMotorbridgeBus, caplog: pytest.LogCaptureFixture
) -> None:
    calls = 0
    recovered = threading.Event()

    def delay_first_stream_send(commands) -> None:
        nonlocal calls
        calls += 1
        if calls == 2:
            time.sleep(0.06)
        elif calls > 2:
            recovered.set()

    bus._send_mit_commands = delay_first_stream_send
    with caplog.at_level(logging.WARNING):
        bus.start_mit_command_stream(
            {"joint_1": (10.0, 1.0, 1.0, 0.0, 0.0)},
            hz=200.0,
            max_consecutive_failures=2,
            max_gap_s=0.05,
        )
        assert recovered.wait(timeout=0.2)

    bus.check_mit_command_stream(max_gap_s=0.1)
    assert bus._mit_stream_fault is None
    assert "recovered after" in caplog.text


def test_mit_stream_latches_repeated_completed_gap_violations(bus: RebotMotorbridgeBus) -> None:
    bus._send_mit_commands = lambda commands: time.sleep(0.02)
    bus.start_mit_command_stream(
        {"joint_1": (10.0, 1.0, 1.0, 0.0, 0.0)},
        hz=200.0,
        max_consecutive_failures=2,
        max_gap_s=0.015,
    )
    deadline = time.monotonic() + 0.2
    while bus._mit_stream_fault is None and time.monotonic() < deadline:
        time.sleep(0.005)

    with pytest.raises(RuntimeError, match="2 consecutive completed-send gap violations"):
        bus.check_mit_command_stream()


def test_mit_stream_latches_one_hard_completed_gap(bus: RebotMotorbridgeBus) -> None:
    bus._send_mit_commands = lambda commands: time.sleep(0.03)
    bus.start_mit_command_stream(
        {"joint_1": (10.0, 1.0, 1.0, 0.0, 0.0)},
        hz=200.0,
        max_consecutive_failures=5,
        max_gap_s=0.01,
    )
    deadline = time.monotonic() + 0.2
    while bus._mit_stream_fault is None and time.monotonic() < deadline:
        time.sleep(0.005)

    with pytest.raises(RuntimeError, match="single completed-send gap.*hard recovered-gap limit"):
        bus.check_mit_command_stream()


def test_enabled_motor_status_updates_feedback(bus: RebotMotorbridgeBus) -> None:
    bus._update_state_cache(
        "joint_1",
        SimpleNamespace(pos=0.5, vel=0.1, torq=0.2, status_code=1),
    )

    assert bus._last_known_states["joint_1"]["position"] == pytest.approx(28.6478898)
    assert bus._last_known_states["joint_1"]["status_code"] == 1.0
    bus.check_motor_status_codes(required_enabled_motors=["joint_1"])


def test_disabled_status_is_valid_until_motor_is_required_enabled(bus: RebotMotorbridgeBus) -> None:
    bus._update_state_cache(
        "joint_1",
        SimpleNamespace(pos=0.5, vel=0.1, torq=0.2, status_code=0),
    )

    bus.check_motor_status_codes()
    with pytest.raises(RuntimeError, match="joint_1.*DISABLED"):
        bus.check_motor_status_codes(required_enabled_motors=["joint_1"])


def test_fault_motor_status_keeps_last_valid_feedback(bus: RebotMotorbridgeBus) -> None:
    bus._update_state_cache(
        "joint_1",
        SimpleNamespace(pos=0.5, vel=0.1, torq=0.2, status_code=0),
    )
    valid_position = bus._last_known_states["joint_1"]["position"]

    bus._update_state_cache(
        "joint_1",
        SimpleNamespace(pos=2.0, vel=1.0, torq=3.0, status_code=8),
    )

    assert bus._last_known_states["joint_1"]["position"] == valid_position
    assert bus._last_known_states["joint_1"]["status_code"] == 8.0
    assert bus.motor_status_codes() == {"joint_1": 8}
    with pytest.raises(RuntimeError, match="joint_1.*OVER_VOLTAGE"):
        bus.check_motor_status_codes()


def test_consecutive_missing_feedback_is_reported(bus: RebotMotorbridgeBus) -> None:
    bus._update_state_cache("joint_1", None)
    bus._update_state_cache("joint_1", None)

    with pytest.raises(RuntimeError, match="joint_1.*2"):
        bus.check_motor_status_codes(max_feedback_misses=2)
