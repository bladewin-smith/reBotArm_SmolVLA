#!/usr/bin/env python

"""Hold a B601 follower pose while deliberately stalling the main thread."""

import argparse
import time

from lerobot.robots.rebot_b601_follower import RebotB601Follower, RebotB601FollowerConfig


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Verify that the B601 follower MIT stream survives camera-like main-thread stalls."
    )
    parser.add_argument("--port", required=True, help="Verified follower MotorBridge port, for example /dev/ttyACM1.")
    parser.add_argument("--id", default="b601_follower_stream_test")
    parser.add_argument("--duration-s", type=float, default=20.0)
    parser.add_argument("--stall-s", type=float, default=5.0)
    parser.add_argument("--stream-hz", type=float, default=100.0)
    parser.add_argument("--max-gap-s", type=float, default=0.25)
    parser.add_argument("--max-feedback-misses", type=int, default=3)
    args = parser.parse_args()

    if args.duration_s <= 0 or args.stall_s <= 0:
        parser.error("--duration-s and --stall-s must be positive.")

    robot = RebotB601Follower(
        RebotB601FollowerConfig(
            id=args.id,
            port=args.port,
            transport="motorbridge",
            cameras={},
            max_relative_target=None,
            command_stream_enabled=True,
            command_stream_hz=args.stream_hz,
            command_stream_max_gap_s=args.max_gap_s,
            motor_feedback_max_consecutive_misses=args.max_feedback_misses,
            disable_torque_on_disconnect=True,
        )
    )

    print("B601 follower command-stream stall test")
    print(f"  port: {args.port}")
    print(f"  stream: {args.stream_hz:.1f} Hz")
    print(f"  deliberate main-thread stall: {args.stall_s:.1f}s")
    print()
    print("Support the follower throughout the test and when it exits. Keep the E-stop ready.")
    input("Place the follower in a safe supported mid-range pose, then press ENTER to enable it... ")

    try:
        robot.connect(calibrate=False)
        initial_states = robot.bus.sync_read_all_states()
        robot._check_motorbridge_health()
        initial_pos = {name: state["position"] for name, state in initial_states.items()}
        print("Follower enabled. The main thread will now stop touching MotorBridge during each stall.")

        start_s = time.monotonic()
        while time.monotonic() - start_s < args.duration_s:
            stall_start_s = time.monotonic()
            time.sleep(min(args.stall_s, args.duration_s - (stall_start_s - start_s)))

            robot.bus.check_mit_command_stream(max_gap_s=args.max_gap_s)
            states = robot.bus.sync_read_all_states()
            robot._check_motorbridge_health()
            max_drift = max(
                abs(float(state["position"]) - initial_pos[name]) for name, state in states.items()
            )
            stream_age_ms = 1000.0 * (
                time.monotonic() - float(robot.bus.mit_command_stream_last_send_s)
            )
            print(
                f"stall survived: {time.monotonic() - stall_start_s:.2f}s, "
                f"max pose drift={max_drift:.2f} deg, stream age={stream_age_ms:.1f} ms"
            )
    except KeyboardInterrupt:
        print("\nCommand-stream test interrupted.")
    finally:
        if robot.bus.is_connected:
            print("Support the follower now; disabling torque and disconnecting.")
            robot.disconnect()


if __name__ == "__main__":
    main()
