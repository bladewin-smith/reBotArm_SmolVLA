#!/usr/bin/env python

"""Read B601-DM follower feedback without enabling torque or sending commands."""

import argparse
import logging

from lerobot.robots.rebot_b601_follower import RebotB601Follower, RebotB601FollowerConfig


def main() -> None:
    parser = argparse.ArgumentParser(description="Read B601-DM follower state without enabling torque.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--id", default="b601_follower")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    robot = RebotB601Follower(
        RebotB601FollowerConfig(
            id=args.id,
            port=args.port,
            transport="motorbridge",
            cameras={},
        )
    )

    print("Read-only follower state check: no torque enable or MIT command will be sent.")
    print("Support the arm in case it was left enabled by another process, and keep the E-stop ready.")
    try:
        robot.bus.connect()
        states = robot.bus.sync_read_all_states()
        violations = robot._startup_position_violations(states)
        for motor_name, state in states.items():
            limits = robot._joint_limits_for(motor_name)
            limits_text = "none" if limits is None else f"[{limits[0]:+.1f}, {limits[1]:+.1f}]"
            marker = "  OUTSIDE STARTUP RANGE" if motor_name in violations else ""
            print(
                f"{motor_name:<8} pos={state['position']:+9.2f} deg  "
                f"vel={state['velocity']:+8.2f} deg/s  status={int(state['status_code']):#x}  "
                f"limits={limits_text}{marker}"
            )
        if violations:
            print(f"Startup guard would refuse torque enable: {violations}")
        else:
            print("All motor positions are inside the configured startup ranges.")
    finally:
        if robot.bus.is_connected:
            robot.bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
