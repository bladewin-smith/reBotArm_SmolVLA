#!/usr/bin/env python

"""Set only the B601-DM follower gripper zero while all motion commands remain disabled."""

import argparse
import time

from lerobot.robots.rebot_b601_follower import RebotB601Follower, RebotB601FollowerConfig


CONFIRMATION = "SET_GRIPPER_ZERO"


def main() -> None:
    parser = argparse.ArgumentParser(description="Set the follower gripper zero at its physical closed pose.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--id", default="b601_follower")
    args = parser.parse_args()

    robot = RebotB601Follower(
        RebotB601FollowerConfig(
            id=args.id,
            port=args.port,
            transport="motorbridge",
            cameras={},
        )
    )

    print("This tool changes only the follower gripper's device zero.")
    print("It does not enable torque and does not send any position or torque command.")
    print("Support the whole arm and keep the E-stop ready in case another process left joints enabled.")
    try:
        robot.bus.connect()
        robot.bus.disable_torque("gripper")
        before = robot.bus.sync_read_all_states("gripper")["gripper"]
        print(f"Current gripper feedback: {before['position']:+.2f} deg")
        print("With gripper torque disabled, move the jaws gently to the same physical closed pose used for training.")
        confirmation = input(f"Type {CONFIRMATION} to set that pose to zero, or press ENTER to cancel: ")
        if confirmation.strip() != CONFIRMATION:
            print("Cancelled; the device zero was not changed.")
            return

        robot.bus.set_zero_position("gripper")
        time.sleep(0.2)
        after = robot.bus.sync_read_all_states("gripper")["gripper"]
        print(f"Gripper zero updated. New feedback: {after['position']:+.2f} deg")
        if abs(float(after["position"])) > 5.0:
            raise RuntimeError(
                "Gripper feedback is still more than 5 degrees from zero. Do not run inference; inspect the "
                "mechanism, MotorBridge feedback, and zeroing procedure."
            )
    finally:
        if robot.bus.is_connected:
            robot.bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
