#!/usr/bin/env python

"""Command only the B601 follower gripper through motorbridge MIT mode."""

import argparse
import time

from lerobot.robots.rebot_b601_follower.config_rebot_b601_follower import RebotB601FollowerConfig
from lerobot.robots.rebot_b601_follower.rebot_b601_follower import RebotB601Follower


def main() -> None:
    parser = argparse.ArgumentParser(description="Debug B601 follower gripper command range.")
    parser.add_argument("--port", default="/dev/ttyACM1")
    parser.add_argument("--id", default="b601_follower")
    parser.add_argument("--target", type=float, required=True, help="Target gripper position in degrees.")
    parser.add_argument("--kp", type=float, default=35.0)
    parser.add_argument("--kd", type=float, default=0.8)
    parser.add_argument("--tau", type=float, default=0.0, help="Optional feedforward torque in Nm.")
    parser.add_argument("--duration-s", type=float, default=3.0)
    parser.add_argument("--min-pos", type=float, default=-90.0)
    parser.add_argument("--max-pos", type=float, default=5.0)
    args = parser.parse_args()

    robot = RebotB601Follower(
        RebotB601FollowerConfig(
            id=args.id,
            port=args.port,
            transport="motorbridge",
            cameras={},
            gripper_min_pos=args.min_pos,
            gripper_max_pos=args.max_pos,
            gripper_position_kp=args.kp,
            gripper_position_kd=args.kd,
            max_relative_target=None,
        )
    )

    try:
        robot.connect(calibrate=False)
        target = max(args.min_pos, min(args.max_pos, args.target))
        print(f"Commanding follower gripper to {target:+.1f} deg for {args.duration_s:.1f}s")
        start = time.perf_counter()
        while time.perf_counter() - start < args.duration_s:
            robot.bus.sync_write_mit({"gripper": (args.kp, args.kd, target, 0.0, args.tau)})
            state = robot.bus.sync_read_all_states("gripper")["gripper"]
            print(
                f"target={target:+.1f}  pos={state['position']:+.2f}  "
                f"vel={state['velocity']:+.2f}  torque={state['torque']:+.2f}"
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping follower gripper command debug...")
    finally:
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
