#!/usr/bin/env python

"""Print B601 leader/follower gripper positions during teleoperation setup."""

import argparse
import time

from lerobot.robots.rebot_b601_follower.config_rebot_b601_follower import RebotB601FollowerConfig
from lerobot.robots.rebot_b601_follower.rebot_b601_follower import RebotB601Follower
from lerobot.teleoperators.rebot_b601_leader.config_rebot_b601_leader import RebotB601LeaderConfig
from lerobot.teleoperators.rebot_b601_leader.rebot_b601_leader import RebotB601Leader


def main() -> None:
    parser = argparse.ArgumentParser(description="Debug B601 gripper mapping.")
    parser.add_argument("--leader-port", default="/dev/ttyACM0")
    parser.add_argument("--follower-port", default="/dev/ttyACM1")
    parser.add_argument("--leader-id", default="b601_leader")
    parser.add_argument("--follower-id", default="b601_follower")
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--duration-s", type=float, default=0.0, help="0 means run until Ctrl+C.")
    args = parser.parse_args()

    leader = RebotB601Leader(
        RebotB601LeaderConfig(
            id=args.leader_id,
            port=args.leader_port,
            transport="motorbridge",
            manual_control_mode="disabled",
        )
    )
    follower = RebotB601Follower(
        RebotB601FollowerConfig(
            id=args.follower_id,
            port=args.follower_port,
            transport="motorbridge",
            cameras={},
        )
    )

    period_s = 1.0 / args.hz
    start = time.perf_counter()

    try:
        leader.connect(calibrate=False)
        follower.connect(calibrate=False)
        leader.bus.disable_torque()
        follower.bus.disable_torque()

        print("Move both grippers by hand. Press Ctrl+C to stop.")
        print("leader_gripper_deg  follower_gripper_deg")
        while True:
            leader_state = leader.bus.sync_read_all_states("gripper")["gripper"]
            follower_state = follower.bus.sync_read_all_states("gripper")["gripper"]
            print(f"{leader_state['position']:+8.2f}            {follower_state['position']:+8.2f}")

            if args.duration_s > 0 and time.perf_counter() - start >= args.duration_s:
                break
            time.sleep(period_s)
    except KeyboardInterrupt:
        print("\nStopping gripper mapping debug...")
    finally:
        if leader.is_connected:
            leader.disconnect()
        if follower.is_connected:
            follower.disconnect()


if __name__ == "__main__":
    main()
