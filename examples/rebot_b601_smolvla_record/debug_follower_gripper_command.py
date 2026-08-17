#!/usr/bin/env python

"""Command only the B601 follower gripper through the recording controller."""

import argparse
import logging
import time

from lerobot.robots.rebot_b601_follower.config_rebot_b601_follower import RebotB601FollowerConfig
from lerobot.robots.rebot_b601_follower.rebot_b601_follower import RebotB601Follower


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    parser = argparse.ArgumentParser(description="Debug B601 follower gripper command range.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--id", default="b601_follower")
    parser.add_argument("--target", type=float, required=True, help="Target gripper position in degrees.")
    parser.add_argument("--kp", type=float, default=35.0)
    parser.add_argument("--kd", type=float, default=0.8)
    parser.add_argument("--duration-s", type=float, default=15.0)
    parser.add_argument("--min-pos", type=float, default=-330.0)
    parser.add_argument("--max-pos", type=float, default=10.0)
    parser.add_argument("--max-relative-target", type=float, default=30.0)
    parser.add_argument(
        "--allow-out-of-range-start",
        action="store_true",
        help="Allow a guarded, relative-limited recovery command when feedback starts outside min/max.",
    )
    parser.add_argument(
        "--control-mode",
        choices=("position", "torque_limited_close"),
        default="torque_limited_close",
    )
    parser.add_argument("--max-torque", type=float, default=0.8)
    parser.add_argument("--close-torque", type=float, default=0.5)
    parser.add_argument("--contact-min-travel-deg", type=float, default=17.0)
    parser.add_argument("--hold-kp", type=float, default=2.0)
    parser.add_argument("--hold-kd", type=float, default=0.5)
    parser.add_argument("--hold-torque", type=float, default=0.12)
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
            gripper_leader_close_pos=None,
            gripper_leader_open_pos=None,
            gripper_follower_close_pos=None,
            gripper_follower_open_pos=None,
            gripper_max_relative_target=args.max_relative_target,
            gripper_control_mode=args.control_mode,
            gripper_max_torque=args.max_torque,
            gripper_close_torque=args.close_torque,
            gripper_contact_min_travel_deg=args.contact_min_travel_deg,
            gripper_contact_hold_kp=args.hold_kp,
            gripper_contact_hold_kd=args.hold_kd,
            gripper_contact_hold_torque=args.hold_torque,
            max_relative_target=args.max_relative_target,
            startup_position_guard_enabled=not args.allow_out_of_range_start,
        )
    )

    try:
        print("Support the follower arm throughout this test and keep the E-stop ready.")
        print("Keep hands out of the gripper jaws; use only a soft expendable object for contact testing.")
        input("Press ENTER to connect and enable the follower...")
        robot.connect(calibrate=False)
        target = max(args.min_pos, min(args.max_pos, args.target))
        print(f"Commanding follower gripper to {target:+.1f} deg for {args.duration_s:.1f}s")
        start = time.perf_counter()
        while time.perf_counter() - start < args.duration_s:
            states = robot.bus.sync_read_all_states()
            robot._check_motorbridge_health()
            robot._last_observation_states = {
                motor_name: motor_state.copy() for motor_name, motor_state in states.items()
            }
            state = states["gripper"]
            sent_action = robot.send_action({"gripper.pos": target})
            if robot._gripper_contact_hold_pos is not None:
                control_phase = "contact_hold"
            elif (
                args.control_mode == "torque_limited_close"
                and target
                > state["position"] + robot.config.gripper_contact_min_closing_error_deg
            ):
                control_phase = "closing_torque"
            else:
                control_phase = "position"
            statuses = ",".join(
                f"{motor_name}={int(motor_state['status_code']):#x}"
                for motor_name, motor_state in states.items()
            )
            diagnostics = robot.bus.mit_command_stream_diagnostics()
            print(
                f"phase={control_phase:<14}  target={target:+.1f}  "
                f"sent={sent_action['gripper.pos']:+.2f}  "
                f"pos={state['position']:+.2f}  vel={state['velocity']:+.2f}  "
                f"torque={state['torque']:+.2f}  status={int(state['status_code']):#x}  "
                f"MOS={state['temp_mos']:.1f}C  rotor={state['temp_rotor']:.1f}C  "
                f"stream_age={diagnostics['age_ms']:.1f}ms  all=[{statuses}]"
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping follower gripper command debug...")
    finally:
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
