#!/usr/bin/env python

"""Hold a B601 follower while reading the production Orbbec camera streams."""

import argparse
import time
from pathlib import Path
from typing import Any

from lerobot.cameras.orbbec import OrbbecCameraConfig
from lerobot.robots.rebot_b601_follower import RebotB601Follower, RebotB601FollowerConfig
from lerobot.teleoperators.rebot_b601_leader import RebotB601Leader, RebotB601LeaderConfig


def _camera_configs(args: argparse.Namespace) -> dict[str, OrbbecCameraConfig]:
    bridge = Path(args.bridge)
    cameras: dict[str, OrbbecCameraConfig] = {}

    if args.camera_mode in {"wrist", "pair"}:
        cameras["wrist"] = OrbbecCameraConfig(
            serial_number=args.wrist_serial,
            bridge_binary=bridge,
            width=args.width,
            height=args.height,
            fps=args.camera_fps,
            warmup_s=args.wrist_warmup_s,
            timeout_ms=args.wrist_timeout_ms,
            use_depth=False,
            record_color=True,
            record_depth=False,
        )

    if args.camera_mode in {"top", "pair"}:
        cameras["top"] = OrbbecCameraConfig(
            serial_number=args.top_serial,
            bridge_binary=bridge,
            width=args.width,
            height=args.height,
            fps=args.camera_fps,
            warmup_s=args.top_warmup_s,
            timeout_ms=args.top_timeout_ms,
            use_depth=True,
            record_color=True,
            record_depth=True,
            depth_key="depths.top",
            record_depth_viz=True,
            depth_viz_key="top_depth",
            depth_viz_min_mm=args.depth_viz_min_mm,
            depth_viz_max_mm=args.depth_viz_max_mm,
            align_depth_to_color=True,
            align_depth_to_color_mode=args.align_depth_to_color_mode,
            use_enhanced_depth_filter=args.enhanced_depth,
            enhanced_depth_model_path=Path(args.enhanced_depth_model) if args.enhanced_depth else None,
            enhanced_depth_confidence_threshold=args.enhanced_depth_confidence_threshold,
        )

    return cameras


def _array_shapes(observation: dict[str, Any]) -> str:
    shapes = []
    for key, value in observation.items():
        shape = getattr(value, "shape", None)
        if shape is not None:
            shapes.append(f"{key}={tuple(shape)}/{getattr(value, 'dtype', '?')}")
    return ", ".join(shapes)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Hold the B601 follower at its current pose while reading Orbbec streams. "
            "This isolates camera/USB/system load from teleoperation and dataset encoding."
        )
    )
    parser.add_argument("--port", required=True, help="Verified follower MotorBridge port.")
    parser.add_argument("--id", default="b601_follower")
    parser.add_argument(
        "--leader-port",
        help="Enable camera-loaded teleoperation using this verified leader MotorBridge port.",
    )
    parser.add_argument("--leader-id", default="b601_leader")
    parser.add_argument("--initial-pose-tolerance-deg", type=float, default=8.0)
    parser.add_argument("--max-relative-target", type=float, default=12.0)
    parser.add_argument("--camera-mode", choices=["none", "wrist", "top", "pair"], default="pair")
    parser.add_argument("--bridge", required=True, help="Path to the built orbbec_rgbd_bridge binary.")
    parser.add_argument("--wrist-serial", default="CV2TC5100075")
    parser.add_argument("--top-serial", default="CP3L44P0001N")
    parser.add_argument("--enhanced-depth-model", help="Path to model.sm4.")
    parser.add_argument("--enhanced-depth", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--enhanced-depth-confidence-threshold", type=int, default=51)
    parser.add_argument(
        "--align-depth-to-color-mode",
        choices=["sw", "software", "hw", "hardware"],
        default="sw",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=10)
    parser.add_argument("--poll-hz", type=float, default=10.0)
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--stream-hz", type=float, default=500.0)
    parser.add_argument("--max-gap-s", type=float, default=0.05)
    parser.add_argument("--hard-gap-s", type=float, default=0.5)
    parser.add_argument("--max-feedback-misses", type=int, default=3)
    parser.add_argument("--wrist-warmup-s", type=int, default=15)
    parser.add_argument("--wrist-timeout-ms", type=int, default=15000)
    parser.add_argument("--top-warmup-s", type=int, default=25)
    parser.add_argument("--top-timeout-ms", type=int, default=25000)
    parser.add_argument("--depth-viz-min-mm", type=int, default=250)
    parser.add_argument("--depth-viz-max-mm", type=int, default=1800)
    args = parser.parse_args()

    if args.duration_s <= 0 or args.poll_hz <= 0 or args.stream_hz <= 0:
        parser.error("--duration-s, --poll-hz, and --stream-hz must be positive.")
    if args.initial_pose_tolerance_deg <= 0 or args.max_relative_target <= 0:
        parser.error("--initial-pose-tolerance-deg and --max-relative-target must be positive.")
    if args.camera_mode in {"top", "pair"} and args.enhanced_depth and not args.enhanced_depth_model:
        parser.error("--enhanced-depth-model is required when top EnhancedDepthFilter is enabled.")

    cameras = _camera_configs(args)
    robot = RebotB601Follower(
        RebotB601FollowerConfig(
            id=args.id,
            port=args.port,
            transport="motorbridge",
            cameras=cameras,
            max_relative_target=args.max_relative_target if args.leader_port else None,
            safety_hold_on_relative_clamp=bool(args.leader_port),
            safety_auto_recover_to_episode_start=False,
            safety_wait_for_leader_start=False,
            command_stream_enabled=True,
            command_stream_hz=args.stream_hz,
            command_stream_max_gap_s=args.max_gap_s,
            command_stream_hard_gap_s=args.hard_gap_s,
            motor_feedback_max_consecutive_misses=args.max_feedback_misses,
            disable_torque_on_disconnect=True,
        )
    )
    leader = (
        RebotB601Leader(
            RebotB601LeaderConfig(
                id=args.leader_id,
                port=args.leader_port,
                transport="motorbridge",
                manual_control_mode="gravity_comp",
                command_stream_enabled=True,
                command_stream_hz=args.stream_hz,
                command_stream_max_gap_s=args.max_gap_s,
                command_stream_hard_gap_s=args.hard_gap_s,
                motor_feedback_max_consecutive_misses=args.max_feedback_misses,
            )
        )
        if args.leader_port
        else None
    )

    test_mode = "camera-loaded teleoperation" if leader is not None else "static camera load"
    print(f"B601 follower + Orbbec {test_mode} test")
    print(f"  follower: {args.port}")
    print(f"  leader: {args.leader_port or 'disabled'}")
    print(f"  camera mode: {args.camera_mode}")
    print(f"  camera/poll rate: {args.camera_fps}/{args.poll_hz:.1f} Hz")
    print(f"  motor stream: {args.stream_hz:.1f} Hz")
    print(f"  duration: {args.duration_s:.1f}s")
    print(f"  EnhancedDepthFilter: {args.enhanced_depth and args.camera_mode in {'top', 'pair'}}")
    print()
    print("Support the follower throughout the test and keep the E-stop ready.")
    if leader is not None:
        input(
            "Place both arms in closely matching, supported, safe mid-range poses, "
            "then press ENTER to connect and enable them... "
        )
    else:
        input("Place it in a supported, safe mid-range pose, then press ENTER to connect and enable it... ")

    try:
        robot.connect(calibrate=False)
        if leader is not None:
            leader.connect(calibrate=False)

            initial_observation = robot.get_observation()
            initial_action = leader.get_action()
            initial_differences = {
                joint: abs(
                    float(initial_action[f"{joint}.pos"])
                    - float(initial_observation[f"{joint}.pos"])
                )
                for joint in ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
            }
            unsafe_differences = {
                joint: difference
                for joint, difference in initial_differences.items()
                if difference > args.initial_pose_tolerance_deg
            }
            if unsafe_differences:
                raise RuntimeError(
                    "Leader/follower initial pose mismatch exceeds "
                    f"{args.initial_pose_tolerance_deg:.1f} deg: {unsafe_differences}"
                )
            print(f"initial leader/follower differences: {initial_differences}")

        start_s = time.monotonic()
        deadline_s = start_s + args.duration_s
        next_frame_s = start_s
        next_report_s = start_s
        frames = 0
        max_observation_ms = 0.0
        printed_shapes = False

        while time.monotonic() < deadline_s:
            observation_start_s = time.monotonic()
            observation = robot.get_observation()
            if leader is not None:
                robot.send_action(leader.get_action())
            observation_ms = 1000.0 * (time.monotonic() - observation_start_s)
            max_observation_ms = max(max_observation_ms, observation_ms)
            frames += 1

            if not printed_shapes:
                print(f"observation arrays: {_array_shapes(observation)}")
                printed_shapes = True

            now_s = time.monotonic()
            if now_s >= next_report_s:
                diagnostics = robot.bus.mit_command_stream_diagnostics()
                status = robot.bus.motor_status_codes()
                print(
                    f"elapsed={now_s - start_s:6.1f}s frames={frames:5d} "
                    f"obs={observation_ms:5.1f}ms max_obs={max_observation_ms:5.1f}ms "
                    f"stream_age={diagnostics['age_ms']:.1f}ms "
                    f"effective_hz={diagnostics['effective_hz']:.1f} "
                    f"max_gap={diagnostics['max_completed_gap_ms']:.1f}ms "
                    f"status={status}"
                )
                next_report_s = now_s + 1.0

            next_frame_s += 1.0 / args.poll_hz
            time.sleep(max(0.0, next_frame_s - time.monotonic()))

        print(f"{test_mode.capitalize()} test completed with all configured streams healthy.")
    except KeyboardInterrupt:
        print("\nStatic-load test interrupted.")
    except Exception:
        robot.hold_after_runtime_error()
        raise
    finally:
        if robot.bus.is_connected:
            print("Support the follower now; disabling torque and disconnecting.")
        try:
            if robot.bus.is_connected or any(camera.is_connected for camera in robot.cameras.values()):
                robot.disconnect()
        finally:
            if leader is not None and leader.bus.is_connected:
                leader.disconnect()


if __name__ == "__main__":
    main()
