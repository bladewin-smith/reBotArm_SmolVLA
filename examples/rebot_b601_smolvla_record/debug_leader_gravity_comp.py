#!/usr/bin/env python

"""Debug gravity compensation on a reBot B601-DM leader arm.

This uses the same motorbridge DM serial path as Seeed's reBotArm_control_py
examples, but keeps the test in the LeRobot B601 setup. It computes gravity
feedforward with the Seeed SDK dynamics helpers and sends MIT commands:

    pos = current position
    vel = 0
    tau = torque_scale * g(q)

Start with one joint and a low torque scale before trying the full arm.
"""

import argparse
import sys
import time
from collections.abc import Sequence
from pathlib import Path

import numpy as np

from lerobot.teleoperators.rebot_b601_leader.rebot_b601_leader import RebotB601Leader
from lerobot.teleoperators.rebot_b601_leader.config_rebot_b601_leader import RebotB601LeaderConfig


def _default_sdk_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        candidate = parent / "rebot_grasp" / "sdk" / "reBotArm_control_py"
        if (candidate / "src" / "reBotArm_control_py").exists():
            return candidate

    home_candidate = Path.home() / "ws" / "rebot_grasp" / "sdk" / "reBotArm_control_py"
    return home_candidate


def _parse_names(value: str) -> list[str]:
    names = [part.strip() for part in value.split(",") if part.strip()]
    if not names:
        raise argparse.ArgumentTypeError("At least one joint name is required.")
    return names


def _parse_gain(value: str | None) -> float | list[float] | None:
    if value is None:
        return None
    parts = [part.strip() for part in value.split(",") if part.strip()]
    gains = [float(part) for part in parts]
    if not gains:
        raise argparse.ArgumentTypeError("Gain value cannot be empty.")
    return gains[0] if len(gains) == 1 else gains


def _gain_for(
    values: float | Sequence[float],
    motor_names: list[str],
    enabled_names: list[str],
    motor_name: str,
) -> float:
    if isinstance(values, float):
        return values
    if len(values) == len(motor_names):
        return float(values[motor_names.index(motor_name)])
    if len(values) == len(enabled_names):
        return float(values[enabled_names.index(motor_name)])
    raise ValueError(
        "Per-joint gain length must match either all motors "
        f"({len(motor_names)}) or enabled joints ({len(enabled_names)})."
    )


def _load_dynamics(sdk_root: Path):
    if sdk_root.exists():
        sys.path.insert(0, str(sdk_root / "src"))
    from reBotArm_control_py.dynamics import compute_generalized_gravity, load_dynamics_model

    model = load_dynamics_model()
    data = model.createData()
    return model, data, compute_generalized_gravity


def _read_q_rad(leader: RebotB601Leader, motor_names: list[str]) -> np.ndarray:
    q_rad = []
    for motor_name in motor_names:
        state = leader.bus._poll_motor_state(motor_name)
        if state is None:
            raise RuntimeError(f"No motor feedback from {motor_name}; stopping gravity compensation.")
        leader.bus._update_state_cache(motor_name, state)
        q_rad.append(float(getattr(state, "pos", 0.0)))
    return np.asarray(q_rad, dtype=np.float64)


def _print_status(step: int, q_rad: np.ndarray, tau_g: np.ndarray, tau_cmd: dict[str, float], motor_names: list[str]) -> None:
    q_deg = np.degrees(q_rad)
    tau_items = "  ".join(f"{name}:{tau_cmd.get(name, 0.0):+.3f}" for name in motor_names)
    print(
        f"[{step:05d}] "
        f"q_deg=[" + ", ".join(f"{value:+.1f}" for value in q_deg) + "] "
        f"tau_g=[" + ", ".join(f"{value:+.3f}" for value in tau_g[: len(motor_names)]) + "] "
        f"tau_cmd={tau_items}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Debug B601-DM leader gravity compensation.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--id", default="b601_leader")
    parser.add_argument("--motorbridge-baudrate", type=int, default=921600)
    parser.add_argument("--sdk-root", type=Path, default=_default_sdk_root())
    parser.add_argument("--enabled-joints", type=_parse_names, default=_parse_names("joint_1,joint_2,joint_3,joint_4,joint_5,joint_6"))
    parser.add_argument("--torque-scale", type=float, default=0.95)
    parser.add_argument("--torque-limit", type=float, default=8.0)
    parser.add_argument("--kp", type=_parse_gain, default=1.9)
    parser.add_argument("--kd", type=_parse_gain, default=0.75)
    parser.add_argument("--hz", type=float, default=100.0)
    parser.add_argument("--print-hz", type=float, default=2.0)
    parser.add_argument("--duration-s", type=float, default=0.0, help="0 means run until Ctrl+C.")
    parser.add_argument("--dry-run", action="store_true", help="Read q/tau_g but do not enable torque or send MIT.")
    parser.add_argument("--no-calibrate", action="store_true")
    args = parser.parse_args()

    if args.hz <= 0 or args.print_hz <= 0:
        raise ValueError("--hz and --print-hz must be positive.")
    if args.torque_scale < 0:
        raise ValueError("--torque-scale must be non-negative.")
    if args.torque_limit <= 0:
        raise ValueError("--torque-limit must be positive.")

    model, data, compute_generalized_gravity = _load_dynamics(args.sdk_root)

    config = RebotB601LeaderConfig(
        id=args.id,
        port=args.port,
        transport="motorbridge",
        motorbridge_baudrate=args.motorbridge_baudrate,
        manual_control_mode="disabled",
    )
    leader = RebotB601Leader(config)
    motor_names = list(config.motor_config)
    for motor_name in args.enabled_joints:
        if motor_name not in motor_names:
            raise ValueError(f"Unknown motor name {motor_name!r}. Available: {motor_names}")

    print("B601-DM leader gravity compensation debug")
    print(f"  port: {args.port}")
    print(f"  baudrate: {args.motorbridge_baudrate}")
    print(f"  sdk root: {args.sdk_root}")
    print(f"  enabled joints: {args.enabled_joints}")
    print(f"  torque scale: {args.torque_scale}")
    print(f"  torque limit: +/-{args.torque_limit} Nm")
    print(f"  dry run: {args.dry_run}")
    print(f"  dynamics model: nq={model.nq}, nv={model.nv}")
    print("\nStart with one hand supporting the arm. Press Ctrl+C to stop.\n")

    period_s = 1.0 / args.hz
    print_period_s = 1.0 / args.print_hz
    next_print_time = time.perf_counter()
    start_time = time.perf_counter()
    step = 0

    try:
        leader.connect(calibrate=not args.no_calibrate)
        leader.bus.disable_torque()
        if not args.dry_run:
            leader.bus.ensure_mit_mode()
            time.sleep(0.1)
            leader.bus.enable_torque(args.enabled_joints)

        while True:
            loop_start = time.perf_counter()
            q_rad = _read_q_rad(leader, motor_names)
            tau_g = compute_generalized_gravity(model=model, q=q_rad, data=data)

            commands = {}
            tau_cmd = {}
            for motor_name in args.enabled_joints:
                index = motor_names.index(motor_name)
                tau = float(np.clip(args.torque_scale * tau_g[index], -args.torque_limit, args.torque_limit))
                tau_cmd[motor_name] = tau
                commands[motor_name] = (
                    _gain_for(args.kp, motor_names, args.enabled_joints, motor_name),
                    _gain_for(args.kd, motor_names, args.enabled_joints, motor_name),
                    float(np.degrees(q_rad[index])),
                    0.0,
                    tau,
                )
            if not args.dry_run:
                leader.bus.sync_write_mit(commands)

            now = time.perf_counter()
            if now >= next_print_time:
                _print_status(step, q_rad, tau_g, tau_cmd, motor_names)
                next_print_time = now + print_period_s

            if args.duration_s > 0 and now - start_time >= args.duration_s:
                break

            step += 1
            sleep_s = period_s - (time.perf_counter() - loop_start)
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        print("\nStopping gravity compensation debug...")
    finally:
        if leader.is_connected:
            leader.bus.disable_torque()
            leader.disconnect()
            print("Leader disconnected; torque disabled.")


if __name__ == "__main__":
    main()
