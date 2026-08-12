#!/usr/bin/env python

"""Debug the reBot B601-DM leader impedance mode without a follower arm.

This script connects only to the leader arm, repeatedly calls get_action(), and
prints motor state snapshots. It is meant for tuning low-stiffness manual
control gains before running full teleoperation data collection.
"""

import argparse
import time
from collections.abc import Sequence

from lerobot.teleoperators.rebot_b601_leader.config_rebot_b601_leader import RebotB601LeaderConfig
from lerobot.teleoperators.rebot_b601_leader.rebot_b601_leader import RebotB601Leader


def _parse_gain(value: str | None) -> float | list[float] | None:
    if value is None:
        return None
    parts = [part.strip() for part in value.split(",") if part.strip()]
    if not parts:
        raise argparse.ArgumentTypeError("Gain value cannot be empty.")
    gains = [float(part) for part in parts]
    return gains[0] if len(gains) == 1 else gains


def _format_gain(value: float | Sequence[float]) -> str:
    if isinstance(value, float):
        return f"{value:.3g}"
    return ",".join(f"{float(item):.3g}" for item in value)


def _print_states(states: dict[str, dict[str, float]], action: dict[str, float]) -> None:
    print("\n" + "=" * 78)
    print(f"{'motor':<12} {'pos(deg)':>10} {'vel(deg/s)':>12} {'torque(Nm)':>12} {'mos(C)':>8} {'rotor(C)':>9}")
    print("-" * 78)
    for motor_name, state in states.items():
        pos = action.get(f"{motor_name}.pos", state.get("position", 0.0))
        print(
            f"{motor_name:<12} "
            f"{pos:>10.2f} "
            f"{state.get('velocity', 0.0):>12.2f} "
            f"{state.get('torque', 0.0):>12.3f} "
            f"{state.get('temp_mos', 0.0):>8.1f} "
            f"{state.get('temp_rotor', 0.0):>9.1f}"
        )


def _connect_leader(leader: RebotB601Leader, *, calibrate: bool, skip_handshake: bool) -> None:
    if not skip_handshake:
        leader.connect(calibrate=calibrate)
        return

    leader.bus.connect(handshake=False)
    if calibrate and not leader.is_calibrated:
        raise RuntimeError(
            "--skip-handshake is intended for low-level CAN debugging. Use --no-calibrate with it, "
            "or run normal calibration after the motors respond to handshake."
        )
    leader.configure()


def main() -> None:
    parser = argparse.ArgumentParser(description="Debug reBot B601-DM leader impedance mode.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Leader port, for example /dev/ttyACM0.")
    parser.add_argument("--id", default="b601_leader", help="Leader calibration id.")
    parser.add_argument("--transport", default="motorbridge", choices=["motorbridge", "socketcan"])
    parser.add_argument("--motorbridge-baudrate", type=int, default=921600)
    parser.add_argument("--can-interface", default="socketcan", choices=["socketcan", "slcan", "auto"])
    parser.add_argument("--no-can-fd", action="store_true", help="Disable CAN FD.")
    parser.add_argument("--can-bitrate", type=int, default=1_000_000)
    parser.add_argument("--can-data-bitrate", type=int, default=5_000_000)
    parser.add_argument("--mode", default="impedance", choices=["disabled", "impedance", "stiff"])
    parser.add_argument("--kp", type=_parse_gain, default=None, help="Scalar or comma-separated 7-joint Kp list.")
    parser.add_argument("--kd", type=_parse_gain, default=None, help="Scalar or comma-separated 7-joint Kd list.")
    parser.add_argument("--hz", type=float, default=30.0, help="Compliance update frequency.")
    parser.add_argument("--print-hz", type=float, default=2.0, help="Terminal print frequency.")
    parser.add_argument("--duration-s", type=float, default=0.0, help="0 means run until Ctrl+C.")
    parser.add_argument("--no-calibrate", action="store_true", help="Do not run calibration if no file exists.")
    parser.add_argument("--skip-handshake", action="store_true", help="Open CAN without requiring all motors to reply.")
    args = parser.parse_args()

    if args.hz <= 0:
        raise ValueError("--hz must be positive.")
    if args.print_hz <= 0:
        raise ValueError("--print-hz must be positive.")

    config = RebotB601LeaderConfig(
        id=args.id,
        port=args.port,
        transport=args.transport,
        motorbridge_baudrate=args.motorbridge_baudrate,
        can_interface=args.can_interface,
        use_can_fd=not args.no_can_fd,
        can_bitrate=args.can_bitrate,
        can_data_bitrate=args.can_data_bitrate,
        manual_control_mode=args.mode,
    )
    if hasattr(config, "handshake"):
        config.handshake = not args.skip_handshake
    elif args.skip_handshake:
        raise RuntimeError(
            "This checkout does not support --skip-handshake yet. Sync "
            "src/lerobot/teleoperators/rebot_b601_leader/config_rebot_b601_leader.py "
            "and rebot_b601_leader.py from the latest local repository."
        )
    if args.kp is not None:
        if args.mode == "stiff":
            config.stiff_kp = args.kp
        else:
            config.impedance_kp = args.kp
    if args.kd is not None:
        if args.mode == "stiff":
            config.stiff_kd = args.kd
        else:
            config.impedance_kd = args.kd

    leader = RebotB601Leader(config)
    print("Leader impedance debug")
    print(f"  port: {args.port}")
    print(f"  mode: {args.mode}")
    print(f"  update hz: {args.hz}")
    if args.mode == "impedance":
        print(f"  impedance_kp: {_format_gain(config.impedance_kp)}")
        print(f"  impedance_kd: {_format_gain(config.impedance_kd)}")
    elif args.mode == "stiff":
        print(f"  stiff_kp: {_format_gain(config.stiff_kp)}")
        print(f"  stiff_kd: {_format_gain(config.stiff_kd)}")
    print("\nKeep one hand near power/E-stop. Press Ctrl+C to stop and disable torque.\n")

    period_s = 1.0 / args.hz
    print_period_s = 1.0 / args.print_hz
    start_time = time.perf_counter()
    next_print_time = start_time

    try:
        _connect_leader(leader, calibrate=not args.no_calibrate, skip_handshake=args.skip_handshake)
        while True:
            loop_start = time.perf_counter()
            action = leader.get_action()

            now = time.perf_counter()
            if now >= next_print_time:
                states = leader.bus.sync_read_all_states()
                _print_states(states, action)
                next_print_time = now + print_period_s

            if args.duration_s > 0 and now - start_time >= args.duration_s:
                break

            sleep_s = period_s - (time.perf_counter() - loop_start)
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        print("\nStopping leader impedance debug...")
    finally:
        if leader.is_connected:
            leader.disconnect()
            print("Leader disconnected; torque disabled.")
        else:
            print("Leader was not connected; no torque command was sent.")


if __name__ == "__main__":
    main()
