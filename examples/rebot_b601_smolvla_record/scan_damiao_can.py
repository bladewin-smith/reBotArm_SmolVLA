#!/usr/bin/env python

"""Low-level CAN probe for Damiao/reBot B601-DM motors.

This scanner does not use LeRobot motor configs. It sends Damiao refresh and,
optionally, enable frames to candidate CAN IDs, then prints every CAN response
ID it sees. Use it to discover whether the bus is physically alive and what
response IDs the motors actually use.
"""

import argparse
import time

import can


CAN_PARAM_ID = 0x7FF
CAN_CMD_REFRESH = 0xCC
CAN_CMD_ENABLE = 0xFC
CAN_CMD_DISABLE = 0xFD


def _parse_ids(value: str) -> list[int]:
    ids: list[int] = []
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start_s, end_s = part.split("-", 1)
            start = int(start_s, 0)
            end = int(end_s, 0)
            step = 1 if end >= start else -1
            ids.extend(range(start, end + step, step))
        else:
            ids.append(int(part, 0))
    if not ids:
        raise argparse.ArgumentTypeError("At least one CAN ID is required.")
    return ids


def _open_bus(args: argparse.Namespace) -> can.BusABC:
    kwargs = {
        "channel": args.interface,
        "interface": "socketcan",
        "bitrate": args.bitrate,
    }
    if args.fd:
        kwargs.update({"fd": True, "data_bitrate": args.data_bitrate})
    return can.interface.Bus(**kwargs)


def _drain(bus: can.BusABC) -> None:
    while bus.recv(timeout=0.001):
        pass


def _collect(bus: can.BusABC, timeout_s: float) -> list[can.Message]:
    messages: list[can.Message] = []
    start = time.time()
    while time.time() - start < timeout_s:
        msg = bus.recv(timeout=0.01)
        if msg is not None:
            messages.append(msg)
    return messages


def _send_simple(bus: can.BusABC, motor_id: int, command: int, is_fd: bool) -> None:
    msg = can.Message(
        arbitration_id=motor_id,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command],
        is_extended_id=False,
        is_fd=is_fd,
    )
    bus.send(msg)


def _send_refresh(bus: can.BusABC, motor_id: int, is_fd: bool) -> None:
    msg = can.Message(
        arbitration_id=CAN_PARAM_ID,
        data=[motor_id & 0xFF, (motor_id >> 8) & 0xFF, CAN_CMD_REFRESH, 0, 0, 0, 0, 0],
        is_extended_id=False,
        is_fd=is_fd,
    )
    bus.send(msg)


def _print_messages(motor_id: int, messages: list[can.Message]) -> None:
    if not messages:
        print(f"0x{motor_id:03X}: no response")
        return

    print(f"0x{motor_id:03X}: {len(messages)} response(s)")
    for msg in messages:
        fd_flag = " FD" if getattr(msg, "is_fd", False) else ""
        print(
            f"  <- id=0x{msg.arbitration_id:03X}{fd_flag} "
            f"dlc={msg.dlc} data={bytes(msg.data).hex()}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Scan Damiao motor CAN IDs.")
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--ids", type=_parse_ids, default=_parse_ids("0x01-0x20"))
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--data-bitrate", type=int, default=5_000_000)
    parser.add_argument("--fd", dest="fd", action="store_true", help="Use CAN FD frames.")
    parser.add_argument("--no-fd", dest="fd", action="store_false", help="Use classic CAN frames.")
    parser.set_defaults(fd=False)
    parser.add_argument("--timeout-s", type=float, default=0.25)
    parser.add_argument("--enable", action="store_true", help="Send enable before refresh.")
    parser.add_argument("--disable-after-enable", action="store_true", help="Send disable after probing each ID.")
    args = parser.parse_args()

    bus = _open_bus(args)
    try:
        print(
            f"Scanning {args.interface} in {'CAN FD' if args.fd else 'classic CAN'} mode; "
            f"ids={','.join(hex(item) for item in args.ids)}"
        )
        _drain(bus)
        try:
            for motor_id in args.ids:
                if args.enable:
                    _send_simple(bus, motor_id, CAN_CMD_ENABLE, args.fd)
                    time.sleep(0.02)
                _send_refresh(bus, motor_id, args.fd)
                messages = _collect(bus, args.timeout_s)
                _print_messages(motor_id, messages)
                if args.enable and args.disable_after_enable:
                    _send_simple(bus, motor_id, CAN_CMD_DISABLE, args.fd)
                time.sleep(0.03)
        except KeyboardInterrupt:
            print("\nScan interrupted.")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
