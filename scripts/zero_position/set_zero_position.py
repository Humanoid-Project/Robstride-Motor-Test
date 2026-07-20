#!/usr/bin/env python3
import argparse
import math
import os
import select
import struct
import sys
import termios
import time
import tty

import can

HOST_ID = 0xFD
DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"

MECH_POS_INDEX = 0x7019
ZERO_STA_INDEX = 0x7029

TYPE_SET_ZERO = 0x06
TYPE_WRITE_PARAM = 0x12
TYPE_SAVE = 0x16
SAVE_PAYLOAD = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])

TWO_PI = 2.0 * math.pi

def build_arb(comm_type, data16, target_id):
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)

def parse_arb(arbitration_id):
    comm_type = (arbitration_id >> 24) & 0x1F
    data16 = (arbitration_id >> 8) & 0xFFFF
    destination = arbitration_id & 0xFF
    return comm_type, data16, destination

def read_mech_position(bus, host_id, motor_id, timeout=0.2):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, MECH_POS_INDEX)
    bus.send(can.Message(arbitration_id=build_arb(0x11, host_id, motor_id),
                         data=bytes(data), is_extended_id=True))

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None or not msg.is_extended_id:
            continue
        comm_type, data16, destination = parse_arb(msg.arbitration_id)
        if comm_type != 0x11 or destination != host_id or (data16 & 0xFF) != motor_id:
            continue
        payload = bytes(msg.data)
        if len(payload) < 8 or int.from_bytes(payload[0:2], "little") != MECH_POS_INDEX:
            continue
        return struct.unpack_from("<f", payload, 4)[0]
    return None

def read_mech_position_retry(bus, host_id, motor_id, attempts=5, timeout=0.2):
    for _ in range(attempts):
        position = read_mech_position(bus, host_id, motor_id, timeout=timeout)
        if position is not None:
            return position
    return None

def set_mechanical_zero(bus, host_id, motor_id):
    data = bytearray(8)
    data[0] = 1
    bus.send(can.Message(arbitration_id=build_arb(TYPE_SET_ZERO, host_id, motor_id),
                         data=bytes(data), is_extended_id=True))
    deadline = time.monotonic() + 0.2
    while time.monotonic() < deadline:
        if bus.recv(timeout=max(0.0, deadline - time.monotonic())) is None:
            break

def write_uint8_parameter(bus, host_id, motor_id, index, value):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, index)
    data[4] = value & 0xFF
    bus.send(can.Message(arbitration_id=build_arb(TYPE_WRITE_PARAM, host_id, motor_id),
                         data=bytes(data), is_extended_id=True))
    time.sleep(0.05)

def save_parameters(bus, host_id, motor_id):
    bus.send(can.Message(arbitration_id=build_arb(TYPE_SAVE, host_id, motor_id),
                         data=SAVE_PAYLOAD, is_extended_id=True))
    time.sleep(0.3)

class RawKeyboard:

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.saved = None

    def __enter__(self):
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *_exc):
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def get_key(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def wait_any(self):
        while self.get_key() is None:
            time.sleep(0.02)

    def wait_yes_no(self, prompt):
        print(prompt, end="", flush=True)
        while True:
            key = self.get_key()
            if key is None:
                time.sleep(0.02)
                continue
            if key in ("y", "Y"):
                print("Y")
                return True
            if key in ("n", "N", "q", "Q", "\x03", "\x1b"):
                print("N")
                return False

def fmt(rad):
    return f"{rad:+8.4f} rad ({math.degrees(rad):+8.2f} deg)"

def angular_diff(a, b):
    return (a - b + math.pi) % TWO_PI - math.pi

def open_bus(args):
    return can.Bus(channel=args.channel, interface=args.interface)

def reconnect(args, keyboard):
    deadline = time.monotonic() + args.reconnect_timeout
    last_error = None
    while time.monotonic() < deadline:
        bus = None
        try:
            bus = open_bus(args)
            position = read_mech_position_retry(bus, args.host_id, args.motor_id, attempts=3)
            if position is not None:
                return bus, position
            bus.shutdown()
        except Exception as exc:
            last_error = exc
            if bus is not None:
                try:
                    bus.shutdown()
                except Exception:
                    pass
        if keyboard.get_key() in ("q", "Q", "\x03"):
            break
        time.sleep(0.5)

    if last_error is not None:
        print(f"  last error: {last_error}")
    return None, None

def verify_power_cycle(args, keyboard, before_zero):
    print("\n--- Power cycle check ---")
    print("Do NOT move the joint from here on; the check compares against 0.")
    print("Power the motor down, wait a moment, power it back up.")
    print("Then press any key to continue (Q to skip the check).")

    key = None
    while key is None:
        key = keyboard.get_key()
        time.sleep(0.02)
    if key in ("q", "Q", "\x03"):
        print("Skipped. The zero was set but not verified across a power cycle.")
        return

    print("\nReconnecting ...")
    bus, after_cycle = reconnect(args, keyboard)
    if bus is None:
        print("Could not reach the motor after the power cycle.")
        print("Bring the CAN link back up and re-read the position manually:")
        print(f"  .venv/bin/python scripts/motor_id/find_motor_id.py --check-id {args.motor_id}")
        return

    try:
        print(f"  position after power cycle: {fmt(after_cycle)}\n")
        from_zero = abs(angular_diff(after_cycle, 0.0))
        from_before = abs(angular_diff(after_cycle, before_zero))

        if from_zero <= args.tolerance:
            print("RESULT: the zero PERSISTED across the power cycle.")
            print("  The motor stores it, so home stays home after a reboot.")
        elif from_before <= args.tolerance:
            print("RESULT: the zero was LOST; the motor reverted to its old reference.")
            print("  Re-run with --save to persist it via type 22, and if that also")
            print("  fails, keep the home offset in software instead of the motor.")
        else:
            print("RESULT: inconclusive.")
            print(f"  {from_zero:.4f} rad from zero, {from_before:.4f} rad from the")
            print("  pre-zero reading. The joint most likely moved during the power")
            print("  cycle, or something is driving it. Re-run and hold it still.")
    finally:
        bus.shutdown()

def parse_args():
    parser = argparse.ArgumentParser(
        description="Set the motor's current position as its mechanical zero (type 6).")
    parser.add_argument("--motor-id", type=lambda v: int(v, 0), required=True,
                        help="motor CAN ID to zero")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL, help="CAN channel, default: can0")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE,
                        help="python-can interface, default: socketcan")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID,
                        help="host CAN ID used in the private protocol, default: 0xFD")
    parser.add_argument("--save", action="store_true",
                        help="send the type 22 save frame after zeroing")
    parser.add_argument("--zero-sta", type=int, choices=[0, 1],
                        help="set power-on position range: 0 for 0..2pi, 1 for -pi..pi "
                             "(implies --save)")
    parser.add_argument("--verify", action="store_true",
                        help="walk through the power-cycle persistence check")
    parser.add_argument("--tolerance", type=float, default=0.05,
                        help="rad tolerance for the power-cycle verdict, default: 0.05")
    parser.add_argument("--reconnect-timeout", type=float, default=30.0,
                        help="seconds to wait for the motor after a power cycle, default: 30")
    parser.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    return parser.parse_args()

def main():
    args = parse_args()
    if args.zero_sta is not None:
        args.save = True

    if not sys.stdin.isatty() and not args.yes:
        print("This tool needs an interactive terminal, or --yes to run unattended.")
        return 1

    bus = open_bus(args)
    try:
        before_zero = read_mech_position_retry(bus, args.host_id, args.motor_id)
        if before_zero is None:
            print(f"Motor ID {args.motor_id} did not respond on {args.channel}. "
                  "Check wiring and ID.")
            return 1

        print(f"Motor ID {args.motor_id} on {args.channel}")
        print(f"  current position: {fmt(before_zero)}\n")
        print("Make sure the joint is held at its home pose and the motor is NOT")
        print("enabled. On older firmware, zeroing while enabled can make the motor")
        print("jump to its target position.\n")

        with RawKeyboard() as keyboard:
            if not args.yes:
                if not keyboard.wait_yes_no("Set this position as mechanical zero? [Y/n] "):
                    print("\nAborted; nothing was changed.")
                    return 0
                print()

            if args.zero_sta is not None:
                print(f"Writing zero_sta = {args.zero_sta} "
                      f"({'-pi..pi' if args.zero_sta else '0..2pi'} on power-on) ...")
                write_uint8_parameter(bus, args.host_id, args.motor_id,
                                      ZERO_STA_INDEX, args.zero_sta)

            print("Sending type 6 (set mechanical zero) ...")
            set_mechanical_zero(bus, args.host_id, args.motor_id)

            if args.save:
                print("Sending type 22 (save parameters) ...")
                save_parameters(bus, args.host_id, args.motor_id)

            after_set = read_mech_position_retry(bus, args.host_id, args.motor_id)
            if after_set is None:
                print("\nNo position reply after zeroing; could not confirm.")
                return 1

            print(f"\n  position after zeroing: {fmt(after_set)}")
            if abs(angular_diff(after_set, 0.0)) <= args.tolerance:
                print("  the motor now reports home as zero.")
            else:
                print("  WARNING: the position did not go to zero. The command may have")
                print("  been rejected -- zeroing is blocked in PP mode (manual 4.2.6).")

            if not args.verify:
                print("\nRun again with --verify to check whether this survives a "
                      "power cycle.")
                return 0

            bus.shutdown()
            bus = None
            verify_power_cycle(args, keyboard, before_zero)
        return 0
    finally:
        if bus is not None:
            bus.shutdown()

if __name__ == "__main__":
    sys.exit(main())
