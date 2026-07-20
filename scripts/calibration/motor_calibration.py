#!/usr/bin/env python3
import argparse
import json
import math
import os
import select
import struct
import sys
import termios
import time
import tty
from datetime import datetime

import can

HOST_ID = 0xFD
DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
MECH_POS_INDEX = 0x7019

MOTOR_IDS = list(range(1, 13))
DEFAULT_OUTPUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "motor_limits.json")

JOINT_MAP = {
    1:  "left_hip_yaw",
    2:  "left_hip_pitch",
    3:  "left_hip_roll",
    4:  "left_knee_pitch",
    5:  "left_ankle_pitch",
    6:  "left_ankle_roll",
    7:  "right_hip_yaw",
    8:  "right_hip_pitch",
    9:  "right_hip_roll",
    10: "right_knee_pitch",
    11: "right_ankle_pitch",
    12: "right_ankle_roll",
}

def build_arb(comm_type, data16, target_id):
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)

def parse_arb(arbitration_id):
    comm_type = (arbitration_id >> 24) & 0x1F
    data16 = (arbitration_id >> 8) & 0xFFFF
    destination = arbitration_id & 0xFF
    return comm_type, data16, destination

def read_mech_position(bus, host_id, motor_id, timeout=0.05):
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
        if len(payload) < 8:
            continue
        if int.from_bytes(payload[0:2], "little") != MECH_POS_INDEX:
            continue
        return struct.unpack_from("<f", payload, 4)[0]
    return None

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

def fmt(rad):
    return f"{rad:+8.4f} rad ({math.degrees(rad):+8.2f} deg)"

def resolve_joint_key(motor_id):
    name = JOINT_MAP.get(motor_id)
    if name is None:
        print(f"Warning: CAN ID {motor_id} is not in JOINT_MAP; saving under 'ID{motor_id}'.")
        return f"ID{motor_id}"
    return name

def joint_key_for_id(motor_id):
    return JOINT_MAP.get(motor_id, f"ID{motor_id}")

def load_document(path):
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as handle:
            document = json.load(handle)
    else:
        document = {}

    for motor_id in MOTOR_IDS:
        document.setdefault(joint_key_for_id(motor_id), None)
    return document

def save_document(path, document):
    ordered_keys = [joint_key_for_id(motor_id) for motor_id in MOTOR_IDS]
    ordered = {key: document.get(key) for key in ordered_keys}
    for key, value in document.items():
        if key not in ordered:
            ordered[key] = value

    tmp_path = f"{path}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        json.dump(ordered, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    os.replace(tmp_path, path)

def print_entry(label, entry):
    print(f"  {label}")
    if "can_id" in entry:
        print(f"    can_id   : {entry['can_id']}")
    print(f"    min      : {fmt(entry['min_rad'])}")
    print(f"    max      : {fmt(entry['max_rad'])}")
    print(f"    range    : {fmt(entry['range_rad'])}")
    print(f"    center   : {fmt(entry['center_rad'])}")
    print(f"    samples  : {entry['samples']}")
    print(f"    recorded : {entry['recorded_at']}")
    if "zeroed_at_reference" in entry:
        print(f"    zeroed   : {'yes' if entry['zeroed_at_reference'] else 'NO'}")

def wait_for_yes_no(prompt, keyboard):
    print(prompt, end="", flush=True)
    while True:
        key = keyboard.get_key()
        if key is None:
            time.sleep(0.02)
            continue
        if key in ("y", "Y"):
            print("Y")
            return True
        if key in ("n", "N", "q", "Q", "\x03", "\x1b"):
            print("N")
            return False

def measure(bus, args, keyboard):
    minimum = None
    maximum = None
    samples = 0
    misses = 0
    period = 1.0 / args.rate
    next_tick = time.monotonic()

    print(f"Recording motor ID {args.motor_id} on {args.channel}.")
    print("Rotate the joint by hand through its full travel. Press Q when done.\n")

    while True:
        key = keyboard.get_key()
        if key in ("q", "Q", "\x03"):
            break

        position = read_mech_position(bus, args.host_id, args.motor_id, timeout=args.timeout)
        if position is None:
            misses += 1
            status = f"no response ({misses})"
        else:
            samples += 1
            minimum = position if minimum is None else min(minimum, position)
            maximum = position if maximum is None else max(maximum, position)
            status = (f"now {fmt(position)}   min {math.degrees(minimum):+8.2f}"
                      f"   max {math.degrees(maximum):+8.2f} deg")
        print(f"\r  {status}    ", end="", flush=True)

        next_tick += period
        sleep_time = next_tick - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            next_tick = time.monotonic()

    print("\n")
    if minimum is None:
        return None

    return {
        "min_rad": minimum,
        "max_rad": maximum,
        "min_deg": math.degrees(minimum),
        "max_deg": math.degrees(maximum),
        "range_rad": maximum - minimum,
        "range_deg": math.degrees(maximum - minimum),
        "center_rad": (maximum + minimum) / 2.0,
        "center_deg": math.degrees((maximum + minimum) / 2.0),
        "samples": samples,
        "recorded_at": datetime.now().isoformat(timespec="seconds"),
    }

def parse_args():
    parser = argparse.ArgumentParser(
        description="Record min/max mechanical position while turning a motor by hand.")
    parser.add_argument("--motor-id", type=lambda v: int(v, 0), required=True,
                        help="motor CAN ID being calibrated (name looked up in JOINT_MAP)")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL, help="CAN channel, default: can0")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE,
                        help="python-can interface, default: socketcan")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID,
                        help="host CAN ID used in the private protocol, default: 0xFD")
    parser.add_argument("--output", default=DEFAULT_OUTPUT,
                        help="JSON file to update, default: scripts/calibration/motor_limits.json")
    parser.add_argument("--rate", type=float, default=50.0, help="polls per second, default: 50")
    parser.add_argument("--timeout", type=float, default=0.05,
                        help="seconds to wait for each reply, default: 0.05")
    return parser.parse_args()

def main():
    args = parse_args()
    key = resolve_joint_key(args.motor_id)
    document = load_document(args.output)
    previous = document.get(key)

    if not sys.stdin.isatty():
        print("This tool needs an interactive terminal for the Q/Y keys.")
        return 1

    bus = can.Bus(channel=args.channel, interface=args.interface)
    try:
        if read_mech_position(bus, args.host_id, args.motor_id, timeout=0.3) is None:
            print(f"Motor ID {args.motor_id} did not respond on {args.channel}. Check wiring and ID.")
            return 1

        with RawKeyboard() as keyboard:
            zeroed = wait_for_yes_no(
                "이 모터는 기준 자세에서 Set Zero Position이 완료된 상태인가? [Y/n] ", keyboard)
            print()

            raw = measure(bus, args, keyboard)
            if raw is None:
                print("No position samples were received; nothing to record.")
                return 1

            entry = {
                "can_id": args.motor_id,
                "zeroed_at_reference": zeroed,
                **raw,
            }

            print(f"Measured range for {key} (can_id {args.motor_id}):")
            print_entry("this session", entry)
            print()

            if not zeroed:
                print("경고: 이 측정값은 기준 자세에서 영점이 잡히지 않은 상태로 기록됨 "
                      "(기준점이 불확실함).\n")

            if previous is None:
                saved = wait_for_yes_no(f"Save this as {key}? [Y/n] ", keyboard)
            else:
                print(f"{key} already has a stored calibration:")
                print_entry("previously stored", previous)
                print()
                saved = wait_for_yes_no(
                    f"Replace the stored {key} values with this session's? [Y/n] ", keyboard)

            if not saved:
                print(f"\nDiscarded. {args.output} was left unchanged.")
                return 0

        document[key] = entry
        save_document(args.output, document)
        print(f"\nSaved {key} to {args.output}.")
        return 0
    finally:
        bus.shutdown()

if __name__ == "__main__":
    sys.exit(main())
