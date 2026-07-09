#!/usr/bin/env python3
import argparse
import struct
import time

import can

HOST_ID = 0xFD
DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
MECH_POS_INDEX = 0x7019


def build_arb(comm_type, data16, target_id):
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_arb(arbitration_id):
    comm_type = (arbitration_id >> 24) & 0x1F
    data16 = (arbitration_id >> 8) & 0xFFFF
    destination = arbitration_id & 0xFF
    return comm_type, data16, destination


def get_device_id(bus, host_id, target_id, timeout=0.3):
    arb = build_arb(0x00, host_id, target_id)
    bus.send(can.Message(arbitration_id=arb, data=bytes(8), is_extended_id=True))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None or not msg.is_extended_id:
            continue
        comm_type, data16, destination = parse_arb(msg.arbitration_id)
        if comm_type == 0x00 and destination == host_id:
            motor_id = data16 & 0xFF
            uid = bytes(msg.data)
            return motor_id, uid
    return None


def read_mech_position(bus, host_id, target_id, timeout=0.3):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, MECH_POS_INDEX)
    arb = build_arb(0x11, host_id, target_id)
    bus.send(can.Message(arbitration_id=arb, data=bytes(data), is_extended_id=True))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None or not msg.is_extended_id:
            continue
        comm_type, data16, destination = parse_arb(msg.arbitration_id)
        if destination == host_id and (data16 & 0xFF) == target_id:
            return True
    return False


def scan(bus, host_id, id_range):
    found = []
    for target in id_range:
        result = get_device_id(bus, host_id, target)
        if result is not None:
            motor_id, uid = result
            found.append((motor_id, uid))
            print(f"  found motor ID {motor_id} (0x{motor_id:02X})  UID={uid.hex()}")
        else:
            if read_mech_position(bus, host_id, target):
                found.append((target, None))
                print(f"  motor answers on ID {target} (0x{target:02X})")
    if not found:
        print("  no motors responded")
    return found


def main():
    parser = argparse.ArgumentParser(description="Find which CAN IDs Robstride motors are answering on.")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument("--interface", default=DEFAULT_INTERFACE)
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID)
    parser.add_argument("--scan", action="store_true", help="scan a range of IDs and report responders")
    parser.add_argument("--scan-max", type=int, default=127, help="highest ID to scan (with --scan)")
    parser.add_argument("--check-id", type=lambda v: int(v, 0), help="check whether one specific ID answers")
    args = parser.parse_args()

    if not args.scan and args.check_id is None:
        parser.error("provide --scan or --check-id")

    bus = can.Bus(channel=args.channel, interface=args.interface)
    try:
        if args.scan:
            print(f"Scanning IDs 0..{args.scan_max} on {args.channel} ...")
            scan(bus, args.host_id, range(0, args.scan_max + 1))
            return

        answers = read_mech_position(bus, args.host_id, args.check_id)
        print(f"Motor ID {args.check_id} (0x{args.check_id:02X}): "
              f"{'responds' if answers else 'NO RESPONSE'}")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
