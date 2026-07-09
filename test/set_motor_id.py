#!/usr/bin/env python3
import argparse
import struct
import time

import can

HOST_ID = 0xFD
DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
MECH_POS_INDEX = 0x7019


def build_arb(comm_type, new_id, host_id, target_id):
    data16 = ((new_id & 0xFF) << 8) | (host_id & 0xFF)
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_arb(arbitration_id):
    comm_type = (arbitration_id >> 24) & 0x1F
    data16 = (arbitration_id >> 8) & 0xFFFF
    destination = arbitration_id & 0xFF
    return comm_type, data16, destination


def read_mech_position(bus, host_id, target_id, timeout=0.3):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, MECH_POS_INDEX)
    arb = build_arb(0x11, 0, host_id, target_id)
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


def set_can_id(bus, host_id, current_id, new_id):
    arb = build_arb(0x07, new_id, host_id, current_id)
    bus.send(can.Message(arbitration_id=arb, data=bytes(8), is_extended_id=True))
    time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description="Change a Robstride motor's CAN ID (permanent).")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument("--interface", default=DEFAULT_INTERFACE)
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID)
    parser.add_argument("--current-id", type=lambda v: int(v, 0), required=True, help="current motor CAN ID")
    parser.add_argument("--new-id", type=lambda v: int(v, 0), required=True, help="new motor CAN ID to assign")
    parser.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    args = parser.parse_args()

    if not 0 <= args.new_id <= 127:
        parser.error("--new-id must be between 0 and 127")

    bus = can.Bus(channel=args.channel, interface=args.interface)
    try:
        answers = read_mech_position(bus, args.host_id, args.current_id)
        print(f"Motor ID {args.current_id} (0x{args.current_id:02X}): "
              f"{'responds' if answers else 'NO RESPONSE'}")
        if not answers:
            print("Aborting: the current ID does not respond. Check wiring/power/ID.")
            return

        if not args.yes:
            reply = input(f"Change motor ID {args.current_id} -> {args.new_id}? "
                          f"This is permanent. [y/N] ")
            if reply.strip().lower() not in ("y", "yes"):
                print("Cancelled.")
                return

        set_can_id(bus, args.host_id, args.current_id, args.new_id)
        print(f"Sent set-ID command. Verifying ID {args.new_id} ...")

        ok = False
        for _ in range(10):
            if read_mech_position(bus, args.host_id, args.new_id):
                ok = True
                break
            time.sleep(0.2)

        if ok:
            print(f"OK: motor now answers on ID {args.new_id} (0x{args.new_id:02X}).")
        else:
            print("New ID did not answer yet. Power-cycle the motor and re-check with:")
            print(f"    python3 find_motor_id.py --check-id {args.new_id}")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
