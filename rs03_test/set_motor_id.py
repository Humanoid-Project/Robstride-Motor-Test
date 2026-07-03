#!/usr/bin/env python3
"""Set / query a Robstride motor's CAN ID over the private protocol.

Replaces the vendor GUI's "change motor ID" function. Robstride (CyberGear-style)
uses:
  - comm type 0 (0x00): get device ID  -> returns the motor's 64-bit MCU UID
  - comm type 7 (0x07): set CAN ID      -> permanently change the motor's CAN ID

Arbitration ID layout (29-bit extended):
  bit28..24 = comm type
  bit23..16 = new CAN ID   (set-id only)
  bit15..8  = host CAN ID
  bit7..0   = target (current) motor CAN ID

⚠️  SAFETY / USAGE
  - Put ONLY ONE motor on the bus while changing its ID. If two motors share the
    same current ID, both would react. Chain the second motor AFTER each has a
    unique ID.
  - The change is permanent. Power-cycle the motor afterwards if it does not
    respond on the new ID immediately.

Examples:
  python3 set_motor_id.py --scan                       # find motor(s) on the bus
  python3 set_motor_id.py --current-id 1 --new-id 2    # change ID 1 -> 2
  python3 set_motor_id.py --current-id 5 --verify-only # just check ID 5 answers
"""
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


def get_device_id(bus, host_id, target_id, timeout=0.15):
    """Comm type 0: ask a motor to report itself. Returns (motor_id, uid) or None."""
    arb = build_arb(0x00, 0, host_id, target_id)
    bus.send(can.Message(arbitration_id=arb, data=bytes(8), is_extended_id=True))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None or not msg.is_extended_id:
            continue
        comm_type, data16, destination = parse_arb(msg.arbitration_id)
        # Type-0 reply: motor's CAN ID is carried in the low byte of the data area.
        if comm_type == 0x00 and destination == host_id:
            motor_id = data16 & 0xFF
            uid = bytes(msg.data)
            return motor_id, uid
    return None


def read_mech_position(bus, host_id, target_id, timeout=0.1):
    """Type 17 read of mechanical position -- used to confirm a motor answers on an ID."""
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


def set_can_id(bus, host_id, current_id, new_id, timeout=0.2):
    """Comm type 7: change the motor's CAN ID (permanent)."""
    arb = build_arb(0x07, new_id, host_id, current_id)
    bus.send(can.Message(arbitration_id=arb, data=bytes(8), is_extended_id=True))
    # Give the motor a moment to commit and (some firmwares) reset.
    time.sleep(0.2)


def scan(bus, host_id, id_range):
    found = []
    for target in id_range:
        result = get_device_id(bus, host_id, target)
        if result is not None:
            motor_id, uid = result
            found.append((motor_id, uid))
            print(f"  found motor ID {motor_id} (0x{motor_id:02X})  UID={uid.hex()}")
        else:
            # Fall back to a position read in case type-0 reply differs by firmware.
            if read_mech_position(bus, host_id, target):
                found.append((target, None))
                print(f"  motor answers on ID {target} (0x{target:02X})")
    if not found:
        print("  no motors responded")
    return found


def main():
    parser = argparse.ArgumentParser(description="Set or query a Robstride motor CAN ID.")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument("--interface", default=DEFAULT_INTERFACE)
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID)
    parser.add_argument("--current-id", type=lambda v: int(v, 0), help="current motor CAN ID")
    parser.add_argument("--new-id", type=lambda v: int(v, 0), help="new motor CAN ID to assign")
    parser.add_argument("--scan", action="store_true", help="scan IDs 0..127 and report responders")
    parser.add_argument("--scan-max", type=int, default=127, help="highest ID to scan (with --scan)")
    parser.add_argument("--verify-only", action="store_true", help="only check that --current-id answers")
    parser.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    args = parser.parse_args()

    bus = can.Bus(channel=args.channel, interface=args.interface)
    try:
        if args.scan:
            print(f"Scanning IDs 0..{args.scan_max} on {args.channel} ...")
            scan(bus, args.host_id, range(0, args.scan_max + 1))
            return

        if args.current_id is None:
            parser.error("provide --current-id (or use --scan)")

        answers = read_mech_position(bus, args.host_id, args.current_id)
        print(f"Motor ID {args.current_id} (0x{args.current_id:02X}): "
              f"{'responds' if answers else 'NO RESPONSE'}")

        if args.verify_only:
            return

        if args.new_id is None:
            parser.error("provide --new-id to change the ID (or use --verify-only)")

        if not answers:
            print("Aborting: the current ID does not respond. Check wiring/power/ID.")
            return

        if not 0 <= args.new_id <= 127:
            parser.error("--new-id must be between 0 and 127")

        if not args.yes:
            reply = input(f"Change motor ID {args.current_id} -> {args.new_id}? "
                          f"This is permanent. [y/N] ")
            if reply.strip().lower() not in ("y", "yes"):
                print("Cancelled.")
                return

        set_can_id(bus, args.host_id, args.current_id, args.new_id)
        print(f"Sent set-ID command. Verifying ID {args.new_id} ...")

        # Some firmwares reset on ID change; retry a few times.
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
            print(f"    python3 set_motor_id.py --current-id {args.new_id} --verify-only")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
