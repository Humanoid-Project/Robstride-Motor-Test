#!/usr/bin/env python3
import argparse
import struct
import time

import can

HOST_ID = 0xFD
MECH_POS_INDEX = 0x7019


def build_arb(comm_type, data16, target_id):
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_arb(arbitration_id):
    comm_type = (arbitration_id >> 24) & 0x1F
    data16 = (arbitration_id >> 8) & 0xFFFF
    destination = arbitration_id & 0xFF
    return comm_type, data16, destination


def drain(bus, deadline):
    frames = []
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None:
            break
        frames.append(msg)
    return frames


def probe_get_device_id(bus, host_id, target_id, timeout):
    arb = build_arb(0x00, host_id, target_id)
    print(f"  -> type0 GetDeviceID  arb=0x{arb:08X}  (target id={target_id})")
    bus.send(can.Message(arbitration_id=arb, data=bytes(8), is_extended_id=True))
    deadline = time.monotonic() + timeout
    frames = drain(bus, deadline)
    for msg in frames:
        if not msg.is_extended_id:
            continue
        comm_type, data16, destination = parse_arb(msg.arbitration_id)
        if comm_type == 0x00 and destination == host_id:
            responder_id = data16 & 0xFF
            uid = bytes(msg.data).hex()
            print(f"     <- reply arb=0x{msg.arbitration_id:08X}  responder_id={responder_id}  uid={uid}")
            return responder_id, uid, frames
    return None, None, frames


def probe_mech_position(bus, host_id, target_id, timeout):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, MECH_POS_INDEX)
    arb = build_arb(0x11, host_id, target_id)
    print(f"  -> type17 ReadParam(mechPos)  arb=0x{arb:08X}  (target id={target_id})")
    bus.send(can.Message(arbitration_id=arb, data=bytes(data), is_extended_id=True))
    deadline = time.monotonic() + timeout
    frames = drain(bus, deadline)
    for msg in frames:
        if not msg.is_extended_id:
            continue
        comm_type, data16, destination = parse_arb(msg.arbitration_id)
        if comm_type == 0x11 and destination == host_id and (data16 & 0xFF) == target_id:
            payload = bytes(msg.data)
            index = int.from_bytes(payload[0:2], "little")
            value = struct.unpack_from("<f", payload, 4)[0]
            print(f"     <- reply arb=0x{msg.arbitration_id:08X}  index=0x{index:04X}  mechPos={value:.4f} rad")
            return value, frames
    return None, frames


def probe_one(bus, host_id, target_id, timeout):
    print(f"\n=== probing motor id {target_id} (0x{target_id:02X}) ===")
    responder_id, uid, frames0 = probe_get_device_id(bus, host_id, target_id, timeout)
    if responder_id is not None:
        return "responded (type0)", frames0

    value, frames1 = probe_mech_position(bus, host_id, target_id, timeout)
    if value is not None:
        return "responded (type17 fallback)", frames0 + frames1

    unrelated = [m for m in frames0 + frames1 if m.is_extended_id]
    if unrelated:
        print("     (no matching reply, but other extended frames were seen on the bus:)")
        for m in unrelated[:10]:
            print(f"       arb=0x{m.arbitration_id:08X} data={bytes(m.data).hex()}")
    return "NO RESPONSE", frames0 + frames1


def main():
    parser = argparse.ArgumentParser(description="Probe Robstride motors on a CAN bus by ID.")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID)
    parser.add_argument("--ids", default="7,5", help="comma-separated motor CAN IDs to probe, default: 7,5")
    parser.add_argument("--timeout", type=float, default=0.3)
    args = parser.parse_args()

    target_ids = [int(v, 0) for v in args.ids.split(",") if v.strip() != ""]

    bus = can.Bus(channel=args.channel, interface=args.interface)
    results = {}
    try:
        for target_id in target_ids:
            status, _ = probe_one(bus, args.host_id, target_id, args.timeout)
            results[target_id] = status
    finally:
        bus.shutdown()

    print("\n---- summary ----")
    for target_id, status in results.items():
        print(f"id {target_id:>3} (0x{target_id:02X}): {status}")


if __name__ == "__main__":
    main()
