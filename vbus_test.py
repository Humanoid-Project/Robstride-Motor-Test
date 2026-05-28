import can
import struct

bus = can.Bus(channel='can0', interface='socketcan')

# Type 17: vbus 읽기 (index 0x701C, 모터 ID 1)
data = bytearray(8)
struct.pack_into('<H', data, 0, 0x701C)

arb_id = (0x11 << 24) | (0xFD << 8) | 0x05
msg = can.Message(arbitration_id=arb_id, data=list(data), is_extended_id=True)
bus.send(msg)

reply = bus.recv(timeout=1.0)
if reply:
    vbus = struct.unpack('<f', bytes(reply.data[4:8]))[0]
    print(f"Vbus = {vbus:.1f} V")
else:
    print("응답 없음")

bus.shutdown()