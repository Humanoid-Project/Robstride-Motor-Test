import can
import struct
import time

bus = can.Bus(channel='can0', interface='socketcan')
MOTOR_ID = 5
HOST_ID = 0xFD

def send(comm_type, data16, data):
    arb_id = ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | MOTOR_ID
    msg = can.Message(arbitration_id=arb_id, data=list(data), is_extended_id=True)
    bus.send(msg)

def recv():
    return bus.recv(timeout=0.5)

def enable():
    send(0x03, HOST_ID, bytes(8))
    return recv()

def stop():
    send(0x04, HOST_ID, bytes(8))
    return recv()

def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))

def control_op(pos, vel, kp, kd, torque=0.0):
    """Operation Control Mode (MIT 방식)"""
    data16 = float_to_uint(torque, -60, 60, 16)
    raw_pos = float_to_uint(pos, -12.57, 12.57, 16)
    raw_vel = float_to_uint(vel, -20, 20, 16)
    raw_kp  = float_to_uint(kp, 0, 5000, 16)
    raw_kd  = float_to_uint(kd, 0, 100, 16)

    data = bytes([
        (raw_pos >> 8) & 0xFF, raw_pos & 0xFF,
        (raw_vel >> 8) & 0xFF, raw_vel & 0xFF,
        (raw_kp  >> 8) & 0xFF, raw_kp  & 0xFF,
        (raw_kd  >> 8) & 0xFF, raw_kd  & 0xFF,
    ])
    send(0x01, data16, data)
    return recv()

# ── 실행 ──────────────────────────────────

print("Enable...")
reply = enable()
if reply:
    print(f"  응답: {reply.data.hex()}")
else:
    print("  응답 없음")

print("1 rad/s로 5초 회전 (Kp=0, Kd=1)")
start = time.time()
while time.time() - start < 5.0:
    reply = control_op(pos=0, vel=1.0, kp=0, kd=1.0)
    if reply:
        raw_pos = (reply.data[0] << 8) | reply.data[1]
        raw_vel = (reply.data[2] << 8) | reply.data[3]
        pos = raw_pos / 65535 * 25.14 - 12.57
        vel = raw_vel / 65535 * 40 - 20
        print(f"  pos={pos:+.3f} rad  vel={vel:+.3f} rad/s")
    time.sleep(0.05)

print("Stop")
stop()
bus.shutdown()