#!/usr/bin/env python3
"""Simple GUI to change a Robstride motor's CAN ID (permanent).

Reuses the private-protocol Type 7 (Set motor CAN_ID) logic from
set_motor_id.py, wrapped in a minimal Tk window:

    bus status  ->  current ID (+ Check)  ->  new ID  ->  Change

All CAN I/O runs on a worker thread so the window never freezes. Only one
process can own the CAN interface at a time -- close the motor_run GUI before
using this tool.
"""
import argparse
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import can

HOST_ID = 0xFD
DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
MECH_POS_INDEX = 0x7019  # mechPos, used only as a "does this ID answer?" probe

# The official sample (RobStride/Python_Sample) rejects motor ID 0.
MIN_MOTOR_ID = 1
MAX_MOTOR_ID = 127
LISTEN_SECONDS = 8.0

# Extended-frame communication types a motor sends unprompted or as a reply:
# 0 = device ID (also emitted twice on power-up), 2 = feedback,
# 21 = fault report, 24 = active report.
REPLY_TYPES = (0x00, 0x02, 0x15, 0x18)


def build_arb_set_id(new_id, host_id, target_id):
    # Type 7: bit23~16 = preset (new) CAN_ID, bit15~8 = host CAN_ID, bit7~0 = target motor ID.
    data16 = ((new_id & 0xFF) << 8) | (host_id & 0xFF)
    return (0x07 << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)


def build_arb_read(host_id, target_id):
    # Type 17: bit15~8 = host CAN_ID, bit7~0 = target motor ID.
    return (0x11 << 24) | ((host_id & 0xFFFF) << 8) | (target_id & 0xFF)


def build_arb_get_id(host_id, target_id):
    # Type 0 (Get device ID): bit15~8 = host CAN_ID, bit7~0 = target motor ID.
    return (0x00 << 24) | ((host_id & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_arb(arbitration_id):
    comm_type = (arbitration_id >> 24) & 0x1F
    data16 = (arbitration_id >> 8) & 0xFFFF
    destination = arbitration_id & 0xFF
    return comm_type, data16, destination


def _await_reply(bus, host_id, target_id, comm_type, timeout):
    """Wait for a reply of `comm_type` whose bit15~8 identifies target_id.

    Firmware disagrees on the reply's low byte -- the manual says 0xFE for a
    Type 0 reply, some builds echo the host ID -- so identify the responder by
    bit15~8 only, and just make sure the frame isn't our own request coming
    back (those carry the host ID in bit15~8).
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None or not msg.is_extended_id:
            continue
        reply_type, data16, _destination = parse_arb(msg.arbitration_id)
        responder = data16 & 0xFF
        if reply_type == comm_type and responder == target_id and responder != host_id:
            return bytes(msg.data)
    return None


def ping(bus, host_id, target_id, timeout=0.3):
    """Return True if a motor answers on target_id.

    Tries Type 0 (Get device ID) first, like the official sample's ping_by_id,
    then falls back to a Type 17 parameter read for firmware that ignores it.
    """
    bus.send(can.Message(arbitration_id=build_arb_get_id(host_id, target_id),
                         data=bytes(8), is_extended_id=True))
    if _await_reply(bus, host_id, target_id, 0x00, timeout) is not None:
        return True

    data = bytearray(8)
    struct.pack_into("<H", data, 0, MECH_POS_INDEX)
    bus.send(can.Message(arbitration_id=build_arb_read(host_id, target_id),
                         data=bytes(data), is_extended_id=True))
    return _await_reply(bus, host_id, target_id, 0x11, timeout) is not None


def send_set_id(bus, host_id, current_id, new_id):
    bus.send(can.Message(arbitration_id=build_arb_set_id(new_id, host_id, current_id),
                         data=bytes(8), is_extended_id=True))
    time.sleep(0.2)


def drain(bus):
    while bus.recv(timeout=0.0) is not None:
        pass


def _blast(bus, arb_builder, host_id, max_id, payload, gap):
    for target in range(MIN_MOTOR_ID, max_id + 1):
        bus.send(can.Message(arbitration_id=arb_builder(host_id, target),
                             data=payload, is_extended_id=True))
        time.sleep(gap)


def _collect(bus, host_id, comm_type, listen, found, uid_from_data):
    deadline = time.monotonic() + listen
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None or not msg.is_extended_id:
            continue
        reply_type, data16, _destination = parse_arb(msg.arbitration_id)
        responder = data16 & 0xFF
        # Identify the responder by bit15~8 only. The reply's low byte is 0xFE
        # per the manual but the host ID on some builds, and our own outgoing
        # requests are the frames that carry the host ID in bit15~8.
        if reply_type != comm_type or responder == host_id:
            continue
        if MIN_MOTOR_ID <= responder <= MAX_MOTOR_ID:
            found.setdefault(responder, bytes(msg.data).hex() if uid_from_data else None)


def scan_ids(bus, host_id, max_id=MAX_MOTOR_ID, listen=0.6, gap=0.002):
    """Find motor IDs answering on the bus.

    Sends every request first and then listens once, instead of waiting per ID:
    127 extended frames are ~17ms of bus time at 1Mbps, so a scan takes ~1s
    rather than the ~25s the official sequential ping_by_id loop needs. Both
    passes are read-only queries.

    Returns {motor_id: uid_hex_or_None}.
    """
    found = {}

    # Pass 1: Type 0 (Get device ID) -- also gives the MCU UID.
    drain(bus)
    _blast(bus, build_arb_get_id, host_id, max_id, bytes(8), gap)
    _collect(bus, host_id, 0x00, listen, found, uid_from_data=True)
    if found:
        return found

    # Pass 2 (fallback): Type 17 read, for firmware that ignores Type 0.
    payload = bytearray(8)
    struct.pack_into("<H", payload, 0, MECH_POS_INDEX)
    drain(bus)
    _blast(bus, build_arb_read, host_id, max_id, bytes(payload), gap)
    _collect(bus, host_id, 0x11, listen, found, uid_from_data=False)
    return found


def listen_for_ids(bus, host_id, seconds=8.0):
    """Passively watch the bus and report every motor ID that speaks.

    A Robstride motor emits two frames carrying its own ID at power-up, so
    power-cycling the motor while this listens is the most reliable way to
    learn the current ID -- it needs no assumption about which query the
    firmware answers, and it also catches motors switched to the MIT protocol,
    which reply on standard (11-bit) frames with the ID in byte 0.

    Returns {motor_id: "private"|"MIT"}.
    """
    found = {}
    drain(bus)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None:
            continue
        if msg.is_extended_id:
            comm_type, data16, _destination = parse_arb(msg.arbitration_id)
            responder = data16 & 0xFF
            if comm_type in REPLY_TYPES and responder != host_id:
                if MIN_MOTOR_ID <= responder <= MAX_MOTOR_ID:
                    found.setdefault(responder, "private")
        elif len(msg.data) == 8:
            responder = msg.data[0]
            if MIN_MOTOR_ID <= responder <= MAX_MOTOR_ID:
                found.setdefault(responder, "MIT")
    return found


def link_state(channel):
    """'up'/'down' for a SocketCAN interface, or None if it can't be determined."""
    try:
        with open(f"/sys/class/net/{channel}/operstate") as handle:
            return handle.read().strip().lower()
    except OSError:
        return None


class Worker(threading.Thread):
    """Owns the CAN bus and processes commands from the GUI, one at a time."""

    def __init__(self, channel, interface, host_id, scan_max, cmd_q, out_q):
        super().__init__(daemon=True)
        self.channel = channel
        self.interface = interface
        self.host_id = host_id
        self.scan_max = scan_max
        self.cmd_q = cmd_q
        self.out_q = out_q
        self.bus = None
        self._stop = threading.Event()

    def emit(self, kind, **kw):
        self.out_q.put((kind, kw))

    def down_hint(self):
        return (f"{self.channel} 인터페이스가 DOWN 입니다  ->  "
                f"sudo ip link set {self.channel} up type can bitrate 1000000")

    def close_bus(self):
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

    def ensure_bus(self):
        """Open the bus if needed. SocketCAN happily opens a DOWN interface and
        only fails on the first send, so check the link state ourselves."""
        if self.bus is not None:
            return True
        if link_state(self.channel) == "down":
            self.emit("bus", ok=False, text=self.down_hint())
            return False
        try:
            self.bus = can.Bus(channel=self.channel, interface=self.interface)
        except Exception as exc:
            self.emit("bus", ok=False, text=f"{self.channel} 열기 실패: {exc}")
            return False
        self.emit("bus", ok=True, text=f"{self.channel} 연결됨")
        return True

    def fail(self, exc):
        text = str(exc)
        if "Network is down" in text or "No such device" in text:
            self.close_bus()
            self.emit("bus", ok=False, text=self.down_hint())
        self.emit("done", ok=False, new=None, text=f"오류: {text}")

    def run(self):
        self.ensure_bus()

        while not self._stop.is_set():
            try:
                cmd = self.cmd_q.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                # Retried per command, so bringing can0 up mid-session just works.
                if not self.ensure_bus():
                    self.emit("done", ok=False, new=None, text=f"오류: {self.down_hint()}")
                    continue
                action = cmd[0]
                if action == "check":
                    _, target = cmd
                    ok = ping(self.bus, self.host_id, target)
                    self.emit("check", target=target, ok=ok)
                elif action == "scan":
                    self.do_scan()
                elif action == "listen":
                    self.do_listen()
                elif action == "change":
                    _, current, new, scan = cmd
                    self.do_change(current, new, scan)
            except Exception as exc:
                self.fail(exc)

    def report_found(self, found, empty_hint):
        for motor_id in sorted(found):
            detail = found[motor_id]
            suffix = f"  ({detail})" if detail else ""
            self.emit("log", text=f"  발견: ID {motor_id} (0x{motor_id:02X}){suffix}")
        if not found:
            self.emit("log", text=f"  {empty_hint}")
        self.emit("scan", ids=sorted(found))

    def do_scan(self):
        self.emit("log", text=f"ID {MIN_MOTOR_ID}~{self.scan_max} 스캔 중...")
        found = scan_ids(self.bus, self.host_id, max_id=self.scan_max)
        self.report_found(found, "응답한 모터가 없습니다. '전원 켜서 찾기'를 써보세요.")

    def do_listen(self):
        self.emit("log", text=f"{LISTEN_SECONDS:.0f}초간 수신 대기 -- 지금 모터 전원을 인가하세요.")
        found = listen_for_ids(self.bus, self.host_id, seconds=LISTEN_SECONDS)
        self.report_found(found, "프레임이 하나도 안 왔습니다. 전원/배선/종단저항/비트레이트를 확인하세요.")

    def do_change(self, current, new, scan):
        self.emit("log", text=f"현재 ID {current} (0x{current:02X}) 응답 확인 중...")
        if not ping(self.bus, self.host_id, current):
            self.emit("done", ok=False, new=None,
                      text=f"현재 ID {current}가 응답하지 않습니다. 배선/전원/ID를 확인하세요.")
            return

        if scan and new != current:
            self.emit("log", text=f"새 ID {new} 충돌 검사 중...")
            if ping(self.bus, self.host_id, new):
                self.emit("done", ok=False, new=None,
                          text=f"새 ID {new}가 이미 버스에서 응답합니다(충돌). 다른 값을 쓰세요.")
                return

        self.emit("log", text=f"ID {current} -> {new} 변경 명령 전송...")
        send_set_id(self.bus, self.host_id, current, new)

        self.emit("log", text=f"새 ID {new} 검증 중...")
        for _ in range(10):
            if ping(self.bus, self.host_id, new):
                self.emit("done", ok=True, new=new,
                          text=f"성공: 모터가 이제 ID {new} (0x{new:02X})로 응답합니다.")
                return
            time.sleep(0.2)

        # Like the official example: tell "write didn't take" apart from
        # "motor went quiet", because the two need different next steps.
        if ping(self.bus, self.host_id, current):
            self.emit("done", ok=False, new=None,
                      text=f"변경 실패: 모터가 여전히 이전 ID {current}로 응답합니다. ID는 바뀌지 않았습니다.")
        else:
            self.emit("done", ok=False, new=None,
                      text=f"ID {new}, {current} 둘 다 무응답입니다. 전원을 재인가한 뒤 '전원 켜서 찾기'로 확인하세요.")

    def shutdown(self):
        self._stop.set()
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass


class App:
    def __init__(self, root, args):
        self.root = root
        self.cmd_q = queue.Queue()
        self.out_q = queue.Queue()
        self.worker = Worker(args.channel, args.interface, args.host_id, args.scan_max,
                             self.cmd_q, self.out_q)
        self.busy = False

        root.title("Robstride Set Motor ID")
        root.geometry("460x470")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        frm = ttk.Frame(root, padding=12)
        frm.pack(fill=tk.BOTH, expand=True)

        self.bus_var = tk.StringVar(value=f"버스 여는 중... ({args.channel})")
        ttk.Label(frm, textvariable=self.bus_var, foreground="#555").pack(anchor="w", pady=(0, 10))

        cur = ttk.Frame(frm)
        cur.pack(fill=tk.X, pady=3)
        ttk.Label(cur, text="현재 ID", width=8).pack(side=tk.LEFT)
        self.current_var = tk.StringVar()
        self.current_box = ttk.Combobox(cur, textvariable=self.current_var, width=6, values=[])
        self.current_box.pack(side=tk.LEFT)
        self.current_box.bind("<Return>", lambda _e: self.check_current())
        self.scan_btn = ttk.Button(cur, text="스캔", command=self.scan)
        self.scan_btn.pack(side=tk.LEFT, padx=(6, 0))
        self.check_btn = ttk.Button(cur, text="확인", command=self.check_current)
        self.check_btn.pack(side=tk.LEFT, padx=(4, 0))
        self.check_var = tk.StringVar(value="-")
        ttk.Label(cur, textvariable=self.check_var).pack(side=tk.LEFT, padx=(8, 0))

        listen_row = ttk.Frame(frm)
        listen_row.pack(fill=tk.X, pady=(2, 0))
        ttk.Label(listen_row, text="", width=8).pack(side=tk.LEFT)
        self.listen_btn = ttk.Button(listen_row, text=f"전원 켜서 찾기 ({LISTEN_SECONDS:.0f}s)",
                                     command=self.listen)
        self.listen_btn.pack(side=tk.LEFT)
        ttk.Label(listen_row, text="스캔이 실패할 때", foreground="#888").pack(side=tk.LEFT, padx=(8, 0))

        new = ttk.Frame(frm)
        new.pack(fill=tk.X, pady=3)
        ttk.Label(new, text="새 ID", width=8).pack(side=tk.LEFT)
        self.new_var = tk.StringVar()
        ttk.Entry(new, textvariable=self.new_var, width=8).pack(side=tk.LEFT)
        ttk.Label(new, text=f"({MIN_MOTOR_ID}~{MAX_MOTOR_ID}, 예: 5 또는 0x05)",
                  foreground="#888").pack(side=tk.LEFT, padx=(8, 0))

        self.scan_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(frm, text="변경 전 새 ID 충돌 검사", variable=self.scan_var).pack(anchor="w", pady=(8, 8))

        self.change_btn = ttk.Button(frm, text="변경", command=self.change_id)
        self.change_btn.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(frm, text="로그").pack(anchor="w")
        self.log = tk.Text(frm, height=8, wrap="word", state="disabled")
        self.log.pack(fill=tk.BOTH, expand=True)

        self.worker.start()
        self.poll()

    def parse_id(self, raw, field):
        raw = raw.strip()
        if not raw:
            messagebox.showerror("입력 오류", f"{field}를 입력하세요.")
            return None
        try:
            value = int(raw, 0)
        except ValueError:
            messagebox.showerror("입력 오류", f"{field}는 숫자여야 합니다 (예: 5 또는 0x05).")
            return None
        if not MIN_MOTOR_ID <= value <= MAX_MOTOR_ID:
            messagebox.showerror("입력 오류",
                                 f"{field}는 {MIN_MOTOR_ID}~{MAX_MOTOR_ID} 범위여야 합니다.")
            return None
        return value

    def scan(self):
        if self.busy:
            return
        self.set_busy(True)
        self.check_var.set("-")
        self.cmd_q.put(("scan",))

    def listen(self):
        if self.busy:
            return
        self.set_busy(True)
        self.check_var.set("-")
        self.append_log(f"[전원 켜서 찾기] {LISTEN_SECONDS:.0f}초 대기 -- 지금 모터 전원을 인가하세요.")
        self.cmd_q.put(("listen",))

    def check_current(self):
        if self.busy:
            return
        current = self.parse_id(self.current_var.get(), "현재 ID")
        if current is None:
            return
        self.check_var.set("확인 중...")
        self.cmd_q.put(("check", current))

    def change_id(self):
        if self.busy:
            return
        current = self.parse_id(self.current_var.get(), "현재 ID")
        if current is None:
            return
        new = self.parse_id(self.new_var.get(), "새 ID")
        if new is None:
            return
        if current == new:
            messagebox.showinfo("변경", "현재 ID와 새 ID가 같습니다.")
            return
        if not messagebox.askyesno(
            "영구 변경 확인",
            f"모터 ID {current} (0x{current:02X}) -> {new} (0x{new:02X})\n"
            "영구적으로 변경합니다. 진행할까요?",
        ):
            return
        self.set_busy(True)
        self.append_log(f"[변경 시작] {current} -> {new}")
        self.cmd_q.put(("change", current, new, self.scan_var.get()))

    def set_busy(self, busy):
        self.busy = busy
        state = "disabled" if busy else "normal"
        for widget in (self.change_btn, self.scan_btn, self.check_btn, self.listen_btn):
            widget.configure(state=state)

    def append_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", time.strftime("[%H:%M:%S] ") + text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def poll(self):
        while True:
            try:
                kind, kw = self.out_q.get_nowait()
            except queue.Empty:
                break
            if kind == "bus":
                self.bus_var.set(kw["text"])
            elif kind == "check":
                self.check_var.set("응답함 ✅" if kw["ok"] else "무응답 ❌")
            elif kind == "scan":
                ids = kw["ids"]
                self.current_box.configure(values=[str(i) for i in ids])
                if len(ids) == 1:
                    self.current_var.set(str(ids[0]))
                    self.check_var.set("응답함 ✅")
                self.set_busy(False)
            elif kind == "log":
                self.append_log(kw["text"])
            elif kind == "error":
                self.append_log("오류: " + kw["text"])
            elif kind == "done":
                self.append_log(kw["text"])
                if kw["ok"] and kw.get("new") is not None:
                    self.current_var.set(str(kw["new"]))
                    self.new_var.set("")
                    self.check_var.set("응답함 ✅")
                elif self.check_var.get() == "확인 중...":
                    self.check_var.set("-")
                self.set_busy(False)
        self.root.after(80, self.poll)

    def on_close(self):
        self.worker.shutdown()
        self.root.destroy()


def parse_args():
    parser = argparse.ArgumentParser(description="Change a Robstride motor's CAN ID from a small GUI.")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL, help="CAN channel, default: can0")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE, help="python-can interface, default: socketcan")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID,
                        help="host CAN ID used in the private protocol, default: 0xFD")
    parser.add_argument("--scan-max", type=int, default=127,
                        help="highest motor ID probed by Scan, default: %(default)s")
    return parser.parse_args()


def main():
    args = parse_args()
    root = tk.Tk()
    App(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
