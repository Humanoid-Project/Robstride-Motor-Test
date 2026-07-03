#!/usr/bin/env python3
"""Two-motor daisy-chain GUI (independent control).

Both motors sit on the SAME CAN bus (daisy chain):
  - RS02: directly wired to the laptop CAN adapter
  - RS03: chained after RS02

Each motor gets its own control panel, its own CAN socket, and its own
model-specific encoding ranges. The heavy lifting (controller thread, safety
logic, graphing) is reused from motor_run.py -- only the per-model encoding and
the two-panel layout are added here.

    python3 daisy_chain_run.py                 # RS02 id=7, RS03 id=5
    python3 daisy_chain_run.py --channel can0
"""
import argparse
import copy
import math
import queue
import struct
import time
import tkinter as tk
from tkinter import messagebox, ttk

import can

import motor_run as mr
from motor_run import (
    GraphCanvas,
    MotorController,
    RS03Motor,
    clamp,
    float_to_uint,
    parse_arbitration_id,
    uint_to_float,
    mode_status_name,
    HOST_ID,
    KP_MIN,
    KP_MAX,
    KD_MIN,
    KD_MAX,
    SAFE_MAX_ACCEL,
    SAFE_MAX_KD,
    SAFE_MAX_SPEED,
    SAFE_MAX_VEL_KP,
    SAFE_DEFAULT_ACCEL,
)


# ---------------------------------------------------------------------------
# Per-model encoding ranges (MIT / operation control mode).
#
# ⚠️  These MUST match each motor's firmware ranges, or commands are scaled
#     wrong (e.g. a "1.0 rad/s" command becomes ~2.2 rad/s). Position is the
#     same for both; velocity and torque differ by model. VERIFY RS02 AGAINST
#     THE MANUAL before running anything faster than a crawl.
# ---------------------------------------------------------------------------
class MotorSpec:
    def __init__(self, name, p_min, p_max, v_min, v_max, t_min, t_max):
        self.name = name
        self.p_min = p_min
        self.p_max = p_max
        self.v_min = v_min
        self.v_max = v_max
        self.t_min = t_min
        self.t_max = t_max


# RS03: from motor_run.py (verified in use).
RS03_SPEC = MotorSpec("RS03", -12.57, 12.57, -20.0, 20.0, -60.0, 60.0)
# RS02: typical Robstride RS02 ranges. VERIFY against your manual.
RS02_SPEC = MotorSpec("RS02", -12.57, 12.57, -44.0, 44.0, -17.0, 17.0)


class SpecMotor(RS03Motor):
    """RS03Motor with model-specific encode/decode ranges."""

    def __init__(self, bus, motor_id, host_id, spec):
        super().__init__(bus=bus, motor_id=motor_id, host_id=host_id)
        self.spec = spec

    def _parse_feedback(self, msg):
        if msg is None or not msg.is_extended_id:
            return None
        comm_type, data16, destination = parse_arbitration_id(msg.arbitration_id)
        response_motor_id = data16 & 0xFF
        if comm_type != 0x02 or destination != self.host_id or response_motor_id != self.motor_id:
            return None
        data = bytes(msg.data)
        if len(data) < 8:
            return None
        raw_pos = (data[0] << 8) | data[1]
        raw_vel = (data[2] << 8) | data[3]
        raw_torque = (data[4] << 8) | data[5]
        raw_temp = (data[6] << 8) | data[7]
        s = self.spec
        return {
            "position": uint_to_float(raw_pos, s.p_min, s.p_max, 16),
            "velocity": uint_to_float(raw_vel, s.v_min, s.v_max, 16),
            "torque": uint_to_float(raw_torque, s.t_min, s.t_max, 16),
            "temperature": raw_temp / 10.0,
            "fault_flags": (data16 >> 8) & 0x3F,
            "mode_status": (data16 >> 14) & 0x03,
        }

    def drain_feedback(self):
        latest = None
        while True:
            msg = self.recv(timeout=0.0)
            if msg is None:
                return latest
            feedback = self._parse_feedback(msg)
            if feedback is not None:
                latest = feedback

    def enable(self):
        self.send(0x03, self.host_id, bytes(8))
        deadline = time.monotonic() + 0.5
        latest = None
        while time.monotonic() < deadline:
            msg = self.recv(timeout=0.02)
            feedback = self._parse_feedback(msg)
            if feedback is not None:
                latest = feedback
        return latest

    def read_float_parameter(self, index, timeout=0.02):
        data = bytearray(8)
        struct.pack_into("<H", data, 0, index)
        self.send(0x11, self.host_id, data)
        deadline = time.monotonic() + timeout
        latest_feedback = None
        while time.monotonic() < deadline:
            msg = self.recv(timeout=max(0.0, deadline - time.monotonic()))
            if msg is None:
                break
            feedback = self._parse_feedback(msg)
            if feedback is not None:
                latest_feedback = feedback
                continue
            if msg.is_extended_id:
                comm_type, data16, destination = parse_arbitration_id(msg.arbitration_id)
                if (
                    comm_type == 0x11
                    and destination == self.host_id
                    and (data16 & 0xFF) == self.motor_id
                ):
                    payload = bytes(msg.data)
                    if len(payload) >= 8:
                        response_index = int.from_bytes(payload[0:2], "little", signed=False)
                        if response_index == index:
                            return struct.unpack_from("<f", payload, 4)[0], latest_feedback
        return None, latest_feedback

    def control_operation_mode(self, pos, vel, kp, kd, torque=0.0, kp_max=KP_MAX):
        s = self.spec
        data16 = float_to_uint(torque, s.t_min, s.t_max, 16)
        raw_pos = float_to_uint(pos, s.p_min, s.p_max, 16)
        raw_vel = float_to_uint(vel, s.v_min, s.v_max, 16)
        raw_kp = float_to_uint(kp, KP_MIN, kp_max, 16)
        raw_kd = float_to_uint(kd, KD_MIN, KD_MAX, 16)
        data = bytes(
            [
                (raw_pos >> 8) & 0xFF,
                raw_pos & 0xFF,
                (raw_vel >> 8) & 0xFF,
                raw_vel & 0xFF,
                (raw_kp >> 8) & 0xFF,
                raw_kp & 0xFF,
                (raw_kd >> 8) & 0xFF,
                raw_kd & 0xFF,
            ]
        )
        self.send(0x01, data16, data)
        return self.drain_feedback()


class SpecMotorController(MotorController):
    """MotorController that builds a spec-aware motor for its model."""

    def __init__(self, args, event_queue, spec):
        super().__init__(args, event_queue)
        self.spec = spec

    def make_motor(self, bus):
        return SpecMotor(
            bus=bus,
            motor_id=self.args.motor_id,
            host_id=self.args.host_id,
            spec=self.spec,
        )


def make_controller_args(channel, interface, motor_id):
    args = argparse.Namespace(
        channel=channel,
        interface=interface,
        motor_id=motor_id,
        host_id=HOST_ID,
        max_speed=clamp(SAFE_MAX_SPEED, 0.01, SAFE_MAX_SPEED),
        jog_speed=0.05,
        accel=SAFE_DEFAULT_ACCEL,
        rate=50.0,
        kp=0.0,
        kd=0.5,
        return_time=4.0,
        return_hold_time=0.4,
        return_kp=1.0,
        return_kd=0.5,
        no_return_home=True,  # daisy-chain default: do not sweep back on close
    )
    return args


class MotorPanel:
    """One motor's controls + graph + live state, bound to its own controller."""

    def __init__(self, parent, app, title, spec, channel, interface, default_id, id_editable):
        self.app = app
        self.spec = spec
        self.channel = channel
        self.interface = interface
        self.id_editable = id_editable
        self.events = queue.Queue()
        self.controller = None
        self.current_motor_id = default_id

        self.frame = ttk.LabelFrame(parent, text=title, padding=8)

        self.motor_id_var = tk.StringVar(value=str(default_id) if default_id is not None else "")
        self.velocity_var = tk.DoubleVar(value=0.0)
        self.accel_var = tk.DoubleVar(value=SAFE_DEFAULT_ACCEL)
        self.kp_var = tk.DoubleVar(value=0.0)
        self.kd_var = tk.DoubleVar(value=0.5)

        self.status_var = tk.StringVar(value="Idle")
        self.active_id_var = tk.StringVar(value="-")
        self.feedback_var = tk.StringVar(value="none")
        self.position_var = tk.StringVar(value="0.000 rad / 0.00 deg")
        self.velocity_read_var = tk.StringVar(value="0.000 rad/s")
        self.target_var = tk.StringVar(value="0.000 rad/s")
        self.vbus_var = tk.StringVar(value="0.00 V")
        self.torque_var = tk.StringVar(value="0.000 N.m")
        self.temp_var = tk.StringVar(value="0.0 deg C")
        self.mode_var = tk.StringVar(value="-")
        self.fault_var = tk.StringVar(value="0x00")

        self._build()

    # -- UI ------------------------------------------------------------------
    def _build(self):
        self.graph = GraphCanvas(self.frame, width=360, height=300)
        self.graph.pack(fill=tk.BOTH, expand=True, pady=(0, 8))

        device = ttk.Frame(self.frame)
        device.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(device, text="Motor ID", width=9).pack(side=tk.LEFT)
        entry = ttk.Entry(device, textvariable=self.motor_id_var, width=6)
        entry.pack(side=tk.LEFT)
        if not self.id_editable:
            entry.configure(state="readonly")
        ttk.Button(device, text="Connect", command=self.apply_motor_id).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(device, text=f"{self.channel} · {self.spec.name}").pack(side=tk.LEFT, padx=(8, 0))

        state = ttk.LabelFrame(self.frame, text="Live State", padding=6)
        state.pack(fill=tk.X, pady=(0, 6))
        for label, var in [
            ("Status", self.status_var),
            ("Motor ID", self.active_id_var),
            ("Feedback", self.feedback_var),
            ("Position", self.position_var),
            ("Velocity", self.velocity_read_var),
            ("Target", self.target_var),
            ("VBUS", self.vbus_var),
            ("Torque", self.torque_var),
            ("Temp", self.temp_var),
            ("Mode", self.mode_var),
            ("Fault", self.fault_var),
        ]:
            row = ttk.Frame(state)
            row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=label, width=10).pack(side=tk.LEFT)
            ttk.Label(row, textvariable=var, anchor="w").pack(side=tk.LEFT, fill=tk.X, expand=True)

        control = ttk.LabelFrame(self.frame, text="Control", padding=6)
        control.pack(fill=tk.X)
        self._slider(control, "Target velocity (rad/s)", self.velocity_var,
                     -SAFE_MAX_SPEED, SAFE_MAX_SPEED, self.on_velocity)
        self._slider(control, "Accel limit (rad/s^2)", self.accel_var,
                     0.01, SAFE_MAX_ACCEL, self.on_accel)
        self._slider(control, "Kp", self.kp_var, 0.0, SAFE_MAX_VEL_KP, self.on_kp)
        self._slider(control, "Kd", self.kd_var, 0.0, SAFE_MAX_KD, self.on_kd)

        buttons = ttk.Frame(control)
        buttons.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(buttons, text="Enable", command=self.enable).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(buttons, text="Stop", command=self.stop).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 0))

        jog = ttk.Frame(control)
        jog.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(jog, text="- Jog", command=lambda: self.adjust(-0.05)).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(jog, text="Zero vel", command=lambda: self.set_velocity(0.0)).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        ttk.Button(jog, text="+ Jog", command=lambda: self.adjust(0.05)).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(control, text="Clear Fault", command=self.clear_fault).pack(fill=tk.X, pady=(6, 0))

    def _slider(self, parent, label, variable, lo, hi, command):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(frame, text=label).pack(anchor="w")
        value_text = tk.StringVar(value=f"{variable.get():.3f}")

        def refresh(*_):
            value_text.set(f"{variable.get():.3f}")

        variable.trace_add("write", refresh)
        row = ttk.Frame(frame)
        row.pack(fill=tk.X)
        ttk.Scale(row, from_=lo, to=hi, variable=variable,
                  command=lambda _v: command()).pack(side=tk.LEFT, fill=tk.X, expand=True)

        def apply_entry(*_):
            try:
                value = float(value_text.get())
            except ValueError:
                value_text.set(f"{variable.get():.3f}")
                return
            variable.set(max(lo, min(hi, value)))
            command()

        entry = ttk.Entry(row, textvariable=value_text, width=8, justify=tk.RIGHT)
        entry.pack(side=tk.LEFT, padx=(6, 0))
        entry.bind("<Return>", apply_entry)
        entry.bind("<FocusOut>", apply_entry)

    # -- controller lifecycle ------------------------------------------------
    def apply_motor_id(self):
        raw = self.motor_id_var.get().strip()
        if not raw:
            messagebox.showerror("Motor ID", f"{self.spec.name}: enter a motor ID first.")
            return
        try:
            motor_id = int(raw, 0)
        except ValueError:
            messagebox.showerror("Motor ID", "Motor ID must be a number, e.g. 2 or 0x02.")
            return
        if not 0 <= motor_id <= 127:
            messagebox.showerror("Motor ID", "Motor ID must be between 0 and 127.")
            return
        self._stop_controller()
        self.set_velocity(0.0, send=False)
        self.graph.reset()
        self.current_motor_id = motor_id
        args = make_controller_args(self.channel, self.interface, motor_id)
        args.accel = self.accel_var.get()
        args.kp = self.kp_var.get()
        args.kd = self.kd_var.get()
        self.controller = SpecMotorController(args, self.events, self.spec)
        self.controller.start()

    def _stop_controller(self):
        if self.controller is not None:
            self.controller.shutdown(return_home=False)
            self.controller.join(timeout=1.5)
            if self.controller.is_alive():
                self.controller.force_shutdown()
                self.controller.join(timeout=0.5)
            self.controller = None

    # -- control actions -----------------------------------------------------
    def set_velocity(self, value, send=True):
        value = max(-SAFE_MAX_SPEED, min(SAFE_MAX_SPEED, float(value)))
        self.velocity_var.set(value)
        if send and self.controller is not None:
            self.controller.set_target_velocity(value)

    def adjust(self, delta):
        self.set_velocity(self.velocity_var.get() + delta)

    def enable(self):
        if self.controller is None:
            messagebox.showinfo("Not connected", f"{self.spec.name}: press Connect first.")
            return
        self.controller.request_enable()

    def stop(self):
        self.set_velocity(0.0)
        if self.controller is not None:
            self.controller.request_stop()

    def clear_fault(self):
        if self.controller is not None:
            self.controller.request_clear_fault()

    def on_velocity(self):
        if self.controller is not None:
            self.controller.set_target_velocity(self.velocity_var.get())

    def on_accel(self):
        if self.controller is not None:
            self.controller.set_accel_limit(self.accel_var.get())

    def on_kp(self):
        if self.controller is not None:
            self.controller.set_kp(self.kp_var.get())

    def on_kd(self):
        if self.controller is not None:
            self.controller.set_kd(self.kd_var.get())

    # -- periodic update -----------------------------------------------------
    def tick(self):
        while True:
            try:
                event_type, message = self.events.get_nowait()
            except queue.Empty:
                break
            if event_type == "error":
                messagebox.showerror(f"{self.spec.name} error", message)
        if self.controller is None:
            return
        state = self.controller.snapshot()
        self.graph.add_sample(state)
        self.graph.redraw()
        self._update_labels(state)

    def _update_labels(self, state):
        now = time.monotonic()
        self.status_var.set(state["status"])
        self.active_id_var.set(str(state["motor_id"]))
        if state["feedback_timestamp"] > 0.0:
            self.feedback_var.set(f"{state['feedback_count']} frames, {now - state['feedback_timestamp']:.2f}s ago")
        else:
            self.feedback_var.set("none")
        self.position_var.set(f"{state['position']:+.4f} rad / {math.degrees(state['position']):+.2f} deg")
        self.velocity_read_var.set(f"{state['velocity']:+.4f} rad/s")
        self.target_var.set(f"{state['target_velocity']:+.4f} rad/s")
        self.vbus_var.set(f"{state['vbus']:.2f} V")
        self.torque_var.set(f"{state['torque']:+.3f} N.m")
        self.temp_var.set(f"{state['temperature']:.1f} deg C")
        self.mode_var.set(f"{state['mode_status']} ({mode_status_name(state['mode_status'])})")
        self.fault_var.set(f"0x{state['fault_flags']:02X}")

    def shutdown(self):
        self._stop_controller()


class DaisyChainApp:
    def __init__(self, root, args):
        self.root = root
        self.closing = False
        root.title("RS02 + RS03 Daisy-Chain")
        root.geometry("820x760")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        container = ttk.Frame(root, padding=10)
        container.pack(fill=tk.BOTH, expand=True)
        container.columnconfigure(0, weight=1, uniform="motor")
        container.columnconfigure(1, weight=1, uniform="motor")
        container.rowconfigure(0, weight=1)

        # RS02: directly wired, ID entered by user. RS03: chained, default 5.
        self.rs02 = MotorPanel(
            container, self, "RS02  (laptop-direct)", RS02_SPEC,
            args.channel, args.interface, default_id=args.rs02_id, id_editable=True,
        )
        self.rs03 = MotorPanel(
            container, self, "RS03  (chained)", RS03_SPEC,
            args.channel, args.interface, default_id=args.rs03_id, id_editable=True,
        )
        self.rs02.frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self.rs03.frame.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        self.panels = [self.rs02, self.rs03]

        self.update_loop()

    def update_loop(self):
        for panel in self.panels:
            panel.tick()
        self.root.after(50, self.update_loop)

    def on_close(self):
        if self.closing:
            return
        self.closing = True
        self.root.protocol("WM_DELETE_WINDOW", lambda: None)
        for panel in self.panels:
            panel.shutdown()
        self.root.destroy()


def parse_args():
    parser = argparse.ArgumentParser(description="Two-motor daisy-chain GUI (RS02 + RS03).")
    parser.add_argument("--channel", default=mr.DEFAULT_CHANNEL, help="CAN channel, default: can0")
    parser.add_argument("--interface", default=mr.DEFAULT_INTERFACE, help="python-can interface, default: socketcan")
    parser.add_argument("--rs02-id", type=lambda v: int(v, 0), default=7, help="RS02 CAN ID, default: 7")
    parser.add_argument("--rs03-id", type=lambda v: int(v, 0), default=5, help="RS03 CAN ID, default: 5")
    return parser.parse_args()


def main():
    args = parse_args()
    root = tk.Tk()
    DaisyChainApp(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
