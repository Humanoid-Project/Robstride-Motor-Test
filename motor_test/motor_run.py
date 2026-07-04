#!/usr/bin/env python3
import argparse
import collections
import math
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import can


HOST_ID = 0xFD
DEFAULT_MOTOR_ID = 5
DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"

KP_MIN = 0.0
KD_MIN = 0.0
SAFE_MAX_SPEED = 2.0
SAFE_MAX_ACCEL = 0.25
SAFE_DEFAULT_ACCEL = 0.10
SAFE_MAX_JOG_SPEED = 0.05
SAFE_MAX_KD = 5.0
SAFE_MAX_RETURN_SPEED = 0.20
SAFE_MAX_POSITION_SPEED = 1.0
SAFE_MIN_RETURN_TIME = 4.0
SAFE_MAX_RETURN_KP = 8.0
SAFE_MAX_RETURN_KD = 5.0
SAFE_OVERSPEED_STOP = 2.0
SAFE_CLOSE_TIMEOUT = 90.0

VBUS_INDEX = 0x701C
MECH_POS_INDEX = 0x7019
RUN_MODE_INDEX = 0x7005
OPERATION_RUN_MODE = 0


class MotorSpec:
    def __init__(self, name, p_min, p_max, v_min, v_max, t_min, t_max, kp_max, kd_max):
        self.name = name
        self.p_min = p_min
        self.p_max = p_max
        self.v_min = v_min
        self.v_max = v_max
        self.t_min = t_min
        self.t_max = t_max
        self.kp_max = kp_max
        self.kd_max = kd_max


# Manual section 4.1.2 (operation control mode, Type 1 frame) encoding ranges.
# Position range is the same for both models; velocity/torque/Kp/Kd are not.
RS03_SPEC = MotorSpec("RS03", -12.57, 12.57, -20.0, 20.0, -60.0, 60.0, kp_max=5000.0, kd_max=100.0)
RS02_SPEC = MotorSpec("RS02", -12.57, 12.57, -44.0, 44.0, -17.0, 17.0, kp_max=500.0, kd_max=5.0)


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))


def uint_to_float(x, x_min, x_max, bits):
    return x / ((1 << bits) - 1) * (x_max - x_min) + x_min


def parse_arbitration_id(arbitration_id):
    comm_type = (arbitration_id >> 24) & 0x1F
    data16 = (arbitration_id >> 8) & 0xFFFF
    destination = arbitration_id & 0xFF
    return comm_type, data16, destination


def parse_feedback(msg, host_id, motor_id, spec):
    if msg is None or not msg.is_extended_id:
        return None

    comm_type, data16, destination = parse_arbitration_id(msg.arbitration_id)
    response_motor_id = data16 & 0xFF
    if comm_type != 0x02 or destination != host_id or response_motor_id != motor_id:
        return None

    data = bytes(msg.data)
    if len(data) < 8:
        return None

    raw_pos = (data[0] << 8) | data[1]
    raw_vel = (data[2] << 8) | data[3]
    raw_torque = (data[4] << 8) | data[5]
    raw_temp = (data[6] << 8) | data[7]
    fault_flags = (data16 >> 8) & 0x3F
    mode_status = (data16 >> 14) & 0x03

    return {
        "position": uint_to_float(raw_pos, spec.p_min, spec.p_max, 16),
        "velocity": uint_to_float(raw_vel, spec.v_min, spec.v_max, 16),
        "torque": uint_to_float(raw_torque, spec.t_min, spec.t_max, 16),
        "temperature": raw_temp / 10.0,
        "fault_flags": fault_flags,
        "mode_status": mode_status,
    }


def parse_float_parameter(msg, host_id, motor_id, index):
    if msg is None or not msg.is_extended_id:
        return None

    comm_type, data16, destination = parse_arbitration_id(msg.arbitration_id)
    response_motor_id = data16 & 0xFF
    if comm_type != 0x11 or destination != host_id or response_motor_id != motor_id:
        return None

    data = bytes(msg.data)
    if len(data) < 8:
        return None

    response_index = int.from_bytes(data[0:2], byteorder="little", signed=False)
    if response_index != index:
        return None

    return struct.unpack_from("<f", data, 4)[0]


class Motor:
    def __init__(self, bus, motor_id, host_id, spec):
        self.bus = bus
        self.motor_id = motor_id
        self.host_id = host_id
        self.spec = spec

    def send(self, comm_type, data16, data):
        arb_id = ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | self.motor_id
        msg = can.Message(arbitration_id=arb_id, data=list(data), is_extended_id=True)
        self.bus.send(msg)

    def recv(self, timeout=0.0):
        return self.bus.recv(timeout=timeout)

    def drain_feedback(self):
        latest = None
        while True:
            msg = self.recv(timeout=0.0)
            if msg is None:
                return latest

            feedback = parse_feedback(msg, self.host_id, self.motor_id, self.spec)
            if feedback is not None:
                latest = feedback

    def enable(self):
        self.send(0x03, self.host_id, bytes(8))
        deadline = time.monotonic() + 0.5
        latest = None
        while time.monotonic() < deadline:
            msg = self.recv(timeout=0.02)
            feedback = parse_feedback(msg, self.host_id, self.motor_id, self.spec)
            if feedback is not None:
                latest = feedback
        return latest

    def stop(self, clear_fault=False):
        data = bytearray(8)
        if clear_fault:
            data[0] = 1
        self.send(0x04, self.host_id, data)
        return self.recv(timeout=0.2)

    def write_uint8_parameter(self, index, value):
        data = bytearray(8)
        struct.pack_into("<H", data, 0, index)
        data[4] = value & 0xFF
        self.send(0x12, self.host_id, data)
        return self.drain_feedback()

    def control_operation_mode(self, pos, vel, kp, kd, torque=0.0):
        s = self.spec
        data16 = float_to_uint(torque, s.t_min, s.t_max, 16)
        raw_pos = float_to_uint(pos, s.p_min, s.p_max, 16)
        raw_vel = float_to_uint(vel, s.v_min, s.v_max, 16)
        raw_kp = float_to_uint(kp, KP_MIN, s.kp_max, 16)
        raw_kd = float_to_uint(kd, KD_MIN, s.kd_max, 16)

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

            feedback = parse_feedback(msg, self.host_id, self.motor_id, self.spec)
            if feedback is not None:
                latest_feedback = feedback
                continue

            value = parse_float_parameter(msg, self.host_id, self.motor_id, index)
            if value is not None:
                return value, latest_feedback

        return None, latest_feedback


class MotorController(threading.Thread):
    def __init__(self, args, event_queue):
        super().__init__(daemon=True)
        self.args = args
        self.spec = args.spec
        self.event_queue = event_queue
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.pending_enable = False
        self.pending_stop = False
        self.pending_clear_fault = False
        self.pending_zero_position = False
        self.pending_move_to = None
        self.pending_shutdown = False
        self.shutdown_return_home = True
        self.shutdown_active = False
        self.motion_active = False
        self.final_stop_sent = False
        self.enabled = False
        self.initial_position = None
        self.position_hold_target = None
        self.target_velocity = 0.0
        self.command_velocity = 0.0
        self.accel_limit = args.accel
        self.kd = args.kd
        self.return_kp = args.return_kp
        self.return_kd = args.return_kd
        self.move_speed = clamp(getattr(args, "move_speed", SAFE_MAX_POSITION_SPEED), 0.01, SAFE_MAX_POSITION_SPEED)
        self.state = {
            "timestamp": 0.0,
            "feedback_timestamp": 0.0,
            "vbus_timestamp": 0.0,
            "position": 0.0,
            "velocity": 0.0,
            "acceleration": 0.0,
            "command_velocity": 0.0,
            "target_velocity": 0.0,
            "torque": 0.0,
            "vbus": 0.0,
            "temperature": 0.0,
            "fault_flags": 0,
            "mode_status": 0,
            "initial_position": None,
            "enabled": False,
            "motor_id": args.motor_id,
            "feedback_count": 0,
            "status": "Disconnected",
        }

    def set_target_velocity(self, value):
        with self.lock:
            self.target_velocity = clamp(float(value), -self.args.max_speed, self.args.max_speed)
            self.state["target_velocity"] = self.target_velocity
            if abs(self.target_velocity) > 1e-6:
                self.position_hold_target = None

    def set_accel_limit(self, value):
        with self.lock:
            self.accel_limit = clamp(float(value), 0.01, SAFE_MAX_ACCEL)

    def set_kd(self, value):
        with self.lock:
            self.kd = clamp(float(value), 0.0, SAFE_MAX_KD)

    def set_return_kp(self, value):
        with self.lock:
            self.return_kp = clamp(float(value), 0.0, SAFE_MAX_RETURN_KP)

    def set_return_kd(self, value):
        with self.lock:
            self.return_kd = clamp(float(value), 0.0, SAFE_MAX_RETURN_KD)

    def set_move_speed(self, value):
        with self.lock:
            self.move_speed = clamp(float(value), 0.01, SAFE_MAX_POSITION_SPEED)

    def request_enable(self):
        with self.lock:
            self.pending_enable = True

    def request_stop(self):
        with self.lock:
            self.pending_stop = True
            self.target_velocity = 0.0
            self.command_velocity = 0.0
            self.position_hold_target = None
            self.state["target_velocity"] = 0.0
            self.state["command_velocity"] = 0.0

    def request_clear_fault(self):
        with self.lock:
            self.pending_clear_fault = True

    def request_zero_position(self):
        with self.lock:
            self.pending_zero_position = True
            self.target_velocity = 0.0
            self.command_velocity = 0.0
            self.position_hold_target = None
            self.state["target_velocity"] = 0.0
            self.state["command_velocity"] = 0.0
            self.state["status"] = "Returning zero..."

    def request_move_to(self, target_position):
        with self.lock:
            self.pending_move_to = clamp(float(target_position), self.spec.p_min, self.spec.p_max)
            self.target_velocity = 0.0
            self.command_velocity = 0.0
            self.position_hold_target = None
            self.state["target_velocity"] = 0.0
            self.state["command_velocity"] = 0.0
            self.state["status"] = "Moving to position..."

    def request_shutdown(self, return_home=True):
        with self.lock:
            self.pending_shutdown = True
            self.shutdown_return_home = return_home
            self.shutdown_active = True
            self.target_velocity = 0.0
            self.command_velocity = 0.0
            self.position_hold_target = None
            self.state["target_velocity"] = 0.0
            self.state["command_velocity"] = 0.0
            self.state["status"] = "Returning home..." if return_home else "Closing..."

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def shutdown(self, return_home=True):
        self.request_shutdown(return_home=return_home)

    def force_shutdown(self):
        self.stop_event.set()

    def publish_status(self, status):
        with self.lock:
            self.state["status"] = status
            self.state["enabled"] = self.enabled
            self.state["motor_id"] = self.args.motor_id

    def publish_initial_position(self, position):
        if position is None:
            return

        with self.lock:
            if self.initial_position is None:
                self.initial_position = position
                self.state["initial_position"] = position

    def publish_feedback(self, feedback, now, dt):
        if feedback is None:
            return

        previous_velocity = self.state["velocity"]
        acceleration = 0.0
        if dt > 1e-4 and self.state["timestamp"] > 0.0:
            acceleration = (feedback["velocity"] - previous_velocity) / dt

        with self.lock:
            if self.initial_position is None:
                self.initial_position = feedback["position"]

            if self.shutdown_active or self.motion_active:
                status = self.state["status"]
            elif self.position_hold_target is not None:
                status = "Holding position"
            else:
                status = "Enabled" if self.enabled else "Stopped"
            self.state.update(
                {
                    "timestamp": now,
                    "feedback_timestamp": now,
                    "position": feedback["position"],
                    "velocity": feedback["velocity"],
                    "acceleration": acceleration,
                    "command_velocity": self.command_velocity,
                    "target_velocity": self.target_velocity,
                    "torque": feedback["torque"],
                    "temperature": feedback["temperature"],
                    "fault_flags": feedback["fault_flags"],
                    "mode_status": feedback["mode_status"],
                    "initial_position": self.initial_position,
                    "enabled": self.enabled,
                    "motor_id": self.args.motor_id,
                    "feedback_count": self.state["feedback_count"] + 1,
                    "status": status,
                }
            )

    def publish_vbus(self, vbus, timestamp):
        if vbus is None:
            return

        with self.lock:
            self.state["vbus"] = vbus
            self.state["motor_id"] = self.args.motor_id
            self.state["timestamp"] = timestamp
            self.state["vbus_timestamp"] = timestamp

    def publish_position_parameter(self, position, timestamp):
        if position is None:
            return

        with self.lock:
            if self.initial_position is None:
                self.initial_position = position
                self.state["initial_position"] = position
            self.state["timestamp"] = timestamp
            self.state["position"] = position

    def current_position(self):
        with self.lock:
            return self.state["position"]

    def send_zero_velocity(self, motor, count=5):
        hold_position = self.current_position()
        kd = clamp(self.kd, 0.1, SAFE_MAX_KD)
        for _ in range(count):
            feedback = motor.control_operation_mode(pos=hold_position, vel=0.0, kp=0.0, kd=kd, torque=0.0)
            self.publish_feedback(feedback, time.monotonic(), 0.0)
            time.sleep(0.01)

    def ensure_enabled_for_motion(self, motor, status):
        if self.enabled:
            return True

        self.publish_status(status)
        motor.write_uint8_parameter(RUN_MODE_INDEX, OPERATION_RUN_MODE)
        feedback = motor.enable()
        self.enabled = True
        self.command_velocity = 0.0
        self.target_velocity = 0.0
        self.publish_feedback(feedback, time.monotonic(), 0.0)
        return True

    def stop_on_overspeed(self, motor, feedback):
        if feedback is None or abs(feedback["velocity"]) <= SAFE_OVERSPEED_STOP:
            return False

        with self.lock:
            self.target_velocity = 0.0
            self.command_velocity = 0.0
            self.position_hold_target = None
            self.state["target_velocity"] = 0.0
            self.state["command_velocity"] = 0.0

        try:
            motor.stop()
        finally:
            self.enabled = False
            self.publish_status(f"Overspeed stop ({feedback['velocity']:+.2f} rad/s)")
        return True

    def move_to_position(self, motor, target_position, status, hold_after=False,
                         max_speed=SAFE_MAX_RETURN_SPEED, interruptible=False):
        move_speed = clamp(abs(max_speed), 0.01, SAFE_MAX_POSITION_SPEED)
        with self.lock:
            start_position = self.state["position"]
            has_feedback = self.state["feedback_timestamp"] > 0.0

        if not has_feedback:
            position, feedback = motor.read_float_parameter(MECH_POS_INDEX, timeout=0.05)
            self.publish_feedback(feedback, time.monotonic(), 0.0)
            self.publish_position_parameter(position, time.monotonic())
            if position is not None:
                start_position = position
                has_feedback = True

        if not has_feedback:
            self.publish_status("Position unknown")
            return False

        target_position = max(self.spec.p_min, min(self.spec.p_max, target_position))
        travel = target_position - start_position
        distance = abs(travel)
        return_time = max(SAFE_MIN_RETURN_TIME, self.args.return_time)
        if distance > 1e-6:
            return_time = max(return_time, 1.5 * distance / move_speed)
        with self.lock:
            return_kp = clamp(self.return_kp, 0.0, SAFE_MAX_RETURN_KP)
            return_kd = clamp(self.return_kd, 0.0, SAFE_MAX_RETURN_KD)
        period = max(0.005, 1.0 / self.args.rate)
        start_time = time.monotonic()
        last_time = start_time
        start_position = self.current_position() if has_feedback else start_position

        def should_interrupt():
            if not interruptible:
                return False
            with self.lock:
                return self.pending_move_to is not None or self.pending_stop

        with self.lock:
            self.motion_active = True
        self.publish_status(status)
        while not self.stop_event.is_set():
            if should_interrupt():
                with self.lock:
                    self.motion_active = False
                return False
            now = time.monotonic()
            elapsed = now - start_time
            progress = min(1.0, elapsed / return_time)
            smooth = progress * progress * (3.0 - 2.0 * progress)
            smooth_velocity = 6.0 * progress * (1.0 - progress) / return_time
            command_position = start_position + travel * smooth
            command_velocity = clamp(
                travel * smooth_velocity,
                -move_speed,
                move_speed,
            )

            feedback = motor.control_operation_mode(
                pos=command_position,
                vel=command_velocity,
                kp=return_kp,
                kd=return_kd,
                torque=0.0,
            )
            dt = max(1e-4, now - last_time)
            last_time = now
            self.publish_feedback(feedback, now, dt)
            if self.stop_on_overspeed(motor, feedback):
                with self.lock:
                    self.motion_active = False
                return False

            if progress >= 1.0:
                break

            sleep_time = period - (time.monotonic() - now)
            if sleep_time > 0:
                time.sleep(sleep_time)

        hold_deadline = time.monotonic() + max(0.0, self.args.return_hold_time)
        while not self.stop_event.is_set() and time.monotonic() < hold_deadline:
            if should_interrupt():
                with self.lock:
                    self.motion_active = False
                return False
            now = time.monotonic()
            feedback = motor.control_operation_mode(
                pos=target_position,
                vel=0.0,
                kp=return_kp,
                kd=return_kd,
                torque=0.0,
            )
            self.publish_feedback(feedback, now, 0.0)
            if self.stop_on_overspeed(motor, feedback):
                with self.lock:
                    self.motion_active = False
                return False
            time.sleep(period)

        with self.lock:
            self.position_hold_target = target_position if hold_after and self.enabled else None
            self.motion_active = False
            self.state["status"] = "Holding position" if self.position_hold_target is not None else self.state["status"]
        return True

    def return_to_initial_position(self, motor):
        with self.lock:
            home_position = self.initial_position

        if home_position is None:
            home_position, feedback = motor.read_float_parameter(MECH_POS_INDEX, timeout=0.05)
            self.publish_feedback(feedback, time.monotonic(), 0.0)
            self.publish_initial_position(home_position)

        if home_position is None:
            self.publish_status("Stopping (home unknown)")
            return

        self.move_to_position(motor, home_position, "Returning home")

    def send_final_stop(self, motor):
        if self.final_stop_sent:
            return

        try:
            self.send_zero_velocity(motor, count=5)
            motor.stop()
        finally:
            self.enabled = False
            self.final_stop_sent = True
            self.shutdown_active = False
            self.publish_status("Closed")

    def graceful_shutdown(self, motor, return_home):
        self.publish_status("Stopping")
        self.command_velocity = 0.0
        self.target_velocity = 0.0

        if return_home:
            self.ensure_enabled_for_motion(motor, "Preparing return home")
            self.send_zero_velocity(motor, count=8)
            self.return_to_initial_position(motor)
        elif self.enabled:
            self.send_zero_velocity(motor, count=8)

        self.send_final_stop(motor)
        self.stop_event.set()

    def make_motor(self, bus):
        return Motor(bus=bus, motor_id=self.args.motor_id, host_id=self.args.host_id, spec=self.spec)

    def run(self):
        bus = None
        try:
            bus = can.Bus(channel=self.args.channel, interface=self.args.interface)
            motor = self.make_motor(bus)
            self.publish_status("Connected")
            self.event_queue.put(("status", "Connected"))
            initial_position, feedback = motor.read_float_parameter(MECH_POS_INDEX, timeout=0.05)
            self.publish_feedback(feedback, time.monotonic(), 0.0)
            self.publish_position_parameter(initial_position, time.monotonic())
            self.publish_initial_position(initial_position)

            period = 1.0 / self.args.rate
            last_time = time.monotonic()
            next_tick = last_time
            next_vbus_poll = last_time

            while not self.stop_event.is_set():
                now = time.monotonic()
                dt = max(1e-4, now - last_time)
                last_time = now

                with self.lock:
                    pending_enable = self.pending_enable
                    pending_stop = self.pending_stop
                    pending_clear_fault = self.pending_clear_fault
                    pending_zero_position = self.pending_zero_position
                    pending_move_to = self.pending_move_to
                    pending_shutdown = self.pending_shutdown
                    shutdown_return_home = self.shutdown_return_home
                    target_velocity = clamp(self.target_velocity, -self.args.max_speed, self.args.max_speed)
                    accel_limit = clamp(self.accel_limit, 0.01, SAFE_MAX_ACCEL)
                    kd = clamp(self.kd, 0.0, SAFE_MAX_KD)
                    self.pending_enable = False
                    self.pending_stop = False
                    self.pending_clear_fault = False
                    self.pending_zero_position = False
                    self.pending_move_to = None
                    self.pending_shutdown = False

                if pending_shutdown:
                    self.graceful_shutdown(motor, return_home=shutdown_return_home)
                    break

                if pending_clear_fault:
                    motor.stop(clear_fault=True)
                    self.publish_status("Fault clear sent")

                if pending_zero_position:
                    if not self.enabled:
                        self.publish_status("Enable before Zero")
                    else:
                        target_velocity = 0.0
                        self.command_velocity = 0.0
                        self.send_zero_velocity(motor, count=5)
                        self.move_to_position(motor, 0.0, "Returning zero", hold_after=True)

                if pending_move_to is not None:
                    if not self.enabled:
                        self.publish_status("Enable before Move")
                    else:
                        target_velocity = 0.0
                        self.command_velocity = 0.0
                        self.send_zero_velocity(motor, count=5)
                        with self.lock:
                            move_speed = self.move_speed
                        self.move_to_position(
                            motor,
                            pending_move_to,
                            "Moving to position",
                            hold_after=True,
                            max_speed=move_speed,
                            interruptible=True,
                        )

                if pending_enable and not self.enabled:
                    motor.write_uint8_parameter(RUN_MODE_INDEX, OPERATION_RUN_MODE)
                    feedback = motor.enable()
                    self.enabled = True
                    self.command_velocity = 0.0
                    self.publish_status("Enabled" if feedback else "Enabled, waiting for feedback")
                    self.publish_feedback(feedback, now, dt)

                if pending_stop and self.enabled:
                    target_velocity = 0.0
                    self.command_velocity = 0.0
                    self.send_zero_velocity(motor, count=5)
                    motor.stop()
                    self.enabled = False
                    self.publish_status("Stopped")

                if self.enabled:
                    with self.lock:
                        hold_target = self.position_hold_target

                    if hold_target is not None and abs(target_velocity) <= 1e-6:
                        self.command_velocity = 0.0
                        feedback = motor.control_operation_mode(
                            pos=hold_target,
                            vel=0.0,
                            kp=clamp(self.return_kp, 0.0, SAFE_MAX_RETURN_KP),
                            kd=clamp(self.return_kd, 0.0, SAFE_MAX_RETURN_KD),
                            torque=0.0,
                        )
                    else:
                        delta = target_velocity - self.command_velocity
                        max_delta = accel_limit * dt
                        if abs(delta) <= max_delta:
                            self.command_velocity = target_velocity
                        else:
                            self.command_velocity += math.copysign(max_delta, delta)
                        self.command_velocity = clamp(
                            self.command_velocity,
                            -self.args.max_speed,
                            self.args.max_speed,
                        )
                        # Kp is always 0 here: `pos` below is just the last-known
                        # actual position, not a real setpoint, so a nonzero Kp
                        # would fight a stale/lagged error and amplify (runaway).
                        feedback = motor.control_operation_mode(
                            pos=self.current_position(),
                            vel=self.command_velocity,
                            kp=0.0,
                            kd=kd,
                            torque=0.0,
                        )
                    self.publish_feedback(feedback, now, dt)
                    self.stop_on_overspeed(motor, feedback)

                if now >= next_vbus_poll:
                    vbus, feedback = motor.read_float_parameter(VBUS_INDEX, timeout=0.015)
                    self.publish_feedback(feedback, now, dt)
                    self.publish_vbus(vbus, now)
                    next_vbus_poll = now + 0.25

                next_tick += period
                sleep_time = next_tick - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_tick = time.monotonic()

        except Exception as exc:
            self.publish_status(f"Error: {exc}")
            self.event_queue.put(("error", str(exc)))
        finally:
            if bus is not None:
                try:
                    motor = self.make_motor(bus)
                    self.send_final_stop(motor)
                except Exception:
                    pass
                bus.shutdown()
            self.enabled = False
            self.publish_status("Closed")


class GraphCanvas(tk.Canvas):
    def __init__(self, parent, history_seconds=10.0, **kwargs):
        super().__init__(parent, bg="#101317", highlightthickness=0, **kwargs)
        self.history_seconds = history_seconds
        self.samples = collections.deque(maxlen=1000)
        self.last_timestamp = 0.0

    def reset(self):
        self.samples.clear()
        self.last_timestamp = 0.0
        self.redraw()

    def add_sample(self, state):
        timestamp = state["timestamp"]
        if timestamp <= 0.0 or timestamp == self.last_timestamp:
            return
        self.last_timestamp = timestamp
        self.samples.append(
            (
                timestamp,
                math.degrees(state["position"]),
                state["velocity"],
                state["acceleration"],
                state["vbus"],
            )
        )

    def redraw(self):
        self.delete("all")
        width = max(1, self.winfo_width())
        height = max(1, self.winfo_height())
        margin_left = 116
        margin_right = 18
        panel_gap = 20
        top = 24
        bottom = 24
        panel_height = (height - top - bottom - panel_gap * 3) / 4
        panels = [
            ("Angle", "deg", 1, "#6CA8FF"),
            ("Velocity", "rad/s", 2, "#63D471"),
            ("Acceleration", "rad/s^2", 3, "#FFB54A"),
            ("VBUS", "V", 4, "#E06CFF"),
        ]

        if not self.samples:
            self.create_text(
                width / 2,
                height / 2,
                text="Enable the motor to start plotting",
                fill="#9AA4B2",
                font=("TkDefaultFont", 14),
            )
            return

        now = self.samples[-1][0]
        visible = [sample for sample in self.samples if now - sample[0] <= self.history_seconds]
        if len(visible) < 2:
            visible = list(self.samples)

        x0 = margin_left
        x1 = width - margin_right
        time_start = max(visible[0][0], now - self.history_seconds)
        time_span = max(1e-3, now - time_start)

        for i, (title, unit, value_index, color) in enumerate(panels):
            y0 = top + i * (panel_height + panel_gap)
            y1 = y0 + panel_height
            values = [sample[value_index] for sample in visible]
            v_min = min(values)
            v_max = max(values)
            if abs(v_max - v_min) < 1e-6:
                center = (v_max + v_min) / 2.0
                v_min = center - 1.0
                v_max = center + 1.0
            pad = max((v_max - v_min) * 0.12, 0.1)
            v_min -= pad
            v_max += pad

            self.create_rectangle(x0, y0, x1, y1, outline="#2A313A", fill="#14191F")
            self.create_text(
                x0,
                y0 - 10,
                text=f"{title} ({unit})",
                anchor="w",
                fill="#DCE3EA",
                font=("TkDefaultFont", 10, "bold"),
            )
            self.create_text(x0 - 8, y0 + 2, text=f"{v_max:.2f}", anchor="ne", fill="#7E8A97")
            self.create_text(x0 - 8, y1 - 2, text=f"{v_min:.2f}", anchor="se", fill="#7E8A97")

            if v_min < 0 < v_max:
                zero_y = y1 - (0 - v_min) / (v_max - v_min) * (y1 - y0)
                self.create_line(x0, zero_y, x1, zero_y, fill="#2E3A45", dash=(4, 4))

            points = []
            for sample in visible:
                x = x0 + (sample[0] - time_start) / time_span * (x1 - x0)
                y = y1 - (sample[value_index] - v_min) / (v_max - v_min) * (y1 - y0)
                points.extend([x, y])
            if len(points) >= 4:
                self.create_line(*points, fill=color, width=2, smooth=True)


def mode_status_name(mode_status):
    names = {
        0: "Reset",
        1: "Calibration",
        2: "Run",
    }
    return names.get(mode_status, "Unknown")


def make_controller_args(channel, interface, motor_id, spec, host_id=HOST_ID, **overrides):
    args = argparse.Namespace(
        channel=channel,
        interface=interface,
        motor_id=motor_id,
        host_id=host_id,
        spec=spec,
        max_speed=clamp(SAFE_MAX_SPEED, 0.01, SAFE_MAX_SPEED),
        jog_speed=0.05,
        accel=SAFE_DEFAULT_ACCEL,
        rate=50.0,
        kd=0.5,
        return_time=SAFE_MIN_RETURN_TIME,
        return_hold_time=0.4,
        return_kp=1.0,
        return_kd=0.5,
        move_speed=min(0.3, SAFE_MAX_POSITION_SPEED),
        no_return_home=True,
    )
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


class MotorPanel:
    """One motor's graph, live state, and controls (Velocity + Position tabs), bound to its own controller thread."""

    def __init__(self, parent, title, spec, channel, interface, default_id, id_editable=True,
                 host_id=HOST_ID, available_specs=None):
        self.spec = spec
        self.available_specs = {s.name: s for s in (available_specs or [RS02_SPEC, RS03_SPEC])}
        self.channel = channel
        self.interface = interface
        self.id_editable = id_editable
        self.host_id = host_id
        self.events = queue.Queue()
        self.controller = None
        self.current_motor_id = default_id
        self.closing = False
        self.close_started_at = 0.0

        self.frame = ttk.LabelFrame(parent, text=title, padding=8)

        self.model_var = tk.StringVar(value=spec.name)
        self.motor_id_var = tk.StringVar(value=str(default_id) if default_id is not None else "")
        self.velocity_var = tk.DoubleVar(value=0.0)
        self.accel_var = tk.DoubleVar(value=SAFE_DEFAULT_ACCEL)
        self.kd_var = tk.DoubleVar(value=0.5)
        self.target_angle_var = tk.StringVar(value="0.0")
        self.move_speed_var = tk.DoubleVar(value=min(0.3, SAFE_MAX_POSITION_SPEED))
        self.return_kp_var = tk.DoubleVar(value=1.0)
        self.return_kd_var = tk.DoubleVar(value=0.5)

        self.status_var = tk.StringVar(value="Idle")
        self.active_id_var = tk.StringVar(value="-")
        self.feedback_var = tk.StringVar(value="none")
        self.position_var = tk.StringVar(value="0.000 rad / 0.00 deg")
        self.home_var = tk.StringVar(value="unknown")
        self.velocity_read_var = tk.StringVar(value="0.000 rad/s")
        self.target_var = tk.StringVar(value="0.000 rad/s")
        self.vbus_var = tk.StringVar(value="0.00 V")
        self.torque_var = tk.StringVar(value="0.000 N.m")
        self.temp_var = tk.StringVar(value="0.0 deg C")
        self.mode_var = tk.StringVar(value="-")
        self.fault_var = tk.StringVar(value="0x00")

        self._build()

    def _build(self):
        self.graph = GraphCanvas(self.frame, width=360, height=300)
        self.graph.pack(fill=tk.BOTH, expand=True, pady=(0, 8))

        device = ttk.Frame(self.frame)
        device.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(device, text="Model", width=6).pack(side=tk.LEFT)
        model_box = ttk.Combobox(
            device, textvariable=self.model_var, values=list(self.available_specs.keys()),
            state="readonly", width=5,
        )
        model_box.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Label(device, text="ID", width=3).pack(side=tk.LEFT)
        entry = ttk.Entry(device, textvariable=self.motor_id_var, width=5)
        entry.pack(side=tk.LEFT)
        if not self.id_editable:
            entry.configure(state="readonly")
        ttk.Button(device, text="Connect", command=self.apply_motor_id).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(device, text=self.channel).pack(side=tk.LEFT, padx=(8, 0))

        state = ttk.LabelFrame(self.frame, text="Live State", padding=6)
        state.pack(fill=tk.X, pady=(0, 6))
        for label, var in [
            ("Status", self.status_var),
            ("Motor ID", self.active_id_var),
            ("Feedback", self.feedback_var),
            ("Position", self.position_var),
            ("Home", self.home_var),
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

        buttons = ttk.Frame(self.frame)
        buttons.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(buttons, text="Enable", command=self.enable).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(buttons, text="Stop", command=self.stop).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 0))
        ttk.Button(buttons, text="Clear Fault", command=self.clear_fault).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 0))

        notebook = ttk.Notebook(self.frame)
        notebook.pack(fill=tk.X)

        velocity_tab = ttk.Frame(notebook, padding=6)
        notebook.add(velocity_tab, text="Velocity")
        self._slider(velocity_tab, "Target velocity (rad/s)", self.velocity_var,
                     -SAFE_MAX_SPEED, SAFE_MAX_SPEED, self.on_velocity)
        self._slider(velocity_tab, "Accel limit (rad/s^2)", self.accel_var,
                     0.01, SAFE_MAX_ACCEL, self.on_accel)
        self._slider(velocity_tab, "Kd", self.kd_var, 0.0, SAFE_MAX_KD, self.on_kd)

        jog = ttk.Frame(velocity_tab)
        jog.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(jog, text="- Jog", command=lambda: self.adjust(-0.05)).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(jog, text="Zero vel", command=lambda: self.set_velocity(0.0)).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        ttk.Button(jog, text="+ Jog", command=lambda: self.adjust(0.05)).pack(side=tk.LEFT, fill=tk.X, expand=True)

        position_tab = ttk.Frame(notebook, padding=6)
        notebook.add(position_tab, text="Position")
        angle_row = ttk.Frame(position_tab)
        angle_row.pack(fill=tk.X)
        ttk.Label(angle_row, text="Target angle (deg)", width=15).pack(side=tk.LEFT)
        angle_entry = ttk.Entry(angle_row, textvariable=self.target_angle_var, width=8, justify=tk.RIGHT)
        angle_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        angle_entry.bind("<Return>", lambda _event: self.go_to_angle())
        ttk.Button(angle_row, text="Go", command=self.go_to_angle).pack(side=tk.LEFT, padx=(6, 0))
        self._slider(position_tab, "Move speed (rad/s)", self.move_speed_var,
                     0.01, SAFE_MAX_POSITION_SPEED, self.on_move_speed)
        self._slider(position_tab, "Position Kp", self.return_kp_var, 0.0, SAFE_MAX_RETURN_KP, self.on_return_kp)
        self._slider(position_tab, "Position Kd", self.return_kd_var, 0.0, SAFE_MAX_RETURN_KD, self.on_return_kd)

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

    def apply_motor_id(self):
        raw = self.motor_id_var.get().strip()
        if not raw:
            messagebox.showerror("Motor ID", f"{self.model_var.get()}: enter a motor ID first.")
            return
        try:
            motor_id = int(raw, 0)
        except ValueError:
            messagebox.showerror("Motor ID", "Motor ID must be a number, e.g. 2 or 0x02.")
            return
        if not 0 <= motor_id <= 127:
            messagebox.showerror("Motor ID", "Motor ID must be between 0 and 127.")
            return
        self.spec = self.available_specs[self.model_var.get()]
        self._stop_controller()
        self.set_velocity(0.0, send=False)
        self.graph.reset()
        self.current_motor_id = motor_id
        args = make_controller_args(
            self.channel, self.interface, motor_id, self.spec, host_id=self.host_id,
            accel=self.accel_var.get(),
            kd=self.kd_var.get(),
            return_kp=self.return_kp_var.get(),
            return_kd=self.return_kd_var.get(),
            move_speed=self.move_speed_var.get(),
        )
        self.controller = MotorController(args, self.events)
        self.controller.start()

    def _stop_controller(self, return_home=False):
        if self.controller is not None:
            self.controller.shutdown(return_home=return_home)
            self.controller.join(timeout=1.5)
            if self.controller.is_alive():
                self.controller.force_shutdown()
                self.controller.join(timeout=0.5)
            self.controller = None

    def set_velocity(self, value, send=True):
        value = max(-SAFE_MAX_SPEED, min(SAFE_MAX_SPEED, float(value)))
        self.velocity_var.set(value)
        if send and self.controller is not None:
            self.controller.set_target_velocity(value)

    def adjust(self, delta):
        self.set_velocity(self.velocity_var.get() + delta)

    def go_to_angle(self):
        try:
            angle_deg = float(self.target_angle_var.get())
        except ValueError:
            messagebox.showerror("Invalid angle", f"{self.spec.name}: target angle must be a number in degrees.")
            return
        target_rad = math.radians(angle_deg)
        clamped = clamp(target_rad, self.spec.p_min, self.spec.p_max)
        if abs(clamped - target_rad) > 1e-9:
            self.target_angle_var.set(f"{math.degrees(clamped):.1f}")
        self.set_velocity(0.0, send=False)
        if self.controller is None:
            messagebox.showinfo("Not connected", f"{self.spec.name}: press Connect first.")
            return
        self.controller.request_move_to(clamped)

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

    def on_kd(self):
        if self.controller is not None:
            self.controller.set_kd(self.kd_var.get())

    def on_move_speed(self):
        if self.controller is not None:
            self.controller.set_move_speed(self.move_speed_var.get())

    def on_return_kp(self):
        if self.controller is not None:
            self.controller.set_return_kp(self.return_kp_var.get())

    def on_return_kd(self):
        if self.controller is not None:
            self.controller.set_return_kd(self.return_kd_var.get())

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
        if state["initial_position"] is None:
            self.home_var.set("unknown")
        else:
            self.home_var.set(f"{state['initial_position']:+.4f} rad / {math.degrees(state['initial_position']):+.2f} deg")
        self.velocity_read_var.set(f"{state['velocity']:+.4f} rad/s")
        self.target_var.set(f"{state['target_velocity']:+.4f} rad/s")
        self.vbus_var.set(f"{state['vbus']:.2f} V")
        self.torque_var.set(f"{state['torque']:+.3f} N.m")
        self.temp_var.set(f"{state['temperature']:.1f} deg C")
        self.mode_var.set(f"{state['mode_status']} ({mode_status_name(state['mode_status'])})")
        self.fault_var.set(f"0x{state['fault_flags']:02X}")

    def shutdown(self):
        self._stop_controller(return_home=False)


class MotorRunApp:
    """Top-level window hosting one or more MotorPanels side by side."""

    def __init__(self, root, args):
        self.root = root
        self.closing = False
        root.title(args.window_title)
        root.geometry(args.geometry)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        container = ttk.Frame(root, padding=10)
        container.pack(fill=tk.BOTH, expand=True)
        for col in range(len(args.panels)):
            container.columnconfigure(col, weight=1, uniform="motor")
        container.rowconfigure(0, weight=1)

        self.panels = []
        for i, panel_cfg in enumerate(args.panels):
            panel = MotorPanel(
                container,
                panel_cfg["title"],
                panel_cfg["spec"],
                args.channel,
                args.interface,
                default_id=panel_cfg["default_id"],
                id_editable=panel_cfg.get("id_editable", True),
                host_id=args.host_id,
            )
            left_pad = 0 if i == 0 else 6
            right_pad = 6 if i < len(args.panels) - 1 else 0
            panel.frame.grid(row=0, column=i, sticky="nsew", padx=(left_pad, right_pad))
            self.panels.append(panel)

        if len(self.panels) == 1:
            self.bind_keys(self.panels[0])

        self.update_loop()

    def bind_keys(self, panel):
        self.root.bind("<Left>", lambda _event: panel.adjust(-0.05))
        self.root.bind("<Right>", lambda _event: panel.adjust(0.05))
        self.root.bind("a", lambda _event: panel.adjust(-0.05))
        self.root.bind("d", lambda _event: panel.adjust(0.05))
        self.root.bind("<space>", lambda _event: panel.set_velocity(0.0))
        self.root.bind("<Escape>", lambda _event: panel.stop())

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
    parser = argparse.ArgumentParser(description="GUI speed/position control and live plotting for a single motor.")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL, help="CAN channel, default: can0")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE, help="python-can interface, default: socketcan")
    parser.add_argument("--model", choices=["rs02", "rs03"], default="rs03",
                         help="motor model -- selects the Kp/Kd/velocity/torque encoding ranges, default: rs03")
    parser.add_argument("--motor-id", type=lambda v: int(v, 0), default=DEFAULT_MOTOR_ID, help="motor CAN ID, default: %(default)s")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID, help="host CAN ID used in the private protocol, default: 0xFD")
    args = parser.parse_args()
    spec = RS02_SPEC if args.model == "rs02" else RS03_SPEC
    args.window_title = f"{spec.name} Motor Run"
    args.geometry = "480x900"
    args.panels = [{"title": f"{spec.name} (id {args.motor_id})", "spec": spec, "default_id": args.motor_id, "id_editable": True}]
    return args


def main():
    args = parse_args()
    root = tk.Tk()
    MotorRunApp(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
