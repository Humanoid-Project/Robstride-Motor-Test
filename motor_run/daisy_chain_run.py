#!/usr/bin/env python3
import argparse
import tkinter as tk

import motor_run as mr
from motor_run import MotorRunApp, RS02_SPEC, RS03_SPEC


def _spec_for(model):
    return RS02_SPEC if model == "rs02" else RS03_SPEC


def parse_args():
    parser = argparse.ArgumentParser(
        description="Two-motor daisy-chain GUI. Each panel's model (RS02/RS03) is also "
        "selectable at runtime from its Model dropdown."
    )
    parser.add_argument("--channel", default=mr.DEFAULT_CHANNEL, help="CAN channel, default: can0")
    parser.add_argument("--interface", default=mr.DEFAULT_INTERFACE, help="python-can interface, default: socketcan")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=mr.HOST_ID, help="host CAN ID used in the private protocol, default: 0xFD")
    parser.add_argument("--panel1-model", choices=["rs02", "rs03"], default="rs02", help="panel 1 default model, default: rs02")
    parser.add_argument("--panel1-id", type=lambda v: int(v, 0), default=7, help="panel 1 default motor CAN ID, default: 7")
    parser.add_argument("--panel2-model", choices=["rs02", "rs03"], default="rs03", help="panel 2 default model, default: rs03")
    parser.add_argument("--panel2-id", type=lambda v: int(v, 0), default=5, help="panel 2 default motor CAN ID, default: 5")
    args = parser.parse_args()
    args.window_title = "Daisy-Chain Motor Run"
    args.geometry = "900x900"
    args.panels = [
        {"title": "Motor 1  (laptop-direct)", "spec": _spec_for(args.panel1_model), "default_id": args.panel1_id, "id_editable": True},
        {"title": "Motor 2  (chained)", "spec": _spec_for(args.panel2_model), "default_id": args.panel2_id, "id_editable": True},
    ]
    return args


def main():
    args = parse_args()
    root = tk.Tk()
    MotorRunApp(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
