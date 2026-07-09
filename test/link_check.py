#!/usr/bin/env python3
import argparse
import re
import subprocess
import sys


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True)


def link_details(iface):
    result = run(["ip", "-details", "-statistics", "link", "show", iface])
    if result.returncode != 0:
        print(f"'{iface}' not found. stderr:\n{result.stderr}")
        sys.exit(1)
    return result.stdout


def parse_state(text):
    state_match = re.search(r"state\s+(\S+)", text)
    bitrate_match = re.search(r"bitrate\s+(\d+)", text)
    errors_match = re.search(
        r"RX:\s*bytes\s+packets\s+errors\s+dropped[^\n]*\n\s*\d+\s+\d+\s+(\d+)\s+(\d+)",
        text,
    )
    berr_match = re.search(
        r"re-started\s+bus-errors\s+arbit-lost\s+error-warn\s+error-pass\s+bus-off\s*\n\s*(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)",
        text,
    )
    return {
        "state": state_match.group(1) if state_match else "unknown",
        "bitrate": bitrate_match.group(1) if bitrate_match else "unknown",
        "rx_errors": errors_match.group(1) if errors_match else None,
        "rx_dropped": errors_match.group(2) if errors_match else None,
        "can_stats": berr_match.groups() if berr_match else None,
    }


def main():
    parser = argparse.ArgumentParser(description="Check a SocketCAN interface's link state and error counters.")
    parser.add_argument("--iface", default="can0")
    args = parser.parse_args()

    text = link_details(args.iface)
    print(text)

    info = parse_state(text)
    print("---- parsed summary ----")
    print(f"state    : {info['state']}")
    print(f"bitrate  : {info['bitrate']}")
    if info["can_stats"] is not None:
        restarts, bus_errors, arbit_lost, error_warn, error_pass, bus_off = info["can_stats"]
        print(f"restarts : {restarts}")
        print(f"bus-errors: {bus_errors}")
        print(f"arbit-lost: {arbit_lost}")
        print(f"error-warn: {error_warn}")
        print(f"error-pass: {error_pass}")
        print(f"bus-off  : {bus_off}")
        if bus_errors not in ("0", None) or bus_off not in ("0", None):
            print("WARNING: non-zero bus-errors/bus-off. Possible termination, wiring, or bitrate mismatch.")
    else:
        print("could not find can-specific error counters in 'ip -details' output; kernel/iproute2 version may format differently.")

    if info["state"] != "UP":
        print(f"\nInterface is not UP. Bring it up with:\n  sudo ip link set {args.iface} up type can bitrate 1000000")


if __name__ == "__main__":
    main()
