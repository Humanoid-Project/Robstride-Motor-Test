# Communication Test Scripts

## link_check.py

can0 interface state, bitrate, error counters.

```bash
python3 link_check.py --iface can0
```

- `--iface`: interface name (default: can0)

## probe_motors.py

Send Type0/Type17 requests to specific motor IDs, print raw arb IDs and decoded reply.

```bash
python3 probe_motors.py --channel can0 --ids 7,5
```

- `--channel`: CAN channel (default: can0)
- `--interface`: python-can interface (default: socketcan)
- `--host-id`: host CAN id (default: 0xFD)
- `--ids`: comma-separated motor ids to probe (default: 7,5)
- `--timeout`: wait time per request in seconds (default: 0.3)

## find_motor_id.py

Find which CAN id a motor is currently answering on.

```bash
python3 find_motor_id.py --channel can0 --scan --scan-max 20
python3 find_motor_id.py --channel can0 --check-id 5
```

- `--channel`, `--interface`, `--host-id`: same as above
- `--scan`: scan a range of ids and report responders
- `--scan-max`: highest id to scan with --scan (default: 127)
- `--check-id`: check whether one specific id answers

## set_motor_id.py

Change a motor's CAN id (permanent).

```bash
python3 set_motor_id.py --channel can0 --current-id 5 --new-id 6
```

- `--channel`, `--interface`, `--host-id`: same as above
- `--current-id`: motor's current id (required)
- `--new-id`: new id to assign (required)
- `--yes`: skip the confirmation prompt

## set_motor_id_gui.py

Same as `set_motor_id.py` but as a small GUI: bus status, current ID (Scan +
Check), new ID, and a Change button. Verifies the current ID responds before
changing, optionally checks the new ID for a collision, then verifies the new
ID after.

Two ways to learn the current ID without typing it:

- **스캔** -- sends all Type 0 (Get device ID) requests first and listens once,
  so probing 1..127 takes ~1s instead of the ~25s a per-ID wait needs. Falls
  back to a Type 17 pass for firmware that ignores Type 0.
- **전원 켜서 찾기** -- listens passively for 8s while you power the motor on. A
  Robstride motor emits two frames carrying its own ID at power-up, so this
  works even when the motor answers no query, and it also catches motors
  switched to the MIT protocol (standard frames, ID in byte 0).

Both are read-only. The only persistent write is Type 7, behind a confirm dialog.

```bash
python3 set_motor_id_gui.py
```

- `--channel`, `--interface`, `--host-id`: same as above
- `--scan-max`: highest id probed by Scan (default: 127)

Bring the interface up first -- SocketCAN opens a DOWN interface without error
and only fails on the first send:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

Note: only one process can own the CAN interface at a time -- close the
`motor_run` GUI before using this tool.
