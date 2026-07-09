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
