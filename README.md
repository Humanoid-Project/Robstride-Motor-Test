# Robstride-Motor-Test

## Setup

```bash
git clone https://github.com/Humanoid-Project/Robstride-03-Test.git
cd Robstride-03-Test
python3 -m venv .venv
source .venv/bin/activate
pip install python-can
```

## CAN Interface

```bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
```

## Usage

### 1. motor_test

[![motor_test](https://img.shields.io/badge/docs-motor__test-blue)](motor_test/README.md)

GUI apps

```bash
cd motor_test/

python3 motor_run.py          # single motor
python3 daisy_chain_run.py    # two motors, daisy-chained
```

### 2. test

[![test](https://img.shields.io/badge/docs-test-green)](test/README.md)

Standalone CAN diagnostic scripts (link check, ID scan/set, probe)

```bash
cd test/

python3 link_check.py
python3 find_motor_id.py --scan
python3 probe_motors.py --ids 7,5
```
