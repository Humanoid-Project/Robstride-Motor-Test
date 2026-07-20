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

### 1. motor_run

[![motor_run](https://img.shields.io/badge/docs-motor__run-blue)](motor_run/README.md)

GUI apps

```bash
cd motor_run/

python3 motor_run.py          # single motor
python3 daisy_chain_run.py    # two motors, daisy-chained
```

### 2. test

[![test](https://img.shields.io/badge/docs-test-green)](test/README.md)