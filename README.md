# Robstride-03 Motor Test

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

```bash
cd rs03_test/

# Communication test
python3 vbus_test.py

# Motor run
python3 motor_run.py
```
