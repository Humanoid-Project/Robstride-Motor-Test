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

## Motor Test

### 1. motor_run

```bash
cd scripts/motor_run/

python3 motor_run.py
python3 daisy_chain_run.py
```

### 2. motor_id

```bash
cd scripts/motor_id/

python3 find_motor_id.py --scan
python3 find_motor_id.py --check-id {ID}
python3 set_motor_id.py --current-id {ID} --new-id {NEW_ID}
python3 set_motor_id_gui.py
```

### 3. calibration

```bash
cd scripts/calibration/

python3 motor_calibration.py --motor-id {ID}
```

### 4. zero_position

```bash
cd scripts/zero_position/

python3 set_zero_position.py --motor-id {ID}
python3 set_zero_position.py --motor-id {ID} --verify
python3 set_zero_position.py --motor-id {ID} --zero-sta 1
```