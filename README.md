# WS-RoArm-M2

Workspace for Waveshare RoArm-M2 robotic arm control.

## Setup

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Code Overview

This repository contains a robust Python driver and a command-line interface for communicating with the RoArm-M2 over a serial USB connection using newline-delimited JSON.

### `roarm_m2/serial_driver.py`
The `RoArmDriver` class provides a thread-safe UART wrapper specifically designed to handle the RoArm's firmware quirks:
- It actively filters out diagnostic "spam" (such as `Servo ID:15 status: failed`) to prevent crashes.
- It automatically drops the arm's JSON *command echoes* so it doesn't mistakenly accept them as the true response payload.
- It exposes safe Python methods for core commands like `set_joint(id, angle)`, `get_status()`, `home()`, and `stop()`.

### `roarm_m2/cli_test.py`
A simple executable script that instantiates the `RoArmDriver` and lets you send commands directly from your terminal. 

**Example Usage:**

Get the arm's global status coordinates and angles:
```bash
python roarm_m2/cli_test.py --port /dev/ttyUSB0 --action status
```

Send the arm safely to structural zero (Home):
```bash
python roarm_m2/cli_test.py --port /dev/ttyUSB0 --action home
```

Change the base LED color (e.g., to Blue):
```bash
python roarm_m2/cli_test.py --port /dev/ttyUSB0 --action color --rgb 0 0 255
```

## Joint Mapping Reference

When using the `--action joint` commands, the joint IDs map to the physical robot as follows:
- **Joint 1**: Base Pan (Rotate left and right)
- **Joint 2**: Shoulder / Base Tilt (Up and down tilt)
- **Joint 3**: Elbow
- **Joint 4**: Claw / Gripper
