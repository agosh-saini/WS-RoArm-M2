# Learning ROS — Step 1: The CLI Layer and Moving the RoArm M2

---

## Why Does the CLI Layer Exist?

When you build a robot driver, you end up with two distinct concerns:

```
[ Hardware ]  <-->  [ serial_driver.py ]  <-->  [ your code / ROS / CLI ]
```

The `serial_driver.py` is a **library** — it is a tool for other code to use.
It has no opinion about *when* to move a joint, *what* angle to move to, or
*how* to display results. It just knows how to talk to the arm over UART.

Without a CLI layer, the only way to test whether the driver works is to write
a Python script from scratch every single time. That is slow and error-prone
during development.

### The CLI layer (`cli_test.py`) solves three problems:

---

### 1. Manual Testing Without Writing Code

During hardware bring-up, you almost always need to answer questions like:

- "Is the arm actually connected?"
- "Does joint 2 respond to commands?"
- "What does the status payload look like?"

With the CLI layer you can answer all of these directly from the terminal,
**without touching Python**:

```bash
python cli_test.py --port /dev/ttyUSB0 --action status
python cli_test.py --port /dev/ttyUSB0 --joint 2 --angle 45
```

This is equivalent to how you might use `curl` to test an HTTP API before
writing a client for it.

---

### 2. A Clean Boundary Between Layers

The CLI layer forces a healthy separation:

| Layer             | Responsibility                                      |
|-------------------|-----------------------------------------------------|
| `serial_driver.py`| Knows HOW to talk to the arm (bytes, JSON, UART)    |
| `cli_test.py`     | Knows WHAT the user asked for (args, pretty output) |
| ROS (later)       | Knows WHEN and WHY to move (topics, planning, TF)   |

This separation is the foundation ROS is built on. ROS nodes are essentially
CLI-like processes that send and receive structured messages — the same idea,
just over a publish/subscribe bus instead of a terminal.

When you eventually wrap this driver in a ROS node, you will only need to
replace the CLI argument parsing with ROS topic subscriptions. The driver
itself does not change at all.

---

### 3. Debugging and Introspection

The CLI prints the raw JSON response from the arm:

```json
Response: {
  "T": 105,
  "joint1": 0.0,
  "joint2": 1.5708,
  "joint3": 0.0,
  "joint4": 0.0,
  "joint5": 0.0,
  "joint6": 0.0
}
```

This lets you verify that the firmware is behaving as expected *before* you
build higher-level logic on top of it. In robotics, hardware surprises are
common — a joint might be wired in reverse, or the firmware might use
different units than the docs describe. The CLI layer surfaces these issues
immediately.

---

## Deep Dive: How the Serial Driver Actually Works

The driver (`serial_driver.py`) is structured to handle all the messiness of the hardware communication so the CLI or ROS node doesn't have to. Here's a breakdown of the key mechanisms:

### 1. Thread Safety (`self._lock`)
In robotics, you often have one thread reading sensor data constantly while another sporadically sends movement commands. Without a lock, both might try to talk to the serial port simultaneously, mangling the bytes. The driver uses Python's `threading.Lock()` to guarantee only one transaction occurs at a time.

### 2. The Command/Response Loop
The RoArm communicates via newline-delimited JSON. When `_send()` is called, it:
- Formats a Python dictionary (like `{"T": 101, "joint": 1, ...}`) into a strict JSON string and appends a newline `\n`.
- Flushes stale data from the serial buffer using `reset_input_buffer()`.
- Writes the JSON payload.
- Computes a safe timeout window and begins reading.

### 3. Handling Firmware Spam and Command Echoes
This is where the magic happens. The firmware often throws raw text strings over UART (like `Servo ID:15 status: failed.\r\n`). Furthermore, it systematically *echoes* exactly what we send it before answering. 

The driver's `while` loop aggressively filters out this noise:
- It reads a line and passes it to `json.loads()`.
- If `loads()` fails (a `JSONDecodeError`), we know it's a diagnostic string. We log it and continue.
- If `loads()` succeeds, we check if the returned dictionary exactly matches the command we just sent. If it does (and we're expecting a dedicated answer, like with the `T: 105` status command), we discard the echo and read the next line. 

This robust parsing logic is the only reason the Python layer doesn't freeze or crash when the physical arm sends unexpected hardware errors.

---

## How to Move the RoArm M2

### Prerequisites

1. Connect the arm to your computer via USB-to-UART cable.
2. Identify the serial port:
   - Linux/Mac: `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`
   - Windows: check Device Manager for `COM` ports
3. Make sure you are inside the `roarm_m2/` package directory:
   ```bash
   cd roarm_m2/roarm_m2
   ```

---

### The Coordinate System

```
          Joint 6 (gripper)
              |
          Joint 5 (wrist pitch)
              |
          Joint 4 (wrist roll)
              |
          Joint 3 (elbow)
              |
          Joint 2 (shoulder)
              |
          Joint 1 (base rotation)
              |
           [base]
```

- **Angles are in degrees** on the command line.
- The driver converts them to radians before sending to the firmware.
- `0°` is the neutral/home position for each joint.
- Positive/negative direction depends on your specific arm wiring.

---

### Command Reference

#### Check the arm is alive — query its status

```bash
python cli_test.py --port /dev/ttyUSB0 --action status
```

Expected output:
```
Requesting arm status…
Response: {
  "T": 105,
  "joint1": 0.0,
  ...
}
```

---

#### Send all joints to the home position

```bash
python cli_test.py --port /dev/ttyUSB0 --action home
```

Do this first whenever you power on the arm — it gives you a known starting
state to work from.

---

#### Move a single joint

```bash
python cli_test.py --port /dev/ttyUSB0 --joint <ID> --angle <degrees>
```

| Flag      | Meaning                         | Valid range    |
|-----------|---------------------------------|----------------|
| `--joint` | Which joint to move             | 1–6            |
| `--angle` | Target position in degrees      | hardware-limited (~±180°) |
| `--speed` | How fast to move (optional)     | 0–100, default 50 |

Examples:

```bash
# Rotate the base 90° to the right
python cli_test.py --port /dev/ttyUSB0 --joint 1 --angle 90

# Raise the shoulder to 45°
python cli_test.py --port /dev/ttyUSB0 --joint 2 --angle 45

# Move the elbow to -30° (below neutral) slowly
python cli_test.py --port /dev/ttyUSB0 --joint 3 --angle -30 --speed 20

# Open/close the gripper
python cli_test.py --port /dev/ttyUSB0 --joint 6 --angle 30
```

---

#### Emergency stop

```bash
python cli_test.py --port /dev/ttyUSB0 --action stop
```

Halts all joint motion immediately. Use this if the arm is moving in an
unexpected direction — **always have this command ready in a second terminal**.

---

#### Control Base LEDs

```bash
python cli_test.py --port /dev/ttyUSB0 --action led --intensity 255
```

Easily change the lighting intensity (0-255) on the robotic arm directly via CLI.

---

### A Safe First-Move Sequence

If this is your first time, run these commands in order:

```bash
# 1. Verify connection
python cli_test.py --port /dev/ttyUSB0 --action status

# 2. Go to a known-safe starting position
python cli_test.py --port /dev/ttyUSB0 --action home

# 3. Rotate base slightly to confirm joint 1 works
python cli_test.py --port /dev/ttyUSB0 --joint 1 --angle 30 --speed 20

# 4. Return base to zero
python cli_test.py --port /dev/ttyUSB0 --joint 1 --angle 0 --speed 20
```

---

## What Comes Next (Step 2 Preview)

Once you are comfortable moving individual joints from the CLI, the natural
next step is **wrapping this driver in a ROS 2 node**.

A ROS node will:
- Subscribe to a `/joint_command` topic instead of reading CLI arguments.
- Publish to a `/joint_states` topic instead of printing to stdout.
- Let other ROS nodes (planners, cameras, GUIs) talk to the arm without
  knowing anything about serial ports or JSON.

The `serial_driver.py` you already have will be imported directly into that
node — no changes required.
