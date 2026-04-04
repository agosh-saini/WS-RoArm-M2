"""
serial_driver.py — Thread-safe UART driver for the Waveshare RoArm M2.

The RoArm M2 communicates over UART using newline-delimited JSON frames.
Each command is a JSON object sent as a single line; the arm replies with
a JSON object on the next line.

Command reference (from Waveshare firmware docs):
  - Joint control : {"T": 101, "joint": <1-6>, "rad": <radians>, "spd": <speed>}
  - Get status    : {"T": 105}
  - Home position : {"T": 100}
  - Emergency stop: {"T": 0}

Usage:
    from roarm_m2 import RoArmDriver

    with RoArmDriver("/dev/ttyUSB0") as arm:
        arm.home()
        arm.set_joint(1, 90.0)
        print(arm.get_status())
"""

import json
import math
import threading
import time
from typing import Any

import serial


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Default serial settings expected by the RoArm M2 firmware.
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT = 2.0  # seconds to wait for a reply

# Waveshare firmware command type codes.
CMD_HOME = 100
CMD_JOINT = 101
CMD_STATUS = 105
CMD_LED = 230
CMD_STOP = 0


class RoArmDriver:
    """Thread-safe serial driver for the Waveshare RoArm M2.

    The driver opens a serial port, sends JSON commands, and returns the
    parsed JSON response.  All public methods acquire a lock so it is safe
    to call them from multiple threads (e.g. a background status poller and
    a foreground control loop).

    Args:
        port:    Serial device path, e.g. "/dev/ttyUSB0".
        baud:    Baud rate — must match firmware setting (default 115200).
        timeout: Seconds to wait for a reply before raising TimeoutError.
    """

    def __init__(self, port: str, baud: int = DEFAULT_BAUD, timeout: float = DEFAULT_TIMEOUT) -> None:
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._serial: serial.Serial | None = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Context-manager support  (with RoArmDriver(...) as arm:)
    # ------------------------------------------------------------------

    def __enter__(self) -> "RoArmDriver":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.disconnect()

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open the serial port.  Safe to call multiple times — a no-op if
        already connected."""
        with self._lock:
            if self._serial and self._serial.is_open:
                return
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self._timeout,
            )
            # Brief pause so the firmware's UART buffer settles after DTR
            # toggles (common on USB-serial adapters that reset the MCU).
            time.sleep(0.1)

    def disconnect(self) -> None:
        """Close the serial port gracefully."""
        with self._lock:
            if self._serial and self._serial.is_open:
                self._serial.close()

    @property
    def is_connected(self) -> bool:
        """True if the serial port is currently open."""
        return self._serial is not None and self._serial.is_open

    # ------------------------------------------------------------------
    # High-level command methods
    # ------------------------------------------------------------------

    def set_joint(self, joint_id: int, angle: float, speed: float = 50.0) -> dict[str, Any]:
        """Move a single joint to the requested angle.

        Args:
            joint_id: Joint number 1–6 (base = 1, gripper = 6).
            angle:    Target position in **degrees**.  Converted to radians
                      internally before transmission.
            speed:    Movement speed (0–100, firmware-defined units).

        Returns:
            Parsed JSON response from the arm.

        Raises:
            ValueError: If joint_id is outside the valid 1–6 range.
        """
        if not 1 <= joint_id <= 6:
            raise ValueError(f"joint_id must be 1–6, got {joint_id}")

        radians = math.radians(angle)
        command = {
            "T": CMD_JOINT,
            "joint": joint_id,
            "rad": round(radians, 4),
            "spd": speed,
        }
        return self._send(command)

    def get_status(self) -> dict[str, Any]:
        """Request the current arm state (joint positions, temps, etc.).

        Returns:
            Parsed JSON response containing current arm state.
        """
        return self._send({"T": CMD_STATUS})

    def home(self) -> dict[str, Any]:
        """Send all joints to their home/zero position.

        Returns:
            Parsed JSON response confirming the motion was queued.
        """
        return self._send({"T": CMD_HOME})

    def stop(self) -> dict[str, Any]:
        """Immediate emergency stop — halts all joint motion.

        Returns:
            Parsed JSON response confirming the stop was received.
        """
        return self._send({"T": CMD_STOP})

    def set_led(self, intensity: int) -> dict[str, Any]:
        """Set the RGB LED intensity on the RoArm base.

        Args:
            intensity: LED intensity value (0-255)

        Returns:
            Parsed JSON response confirming the LED change.
        """
        intensity = max(0, min(255, int(intensity)))
        return self._send({"T": CMD_LED, "intensity": intensity})

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _send(self, command: dict[str, Any]) -> dict[str, Any]:
        """Serialize *command* to JSON, write it to the serial port, and
        return the parsed response.

        The protocol frame is:
            TX: <json>\n
            RX: <json>\n

        Args:
            command: Python dict to serialize and transmit.

        Returns:
            Parsed response dict.

        Raises:
            RuntimeError:  If not connected.
            TimeoutError:  If no valid JSON response arrives within self._timeout.
        """
        if not self.is_connected:
            raise RuntimeError("Not connected — call connect() first.")

        payload = json.dumps(command, separators=(",", ":")) + "\n"

        with self._lock:
            # Flush stale bytes before each transaction.
            self._serial.reset_input_buffer()
            self._serial.write(payload.encode("utf-8"))

            end_time = time.time() + self._timeout

            while time.time() < end_time:
                raw = self._serial.readline()
                if not raw:
                    # Timeout reached (readline returned empty bytes)
                    break

                decoded = raw.decode("utf-8", errors="replace").strip()
                if not decoded:
                    continue

                try:
                    data = json.loads(decoded)
                    if isinstance(data, dict):
                        if data == command:
                            # The arm echoes commands. For T:105 (status), it follows the echo with a T:1051 response. 
                            # For motion commands like T:101, the echo IS the only response.
                            if command.get("T") == 105:
                                continue
                        return data
                except json.JSONDecodeError:
                    # Print and ignore non-JSON diagnostic messages like "Servo ID:14 status: failed."
                    print(f"Arm returned non-JSON data: {raw!r}")

        raise TimeoutError(
            f"No valid JSON response from arm within {self._timeout}s "
            f"(command T={command.get('T')})"
        )
