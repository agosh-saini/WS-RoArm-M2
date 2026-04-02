"""
cli_test.py — Command-line interface for testing the RoArm M2 driver.

Run directly without installing the package:
    python cli_test.py --port /dev/ttyUSB0 --joint 1 --angle 90

The script always:
  1. Connects to the arm.
  2. Runs the requested action (joint move, status query, home, or stop).
  3. Prints the arm's JSON response to stdout.
  4. Disconnects cleanly.

Exit codes:
  0 — success
  1 — connection or communication error
  2 — invalid arguments (handled automatically by argparse)
"""

import argparse
import json
import sys

# Resolve the package whether the script is run as a standalone file inside
# the package directory or via `python -m roarm_m2.cli_test`.
try:
    from roarm_m2.serial_driver import RoArmDriver
except ModuleNotFoundError:
    # Running as a plain script from inside the package directory.
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
    from roarm_m2.serial_driver import RoArmDriver


# ---------------------------------------------------------------------------
# CLI argument definition
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Test the Waveshare RoArm M2 over UART.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Move joint 1 to 90°:
    python cli_test.py --port /dev/ttyUSB0 --joint 1 --angle 90

  Move joint 3 to -45° at speed 30:
    python cli_test.py --port /dev/ttyUSB0 --joint 3 --angle -45 --speed 30

  Query arm status:
    python cli_test.py --port /dev/ttyUSB0 --action status

  Send all joints home:
    python cli_test.py --port /dev/ttyUSB0 --action home

  Emergency stop:
    python cli_test.py --port /dev/ttyUSB0 --action stop
        """,
    )

    parser.add_argument(
        "--port",
        required=True,
        help="Serial device path, e.g. /dev/ttyUSB0",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Serial read timeout in seconds (default: 2.0)",
    )
    parser.add_argument(
        "--action",
        choices=["joint", "status", "home", "stop"],
        default="joint",
        help="Action to perform (default: joint)",
    )
    parser.add_argument(
        "--joint",
        type=int,
        choices=range(1, 7),
        metavar="1-6",
        help="Joint ID to move (required when --action joint)",
    )
    parser.add_argument(
        "--angle",
        type=float,
        help="Target angle in degrees (required when --action joint)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=50.0,
        help="Movement speed 0–100 (default: 50, used with --action joint)",
    )

    return parser


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    # Validate joint-move arguments upfront for a clear error message.
    if args.action == "joint":
        if args.joint is None or args.angle is None:
            parser.error("--joint and --angle are required when --action is 'joint'.")

    try:
        with RoArmDriver(args.port, baud=args.baud, timeout=args.timeout) as arm:
            if args.action == "joint":
                print(f"Moving joint {args.joint} to {args.angle}° at speed {args.speed}…")
                response = arm.set_joint(args.joint, args.angle, args.speed)

            elif args.action == "status":
                print("Requesting arm status…")
                response = arm.get_status()

            elif args.action == "home":
                print("Sending arm to home position…")
                response = arm.home()

            elif args.action == "stop":
                print("Sending emergency stop…")
                response = arm.stop()

        # Pretty-print so the response is easy to read in a terminal.
        print("Response:", json.dumps(response, indent=2))

    except (OSError, serial.SerialException) as exc:
        # Serial port errors: wrong path, permissions, device unplugged, etc.
        print(f"Serial error: {exc}", file=sys.stderr)
        sys.exit(1)

    except TimeoutError as exc:
        print(f"Timeout: {exc}", file=sys.stderr)
        sys.exit(1)

    except ValueError as exc:
        # Bad JSON back from the arm, or invalid joint_id.
        print(f"Protocol error: {exc}", file=sys.stderr)
        sys.exit(1)


# Make `import serial` available even when run as a plain script.
import serial  # noqa: E402 — placed here so the try/except above still works

if __name__ == "__main__":
    main()
