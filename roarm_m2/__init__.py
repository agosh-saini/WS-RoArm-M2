"""
roarm_m2 — Pure Python serial driver for the Waveshare RoArm M2.

Exposes the main driver class so callers can do:
    from roarm_m2 import RoArmDriver
"""

from .serial_driver import RoArmDriver

__all__ = ["RoArmDriver"]
