"""AVATIC hardware-in-the-loop: the real Pluto X behind the participant
Drone interface (organisers' code; participants use avatic_drone and
`my_controller.py --hardware`).

  hardware.py  HardwareDrone - the same methods as the simulator's Drone
  msp.py       MSP v1 client (commands + telemetry) for the MagisV2 firmware
  video.py     camera module stream (plutocam H.264 -> ffmpeg -> NumPy)
"""
from .hardware import HardwareDrone

__all__ = ['HardwareDrone']
