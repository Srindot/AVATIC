"""AVATIC participant interface: camera in, stick commands out.

    from avatic_drone import Drone
"""
from .drone import ArenaStatus, Drone, Frame, Telemetry

__all__ = ['Drone', 'Frame', 'Telemetry', 'ArenaStatus']
