"""AVATIC participant interface: camera in, stick commands out.

    from avatic_drone import Command, Drone

    drone = Drone()                        # the simulator (default)
    drone = Drone(backend='hardware')      # the real Pluto X (camera-module Wi-Fi)

The hardware backend is in the repository's hitl/ directory (loaded on
demand). Both backends have the same methods (get_frame, get_telemetry, arena,
send_command/send, wait_until_ready, arm, disarm, loop, sleep, time,
running, close), so a controller runs unchanged on either. The backend can
also be chosen with the environment variable AVATIC_BACKEND=sim|hardware
(and, for hardware, AVATIC_HOST, AVATIC_MSP_PORT, AVATIC_VIDEO).
"""

import os

from .types import ArenaStatus, Command, Frame, Telemetry

__all__ = ['Drone', 'Command', 'Frame', 'Telemetry', 'ArenaStatus', 'connect']

BACKENDS = ('sim', 'hardware')


def Drone(backend: str = None, **options):  # noqa: N802 - reads like a class
    """Connects to the simulator (backend='sim') or the real Pluto X
    (backend='hardware'; options: host, msp_port, video, time_limit_s).
    Default: $AVATIC_BACKEND, else 'sim'."""
    backend = backend or os.environ.get('AVATIC_BACKEND', 'sim')
    if backend == 'sim':
        from .sim import SimDrone  # needs ROS: imported only when used
        return SimDrone(**options)
    if backend == 'hardware':
        HardwareDrone = _load_hitl()
        for key, variable in (('host', 'AVATIC_HOST'), ('msp_port', 'AVATIC_MSP_PORT'),
                              ('video', 'AVATIC_VIDEO')):
            if key in options or variable not in os.environ:
                continue
            value = os.environ[variable]
            if key == 'msp_port':
                value = int(value)
            elif key == 'video' and value in ('', 'none'):
                value = None
            options[key] = value
        return HardwareDrone(**options)
    raise ValueError(f'backend must be one of {BACKENDS}, got {backend!r}')


def _load_hitl():
    """The hardware backend lives in <repository>/hitl (organisers' code)."""
    import sys
    from pathlib import Path
    hitl = Path(__file__).resolve().parents[2] / 'hitl'
    if not (hitl / 'avatic_hitl').is_dir():
        raise ImportError(f'hardware backend not found: expected {hitl}/avatic_hitl')
    if str(hitl) not in sys.path:
        sys.path.insert(0, str(hitl))
    from avatic_hitl.hardware import HardwareDrone
    return HardwareDrone


def connect():
    """Shared simulator connection for interactive use (see sim.connect)."""
    from .sim import connect as _connect
    return _connect()
