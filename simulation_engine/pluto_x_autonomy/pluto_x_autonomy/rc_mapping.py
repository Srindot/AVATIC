"""StickCommand <-> RC channels of the Pluto X (MagisV2, Rx_ESP config).

Channel order AETR1234 (roll, pitch, throttle, yaw, AUX1..AUX4), values in
microseconds. The mode ranges below are the firmware's defaults for the
Pluto X receiver setup (MagisV2 config: ARM = AUX4 1300..2100; ANGLE always
on; BARO = AUX3 1300..2100; MAG = AUX1 900..1300; HEADFREE = AUX1
1300..1700; developer mode = AUX2 1450..1550).
"""

from __future__ import annotations

from dataclasses import dataclass

from .api import StickCommand

CENTRE_US = 1500
HALF_RANGE_US = 500
THROTTLE_MIN_US = 1000
THROTTLE_RANGE_US = 1000

ARM_ON_US = 1500      # AUX4 inside 1300..2100
ARM_OFF_US = 1000
BARO_ON_US = 1500     # AUX3 inside 1300..2100
BARO_OFF_US = 1000
AUX1_NEUTRAL_US = 2000  # outside the MAG and HEADFREE ranges
AUX2_NEUTRAL_US = 1000  # outside developer mode


@dataclass(frozen=True)
class RcChannels:
    roll_us: int
    pitch_us: int
    throttle_us: int
    yaw_us: int
    aux1_us: int
    aux2_us: int
    aux3_us: int
    aux4_us: int

    def as_list(self) -> list:
        return [self.roll_us, self.pitch_us, self.throttle_us, self.yaw_us,
                self.aux1_us, self.aux2_us, self.aux3_us, self.aux4_us]


def to_rc(command: StickCommand, arm: bool) -> RcChannels:
    """Maps a (clipped) stick command to RC channels."""
    c = command.clipped()
    return RcChannels(
        roll_us=round(CENTRE_US + HALF_RANGE_US * c.roll),
        pitch_us=round(CENTRE_US + HALF_RANGE_US * c.pitch),
        throttle_us=round(THROTTLE_MIN_US + THROTTLE_RANGE_US * c.throttle),
        yaw_us=round(CENTRE_US + HALF_RANGE_US * c.yaw),
        aux1_us=AUX1_NEUTRAL_US,
        aux2_us=AUX2_NEUTRAL_US,
        aux3_us=BARO_ON_US if c.altitude_hold else BARO_OFF_US,
        aux4_us=ARM_ON_US if arm else ARM_OFF_US)


IDLE = StickCommand()  # sticks centred, throttle minimum
