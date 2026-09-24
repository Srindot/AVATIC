import math

import pytest

from pluto_x_autonomy.api import StickCommand
from pluto_x_autonomy.rc_mapping import to_rc


def test_idle_disarmed():
    rc = to_rc(StickCommand(), arm=False)
    assert rc.as_list() == [1500, 1500, 1000, 1500, 2000, 1000, 1000, 1000]


def test_full_deflection_and_modes():
    rc = to_rc(StickCommand(roll=1, pitch=-1, yaw=0.2, throttle=1,
                            altitude_hold=True), arm=True)
    assert (rc.roll_us, rc.pitch_us, rc.yaw_us, rc.throttle_us) == \
        (2000, 1000, 1600, 2000)
    assert rc.aux3_us == 1500 and rc.aux4_us == 1500


def test_clipping_and_nan():
    rc = to_rc(StickCommand(roll=3, throttle=-1), arm=True)
    assert rc.roll_us == 2000 and rc.throttle_us == 1000
    with pytest.raises(ValueError):
        to_rc(StickCommand(pitch=math.nan), arm=True)
