"""Safety caps (both backends) and the hardware backend's failsafe rule."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))                               # avatic_hitl
sys.path.insert(0, os.path.join(HERE, '..', '..', 'outerloop_controller'))  # avatic_drone.types
from avatic_drone.types import SAFETY_LIMITS, Command, apply_safety_limits  # noqa: E402
from avatic_hitl.hardware import HardwareDrone  # noqa: E402


def test_commands_inside_the_limits_pass_unchanged():
    command = Command(roll=0.3, pitch=-0.5, yaw_rate=0.8, throttle=0.76, altitude_hold=True)
    safe, capped = apply_safety_limits(command)
    assert safe == command and capped == []


def test_each_axis_is_clipped_and_reported():
    safe, capped = apply_safety_limits(Command(roll=-1.0, pitch=0.9, yaw_rate=-1.0, throttle=1.0))
    assert safe.roll == -SAFETY_LIMITS.max_tilt_stick
    assert safe.pitch == SAFETY_LIMITS.max_tilt_stick
    assert safe.yaw_rate == -SAFETY_LIMITS.max_yaw_rate_stick
    assert safe.throttle == SAFETY_LIMITS.max_throttle
    assert capped == ['throttle', 'tilt', 'yaw rate']


def test_negative_throttle_is_clipped_to_zero():
    safe, capped = apply_safety_limits(Command(throttle=-0.2))
    assert safe.throttle == 0.0 and capped == ['throttle']


def _drone(hover=0.78):
    drone = HardwareDrone.__new__(HardwareDrone)   # no connection: only the rule is tested
    drone._hover_estimate, drone._timed_out = hover, False
    return drone


def test_failsafe_holds_hover_when_flying():
    safe = _drone()._failsafe_command(Command(roll=0.4, yaw_rate=0.5, throttle=0.8))
    assert (safe.roll, safe.pitch, safe.yaw_rate) == (0.0, 0.0, 0.0)
    assert safe.throttle == 0.78 and not safe.altitude_hold


def test_failsafe_never_lifts_a_grounded_drone():
    assert _drone()._failsafe_command(Command(throttle=0.0)).throttle == 0.0
    assert _drone()._failsafe_command(Command(throttle=0.2)).throttle == 0.0


def test_failsafe_keeps_altitude_hold():
    safe = _drone()._failsafe_command(Command(throttle=0.7, altitude_hold=True))
    assert safe.altitude_hold and safe.throttle == 0.5


def test_failsafe_hover_is_limited_to_a_sane_range():
    assert _drone(hover=0.95)._failsafe_command(Command(throttle=0.9)).throttle == 0.85
    assert _drone(hover=0.40)._failsafe_command(Command(throttle=0.5)).throttle == 0.6


def test_stick_mapping_matches_the_simulator():
    """hardware.py copies pluto_x_autonomy's rc_mapping (to need no ROS):
    both must give the same RC microseconds."""
    import itertools
    sys.path.insert(0, os.path.join(HERE, '..', '..', 'simulation_engine', 'pluto_x_autonomy'))
    from pluto_x_autonomy.api import StickCommand
    from pluto_x_autonomy.rc_mapping import to_rc
    from avatic_hitl.hardware import _to_channels
    values = (-1.0, -0.37, 0.0, 0.2, 1.0)
    for roll, pitch, yaw, throttle, hold, arm in itertools.product(
            values, values[:3], values[2:], (0.0, 0.5, 0.76, 1.0), (False, True), (False, True)):
        sim = to_rc(StickCommand(roll=roll, pitch=pitch, yaw=yaw, throttle=throttle,
                                 altitude_hold=hold), arm).as_list()
        hw = _to_channels(Command(roll=roll, pitch=pitch, yaw_rate=yaw, throttle=throttle,
                                  altitude_hold=hold), arm)
        assert list(sim) == hw, (roll, pitch, yaw, throttle, hold, arm)
