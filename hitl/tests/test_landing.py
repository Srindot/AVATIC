"""HardwareDrone.land() against a simple vertical model of the drone, with a
fake clock: land() runs synchronously, in milliseconds of real time.

Model: thrust = weight x (throttle / true hover)^2 while armed, ground at
z = 0. The barometer reads z (plus optional noise). The landing must reach
the ground slowly and disarm there, whatever the hover estimate was."""
import os
import random
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'outerloop_controller'))
from avatic_drone.types import Command, Telemetry  # noqa: E402
from avatic_hitl import hardware  # noqa: E402

G = 9.81


class World:
    """Fake time + the vertical model; the drone's time.sleep() steps it."""

    def __init__(self, drone, z, true_hover, baro_noise_m=0.0, seed=1):
        self.drone, self.z, self.vz, self.true_hover = drone, z, 0.0, true_hover
        self.now, self.noise, self.rng = 0.0, baro_noise_m, random.Random(seed)
        self.max_descent = self.max_climb = self.touchdown_speed = 0.0
        self.max_z = z

    # time module stand-in
    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        steps = max(int(seconds / 0.005), 1)
        for _ in range(steps):
            self._step(seconds / steps)

    def _step(self, dt):
        d = self.drone
        armed = d._arm_switch and not d._estop
        thrust = G * (d._command.throttle / self.true_hover) ** 2 if armed else 0.0
        self.vz += (thrust - G) * dt
        self.z += self.vz * dt
        if self.z <= 0.0:
            if self.vz < -0.01:
                self.touchdown_speed = max(self.touchdown_speed, -self.vz)
            self.z, self.vz = 0.0, max(self.vz, 0.0)
        self.max_descent = max(self.max_descent, -self.vz)
        self.max_climb = max(self.max_climb, self.vz)
        self.max_z = max(self.max_z, self.z)
        self.now += dt

    def telemetry(self):
        return Telemetry(time_s=self.now, armed=self.drone._arm_switch and not self.drone._estop,
                         ready_to_arm=False, roll_deg=0.0, pitch_deg=0.0, heading_deg=0.0,
                         altitude_m=self.z + self.rng.gauss(0.0, self.noise),
                         battery_v=3.9)


def make_drone(monkeypatch, z, hover_estimate, true_hover, last_throttle, noise=0.0):
    drone = hardware.HardwareDrone.__new__(hardware.HardwareDrone)   # no connection
    drone._lock = hardware.threading.RLock()
    drone._land_lock = hardware.threading.Lock()
    drone._command = Command(throttle=last_throttle)
    drone._arm_switch, drone._estop, drone._landing = True, False, False
    drone._link_dead, drone._sigint_count = False, 0
    drone._hover_estimate = hover_estimate
    drone._link = types.SimpleNamespace(closed=False)
    world = World(drone, z, true_hover, noise)
    drone.get_telemetry = world.telemetry
    drone.telemetry_age_s = lambda: 0.0
    monkeypatch.setattr(hardware, 'time', types.SimpleNamespace(
        monotonic=world.monotonic, sleep=world.sleep))
    monkeypatch.setattr('builtins.print', lambda *a, **k: None)
    return drone, world


@pytest.mark.parametrize('hover_estimate,true_hover', [
    (0.76, 0.76),   # estimate right
    (0.85, 0.72),   # estimate far too high (a climbing controller; a fresh battery)
    (0.60, 0.80),   # estimate far too low (learned while idling)
])
def test_lands_slowly_and_disarms_on_the_ground(monkeypatch, hover_estimate, true_hover):
    drone, world = make_drone(monkeypatch, z=1.5, hover_estimate=hover_estimate,
                              true_hover=true_hover, last_throttle=true_hover, noise=0.02)
    drone.land()
    assert not drone._arm_switch                        # disarmed ...
    assert world.z == 0.0                               # ... on the ground
    assert world.now < hardware.LAND_HARD_LIMIT_S       # not by the 30 s limit
    assert world.max_z < 1.5 + 0.5                      # never climbs away
    assert world.touchdown_speed < 0.6, world.touchdown_speed


def test_no_hop_when_landing_starts_on_the_pad(monkeypatch):
    drone, world = make_drone(monkeypatch, z=0.0, hover_estimate=0.85, true_hover=0.76,
                              last_throttle=0.0)
    drone.land()
    assert not drone._arm_switch
    assert world.max_z == 0.0 and world.now < 0.5


def test_emergency_stop_ends_the_landing(monkeypatch):
    drone, world = make_drone(monkeypatch, z=1.0, hover_estimate=0.76, true_hover=0.76,
                              last_throttle=0.76)
    original_step = world._step

    def step(dt):
        if world.now > 1.0:
            drone._estop = True                 # what a Ctrl-C while landing does
        original_step(dt)
    world._step = step
    drone.land()
    assert not drone._arm_switch and world.now < 2.0


def test_link_loss_ends_the_landing(monkeypatch):
    drone, world = make_drone(monkeypatch, z=1.0, hover_estimate=0.76, true_hover=0.76,
                              last_throttle=0.76)
    drone._link_dead = True
    drone.land()
    assert world.now < 0.5
