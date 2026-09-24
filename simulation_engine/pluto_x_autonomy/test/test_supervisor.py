from pluto_x_autonomy.api import OuterLoopController, StickCommand
from pluto_x_autonomy.examples.waypoint import WaypointController
from pluto_x_autonomy.supervisor import Phase, Supervisor, SupervisorParams
from sim_helpers import PointMass

DT = 0.02


class Climb(OuterLoopController):
    """Climbs for a while, then reports done (or raises, if asked)."""

    def __init__(self, params):
        super().__init__(params)
        self.t = 0.0
        self.resets = 0

    def reset(self, obs, status):
        self.resets += 1

    def update(self, obs, status, dt):
        self.t += dt
        if self.params.get('raise_at') and self.t > self.params['raise_at']:
            raise RuntimeError('participant bug')
        return StickCommand(throttle=0.75 if self.t < 2.0 else 0.7)

    def done(self):
        return self.t > 4.0


def run(supervisor, vehicle, seconds, status_kwargs=None, obs_until=None):
    phases = []
    t = 0.0
    for _ in range(int(seconds / DT)):
        t += DT
        obs = vehicle.observation(t) if obs_until is None or t < obs_until \
            else None
        command, arm = supervisor.step(t, obs,
                                       vehicle.status(t, **(status_kwargs or {})))
        vehicle.step(command, arm, DT)
        if not phases or phases[-1] != supervisor.phase:
            phases.append(supervisor.phase)
        if supervisor.finished:
            break
    return phases


def test_full_flight_sequence():
    vehicle = PointMass()
    controller = Climb({})
    supervisor = Supervisor(controller)
    phases = run(supervisor, vehicle, 60.0)
    # (WAIT_LINK is left on the first step: status is available at once)
    assert phases == [Phase.WAIT_READY, Phase.ARMING,
                      Phase.FLYING, Phase.LANDING, Phase.DISARMED]
    assert controller.resets == 1
    assert vehicle.p[2] < 0.06 and not vehicle.armed
    assert abs(supervisor.hover_throttle_estimate - 0.72) < 0.05


def test_waits_for_calibration():
    vehicle = PointMass()
    supervisor = Supervisor(Climb({}), SupervisorParams(ready_timeout_s=1.0))
    phases = run(supervisor, vehicle, 5.0, status_kwargs={'ok_to_arm': False})
    assert phases[-1] == Phase.ABORTED
    assert 'ok_to_arm' in supervisor.reason
    assert not vehicle.armed


def test_controller_exception_triggers_failsafe():
    vehicle = PointMass()
    supervisor = Supervisor(Climb({'raise_at': 1.5}))
    phases = run(supervisor, vehicle, 60.0)
    assert Phase.FAILSAFE in phases and phases[-1] == Phase.DISARMED
    assert 'participant bug' in supervisor.reason


def test_stale_observation_triggers_failsafe():
    vehicle = PointMass()
    supervisor = Supervisor(Climb({}))
    phases = run(supervisor, vehicle, 60.0, obs_until=1.5)
    assert Phase.FAILSAFE in phases
    assert 'observation' in supervisor.reason


def test_geofence():
    vehicle = PointMass()
    supervisor = Supervisor(Climb({}), SupervisorParams(max_altitude_m=0.3))
    phases = run(supervisor, vehicle, 60.0)
    assert Phase.FAILSAFE in phases and 'geofence' in supervisor.reason


def test_waypoint_mission_on_point_mass():
    vehicle = PointMass()
    controller = WaypointController({
        'waypoints_enu_m': [[0, 0, 1], [1, 0, 1], [1, 1, 1.5], [0, 0, 1]],
        'hover_throttle': 0.65})
    supervisor = Supervisor(controller)
    phases = run(supervisor, vehicle, 120.0)
    assert phases[-1] == Phase.DISARMED, supervisor.reason
    assert Phase.LANDING in phases and Phase.FAILSAFE not in phases
    assert controller.done()


def test_heading_scan_on_point_mass():
    import math
    vehicle = PointMass()
    yaw0 = vehicle.yaw
    controller = WaypointController({
        'waypoints_enu_m': [[0, 0, 1, 0], [0, 0, 1, 180], [0, 0, 1, 360],
                            [0, 0, 1, -90]],
        'hover_throttle': 0.65, 'hold_time_s': 0.5})
    supervisor = Supervisor(controller)
    headings = []
    t = 0.0
    for _ in range(int(120 / DT)):
        t += DT
        command, arm = supervisor.step(t, vehicle.observation(t), vehicle.status(t))
        vehicle.step(command, arm, DT)
        headings.append(controller.yaw_unwrapped - yaw0)
        if supervisor.finished:
            break
    assert supervisor.phase == Phase.DISARMED, supervisor.reason
    assert max(headings) > math.radians(355)      # went a full turn left
    assert min(headings) < math.radians(-85)      # and back past the start


def test_landing_holds_position_against_drift():
    import numpy as np
    for hold, max_drift in ((True, 0.3), (False, None)):
        vehicle = PointMass()
        supervisor = Supervisor(Climb({}), SupervisorParams(land_hold_position=hold))
        t, start = 0.0, None
        for _ in range(int(60 / DT)):
            t += DT
            command, arm = supervisor.step(t, vehicle.observation(t), vehicle.status(t))
            if supervisor.phase == Phase.LANDING and start is None:
                start = vehicle.p[:2].copy()
                vehicle.v[:2] = [0.6, -0.4]  # gust-like sideways velocity at landing start
            vehicle.step(command, arm, DT)
            if supervisor.finished:
                break
        drift = float(np.linalg.norm(vehicle.p[:2] - start))
        assert supervisor.phase == Phase.DISARMED
        if hold:
            assert drift < max_drift, drift
        else:
            unheld = drift
    assert unheld > 1.0  # without the hold the drift is carried to the ground
