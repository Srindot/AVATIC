"""A crude point-mass stand-in for the vehicle + MagisV2 angle mode, for
testing outer-loop logic without Gazebo. NOT a vehicle model."""

import math

import numpy as np

from pluto_x_autonomy.api import FcStatus, Observation

G = 9.81
TILT_PER_STICK = math.radians(32.0)
HOVER_THROTTLE = 0.7


class PointMass:

    def __init__(self):
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.yaw = math.radians(90.0)  # facing north
        self.roll = self.pitch = 0.0
        self.armed = False

    def step(self, command, arm, dt):
        self.armed = arm
        if not arm:
            self.v[:] = 0.0
            return
        # attitude follows the stick setpoint with a 0.1 s lag
        alpha = min(dt / 0.1, 1.0)
        self.roll += alpha * (command.roll * TILT_PER_STICK - self.roll)
        self.pitch += alpha * (command.pitch * TILT_PER_STICK - self.pitch)
        self.yaw -= command.yaw * math.radians(90.0) * dt  # + stick = clockwise
        a_fwd = G * math.tan(self.pitch)      # nose down -> forward
        a_right = G * math.tan(self.roll)     # bank right -> right
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        a = np.array([cy * a_fwd + sy * a_right, sy * a_fwd - cy * a_right,
                      G * (command.throttle / HOVER_THROTTLE - 1.0)])
        self.v += (a - 0.3 * self.v) * dt
        self.p += self.v * dt
        if self.p[2] < 0.0:
            self.p[2] = 0.0
            self.v[2] = max(self.v[2], 0.0)

    def observation(self, t):
        return Observation(time_s=t, position_enu_m=self.p.copy(),
                           velocity_enu_m_s=self.v.copy(), roll_rad=self.roll,
                           pitch_rad=self.pitch, yaw_rad=self.yaw,
                           angular_velocity_flu_rad_s=np.zeros(3))

    def status(self, t, ok_to_arm=True, healthy=True):
        return FcStatus(time_s=t, armed=self.armed, ok_to_arm=ok_to_arm,
                        calibrated=ok_to_arm, angle_mode=True,
                        altitude_hold=False, healthy=healthy, roll_deg=0.0,
                        pitch_deg=0.0, heading_deg=0.0, altitude_m=self.p[2],
                        battery_v=4.0)
