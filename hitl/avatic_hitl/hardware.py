"""The hardware backend (hitl/): the same Drone interface, on the real Pluto X.

    drone = HardwareDrone()            # camera-module Wi-Fi: 192.168.0.1:9060
    drone = HardwareDrone(host='192.168.4.1', msp_port=23, video=None)  # drone Wi-Fi, no camera

Used through avatic_drone.Drone(backend='hardware') (or my_controller.py --hardware).

Commands: MSP_SET_RAW_RC streamed at 50 Hz from the latest send_command(),
exactly as the simulator backend (same stick mapping, pluto_x_autonomy
rc_mapping values duplicated in _to_channels so this file needs no ROS).
Telemetry: MSP_STATUS / ATTITUDE / ALTITUDE / ANALOG polled at 20 Hz.
Camera: H.264 from the camera module, decoded by ffmpeg (video.py).

Safety caps (the same as the simulator's, avatic_drone.types.SafetyLimits):
|roll|, |pitch| <= 0.6, |yaw_rate| <= 0.8, throttle <= 0.95; above 2.5 m
(barometric) the throttle is limited below hover (0.9 x the hover
estimate, itself at most 0.8; 0.4 in altitude hold) so the drone descends.

Differences from the simulator (intentional):
  * time() is the host's monotonic clock (seconds since connecting);
  * arena() is a local run timer only: time_limit_s from arming. There is
    no automatic scoring on hardware (the judges count the balloons);
  * the end of a run (time up, close(), Ctrl-C) LANDS the drone: level
    sticks and a 0.3 m/s descent-rate control on the throttle (around the
    learned hover throttle, corrected on the way down; climb rate from the
    barometric altitude) down to 0.3 m, then 0.2 m/s under a throttle cap
    that only ramps down (so the landing can never climb back up), and
    disarm once the cap is below 0.35; disarms after 30 s whatever happens.
    Not the firmware's land command: it forces 1300 us, which is a descent
    rate only in altitude hold, and altitude hold assumes hover at 1500 us
    (docs/architecture.md, firmware finding 7) - unverified for the real
    drone. A Ctrl-C while landing, or a second Ctrl-C, disarms immediately
    (emergency stop).

UNVERIFIED ON HARDWARE (2026-09-25, drone not delivered): message formats
come from the MagisV2 source; the whole command/telemetry path was tested
against the simulator through the MSP test bridge. Check on the drone,
propellers OFF first: telemetry values, arming, disarming, and the video
stream (hitl/README.md, first-session checklist).
"""

from __future__ import annotations

import atexit
import math
import signal
import threading
import time
from typing import Iterator, List, Optional

from . import msp
from avatic_drone.types import (SAFETY_LIMITS, ArenaStatus, Command, Frame, Telemetry,
                                apply_safety_limits)

COMMAND_RATE_HZ = 50.0
TELEMETRY_RATE_HZ = 20.0
COMMAND_TIMEOUT_S = 0.5
TELEMETRY_STALE_S = 1.0
HOVER_ESTIMATE_TIME_CONSTANT_S = 2.0
FAILSAFE_THROTTLE_RANGE = (0.6, 0.85)
DEFAULT_HOVER_THROTTLE = 0.76
LAND_DESCENT_M_S = 0.3         # landing: descent-rate control ...
LAND_GAIN_PER_M_S = 0.15       # ... throttle = hover + gain * (target - climb rate) ...
LAND_THROTTLE_BAND = (-0.10, 0.10)  # ... within hover + this band ...
LAND_HOVER_KI = 0.08           # ... hover corrected by this per (m/s of descent error) per s,
LAND_HOVER_KI_FALLING = 0.4    # ... faster when falling too fast by more than
LAND_FALLING_ERROR_M_S = 0.4   # ... this (the dangerous direction; gentle otherwise: lag)
LAND_HOVER_RANGE = (0.45, 0.85)  # ... within this range
LAND_FINAL_ALT_M = 0.3         # below this: the final phase (never left again: the baro
LAND_FINAL_DESCENT_M_S = 0.2   # wanders on the ground) - a slower descent under a throttle
LAND_CAP_START_ABOVE_HOVER = 0.08  # cap starting a little above hover (to brake),
LAND_CAP_RAMP_PER_S = (0.06, 0.2)  # ramping down slowly, then fast once
LAND_CAP_FAST_BELOW = 0.15     # the cap is this far below hover (on the ground by then)
LAND_DISARM_THROTTLE = 0.35    # disarm when the cap is below this (cannot fly)
LAND_HARD_LIMIT_S = 30.0       # disarm after this long whatever happens
CEILING_THROTTLE_ALT_HOLD = 0.4  # above the ceiling in altitude hold: 1400 us = -25 cm/s
CEILING_HOVER_FRACTION = 0.9     # above the ceiling otherwise: 0.9 x the hover estimate,
CEILING_HOVER_MAX = 0.8          # ... the estimate capped (a climbing controller inflates it)
FLYING_THROTTLE = 0.3          # failsafe: above this the drone is assumed airborne
AIRBORNE_ALT_M = 0.3           # the hover estimate learns only above this altitude
ARM_TIMEOUT_S = 5.0

# stick mapping (identical to pluto_x_autonomy/rc_mapping.py)
CENTRE_US, HALF_RANGE_US = 1500, 500
THROTTLE_MIN_US, THROTTLE_RANGE_US = 1000, 1000
ARM_ON_US, ARM_OFF_US = 1500, 1000       # AUX4
BARO_ON_US, BARO_OFF_US = 1500, 1000     # AUX3
AUX1_NEUTRAL_US, AUX2_NEUTRAL_US = 2000, 1000


def _clip(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def _to_channels(command: Command, arm: bool) -> List[int]:
    """Command -> AETR1234 microseconds (roll, pitch, throttle, yaw, AUX1..4)."""
    return [round(CENTRE_US + HALF_RANGE_US * _clip(command.roll, -1, 1)),
            round(CENTRE_US + HALF_RANGE_US * _clip(command.pitch, -1, 1)),
            round(THROTTLE_MIN_US + THROTTLE_RANGE_US * _clip(command.throttle, 0, 1)),
            round(CENTRE_US + HALF_RANGE_US * _clip(command.yaw_rate, -1, 1)),
            AUX1_NEUTRAL_US, AUX2_NEUTRAL_US,
            BARO_ON_US if command.altitude_hold else BARO_OFF_US,
            ARM_ON_US if arm else ARM_OFF_US]


class HardwareDrone:
    """Connection to a real Pluto X over MSP (see the module docstring)."""

    def __init__(self, host: str = '192.168.0.1', msp_port: int = 9060,
                 video: Optional[str] = 'plutocam', time_limit_s: float = 25.0,
                 connect_timeout_s: float = 10.0):
        self.closed = False
        self._t0 = time.monotonic()
        self._lock = threading.RLock()
        self._land_lock = threading.Lock()   # one landing at a time
        self._command = Command()
        self._arm_switch = False
        self._last_command_s: Optional[float] = None
        self._timed_out = False
        self._hover_estimate = DEFAULT_HOVER_THROTTLE
        self._landing = False
        self._armed_at: Optional[float] = None
        self._time_limit_s = time_limit_s
        self._baro_bit: Optional[int] = None
        self._stop = threading.Event()
        self._warned_stale = False
        self._warned_caps = set()
        self._link_dead = False
        self._auto_land_started = False
        self._sigint_count = 0
        self._estop = False                  # emergency stop: set by Ctrl-C, read by the command loop
        self._previous_handlers = {}
        self._video = None
        self._threads = []

        try:
            self._link = msp.Connection(host, msp_port, timeout_s=connect_timeout_s)
        except OSError as error:
            raise ConnectionError(
                f'cannot reach the Pluto X at {host}:{msp_port} ({error}). Joined its Wi-Fi? '
                '(camera module: 192.168.0.1:9060, drone Wi-Fi: 192.168.4.1:23)') from error
        try:
            self._threads = [threading.Thread(target=self._command_loop, daemon=True),
                             threading.Thread(target=self._telemetry_loop, daemon=True)]
            for thread in self._threads:
                thread.start()
            self._resolve_boxes()
            deadline = time.monotonic() + connect_timeout_s
            while self.get_telemetry() is None:
                if time.monotonic() > deadline:
                    raise TimeoutError(f'no telemetry from {host}:{msp_port} (is it a Pluto X?)')
                time.sleep(0.02)
            if video:
                from .video import VideoStream
                self._video = VideoStream(video, host=host, clock=self.time)
        except BaseException:          # incl. Ctrl-C: never leave threads + socket behind
            self._shutdown_threads()
            raise
        atexit.register(self.close)  # land even if the script forgets close()
        self._previous_handlers = {}
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            try:   # a closed terminal or `kill` lands the drone like Ctrl-C
                self._previous_handlers[sig] = signal.signal(sig, self._on_sigint)
            except ValueError:  # not the main thread
                pass

    # ------------------------------------------------------------ internals

    def _resolve_boxes(self):
        """Finds the BARO mode's bit in MSP_STATUS (ARM is always bit 0)."""
        sent = time.monotonic()
        self._link.request(msp.MSP_BOXIDS)
        frame = self._link.wait_for(msp.MSP_BOXIDS, sent, 2.0)
        if frame and not frame.is_error:
            ids = msp.decode_boxids(frame.payload)
            if msp.BOX_BARO in ids:
                self._baro_bit = ids.index(msp.BOX_BARO)
        if self._baro_bit is None:
            print('[hitl] WARNING: no MSP_BOXIDS reply: telemetry.altitude_hold will read False',
                  flush=True)

    def _command_loop(self):
        period = 1.0 / COMMAND_RATE_HZ
        next_t = time.monotonic()
        while not self._stop.is_set():
            now = self.time()
            _, altitude = self._decoded(msp.MSP_ALTITUDE, msp.decode_altitude)
            altitude_m = altitude.altitude_cm * 0.01 if altitude is not None else None
            with self._lock:
                command, arm = self._command, self._arm_switch
                if self._estop:
                    command, arm = Command(), False
                elif (arm and not self._landing and self._last_command_s is not None
                        and now - self._last_command_s > COMMAND_TIMEOUT_S):
                    command = self._failsafe_command(command, altitude_m)
                elif (arm and not command.altitude_hold and not self._landing
                      and altitude_m is not None
                      and AIRBORNE_ALT_M < altitude_m <= SAFETY_LIMITS.max_altitude_m):
                    # running average of the throttle sent while flying: ~hover
                    alpha = min(period / HOVER_ESTIMATE_TIME_CONSTANT_S, 1.0)
                    self._hover_estimate = _clip(
                        self._hover_estimate + alpha * (command.throttle - self._hover_estimate),
                        *FAILSAFE_THROTTLE_RANGE)
            # altitude ceiling (safety): above it, a throttle below hover so
            # the drone descends. Not by engaging altitude hold: the firmware
            # enters it assuming hover at 1500 us (altitudehold.cpp:283),
            # which drops the simulated drone (docs/architecture.md finding 7)
            ceiling = (CEILING_THROTTLE_ALT_HOLD if command.altitude_hold else
                       CEILING_HOVER_FRACTION * min(self._hover_estimate, CEILING_HOVER_MAX))
            if (arm and altitude_m is not None and not self._landing
                    and altitude_m > SAFETY_LIMITS.max_altitude_m
                    and command.throttle > ceiling):
                command = Command(command.roll, command.pitch, command.yaw_rate,
                                  ceiling, command.altitude_hold)
                if 'ceiling' not in self._warned_caps:
                    self._warned_caps.add('ceiling')
                    print(f'[hitl] above the {SAFETY_LIMITS.max_altitude_m} m safety ceiling: '
                          'throttle limited, descending (warned once)', flush=True)
            try:
                self._link.send(msp.encode_set_raw_rc(_to_channels(command, arm)))
            except OSError:
                # incl. a send timeout: that means the TCP buffer filled up
                # (seconds of stall), and a partly sent frame desynchronises
                # the stream - treat it as a lost link
                if not self._stop.is_set():
                    print('[hitl] MSP link lost (commands): control ended', flush=True)
                    self._link_dead = True
                return
            next_t += period
            time.sleep(max(next_t - time.monotonic(), 0.0))

    def _telemetry_loop(self):
        requests = (msp.MSP_STATUS, msp.MSP_ATTITUDE, msp.MSP_ALTITUDE, msp.MSP_ANALOG)
        period = 1.0 / TELEMETRY_RATE_HZ
        while not self._stop.is_set():
            try:
                for cmd in requests:
                    self._link.request(cmd)
            except OSError:
                # incl. a send timeout (a partly sent frame desynchronises the
                # stream): a lost link, as in the command loop
                if not self._stop.is_set():
                    print('[hitl] MSP link lost (telemetry)', flush=True)
                    self._link_dead = True
                return
            age = self.telemetry_age_s()
            if TELEMETRY_STALE_S < age < float('inf') and not self._warned_stale:
                print(f'[hitl] WARNING: no telemetry for {age:.1f} s - Wi-Fi link problem?',
                      flush=True)
                self._warned_stale = True
            elif age <= TELEMETRY_STALE_S and self._warned_stale:
                print('[hitl] telemetry back', flush=True)
                self._warned_stale = False
            # time up: land on our own, even if the script never calls close()
            if (self.arena().finished and not self._auto_land_started and not self._landing
                    and not self.closed):
                self._auto_land_started = True
                print('[hitl] time is up: landing', flush=True)
                threading.Thread(target=self.land, daemon=True).start()
                # land() takes _land_lock at once: a close() racing with it
                # waits for this landing instead of starting a second one
            time.sleep(period)

    def _failsafe_command(self, last: Command, altitude_m: Optional[float] = None) -> Command:
        """No send_command() for COMMAND_TIMEOUT_S: what to stream instead."""
        airborne = last.throttle > FLYING_THROTTLE or (altitude_m is not None and altitude_m > 0.3)
        if last.altitude_hold:
            safe, what = Command(throttle=0.5, altitude_hold=True), 'altitude hold'
        elif airborne:
            safe = Command(throttle=_clip(self._hover_estimate, *FAILSAFE_THROTTLE_RANGE))
            what = f'throttle {safe.throttle:.2f} (estimated hover)'
        else:
            safe, what = Command(throttle=0.0), 'throttle 0 (was on the ground)'
        if not self._timed_out:
            print(f'[hitl] no send_command() for {COMMAND_TIMEOUT_S} s: failsafe - sticks level, '
                  f'{what}', flush=True)
            self._timed_out = True
        return safe

    def telemetry_age_s(self) -> float:
        """Seconds since the last MSP_STATUS reply (inf before the first)."""
        entry = self._link.latest(msp.MSP_STATUS)
        return time.monotonic() - entry[0] if entry else float('inf')

    def _decoded(self, cmd, decoder):
        entry = self._link.latest(cmd)
        if entry is None or entry[1].is_error:
            return None, None
        try:
            return entry[0], decoder(entry[1].payload)
        except ValueError:
            return None, None

    def _on_sigint(self, signum, frame):
        # no locks here: a signal handler can run inside any `with self._lock`
        if self._landing or self._sigint_count >= 1:
            self._estop = True          # the command loop streams disarm from now on
            print('\n[hitl] Ctrl-C: EMERGENCY DISARM', flush=True)
            return
        self._sigint_count = 1
        print('\n[hitl] Ctrl-C: landing (press Ctrl-C again to disarm now)', flush=True)
        raise KeyboardInterrupt

    def _shutdown_threads(self):
        self._stop.set()
        for thread in self._threads:
            if thread.is_alive() and thread is not threading.current_thread():
                thread.join(timeout=1.0)
        self._link.close()

    # --------------------------------------------------------------- inputs

    def get_frame(self) -> Optional[Frame]:
        return self._video.get_frame() if self._video else None

    def get_telemetry(self) -> Optional[Telemetry]:
        t_status, status = self._decoded(msp.MSP_STATUS, msp.decode_status)
        _, attitude = self._decoded(msp.MSP_ATTITUDE, msp.decode_attitude)
        _, altitude = self._decoded(msp.MSP_ALTITUDE, msp.decode_altitude)
        _, analog = self._decoded(msp.MSP_ANALOG, msp.decode_analog)
        if status is None or attitude is None or altitude is None or analog is None:
            return None
        altitude_hold = (self._baro_bit is not None and
                         bool(status.mode_flags >> self._baro_bit & 1))
        return Telemetry(time_s=t_status - self._t0, armed=status.armed,
                         ready_to_arm=status.ok_to_arm,
                         roll_deg=attitude.roll_decideg * 0.1,
                         pitch_deg=-attitude.pitch_decideg * 0.1,  # firmware: nose down +
                         heading_deg=float(attitude.heading_deg),
                         altitude_m=altitude.altitude_cm * 0.01,
                         battery_v=analog.battery_mv * 0.001,
                         altitude_hold=altitude_hold)

    def flags(self) -> dict:
        """Pluto status flags (crash, low battery, signal loss, ...)."""
        _, status = self._decoded(msp.MSP_STATUS, msp.decode_status)
        if status is None:
            return {}
        names = {msp.FLAG_ACCEL_GYRO_CALIBRATION: 'calibrating_accel_gyro',
                 msp.FLAG_MAG_CALIBRATION: 'calibrating_mag', msp.FLAG_CRASH: 'crash',
                 msp.FLAG_LOW_BATTERY: 'low_battery',
                 msp.FLAG_LOW_BATTERY_IN_FLIGHT: 'low_battery_in_flight',
                 msp.FLAG_SIGNAL_LOSS: 'signal_loss', msp.FLAG_NOT_OK_TO_ARM: 'not_ok_to_arm',
                 msp.FLAG_OK_TO_ARM: 'ok_to_arm', msp.FLAG_ARMED: 'armed'}
        return {name: status.flag(bit) for bit, name in names.items()}

    def arena(self) -> ArenaStatus:
        """Local run timer only (no scoring on hardware)."""
        if self._armed_at is None:
            return ArenaStatus(time_remaining_s=self._time_limit_s)
        remaining = max(self._time_limit_s - (self.time() - self._armed_at), 0.0)
        return ArenaStatus(time_remaining_s=remaining, finished=remaining <= 0.0)

    def time(self) -> float:
        return time.monotonic() - self._t0

    # -------------------------------------------------------------- outputs

    def send_command(self, roll: float = 0.0, pitch: float = 0.0, yaw_rate: float = 0.0,
                     throttle: float = 0.0, altitude_hold: bool = False) -> None:
        for name, value in (('roll', roll), ('pitch', pitch), ('yaw_rate', yaw_rate),
                            ('throttle', throttle)):
            if not math.isfinite(value):
                raise ValueError(f'{name} must be a finite number, got {value}')
        safe, capped = apply_safety_limits(Command(roll, pitch, yaw_rate, throttle, altitude_hold))
        for name in capped:
            if name not in self._warned_caps:
                self._warned_caps.add(name)
                print(f'[hitl] {name} command beyond the safety cap: clipped '
                      '(avatic_drone.types.SafetyLimits); warned once', flush=True)
        with self._lock:
            if self._landing:
                return  # the land command is in charge
            self._command = safe
            self._last_command_s = self.time()
            self._timed_out = False

    def send(self, command: Command) -> None:
        if not isinstance(command, Command):
            raise TypeError(f'send() needs a Command, got {type(command).__name__}')
        self.send_command(command.roll, command.pitch, command.yaw_rate, command.throttle,
                          command.altitude_hold)

    # --------------------------------------------------------------- arming

    def wait_until_ready(self, timeout_s: float = 60.0) -> None:
        deadline = time.monotonic() + timeout_s
        while True:
            t = self.get_telemetry()
            if t is not None and t.ready_to_arm:
                return
            if time.monotonic() > deadline:
                raise TimeoutError(f'the Pluto X is not ready to arm: {self.flags()}')
            time.sleep(0.05)

    def arm(self) -> None:
        self.send_command(throttle=0.0)
        with self._lock:
            self._estop = False
            self._armed_at = None          # a new run: the previous one's timer is over
            self._auto_land_started = False
            self._arm_switch = True
        deadline = time.monotonic() + ARM_TIMEOUT_S
        while not getattr(self.get_telemetry(), 'armed', False):
            self.send_command(throttle=0.0)
            if time.monotonic() > deadline:
                with self._lock:
                    self._arm_switch = False
                raise RuntimeError(f'the Pluto X did not arm: {self.flags()}')
            time.sleep(0.02)
        self._armed_at = self.time()

    def land(self, timeout_s: float = LAND_HARD_LIMIT_S) -> None:
        """Lands, then disarms (see the module docstring): level sticks,
        descent-rate control down to LAND_FINAL_ALT_M, then a throttle
        ramp-down and disarm; also disarms after timeout_s, and at once if
        the link is lost. While landing, a Ctrl-C disarms immediately
        (emergency stop). If a landing is already running (time up), waits
        for it."""
        if not self._land_lock.acquire(blocking=False):
            with self._land_lock:       # another landing is running: wait for it
                return
        try:
            if not getattr(self.get_telemetry(), 'armed', False) or self._estop:
                return
            with self._lock:
                last_throttle = self._command.throttle
            t = self.get_telemetry()
            if (last_throttle < FLYING_THROTTLE and t is not None
                    and t.altitude_m < LAND_FINAL_ALT_M):
                return   # still on the pad: disarm, no hop
            print('[hitl] landing...', flush=True)
            # hover: the learned estimate, capped (a climbing controller
            # inflates it) and corrected on the way down (integral term)
            hover = min(_clip(self._hover_estimate, *FAILSAFE_THROTTLE_RANGE), CEILING_HOVER_MAX)
            if last_throttle > FLYING_THROTTLE:
                # flying: the last throttle is another hover guess (capped);
                # the higher one avoids starting with a drop
                hover = max(hover, min(last_throttle, CEILING_HOVER_MAX))
            throttle = hover
            with self._lock:
                self._landing = True
                self._command = Command(throttle=throttle)
            start = previous = time.monotonic()
            last_alt = last_t = None
            climb = 0.0
            final = False               # the final phase (never left again)
            cap = 1.0
            while time.monotonic() - start < timeout_s:
                if self._estop:
                    return
                if self._link_dead or self._link.closed:
                    print('[hitl] link lost while landing: the firmware\'s own RC-loss '
                          'failsafe is in charge', flush=True)
                    return
                t = self.get_telemetry()
                if t is not None and not t.armed:
                    return  # disarmed (by the firmware or an emergency stop)
                now = time.monotonic()
                dt, previous = now - previous, now
                fresh = t is not None and self.telemetry_age_s() < TELEMETRY_STALE_S
                if fresh and last_alt is None:
                    last_alt, last_t = t.altitude_m, now
                elif fresh and t.altitude_m != last_alt:     # a new baro sample
                    climb += 0.3 * ((t.altitude_m - last_alt) / (now - last_t) - climb)
                    last_alt, last_t = t.altitude_m, now
                if fresh and t.altitude_m < LAND_FINAL_ALT_M and not final:
                    final = True
                    cap = hover + LAND_CAP_START_ABOVE_HOVER   # room to brake
                target = -LAND_FINAL_DESCENT_M_S if final else -LAND_DESCENT_M_S
                if fresh:
                    # correct the hover estimate while not descending as planned
                    error = target - climb          # + = falling faster than planned
                    ki = LAND_HOVER_KI_FALLING if error > LAND_FALLING_ERROR_M_S else LAND_HOVER_KI
                    hover = _clip(hover + ki * dt * error, *LAND_HOVER_RANGE)
                    low_band, high_band = LAND_THROTTLE_BAND
                    throttle = _clip(hover + LAND_GAIN_PER_M_S * (target - climb),
                                     hover + low_band, hover + high_band)
                    if t.altitude_m > SAFETY_LIMITS.max_altitude_m:   # the ceiling holds too
                        throttle = min(throttle, CEILING_HOVER_FRACTION * hover)
                else:
                    throttle = CEILING_HOVER_FRACTION * hover   # no telemetry: sink
                if final:
                    slow, fast = LAND_CAP_RAMP_PER_S
                    cap -= (fast if cap < hover - LAND_CAP_FAST_BELOW else slow) * dt
                    throttle = min(throttle, cap)
                    if cap < LAND_DISARM_THROTTLE:
                        return  # on the ground: disarm
                with self._lock:
                    self._command = Command(throttle=throttle)
                time.sleep(0.05)
            print(f'[hitl] landing not finished after {timeout_s:.0f} s: disarming',
                  flush=True)
        except KeyboardInterrupt:       # only if the SIGINT handler is not installed
            print('\n[hitl] Ctrl-C while landing: EMERGENCY DISARM', flush=True)
        finally:
            self.disarm()
            with self._lock:
                self._landing = False
            self._sigint_count = 0      # Ctrl-C interrupts the script again
            self._land_lock.release()

    def disarm(self) -> None:
        """Stops the motors immediately (the drone falls if airborne)."""
        with self._lock:
            self._arm_switch = False
            self._command = Command()

    # --------------------------------------------------------------- timing

    def running(self) -> bool:
        """False when the run is over, or the link is gone (no telemetry for
        2 x TELEMETRY_STALE_S, or a dead socket)."""
        return (not self.arena().finished and not self._link.closed and not self._link_dead
                and self.telemetry_age_s() < 2 * TELEMETRY_STALE_S)

    def sleep(self, seconds: float) -> None:
        end = self.time() + seconds
        while self.time() < end and self.running():
            time.sleep(0.002)

    def loop(self, hz: float) -> Iterator[int]:
        if not hz > 0.0:
            raise ValueError('hz must be > 0')
        period, step = 1.0 / hz, 0
        next_t = self.time()
        warned = False
        while self.running():
            yield step
            step += 1
            next_t += period
            now = self.time()
            if now > next_t + period:
                next_t += int((now - next_t) / period) * period
                if not warned:
                    print(f'[hitl] loop({hz:g} Hz): an iteration took longer than '
                          f'{period * 1000:.0f} ms; skipping missed periods', flush=True)
                    warned = True
            while self.time() < next_t and self.running():
                time.sleep(0.001)

    def close(self) -> None:
        """Lands (if flying), disarms and disconnects."""
        if self.closed:
            return
        self.closed = True
        try:
            try:
                self.land()  # waits for the time-up landing if it is running
            except KeyboardInterrupt:
                # a Ctrl-C in the instant before landing started: land anyway
                # (the next Ctrl-C is an emergency stop)
                self._sigint_count = 1
                self.land()
            time.sleep(0.2)  # let the disarm frames go out
        finally:
            if self._video:
                self._video.close()
            self._shutdown_threads()
            atexit.unregister(self.close)
            for sig, handler in self._previous_handlers.items():
                try:
                    signal.signal(sig, handler)
                except ValueError:
                    pass  # close() from another thread: the handler stays (harmless)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
