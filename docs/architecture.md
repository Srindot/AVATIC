# Pluto X simulation: architecture, MagisV2 in the loop, outer-loop controllers

Status 2026-09-25. This is the current architecture. The legacy kwad.cpp
stack is still available as an alternative flight controller; its port is
documented in [legacy_port.md](legacy_port.md) and its parameters in
[pluto_x_parameters.md](pluto_x_parameters.md).

## 1. Overview

```text
 participant code                  ROS 2                        Gazebo Harmonic (gz sim, 1 ms steps)
 ────────────────                  ─────                        ────────────────────────────────────
 my_controller.py
   avatic_drone.Drone ── /pluto/rc (50 Hz) ──►  fc_link ── gz /pluto/rc ──► VehicleSystem plugin
   (safety caps,        RcCommand                        (Int32_V, AETR1234)   ├─ SimulatedVehicle
    failsafe, ceiling)                                                         │   ├─ wind, RC link latency
        ▲  ◄── /pluto/fc_status ◄── fc_link ◄── gz /pluto/fc_telemetry        │   ├─ FlightController:
        │      (FC estimates only)                                             │   │    MagisV2 firmware
        ◄── /pluto/camera/image_raw ◄── ros_gz_bridge ◄── camera sensor       │   ├─ propulsion, battery
        ◄── /arena/score, time_remaining, events ◄── arena_system         │   └─ external wrench
                                                                               └─ AddWorldWrench → physics
 run_recorder ◄── everything above + /sim/pluto/odometry (ground truth, judges only)
              ──► analysis/runs/<date-time>/  (or evaluation/sessions/...)

 on the real Pluto X the same controller file runs with --hardware:
   avatic_drone.Drone(backend='hardware') → hitl/avatic_hitl: MSP v1 over TCP
   (192.168.0.1:9060) + the plutocam H.264 camera stream
```

What runs where:

| Layer | What | Where |
|---|---|---|
| Outer loop | the participant's visual-servoing controller (camera + FC telemetry only) | `outerloop_controller/my_controller.py`, through `avatic_drone` |
| Link | RC channels to the vehicle, telemetry back (the real Pluto X uses MSP over Wi-Fi) | sim: `pluto_x_ros/fc_link`; hardware: `hitl/avatic_hitl` |
| Flight controller | the **production MagisV2 firmware**: sensor calibration, attitude estimator, angle mode, altitude hold, PID, mixer, arming, failsafe | `pluto_x_magisv2` (host build of the unmodified vendored source; two `MAGIS_HOST`-guarded patches, §4) |
| Vehicle | sensors (IMU, baro, magnetometer, battery monitor, camera), motors, battery, aerodynamics | `pluto_x_core` (`SimulatedVehicle`), in the Gazebo plugin |
| Rigid body | integration, gravity, ground contact | Gazebo physics (DART) |

The participant interface is **sticks in, camera + telemetry out**, the
same as the real Pluto X offers; the same controller file flies either
backend. `pluto_x_ros/scripts/msp_sim_bridge.py` serves the simulator over
the hardware protocol (MSP on TCP 9060, H.264 on 9061), to test the
hardware backend without a drone.

The developer path of section 5 (`outer_loop_host`, ground-truth
observation) is kept for simulator validation (sim.launch.py,
arena.launch.py, the check scripts); participants do not use it.

## 2. Packages

```text
firmware/magisv2      MagisV2 firmware, vendored unmodified (GPL-3.0-or-later; PROVENANCE.md)
simulation_engine/pluto_x_core         vehicle model, sensors, legacy stack, FlightController interface, config (no ROS/Gazebo)
simulation_engine/pluto_x_magisv2      host build of MagisV2 + MagisHost facade + MagisFlightController (GPL)
simulation_engine/pluto_x_gazebo       VehicleSystem plugin (links pluto_x_magisv2, so GPL), model, world
simulation_engine/pluto_x_interfaces   RcCommand.msg, FlightControllerStatus.msg, OuterLoopSetpoint.msg
simulation_engine/pluto_x_ros          fc_link (RC + telemetry link), msp_sim_bridge.py (hardware protocol test bridge)
simulation_engine/pluto_x_autonomy     developer OuterLoopController API, outer_loop_host, supervisor, examples (Python)
outerloop_controller/                  participant template + avatic_drone API (sim / hardware backends)
hitl/                                  hardware backend: MSP client, camera decoder
analysis/, evaluation/                 run recordings, notebooks, the multi-layout evaluator
simulation_engine/pluto_x_bringup      competition / arena / sim / legacy_sim launches, run_recorder, bridge config, end-to-end checks
```

## 3. Flight controller interface (`pluto_x_core`)

`pluto_x::FlightController` (`control/flight_controller.hpp`) is what
`SimulatedVehicle` calls every physics step:

* input: simulation time, physical truth (state and **acceleration**, from
  which each implementation builds its own sensor readings), the latest RC
  frame after the command-link latency, battery voltage and current;
* output: motor duty per rotor in [0, 1], plus `armed`, `healthy` and
  telemetry (the controller's own estimates).

Implementations:

| `flight_controller.type` | Class | Sensors | Commands |
|---|---|---|---|
| `magisv2` (default in `pluto_x_estimated.yaml`) | `pluto_x_magisv2::MagisFlightController` | IMU (ICM-20948 counts), baro (ICP-10111), magnetometer (AK09916), INA219 battery monitor | RC frames |
| `legacy` | `pluto_x::LegacyFlightController` | full state + `state_error` model | fixed by `pilot.*` (RC ignored) |

`PropulsionModel` turns duty into rotor speed as ω_target = (V/V_ref) · d ·
ω_max. This is linear in duty (an assumption, see pluto_x_parameters.md §4),
followed by the first-order motor lag.

Selecting the controller: set `flight_controller.type` in the YAML, or pass
`flight_controller:=legacy|magisv2` to `sim.launch.py`. The launch argument
is passed to the plugin as the SDF `<flight_controller>` and overrides the
YAML before validation.

## 4. MagisV2 in the loop (`pluto_x_magisv2`)

### Build

* `firmware/magisv2` is never edited. At configure time CMake copies it
  into the build tree and applies `simulation_engine/pluto_x_magisv2/patches/*.patch`.
  Every patch is guarded by `MAGIS_HOST`:
  * `0001-host-config-flash-buffer`: the configuration "flash" is a host
    buffer instead of the STM32 flash address;
  * `0002-arm-divide-by-zero-filterRc`: `filterRc` divides by a cycle time
    that is 0 on the first pass. On Cortex-M4 an integer division by zero
    returns 0; x86 traps (SIGFPE). The patch reproduces the ARM result.
* Compiled: the Makefile's PRIMUSX2 module list without `main.cpp` and the
  hardware drivers, with the target's defines and ARM-compatible flags:
  `-funsigned-char` (ARM `char` is unsigned) and `-fsingle-precision-constant`.
  The link uses `-Wl,--no-undefined`, so a firmware symbol with no host
  implementation is a build error rather than a silent gap.
* Replaced: the driver layer (`host/hal_*.cpp`): time, flash, sensor
  drivers, PWM, power monitor. It also replaces `main.cpp init()`:
  `MagisHost::Initialise` makes its flight-logic calls in the same order and
  skips the hardware bring-up (the list is in `magis_host.cpp`).
* Executed: `mw.cpp loop()`, unmodified, called every
  `busy_loop_period_us` (100 µs) of simulated time. The firmware then runs
  its control step every `looptime` (3.5 ms) and its round-robin tasks in
  between, as on the MCU.

### Conventions (derived from the firmware, verified by tests)

* Firmware body frame FLU, world z up; roll + = right side down, **pitch + =
  nose down**, yaw + = counter-clockwise (heading = −yaw).
* Motors M0..M3 = rear-right, front-right, rear-left, front-left.
  **M1 and M2 spin CCW** (the firmware's yaw control requires it;
  `test_magis_host.cpp` checks it).
* Motor output: PWM 1000..2000 µs, brushed duty = (PWM − 1000)/1000.
* Pluto X receiver setup (Rx_ESP): RC over MSP, channel order AETR1234;
  ARM = AUX4 1300..2100; ANGLE always on; BARO (altitude hold) = AUX3
  1300..2100; MAG = AUX1 900..1300; HEADFREE = AUX1 1300..1700.
* Sensors given to the firmware: gyro 16.4 LSB/(°/s) and accelerometer
  4096 LSB/g (`GYRO_CONFIG_1 = 0x1F` gives ±2000 °/s and `ACCEL_CONFIG =
  0x35` gives ±8 g, as the firmware configures the ICM-20948); the
  magnetometer in AK09916 raw axes, which equal FRD (the firmware maps raw
  (x, y, z) to FLU (x, −y, −z)).
* One firmware instance per process (its state lives in globals). The
  simulated time is `uint32` µs, as on the MCU, which limits a run to
  4294 s (checked).

### Findings from running the real firmware

1. **Gyro scale mismatch in the firmware (hardware question).**
   `accgyro_icm20948.cpp` configures the gyro for ±2000 °/s (16.4 LSB per
   °/s) but sets `gyro->scale = 1/131` (the ±250 °/s value; the 16.4 line is
   commented out). The attitude estimator therefore integrates rotation
   about 8× too slowly and relies on accelerometer correction. In the
   simulator the estimated bank lags the true bank by about 20 % in a
   sustained turn (6.5° estimated against 8.1° true). The simulator gives
   the firmware what the chip would deliver, so the real vehicle should show
   the same behaviour. Worth confirming on hardware before tuning an outer
   loop on attitude telemetry.
2. **Rotor inertia constrains the yaw loop.** With the spin-up reaction
   torque modelled, the first rotor-inertia estimate (3e-7 kg m²) put the
   firmware's yaw rate loop into a ±200 µs limit cycle and the vehicle spun
   on the ground. The stable region is J ≤ 4e-8 kg m². The revised
   estimate, 4e-8 (pluto_x_parameters.md §4), is also the better-founded
   one. The margin is thin, so J, the motor lag and Izz need measuring
   (yaw step test).
3. **Baro model consistency.** The firmware converts pressure to altitude
   with an isothermal formula at the sensor's temperature. The BaroModel
   used to follow the ISA temperature profile while reporting 30 °C, which
   gave a 6 % altitude scale error. It now uses a lapse-rate atmosphere
   anchored at the configured air temperature.
4. **Hover throttle** in the simulator is about 1760 µs (duty 0.76) at 3.9 V
   with the 8 g camera module fitted (68 g; 1720 µs without it).
   This follows from the linear duty-to-speed assumption and the estimated
   thrust coefficient. A thrust-stand measurement will move it.
5. **The firmware re-zeroes the baro in near-free-fall (affects hardware
   too).** With the arm switch on and throttle above `mincheck`,
   `io/rc_controls.cpp:183-186` calls `mwArm()` whenever
   `netAccMagnitude < 11` (under 0.33 g of specific force: the "throw to
   arm" feature), also when already armed; `mw.cpp:579-583` then calls
   `baroResetGroundLevel()`, which takes the current pressure as the new
   ground. A throttle cut that gives a downward acceleration beyond about
   6.5 m/s² in flight therefore resets the altitude to about 0 at the
   current height, once per RC frame while it lasts, and the offsets add
   up. Found 2026-09-25 with an undamped altitude loop (throttle swinging
   between 0 and the 0.95 cap): the true height reached 12.9 m while the
   estimate read about 3 m, and the 2.5 m safety ceiling (which uses the
   estimate) did not engage. Replaying the run with the ground pressure held
   after arming keeps the baro within 0.5 m of the truth, so the sensor
   model is not the cause. Normal flight never triggers it (baro error
   ≤ 0.3 m in the recorded runs). Participant guidance: damp the altitude
   loop, never cut the throttle hard in flight (outerloop_controller/README.md).
6. **Boot-time baro calibration (fixed in the host facade, 2026-09-25).**
   `icp10111BaroCalibrate()` waits for conversions with `delay()`, which
   used to return without advancing time on the host, so the calibration
   never saw a reading and left the ground pressure at 0. It was hidden
   because the repeated `mwArm()` on the pad (finding 5) set the ground
   level. `delay()` now advances the firmware clock during start-up
   (host/hal_system.cpp); `MagisHost` keeps the resulting boot time (about
   1.85 s) as an offset between the firmware and simulation clocks. Checked
   by test_magis_host (ground pressure after boot = the simulated pressure).
7. **Altitude hold assumes hover at 1500 us: the simulated drone drops
   (OPEN, 2026-09-25).** Entering BARO mode sets `initialThrottleHold =
   1500` (`flight/altitudehold.cpp:283`); the hold throttle is then 1500 us
   + the altitude PID's adjustment. The simulated Pluto X hovers at about
   1760 us (finding 4, an estimate), so 1500 us gives about 43 % of the
   weight (thrust ~ duty^2) and the vehicle hits the ground from 1.2 m in
   under a second, before the PID catches up (measured: engaging altitude
   hold at 1.24 m, truth 0.02 m one second later). The firmware's land
   command relies on altitude hold (it forces the throttle to 1300 us =
   -50 cm/s in BARO mode, a raw 30 % throttle without it). Either the real
   Pluto X hovers near 1500 us (then the thrust model's hover is wrong and
   should be re-estimated) or the real drone dips too. Measure hover
   throttle on hardware (first session) before changing the model. Until
   then: participants fly without altitude hold; the hardware backend lands
   in altitude hold (the firmware's design); the MSP test bridge emulates
   that landing without the simulated firmware's altitude hold.

## 5. Developer outer-loop controllers (`pluto_x_autonomy`)

For simulator validation only; participants use `avatic_drone`
(outerloop_controller/README.md).

Subclass `pluto_x_autonomy.api.OuterLoopController`:

```python
from pluto_x_autonomy.api import OuterLoopController, StickCommand

class Hover(OuterLoopController):
    def reset(self, obs, status):            # once, when armed
        self.z0 = obs.position_enu_m[2]

    def update(self, obs, status, dt):       # every 1/rate_hz of sim time
        climb_error = 0.3 * (self.z0 + 1.0 - obs.position_enu_m[2]) - obs.velocity_enu_m_s[2]
        return StickCommand(throttle=0.76 + 0.25 * climb_error)

    def done(self):                          # True -> the host lands and disarms
        return False
```

Run it (no package needed; a plain file works):

```bash
ros2 launch pluto_x_bringup sim.launch.py controller:=/path/to/hover.py:Hover controller_params_file:=/path/to/params.yaml
```

For an installed package, use `controller:=my_pkg.my_module:MyController`.
The YAML mapping in `controller_params_file` is passed to the constructor
as `params`.

What the controller receives:

* `Observation`: position and velocity in ENU (origin = spawn), roll,
  pitch and yaw (REP-103: yaw counter-clockwise from east), body rates in
  FLU, `source`. In the simulator `source = 'ground_truth'` (from
  `/sim/pluto/odometry`). **No real vehicle has this**, so a flight-ready
  controller has to get its state from the camera or an estimator.
* `FcStatus`: the flight controller's telemetry: armed, ok_to_arm,
  calibrated, modes, *estimated* attitude, heading, altitude, battery.

What it returns, `StickCommand` (normalised, clipped by the host):

| field | range | meaning (verified in the closed-loop tests) |
|---|---|---|
| `roll` | −1..1 | +1 = bank right |
| `pitch` | −1..1 | +1 = nose down, accelerate forward |
| `yaw` | −1..1 | +1 = yaw rate clockwise seen from above |
| `throttle` | 0..1 | collective (hover about 0.76 in the sim, with the camera) |
| `altitude_hold` | bool | engage the firmware's baro altitude hold (AUX3) |

In angle mode roll and pitch set an attitude setpoint; about 0.2 stick
gives a 6.5° bank (measured in the sim). Yaw sets a yaw rate.

What the host does (`supervisor.py`, unit-tested):

* waits for telemetry and for `ok_to_arm` (firmware calibration, about 4 s
  in Gazebo);
* arms through AUX4 with the throttle at minimum, then calls `reset()` and
  starts `update()`;
* when `done()` returns True it lands (vertical-speed PI on the observation,
  holding horizontal position over the point where the landing started)
  and disarms after touchdown. This only ends a development run safely:
  the competition task (pop balloons within the time limit) has no landing
  requirement, and in the arena the run ends at the time limit, not with a
  landing;
* failsafe: a controller exception or a non-`StickCommand` return, a stale
  observation or telemetry (> 0.5 s), an unhealthy flight controller, or a
  geofence violation (default 5 m altitude, 20 m radius) → level sticks
  and a throttle ramp down from the learned hover throttle, then disarm;
* if the firmware disarms by itself in flight, the host stops (ABORTED).

All supervisor limits are ROS parameters (`supervisor.<field>`).

Example controller: `pluto_x_autonomy/examples/waypoint.py`
(`WaypointController`). It is a cascade of position P, velocity PI and tilt
on the horizontal axes, with altitude P and climb-rate PI on the throttle
and heading hold. It flies `config/waypoint_square.yaml` by default. It
demonstrates the interface; it is not a tuned mission controller.

## 6. Verification

| Check | Command | Result (2026-09-24) |
|---|---|---|
| Core unit tests | `colcon test --packages-select pluto_x_core` | 129 pass (2026-09-25) |
| MagisV2 host: boot, calibrate, arm, attitude and yaw responses | `pluto_x_magisv2_tests` | pass |
| MagisV2 closed loop (reference integrator): on the ground at mid throttle, climb, roll, pitch and yaw responses, telemetry signs, disarm | `pluto_x_magisv2_flight_controller_tests` | pass |
| Outer-loop layer: RC map, loader, supervisor phases and failsafes, waypoint geometry, point-mass mission | `colcon test --packages-select pluto_x_autonomy` | 20 pass (2026-09-25) |
| Gazebo end-to-end: MagisV2 + fc_link + host fly the square mission | `simulation_engine/scripts/check_mission.sh` | all waypoints within 0.10 m, settled tracking error 0.15 m (wind on), max tilt 9.4° (68 g with camera); touch-down point reported, not checked |
| Same with a rotated start (checks the body-frame handling) | `SPAWN_YAW=2.0 simulation_engine/scripts/check_mission.sh` | mission completed |
| Yaw axis: stick steps (open loop) and a 450° heading scan (closed loop) | `simulation_engine/scripts/check_yaw.sh` | all pass; see "Yaw axis" in §6 |
| Legacy stack regression | `simulation_engine/scripts/validate_legacy_sim.sh`, `simulation_engine/scripts/check_dynamics.sh` | all pass (unchanged numbers) |

The link acceleration that Gazebo reports (the input to the accelerometer
model) was checked in a motors-off drop from 10 m. It reads 9.75 m/s² at
release, decreases as body drag builds, and matches Δv/Δt.

### Yaw axis (`simulation_engine/scripts/check_yaw.sh`, 2026-09-24, wind off)

Participants will yaw to pan the camera across the field, so the yaw axis
has its own checks.

Open loop: yaw-stick steps while the outer loop holds position.

| yaw stick | steady rate | per unit stick | rise 10–90 % | overshoot | ripple | position disturbance |
|---|---|---|---|---|---|---|
| +0.1 | −7.9 °/s | 79 °/s | 30 ms | 6 % | 0.09 °/s | 0.014 m |
| +0.2 | −15.5 °/s | 77 °/s | 30 ms | 5 % | 0.08 °/s | 0.022 m |
| +0.4 | −30.6 °/s | 77 °/s | 30 ms | 5 % | 0.08 °/s | 0.030 m |
| −0.4 | +30.6 °/s | 76 °/s | 30 ms | 4 % | 0.09 °/s | 0.020 m |
| +1.0 | −76.5 °/s | 77 °/s | 40 ms | 5 % | 0.08 °/s | 0.014 m |

* A + stick turns the vehicle clockwise, and the rate is linear in the
  stick (about 77 °/s per unit) up to full stick. The yaw-rate limit is the
  firmware's rate setting, not motor saturation: a full turn takes at least
  4.7 s.
* There is no limit cycle and no oscillation (steady ripple 0.09 °/s),
  including at full stick. This is the regression check for the
  rotor-inertia finding in §4.
* Yaw couples very little into position (≤ 3 cm) or altitude (≤ 1 mm).
* The 30 ms response comes mainly from the modelled spin-up reaction
  torque, which rests on the estimated rotor inertia and motor lag. It is
  the least certain part of the yaw model, so measure the yaw step response
  on hardware.

Closed loop: `WaypointController` with headings (`config/yaw_scan.yaml`:
0 → 90 → 180 → 360 → 270 → −90 → 0°, slewed at 45 °/s), hovering at 1 m.

| | result |
|---|---|
| heading error while turning | mean 0.7°, max 3.9° (the lag behind a 45 °/s ramp) |
| settled heading error | mean 0.06°, max 0.14° |
| position drift over the 450° scan | max 0.087 m (no wind); max 0.58 m with the then-default 0.3 m/s gusts (now 0.15 m/s), the same as hovering without yawing (the example outer loop's gust rejection) |
| altitude deviation | max 4 mm |
| firmware heading estimate vs truth | mean +0.36°, max 1.3°. The mean is the magnetic declination of the configured field (−0.43°): the firmware reports magnetic heading. Its heading estimate is good despite the gyro-scale mismatch in §4 finding 1, because the magnetometer corrects it. |

For a camera pan, use waypoints with the same position and increasing
`yaw_deg` (unwrapped, so 360 is a full turn), or command the yaw stick
directly (77 °/s per unit). The host publishes the controller's heading
setpoint in `/pluto/outer_loop/setpoint` (`OuterLoopSetpoint.msg`).

## 7. Known limitations

* The stick-to-angle and throttle-to-thrust maps come from estimated
  parameters. Measure them on hardware (pluto_x_parameters.md §8).
* Not modelled: the IMU's internal low-pass filters (gyro DLPF ~51 Hz,
  accelerometer ~5.7 Hz, per the configured DLPFCFG values), vibration, magnetic disturbances, the
  ICP-10111's internal conversion, and non-ideal ESC or brushed-motor
  curves.
* The MSP link is modelled as a pure delay (`latency.command_s`): no
  packet loss, no jitter.
* Stopping `sim.launch.py` with a signal can leave the `gz sim` server
  running (the launch stops the `ruby gz` wrapper). The check scripts run it
  in its own process group and stop the whole group.
* One simulated Pluto X per Gazebo process (the firmware is a singleton).

## 8. Licensing

MagisV2 is GPL-3.0-or-later. `pluto_x_magisv2` links it, and so does the
Gazebo plugin (`pluto_x_gazebo`), so both are GPL-3.0-or-later.
`pluto_x_core`, `pluto_x_ros`, `pluto_x_interfaces` and `pluto_x_autonomy`
do not link it; they talk to it through topics, and their licence is still
to be decided. Participant controllers run in a separate process and only
exchange ROS messages. The legacy meshes have no licence; clarify that
before redistributing them.
