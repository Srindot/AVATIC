# Pluto X simulation — legacy controller stack on ROS 2 / Gazebo Harmonic

> Current architecture (MagisV2 firmware in the loop, outer-loop
> controllers): [architecture.md](architecture.md). This document covers the
> legacy kwad.cpp stack, which remains available as
> `flight_controller: legacy`.

Status: **preliminary**. This is the controller cascade and dynamics model of
the legacy `PlutoX-ROS-Joystick-Control` repository (`src/kwad.cpp`), ported
onto a ROS 2 Humble + Gazebo Harmonic foundation. The controller is not
MagisV2.

Two parameter sets ship:
* `pluto_x_estimated.yaml` (**default**): estimated Pluto X parameters
  (68 g with the camera module, X layout, rescaled legacy gains). These are estimates, not
  measurements; see [pluto_x_parameters.md](pluto_x_parameters.md).
* `legacy_kwad.yaml`: the original `kwad.cpp` values (1.4 kg, 0.56 m arm,
  "+" layout), kept for reproducing the legacy behaviour. §5–§9 below
  describe this set.

---

## 1. System architecture

```text
            ROS 2 Humble                          Gazebo Harmonic (gz-sim 8)
 ┌──────────────────────────────┐        ┌──────────────────────────────────────────┐
 │                              │        │  pilot command fixed by config           │
 │                              │        │        │                                  │
 │                              │        │        ▼                                  │
 │                              │        │  VehicleSystem (model plugin)             │
 │                              │        │   PreUpdate, every 1 ms physics step:     │
 │                              │        │    read link pose/twist (ENU/FLU)         │
 │                              │        │    ── every 10 ms (sim time) ──           │
 │                              │        │    ENU→NED adapter                        │
 │                              │        │    LegacyControllerStack::Step            │
 │                              │        │      position → attitude → rate → mixer   │
 │                              │        │    ─────────────────────────              │
 │                              │        │    legacy external wrench (NED/FRD)       │
 │                              │        │    NED→ENU adapter → Link::AddWorldWrench │
 │                              │        │        ▼                                  │
 │                              │        │  DART rigid-body physics (+gravity,       │
 │                              │        │  ground contact)                          │
 │ /pluto/imu ◄─────────────────┼─bridge─┼── gz IMU sensor (observation only)        │
 │ /sim/pluto/odometry ◄────────┼─bridge─┼── OdometryPublisher (ground truth)        │
 │ /pluto/battery ◄─────────────┼─bridge─┼── battery state (10 Hz)                   │
 │ /clock ◄─────────────────────┼─bridge─┼── sim clock                               │
 └──────────────────────────────┘        └──────────────────────────────────────────┘
```

All algorithms live in `pluto_x_core` (no ROS, no Gazebo). Everything that
happens to the vehicle in one physics step is in
`pluto_x::SimulatedVehicle`:
* wind;
* the controller at its fixed period, with command latency and sensor
  error;
* propulsion, with motor lag and battery sag;
* the external wrench.

The Gazebo plugin and the offline reference sim both call that same class.
The plugin is glue: it copies the state in from the ECM, applies the
returned wrench and publishes battery state. The added effects are
described in [pluto_x_parameters.md §7](pluto_x_parameters.md). The vehicle pose is never
set directly (`SetModelState`-style teleporting is not used).

## 2. Package structure

```text
simulation_engine/
├── pluto_x_core/            pure C++17 (Eigen, yaml-cpp) + gtest
│   ├── config/pluto_x_estimated.yaml    default parameters (estimated Pluto X)
│   ├── config/legacy_kwad.yaml          original kwad.cpp parameters
│   ├── include/pluto_x/
│   │   ├── common/   frames, validation, backward_difference, math_types
│   │   ├── config/   legacy_params (structs), config_loader
│   │   ├── control/  legacy_pid_term, legacy_cascade, quad_mixer,
│   │   │             legacy_controller_stack, pilot_command
│   │   └── dynamics/ vehicle_state, legacy_dynamics, enu_adapter
│   ├── src/…                            implementations
│   ├── tools/legacy_reference_sim.cpp   offline closed-loop reference run
│   └── test/                            unit tests (129 on 2026-09-25)
├── pluto_x_gazebo/          gz-sim system plugin + model + world
│   ├── src/vehicle_system.cpp           ECM glue, RC in, telemetry out, wrench
│   ├── models/pluto_x/model.sdf.xacro   model; mass/inertia read from the YAML
│   ├── worlds/legacy_flat.sdf           flat world, 1 ms DART step
│   └── hooks/                           sets GZ_SIM_RESOURCE_PATH / _SYSTEM_PLUGIN_PATH
└── pluto_x_bringup/
    ├── launch/legacy_sim.launch.py      gz sim + spawn + bridge
    ├── config/bridge.yaml               ros_gz_bridge topics
    └── scripts/legacy_flight_check.py   end-to-end flight checks
                compare_trajectories.py  trajectory comparison / determinism
                dynamics_check.py        dynamics / frame checks
simulation_engine/scripts/dev.sh                           run any command in the dev container
simulation_engine/scripts/validate_legacy_sim.sh           full end-to-end validation
```

`resources/COLCON_IGNORE` keeps the reference repositories (ROS 1, firmware)
out of the colcon workspace.

## 3. Data flow and timing

| Item | Value | Source |
|---|---|---|
| Physics step | 1 ms (DART) | `worlds/legacy_flat.sdf` |
| Controller period | 10 ms, on **simulation time** | `controller.period_s` |
| Wrench update | every physics step (state-dependent terms recomputed; rotor output held between controller ticks) | plugin |

The controller runs inside the physics loop (Gazebo `PreUpdate`), not as a
separate ROS node. There is no asynchronous input, so runs are
bit-for-bit repeatable (§12).

The plugin warns if the controller period is not an integer multiple of the
physics step.

## 4. Coordinate frames

| Frame | Axes | Used by |
|---|---|---|
| ENU world / FLU body | x East, y North, z Up / x fwd, y left, z up | Gazebo, ROS (REP-103) |
| NED world / FRD body | x North, y East, z Down / x fwd, y right, z down | legacy controller and dynamics |

The legacy equations are NED/FRD (thrust along −z_body, gravity along +z).
They run unchanged in that frame. All conversion happens in
`pluto_x/common/frames.hpp` and `pluto_x/dynamics/enu_adapter.hpp`, and is
unit-tested (including a test that the ENU rigid-body equations driven by
the converted wrench reproduce the NED legacy equations).

The controller holds the heading it starts with (`pilot.hold_initial_heading`,
deviation D12). The launch file spawns the vehicle facing +x (`spawn_yaw`
0), the direction Gazebo's default camera looks, so stick right is screen
right.

## 5. Vehicle model

Single rigid link `base_link` carrying all mass. The inertial frame coincides
with the link frame, because the wrench is applied at the link origin (the
plugin checks this).

| Parameter | Value | Provenance |
|---|---|---|
| mass | 1.4 kg | kwad.cpp `m` |
| Ixx, Iyy | 0.05 kg m² | kwad.cpp `jx`, `jy` |
| Izz | **0.10 kg m²** | **changed**; kwad.cpp `jz` = 0.24 is non-physical (§9) |
| collision | box 0.10 × 0.10 × 0.03 m at the CoM | ASSUMPTION, envelope of the legacy meshes |
| visuals | legacy meshes (body, cap, motors, propellers, guards) | legacy URDF layout |

Motors and propellers are visual only (no joints, no mass). The legacy model
applies the total rotor wrench at the centre of mass, so rotor links would
not change the dynamics yet. The meshes describe a ~10 cm frame, while the
mass properties describe a 1.4 kg / 0.56 m-arm vehicle. This inconsistency is
inherited and deliberate at this stage.

The mass, inertia and collision box are read by the xacro template from the
same YAML that the plugin loads. The plugin additionally refuses to start if
the link mass/inertia or the world gravity disagree with the YAML.

## 6. Dynamics model

`kwad.cpp` equations (NED/FRD), `u1` = thrust, `(u2, u3, u4)` = torque
command, `o = w1 − w2 + w3 − w4`:

```text
a    = ( −u1 R e_z − diag(kdx, kdy, kdz) v ) / m + g e_z
p_dot = ( q r (Jy − Jz) − jp p o + l u2 ) / Jx
q_dot = ( p r (Jz − Jx) + jp q o + l u3 ) / Jy
r_dot = ( p q (Jx − Jy)          + u4 ) / Jz
```

In Gazebo these are realised as an **external wrench on a rigid body**:

```text
F_ext   = −u1 R e_z − diag(kd) v                (gravity is added by Gazebo)
τ_ext   = ( l u2 − jp p o,  l u3 + jp q o,  u4 )  (FRD)
```

The rigid-body terms (`q r (Jy − Jz)` etc.) come from Gazebo's own Euler
equations, since −ω × Jω supplies exactly those terms. The equivalence is
verified numerically over 500 random states in
`test_legacy_dynamics.cpp::ExternalWrenchReproducesLegacyEquations` and,
through the frame adapter, in `test_enu_adapter.cpp`.

Inherited modelling quirks (kept, documented, **not** validated):

* `u2`, `u3` already contain `l` (mixer), and the dynamics multiply by `l`
  again, giving an effective roll/pitch torque of `kt l² Δω²`. This is
  selectable with `vehicle.legacy_double_arm_torque` (true in
  `legacy_kwad.yaml`, false in `pluto_x_estimated.yaml`).
* The gyroscopic terms are `−jp p o` (roll) and `+jp q o` (pitch). The
  textbook form uses `q` on roll and `p` on pitch.
* Drag is linear in world velocity with per-axis coefficients.
* There are no rotor dynamics. Rotor speed follows the command
  instantaneously.

The offline `LegacyReferenceIntegrator` reproduces `kwad.cpp`'s
explicit-Euler integrator with its state clamps (body rate, attitude). It is
used only for tests and as a comparison baseline, never as the Gazebo
dynamics. Gazebo physics does not clamp state.

## 7. Controller stack

Four stages, one class each, run at 10 ms:

1. `LegacyPositionController` (position hold only) → roll/pitch setpoint, thrust
2. `LegacyAttitudeController` → body-rate setpoint (clamped ±0.87 rad/s)
3. `LegacyRateController` → torque command (clamped ±6.25/±6.25/±2.25 N m)
4. `QuadMixer` → rotor speeds (w² clamped to the configured limits),
   achieved wrench. The mixer is geometric: `rotor.layout: legacy_plus`
   reproduces the `kwad.cpp` allocation exactly (tested), and `magis_quad_x`
   uses MagisV2's motor order M0..M3.

Every loop uses `LegacyPidTerm`, which reproduces the legacy semantics
exactly:
* the integrator accumulates only while |e| < window;
* only the integral **term** is clamped, not the accumulator;
* the derivative acts on a measured signal (velocity, body rate or angular
  acceleration), not on de/dt.

In Gazebo the rate loop's angular acceleration comes from a backward
difference of the body rate over one controller period (`kwad.cpp` read it
from its own integrator).

The yaw setpoint is always 0, and angle errors are not wrapped (legacy
behaviour).

Invalid (non-finite) controller input produces minimum thrust and zero
torque, with a status flag and an error log. It is never propagated.

## 8. Pilot input

There is **no pilot input device** (the joystick path was removed on
2026-09-24). The vehicle flies the mode in the config, `pilot.initial_mode`:
* `position_hold`: fly to `position_hold_target_enu_m` and hold (the
  default);
* `manual_attitude`: constant `hover_thrust_n`, level.

The RC-command interface now exists, together with the MagisV2 firmware as
flight controller and the outer-loop controller host: see
[architecture.md](architecture.md). The legacy stack ignores RC and is
selected with `flight_controller:=legacy` (`legacy_sim.launch.py` does
this).

## 9. Deviations from kwad.cpp

| # | Legacy behaviour | Here | Reason / evidence |
|---|---|---|---|
| D1 | Own Euler integrator + `SetModelState` teleport | Gazebo rigid-body physics + external wrench | Required architecture; same equations (§6, tested) |
| D2 | Wall-clock 10 ms gate (`chrono`) | Simulation-time scheduling | Determinism |
| D3 | `z < 0 → 0` clamp in NED (a ceiling; the vehicle "flew" at positive NED z, i.e. below the ground) | Real ground contact; reference integrator clamps `z > 0` and zeroes downward velocity | Physically consistent world |
| D4 | Published pose swapped x/y and roll/pitch | Single tested frame adapter | Frame correctness |
| D5 | `x_des`, `y_des` overwritten with their yaw-rotated copies every step | Target is an input, never mutated | Bug (repeated rotation for yaw ≠ 0); test `TargetIsNotMutatedAcrossStepsWithYaw` |
| D6, D7 | Joystick handling (toggle per message, inverted throttle) | Joystick removed entirely | Not part of the competition architecture |
| D8 | `jz = 0.24` | `Izz = 0.10` | 0.05 + 0.05 < 0.24 violates the triangle inequality; no rigid body has it and Gazebo refuses to load it. Izz = Ixx + Iyy (flat-plate limit) keeps roll/pitch unchanged. The loader now rejects non-physical inertia. |
| D9 | Position/velocity rotated by the **full attitude** before forming the error | Default `error_frame: yaw_only`; `legacy_full_attitude` still available | See "Position-hold instability" below |
| D10 | Manual mode: `u1` persists as the mixer's achieved thrust between joystick messages | Pilot thrust re-applied every tick | Identical unless rotors saturate |
| D12 | Yaw setpoint fixed at 0 (north) | Heading at the first controller step (`hold_initial_heading`) | Lets the vehicle spawn facing the Gazebo camera. Identical to kwad.cpp when the vehicle starts facing north |

### Position-hold instability (D9)

Rotating the absolute position by the full attitude puts a term
`sin(tilt) · altitude` into the horizontal error; for example `y_body`
contains `sin(roll) · z`. Because of D3, `kwad.cpp` only ever flew at
positive NED z, where this term acts as negative (stabilising) feedback.
Above real ground the sign flips, and the feedback gain grows with altitude.
Measured on the reference integrator (60 s position hold from the ground to
(1, 1, alt), final distance to target):

| error_frame | Izz | 0.5 m | 1 m | 2 m | 3 m | 5 m | 8 m |
|---|---|---|---|---|---|---|---|
| legacy_full_attitude | 0.10 | 0.000 | 0.000 | 343 | 146 | 265 | 71 |
| legacy_full_attitude | 0.24 (legacy) | 0.000 | 1277 | 678 | 754 | 347 | 535 |
| yaw_only | 0.10 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 |
| yaw_only | 0.24 (legacy) | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 |

The unchanged legacy stack in the original "below the ceiling" world
converges to its target; above real ground it diverges once the target is
2 m or higher. The regression test
`LegacyFullAttitudeFrameDivergesAtTwoMetres` pins this result.

## 10. Assumptions and parameters

All parameters live in `simulation_engine/pluto_x_core/config/legacy_kwad.yaml`, each
annotated with its legacy name. Every key is required: the loader has no
silent defaults, and reports errors with the full key path. The loader
checks finiteness, signs, ordered limits, the inertia triangle inequality,
that the hover thrust lies within the thrust limits, and the period range.

Explicit assumptions:

* ASSUMPTION: all numeric vehicle/controller values are `kwad.cpp`'s, not
  Pluto X measurements.
* ASSUMPTION: diagonal inertia, CoM at the link origin.
* ASSUMPTION: rotor thrust `T = kt ω²`, yaw torque `Q = kd ω²`, no rotor
  dynamics.
* ASSUMPTION: linear world-frame drag.
* ASSUMPTION: collision box 0.10 × 0.10 × 0.03 m.
* ASSUMPTION: IMU ideal (no noise/bias), 100 Hz, at the CoM. It is published
  for observation only, because the legacy controller uses full-state
  feedback, as `kwad.cpp` did.

## 11. Known limitations

* Not a Pluto X model (parameters, geometry-vs-mass mismatch, "+" mixer on
  an "x"-looking frame).
* Controller uses ground-truth full state, not an estimator. No sensor noise.
* Not MagisV2: no RC/MSP interface, no arming, no DCM estimator, no PWM
  motor model.
* No camera.
* Motors/propellers are not separate links. Propellers do not spin
  visually.
* The yaw setpoint is fixed at 0, and yaw error is not wrapped (a vehicle
  pushed near ±180° yaw will turn the long way).
* The meshes come from a repository without a licence; clarify before
  redistribution.
* `model.config` points at the xacro template. The model must be spawned via
  the launch file (xacro), not `<include>`d directly.

## 12. Validation

Last run 2026-09-24, native build, Gazebo Sim 8.15. Artefacts are written to
`log/validation/` and `log/dynamics/`.

**Unit tests** (`pluto_x_core_tests`): 109 / 109 pass. They cover:
* frame conversions, including equality with the verbatim legacy
  `rotateGFtoBF`;
* the PID term semantics and the cascade signs and saturation;
* the mixer, for both layouts, including exact equality with the
  `kwad.cpp` allocation;
* the dynamics:
  - the legacy equations and wrench/Newton–Euler equivalence in NED and ENU;
  - drag;
  - the gyroscopic term;
  - spin-up torque and angular momentum conservation;
* motor lag, battery, wind, sensor error and the delay line;
* the `SimulatedVehicle` pipeline:
  - transparency with all effects off;
  - determinism;
  - latency;
  - closed-loop hold with all effects, and without state clamps;
* the configuration loader, covering every error class.

**End-to-end** (`simulation_engine/scripts/validate_legacy_sim.sh`):

| Config | Hold, all effects (last 10 s mean) | Determinism | Gazebo vs reference | Hold, effects off |
|---|---|---|---|---|
| `pluto_x_estimated.yaml` | 0.264 m (tolerance 0.3 m: battery altitude offset + gusts) | identical | RMS 0.0035 m | 0.0001 m |
| `legacy_kwad.yaml` (no effects) | 0.0000 m | identical | RMS 0.0071 m | 0.0000 m |

"Effects off" means battery, sensor error, wind and latency off. The
dynamics are kept (motor lag, drag, spin-up torque).

What this validates:
* the controller drives a Gazebo rigid body through `AddWorldWrench` only;
* the frame conversions are consistent end to end;
* the simulation is deterministic;
* Gazebo and the independent offline integrator agree to millimetres.

What it does **not** validate: anything about the real Pluto X. Every
parameter is an estimate or a legacy value.

### Dynamics and frame checks (`simulation_engine/scripts/check_dynamics.sh`)

These run in Gazebo with the estimated Pluto X parameters. Battery, sensor
error, wind and latency are off; motor lag, drag and spin-up torque are on.
The vehicle spawns facing +x.

| Scenario | Check | Result |
|---|---|---|
| rest (motors off) | IMU specific force = (0, 0, +g) in FLU | (−0.000, 0.000, 9.810) m/s² |
| free fall from 5 m | matches v = v_t tanh(g t / v_t) with body drag | 4.7 mm worst error over a 4.74 m drop |
| target 1 m ahead | nose down (FRD pitch < 0), moves +x only | pitch −0.222 rad; dx +0.948, dy −0.000 m |
| target 1 m right | right side down (FRD roll > 0), moves −y only | roll +0.222 rad; dx +0.000, dy −0.948 m |
| heading north from east | turns counter-clockwise from above, settles | IMU yaw rate +0.84 rad/s; final 1.5697 rad (≤ 0.001 rad error) |
| all manoeuvres | IMU gyro (FLU) = rate of ground-truth attitude | worst 0.030 rad/s (finite-difference noise) |

Together these confirm:
* **World frame:** ENU, gravity −z at 9.81 m/s².
* **Body frame:** FLU in Gazebo, x forward, z up. The NED/FRD
  controller/dynamics convert through the tested adapter.
* **Signs:** every attitude and translation sign is correct.
* **Yaw:** the yaw dynamics and yaw torque path work, including the heading
  turning direction.

## 13. Build, run, test

Native, from the repository root (ROS 2 Humble + Gazebo Harmonic;
`simulation_engine/scripts/dev.sh` runs the same commands in a container,
optional: set `PLUTO_DEV_IMAGE` to your image).

```bash
source /opt/ros/humble/setup.bash && colcon build
```

```bash
colcon test --packages-select pluto_x_core
```

```bash
./build/pluto_x_core/pluto_x_core_tests
```

Dynamics and coordinate-frame checks (≈ 2 min, headless):

```bash
simulation_engine/scripts/check_dynamics.sh
```

Full end-to-end validation (≈ 4 min, headless):

```bash
simulation_engine/scripts/validate_legacy_sim.sh
```

Interactive, with the Gazebo GUI (flies the configured position-hold
target):

```bash
source install/setup.bash && ros2 launch pluto_x_bringup legacy_sim.launch.py
```
