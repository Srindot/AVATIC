# Pluto X vehicle parameters — sources, estimates, and what to measure

File: `simulation_engine/pluto_x_core/config/pluto_x_estimated.yaml` (default config of
`legacy_sim.launch.py` since 2026-09-24).

**Status: estimated, not measured.** No official datasheet publishes mass
properties, geometry or thrust. A web search (2026-09-24) found the published
figures in §1; everything else is derived in §2–§6. Replace each [EST] value
with a measurement (§8) when a Pluto X is available.

## 1. Published figures [SRC]

| Item | Value | Source |
|---|---|---|
| Mass | ~60 g | [Drone Deliver review](https://dronedeliver.co.uk/pluto-x-drone-review/); store listing |
| Pluto WiFi Camera Module (add-on, UniBus) | 8 g with casing; video 720p / 1080p; live stream ~18 FPS (25 FPS to SD); min supply 3.4 V; 30 m range. No field of view given. | [supplier listing](https://sparshhacks.com/product/wifi-camera-module-drone-accessory/) (checked 2026-09-24) |
| Flying mass used (drone + camera) | **68 g** | sum of the two rows above |
| Frame | "90mm frame", 3D-printed Nylon 6 | [DroneBot Workshop](https://dronebotworkshop.com/plutox-introduction/); [Drona Aviation](https://dronaaviation.com/plutox/) |
| Motors | brushed coreless, **7 × 20 mm ("720")**, 3.7 V | [spare-part listing](https://www.amazon.in/Pluto-Micro-Coreless-Propeller-Quadcopter-Metallic/dp/B07ZWZHQVP). **Conflict:** [Drona's motor page](https://www.dronaaviation.com/product/brushed-coreless-motors) says "1020 motors" |
| 720 motor | 45,000 rpm @ 3.4 V, 140 mA, ~5 g, Ø1 mm shaft | [Handson 720 datasheet](https://handsontec.com/dataspecs/motor_fan/Coreless%20Motor%20720.pdf) (generic 720, not Drona-specific) |
| Propellers | 55 mm, 2 CW + 2 CCW | spare-part listing |
| Battery | 1S 3.7 V 600 mAh 20C LiPo | [Drona store](https://www.dronaaviation.com/product/lipo-battery/) |
| Payload | 15 g | [Fab.to.Lab](https://www.fabtolab.com/drona-aviation-plutoX-drone-innovation-open-sourced); DroneBot Workshop |
| Flight time | 9–15 min | Fab.to.Lab; review |
| MCU | STM32F303, 72 MHz | Fab.to.Lab (matches MagisV2 `STM32F303xC`) |
| Camera | 720p, 1 MP, Wi-Fi | Drona Aviation; review |
| Motor layout | QUADX, M0 rear-right, M1 front-right, M2 rear-left, M3 front-left | MagisV2 `flight/mixer.cpp` `mixerQuadX` |

A JSON "spec sheet" produced by a language model was **rejected**:
* Its sensors (MPU9250, MS5611) contradict the MagisV2 drivers (ICM-20948,
  ICP10111).
* Its `kf` and `max_rpm` values give 0.41 N total thrust for a 0.59 N
  vehicle, which cannot hover.

Its `kf` value turned out consistent with §4 (see there).

## 2. Geometry [SRC + EST] (revised 2026-09-24)

Published figures:

| Item | Value | Source |
|---|---|---|
| Overall footprint | 16 × 16 cm (×4 cm); 213 mm diagonal | [Beyond Sky dealer listing](https://beyondsky.xyz/pluto-x-educational-drone-standard-kit-328/); "16 × 16 × 4 cm" product dimensions in store listings |
| Frame | 10 × 10 cm, 12 g | [Fab.to.Lab parts listing](https://www.fabtolab.com/drona-aviation-plutoX-drone-innovation-open-sourced) |
| Propellers | 55 mm diameter, 25 mm pitch | same |
| Motors | 7 × 20 mm coreless | same |
| Battery | 37 × 24 × 8 mm, 1S 600 mAh | same |

(A search summary also returned "159 × 203 × 55 mm unfolded": those are
the DJI Mini 2's dimensions, not the Pluto X's, and were not used.)

Used: **`arm_length_m = 0.065`**, with motors at (±46, ±46) mm. This agrees
with the 10 × 10 cm frame (motors near its corners), with the legacy CAD
(motors at ±46 mm), and with the overall size: motor 46 mm + prop radius
27.5 mm + guard ≈ 77 mm half-width, i.e. 153 mm in the model against
160 mm published. The adjacent-motor spacing of 92 mm leaves 37 mm between
the 55 mm propellers.

**Correction:** until 2026-09-24 the model used 0.045 m, reading
"90 mm frame" as the motor-to-motor diagonal. That gave 30 % less roll and
pitch torque and half the inertia. The visuals were already at ±46 mm, so
physics and visuals disagreed.

Model envelope, measured from the rendered meshes: 153 × 153 × 47 mm (guards
18–77 mm from the centre on each axis). The collision box (ground contact)
is 150 × 150 × 45 mm. The propellers are drawn as 55 mm translucent discs,
because the legacy propeller mesh is only 20 mm long. Balloon contact uses
the sphere outline in docs/arena.md.

## 3. Inertia [EST]

Lumped-component model about the centre of mass (60 g drone, plus the 8 g
camera module, 68 g in total):

| Component | Mass | Model | Ixx | Iyy | Izz |
|---|---|---|---|---|---|
| 4 motors + props | 4 × 5 g | points at (±46, ±46) mm | 4.23e-5 | 4.23e-5 | 8.46e-5 |
| Frame | 12 g [SRC] | two 130 mm rods on the diagonals (10 × 10 cm frame) | 8.5e-6 | 8.5e-6 | 1.69e-5 |
| Battery | 16 g | 37 × 24 × 8 mm box [SRC size], 10 mm below the CoM | 2.5e-6 | 3.5e-6 | 2.6e-6 |
| Board + electronics | 12 g | 40 × 40 mm plate, 10 mm above the CoM | 2.8e-6 | 2.8e-6 | 3.2e-6 |
| Camera module | 8 g [SRC] | 20 mm forward, 15 mm above the CoM [EST], plus its own box | 2.2e-6 | 5.6e-6 | 3.9e-6 |
| **Sum** | 68 g | | **5.83e-5** | **6.27e-5** | **1.11e-4** |

Used: Ixx = 5.8e-5, Iyy = 6.3e-5, Izz = 1.11e-4 kg m² (triangle
inequality: 1.21e-4 ≥ 1.11e-4). Before the geometry correction (§2) it was
3.2e-5 / 3.6e-5 / 6.0e-5. ASSUMPTION: the camera's ~2.4 mm forward and
~1.8 mm upward shift of the centre of mass is neglected; the wrench is
applied at the link origin. Uncertainty: about ±40 %, because the
component masses (other than the frame, camera and total) are guesses.

## 4. Thrust and rotor model [EST]

No thrust data exists for the 720 motor with a 55 mm propeller. The estimate:

* **Per-motor maximum thrust 0.26 N (26.5 gf).** Constraints: take-off with
  the 15 g payload (75 g) needs more than 0.74 N total. Brushed micro quads
  of this class typically fly at T/W ≈ 1.5–2. 4 × 0.26 N = 1.04 N gives T/W
  1.77 at 60 g and 1.41 at 75 g.
* **Loaded maximum speed 4163 rad/s (39,750 rpm).** The 720 datasheet gives
  45,000 rpm unloaded at 3.4 V; loaded speed with a propeller is lower.
* `kt = T_max / w_max² = 1.5e-8 N s²`. This is the same value as the
  rejected JSON's `kf`, whose own `max_rpm` was therefore the inconsistent
  field.
* `kd = 2.4e-10 N m s²`, i.e. Q/T = 0.016 m, typical for small propellers.
* Hover at 0.589 N means each rotor runs at 57 % of maximum thrust, 75 % of
  maximum speed.
* `rotor_inertia = 4e-8 kg m²` (revised from 3e-7). A 55 mm propeller of
  ~0.35 g with its mass concentrated towards the hub (uniform rod
  m L²/12 ≈ 9e-8, halved for the taper) plus a ~1e-9 coreless rotor cup.
  The first estimate (1 g blade, uniform rod) was ~7× too high. It also
  failed a closed-loop check: with the production MagisV2 firmware in the
  loop (pluto_x_magisv2 flight-controller test), the spin-up reaction
  torque of rotors with J ≥ 6e-8 kg m² (motor lag 30 ms) drives the
  firmware's yaw rate loop into a ±200 µs limit cycle and the vehicle
  spins; J ≤ 4e-8 is stable. The real firmware flies the real vehicle, so
  the real value lies in the stable region. The margin is thin: J_rotor,
  the motor lag and Izz should be measured together (yaw step test, §8).

Uncertainty: thrust ±30 %. This is **the most important value to measure**
(§8).

## 5. Drag [EST]

Model `rotor_and_body` (`dynamics/aerodynamic_drag`), acting on the
air-relative velocity:

* **Rotor drag** (in the rotor plane): F = −k_rd · Σω_i · v_body,xy with
  k_rd = 6.4e-6 N s/(m rad/s). Calibration: 0.08 N per m/s of horizontal
  speed at hover (Σω = 4 × 3132 rad/s), i.e. a horizontal velocity time
  constant m/c ≈ 0.75 s and ≈ 7 m/s steady speed at 45° tilt. It vanishes
  when the rotors stop.
* **Body drag** (quadratic, per body axis): F = −½ ρ C_dA |v| v with
  C_dA = 0.0044 m² (x, y) and 0.0096 m² (z), ρ = 1.225 kg/m³. This gives a
  vertical terminal velocity of 10 m/s with the motors off.

**Why this replaced the single linear coefficient:** the earlier estimate
of 0.05 N s/m vertical drag acted even with the motors off. It made a free
fall visibly slow (τ = 1.2 s; −5.7 m/s after 0.8 s instead of −7.8 m/s).
Measured in Gazebo after the change, a motors-off fall from 5 m matches the
analytic solution with body drag to 4.7 mm over the 4.7 m drop
(`simulation_engine/scripts/check_dynamics.sh`).

## 6. Controller values [CTRL]

The controller is still the legacy `kwad.cpp` cascade. Its gains were
**rescaled, not tuned**, so that each loop's closed-loop behaviour is
unchanged:

| Loop | Output unit | Scaling |
|---|---|---|
| position north/east | rad | none |
| position down | N | × m_new / m_legacy = 0.068 / 1.4 |
| attitude | rad/s | none |
| rate roll / pitch | N m | × J_new / (J_legacy / l_legacy) = 5.8e-5 and 6.3e-5 / (0.05 / 0.56) |
| rate yaw | N m | × Izz_new / jz_legacy = 1.11e-4 / 0.24 |
| hover thrust / throttle gain | N | m g; × mass ratio |

Torque and thrust limits come from the rotor model (§4), not from the
legacy numbers. The legacy "arm length applied twice" quirk is off
(`legacy_double_arm_torque: false`), and the mixer uses MagisV2's X layout.

This is **not MagisV2**. The real Pluto X response (angle mode, 285 Hz loop,
altitude hold) will differ until MagisV2 runs in the loop.

## 7. "Close enough" effects (added 2026-09-24)

The goal is not sim-to-real. It is that teams meet the same kinds of
problems as on the hardware. Each effect is a separate model in
`pluto_x_core` with its own section in the YAML, and can be switched off
there. All values are estimates.

| Effect | Model | Value | Tag | Code |
|---|---|---|---|---|
| Motor lag | first-order rotor speed, exact discretisation | τ = 30 ms | [EST] | `dynamics/rotor_actuator` |
| Battery sag | V = V_oc(soc) − R·I; I = I_hover (T/T_hover)^1.5; thrust × (V/V_ref)² | 600 mAh [SRC], R = 0.08 Ω, I_hover = 3.2 A (80 % of 600 mAh in ~9 min [SRC]), V_ref = 3.7 V, typical LiPo curve | [EST] | `dynamics/battery_model`, `dynamics/propulsion_model` |
| Command latency | pure delay on the pilot command, simulation time | 50 ms | [EST] | `common/delay_line` |
| Sensor/estimator error | constant roll/pitch bias + white noise on attitude; constant bias + white noise on gyro | bias 1° (σ), noise 0.1°; gyro bias 0.05 °/s, noise 0.0018 rad/s (ICM-20948 0.015 °/s/√Hz [SRC] × 50 Hz [EST]) | [EST] | `sensors/state_error_model` |
| Wind | mean + per-axis Gauss–Markov gust, acting through air-relative drag | mean 0, σ 0.3 m/s, τ 2 s | [EST] | `environment/wind_model` |
| Gyroscopic term | physical −ω × h (legacy form selectable) | J_rotor = 4e-8 kg m² | [EST] | `dynamics/legacy_dynamics` |
| Rotor spin-up yaw torque | Σ s_i J_rotor dω_i/dt on the yaw axis (angular-momentum exchange; tested) | J_rotor = 4e-8 kg m²; requires motor lag > 0 (enforced by the loader) | [EST] | `dynamics/propulsion_model` |
| Drag | rotor drag + quadratic body drag (§5) | see §5 | [EST] | `dynamics/aerodynamic_drag` |

**Spin-up torque needs motor lag.** With an instantaneous motor model
(`time_constant_s: 0`), every controller step changes rotor speed within one
physics step. The rotor angular momentum then lands on the body as a yaw
kick about 20× the drag torque. In a test this destabilised yaw, saturated
the rotors and left the vehicle hovering 0.27 m low. It was reproduced
offline, and the legacy state clamps had hidden it. Real motors always have
lag, so the loader rejects that combination.

All random sources are seeded (`state_error.seed`, `wind.seed`), so a run
is repeatable. Change the seed to get a different "drone" (bias) or a
different gust history.

**Effect-by-effect result** (reference integrator, position hold from the
ground to (1, 1, 1) m, 20–60 s window):

| Enabled | Mean error | Max error |
|---|---|---|
| none | 0.000 m | 0.000 m |
| motor lag only | 0.000 m | 0.000 m |
| battery only | 0.199 m | 0.241 m |
| latency only | 0.000 m | 0.000 m |
| sensor error only | 0.000 m | 0.001 m |
| wind only | 0.118 m | 0.246 m |
| all | 0.255 m | 0.376 m |

How to read this table:
* The battery offset is an altitude offset. The legacy altitude loop has no
  integral term (`z_ki = 0`), so a fresh battery's extra thrust holds the
  vehicle about 0.2 m high. A real proportional-only altitude hold behaves
  the same way.
* Latency, motor lag and sensor error barely affect position hold, because
  the hold uses ground-truth position and its integrator absorbs the bias.
* Those three effects appear in **manual** flight, which is what teams fly.
  There, a 1° bias alone gives about 0.17 m/s² of drift, and commands arrive
  50 ms late.

Consequence: a fixed thrust command means "hover at 3.7 V". A fresh
battery therefore climbs slowly with it, and a depleted one sinks, as on
real brushed quads.

Still not modelled:
* the camera and its latency (the delay model is ready);
* ground effect;
* rotor drag and quadratic drag;
* IMU samples feeding a real estimator. MagisV2 in the loop replaces the
  sensor-error stand-in.

## 8. Measurement plan (replaces [EST] values)

| Priority | Measurement | Method | Replaces |
|---|---|---|---|
| 1 | Mass (with/without battery, with camera) | 0.1 g scale | `mass_kg` |
| 1 | Thrust vs throttle (1000–2000), with battery voltage | drone upside-down on the scale, throttle stepped over MSP (`plutocontrol`) | `kt`, `speed_squared_max`, limits |
| 1 | Hover throttle | fly, log over MSP | thrust curve check |
| 1 | Motor positions, prop diameter, camera pose | calipers | `arm_length_m`, camera mount |
| 2 | Centre of mass | balance on an edge, both axes | CoM offset |
| 2 | Inertia | bifilar pendulum (two threads, time the twist period) | `inertia_kg_m2` |
| 2 | Camera intrinsics, FOV, latency | OpenCV checkerboard; film a ms clock | camera model |
| 3 | Motor response time | throttle step, slow-motion video / scale | motor lag (new) |
| 3 | Battery sag | log voltage over a full flight | battery model (new) |
| 3 | Motor spin directions | observe with the props off | confirms the MagisV2 QUADX assumption |
