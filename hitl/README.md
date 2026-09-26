# HITL: flying the participant controller on the real Pluto X

Status 2026-09-25: **written and tested against the simulator only.** The
drone had not been delivered. The code is complete, but every part marked
UNVERIFIED below has to be checked on the drone, propellers off first.

## What it is

This directory is organisers' code. Participants never edit it: their
controller in `outerloop_controller/` reaches it through
`Drone(backend='hardware')`, which is what `my_controller.py --hardware`
does. It provides the same `Drone` interface participants use in the
simulator, backed by the real drone:

```text
hitl/
  avatic_hitl/hardware.py   HardwareDrone: the same methods as the simulator's Drone
  avatic_hitl/msp.py        MSP v1 client for the MagisV2 firmware
  avatic_hitl/video.py      camera module stream (plutocam, then H.264 decode)
  tests/test_msp.py         codec tests: python3 -m pytest hitl/tests
  tests/test_safety.py      safety caps, the failsafe rule, stick mapping = simulator's
  tests/test_landing.py     land() against a vertical model (fake clock)
```

```text
my_controller.py ──► avatic_drone.Drone(backend='hardware') ──► hitl/avatic_hitl/hardware.py
                        ├─ commands   MSP_SET_RAW_RC, 50 Hz, over TCP        hitl/avatic_hitl/msp.py (our own MSP v1 client)
                        ├─ telemetry  MSP_STATUS/ATTITUDE/ALTITUDE/ANALOG, 20 Hz
                        └─ camera     H.264 from the camera module ─► ffmpeg ─► NumPy frames   hitl/avatic_hitl/video.py (plutocam)
```

A controller runs unchanged on either backend:

```bash
python3 outerloop_controller/my_controller.py              # simulator
python3 outerloop_controller/my_controller.py --hardware   # real Pluto X
```

With `--hardware` the laptop needs no ROS: only Python 3, NumPy, `ffmpeg`
(for video) and `pip install plutocam`.

## Connection

| Setup | MSP address | Video |
|---|---|---|
| Camera module fitted (competition) | `192.168.0.1:9060` (default) | camera module, via `plutocam` (default) |
| Drone Wi-Fi, no camera | `--host 192.168.4.1 --msp-port 23 --video none` | none |

With the camera module fitted, the drone's own Wi-Fi is off. Join the
camera module's network: control and video both go through it.

## The protocol (from the MagisV2 source)

Every message layout comes from `firmware/magisv2/src/main/io/serial_msp.cpp`.
Several differ from generic MultiWii/Cleanflight documentation:

| Message | Layout used |
|---|---|
| `MSP_SET_RAW_RC` 200 | 8 × u16 µs, AETR1234. The stick mapping is identical to the simulator's. |
| `MSP_STATUS` 101 | u16 **Pluto status flags** (armed = bit 8, ok to arm = bit 7, low battery 3, crash 2, signal loss 5), u16 I²C errors, u16 sensors, u32 active modes (ARM = bit 0), u8 profile |
| `MSP_ATTITUDE` 108 | roll, pitch in decidegrees (firmware pitch + = nose down; reported to the controller as + = nose up, as in the simulator), heading in degrees |
| `MSP_ALTITUDE` 109 | i32 estimated altitude in cm, i16 vario |
| `MSP_ANALOG` 110 | u16 battery **mV**, current, mAh drawn and remaining, u8 state of charge, u8 warning |
| `MSP_SET_COMMAND` 217 | **u16**, 2 = land |

The firmware reads `MSP_SET_COMMAND` as two bytes (`read16`). The
`plutocontrol` library sends one, which works only because of leftover
buffer bytes. Our client sends two.

## Safety behaviour

- **Command caps** (both backends, `avatic_drone.types.SafetyLimits`):
  roll and pitch stick ≤ 0.6 (the firmware limits the angle to 20°,
  reached at about 0.45 stick), yaw rate ≤ 0.8 (about
  62°/s), throttle ≤ 0.95. Larger values are clipped, with one warning per
  axis.
- **Altitude ceiling 2.5 m** (baro, both backends): above it the throttle
  is limited to 0.9 × the hover estimate (the estimate itself at most 0.8), so the drone descends. It relies on the
  firmware's altitude estimate, which the firmware re-zeroes in
  near-free-fall (a hard throttle cut in flight; docs/architecture.md,
  firmware finding 5): after that the ceiling is offset by the height at
  the reset. Fly in a net or a room with a ceiling-height margin on the
  first sessions.
- **End of run** (25 s from arming, the official limit): the backend starts landing by itself,
  even if the script never calls `close()`; `close()`, the end of the
  script (`atexit`), Ctrl-C, closing the terminal (SIGHUP) or `kill`
  (SIGTERM) land too.
- **Landing:** level sticks and a 0.3 m/s descent-rate control on the
  throttle, around a hover guess (the learned estimate, or the last
  throttle if higher, capped at 0.8) that is corrected on the way down.
  Below 0.3 m: 0.2 m/s under a throttle cap that only ramps down, so the
  landing can never climb back up; the motors disarm when the cap falls
  below 0.35. On the pad (low throttle, below 0.3 m) it disarms at once.
  Disarms after 30 s whatever happens, and at once if the link is lost (the
  firmware's RC-loss failsafe then takes over). Tested against the
  simulated firmware through the MSP bridge (1 m to the ground in 8 s,
  touchdown 0.26 m/s), and in `tests/test_landing.py` with hover estimates
  up to 0.2 off (touchdown under 0.4 m/s). It does **not** use the
  firmware's land command: that forces 1300 µs, a descent rate only in
  altitude hold, and altitude hold assumes a 1500 µs hover
  (docs/architecture.md, firmware finding 7) - unverified on the real drone.
- **Ctrl-C while landing, or a second Ctrl-C:** immediate disarm
  (emergency stop). The drone falls.
- **No `send_command()` for 0.5 s:** level sticks and, if the drone was
  flying (last throttle > 0.3, or baro altitude > 0.3 m), the estimated
  hover throttle (bounded to 0.6-0.85); otherwise throttle 0 (the same failsafe as the simulator). A
  flying drone may slowly sink.
- **Telemetry stops for 1 s:** a warning is printed (Wi-Fi problem); after
  2 s, or if the socket fails, `running()` returns False so the control
  loop ends and the script lands. A send that times out (the TCP buffer
  full: seconds of stall) counts as a lost link, because a partly sent
  frame would desynchronise the stream. If the RC stream stops, the
  firmware's own signal-loss failsafe takes over (unverified on the bench:
  check it).
- **Camera frames** are writable NumPy copies (the same as the simulator).
- **No scoring on hardware:** `drone.arena()` is a local 25 s run timer
  only; the judges count the balloons.

## Testing without the drone: the MSP bridge

`simulation_engine/pluto_x_ros/scripts/msp_sim_bridge.py` puts the
simulated Pluto X behind a real MSP-over-TCP link and serves the simulated
camera as H.264 over TCP. The hardware backend then flies the simulated
drone through the same byte-level protocol as the real one. The bridge's
MSP encoding is written independently of the client's, so the two check
each other.

```bash
ros2 launch pluto_x_bringup competition.launch.py msp_bridge:=true
```

```bash
python3 outerloop_controller/my_controller.py --hardware --host 127.0.0.1 --video tcp://127.0.0.1:9061
```

Result on 2026-09-25:

| Check | Result |
|---|---|
| Connect and decode telemetry | battery 4.20 V, heading 91° (facing east, correct) |
| Ready to arm, arm over MSP | armed 0.2 s after the request |
| Barometric hover from MSP telemetry | 1.02–1.03 m |
| Video: H.264 over TCP → ffmpeg → NumPy | 1280 × 720, about 19 frames/s |
| `yaw_rate +0.3` for 1 s | +21° clockwise |
| Land at end of run | steady 0.25–0.3 m/s descent, touchdown, then disarm |
| `my_controller.py --hardware` (the unchanged template) | connects, arms, flies the run, lands, exits 0 |

MSP codec unit tests: `python3 -m pytest hitl/tests`.

The backend's own landing needs no emulation: it is ordinary RC, flown
by the simulated firmware. The bridge still emulates `MSP_SET_COMMAND`
land (the simulator's firmware receives RC only), the way the firmware
lands: 50 cm/s in altitude hold, a raw 30 % throttle (a drop) without it,
disarm once no longer descending.

## Organisers: first hardware session checklist (UNVERIFIED on hardware)

Propellers **off** for steps 1–5.

1. Join the camera Wi-Fi, then run a script that only connects and prints
   `drone.get_telemetry()` and `drone.flags()` (`flags()` exists only on the
   hardware backend). Check that the battery
   voltage matches a multimeter, and that the heading changes when you
   rotate the drone by hand.
2. Tilt the drone by hand. Right side down should give positive
   `roll_deg`; nose up should give positive `pitch_deg`.
3. Check the video: frames arrive, and measure their rate and delay (film
   a stopwatch shown on screen next to the stream).
4. Arm, then disarm: `drone.arm()` should succeed within 5 s, and
   `drone.disarm()` should stop the motors.
5. Pull the Wi-Fi (or stop the script with the drone armed on the bench)
   and watch what the firmware's RC-loss failsafe does with the motors.
6. With propellers on, in a safe open space: take off to about 0.5 m, hold
   briefly, then land via `close()`. **Measure the hover throttle** (the
   `commands` the template sends while hovering). The simulator assumes
   0.76 (1760 µs); the firmware's altitude hold assumes 0.5 (1500 µs). The
   answer decides how the simulator's thrust model is corrected
   (docs/architecture.md, firmware finding 7). Retune `HOVER_THROTTLE`.
7. Check that the double-Ctrl-C emergency disarm works.
