# 3. Writing your controller

## The template

Open `outerloop_controller/my_controller.py`. It has three sections:

1. **Settings**: numbers you may change (control rate, hover throttle).
2. **`MyController`**: **your code goes here.**
3. **Runner**: connects to the drone, arms it and calls your controller.
   You do not need to change it.

The runner calls your `step()` method 20 times per second:

```python
class MyController:
    def step(self, frame, telemetry, t):
        # frame:     the latest camera image (or None at the very start)
        # telemetry: the flight controller's readings
        # t:         seconds since arming (0 ... 25)
        ...
        return Command(roll=0.0, pitch=0.0, yaw_rate=0.0, throttle=0.76)
```

`step()` looks at the camera and the readings, decides, and returns **one
command**. That is the whole job. As shipped, the template climbs to 1 m and
hovers.

If your `step()` crashes, the error is printed, your commands stop, and a
safety system keeps the drone level until the end of the run.

## Commands: how to fly

A `Command` has four numbers, like the two sticks of a remote control:

| field | range | what it does |
|---|---|---|
| `roll` | −1 … +1 | **tilt sideways.** + = tilt right, so the drone slides right |
| `pitch` | −1 … +1 | **tilt forwards.** + = nose down, so the drone flies forward |
| `yaw_rate` | −1 … +1 | **turn on the spot.** + = turn clockwise (right), seen from above |
| `throttle` | 0 … 1 | **lift.** About **0.76 hovers**; more climbs, less sinks |

**Fields you leave out are 0, including `throttle`.** `Command(pitch=0.2)`
alone means "no lift": always set the throttle.

Useful numbers:

- **Tilt:** `pitch` or `roll` 0.2 is about 7°, 0.4 is about 16°, and the
  drone never tilts more than 20° (reached at about 0.45). A small tilt of
  0.1–0.25 is plenty in the small arena.
- **Turning:** about 77° per second per unit of `yaw_rate`, capped at 0.8
  (about 62°/s). 0.5 is about 38°/s, so it turns
  half a circle in about 5 s.
- **Throttle:** keep it near 0.76 and correct it with the altitude (the
  template shows how). Tilting needs a little more throttle to keep height.
- `roll` and `pitch` hold an **angle**, not a speed. Hold `pitch=0.2` and the
  drone keeps speeding up forward until air drag limits it. Set it back to
  0 and it coasts, slowing down only gradually. To stop quickly, tilt the
  other way for a moment.

**Safety limits** (the same in the simulator and on the real drone):

- roll and pitch are cut to ±0.6, `yaw_rate` to ±0.8, throttle to 0.95;
- above **2.5 m** the throttle is lowered so the drone comes down;
- if you send no command for 0.5 s, the drone levels itself and holds about
  hover throttle (if it is still on the ground: throttle 0, it never takes
  off by itself).

**Two things to avoid:**

- **Do not cut the throttle suddenly in the air** (for example to 0.3). The
  flight controller then thinks the drone was thrown, resets its altitude
  reading to 0 in mid-air, and your altitude control goes wrong. Keep the
  throttle above about 0.5 while flying and change it smoothly.
- **Do not hard-code the hover throttle.** 0.76 is the simulator's value; the
  real drone's is not measured yet and changes as the battery drains. Let
  your height loop correct it (for example with an integral term).

## Height control is yours

There is no altitude hold: the throttle is thrust, not a height or a climb
rate. Holding and changing height is part of the challenge.

- The template's height loop (`step()`, the lines using `HOVER_THROTTLE`)
  holds 1 m and is a working starting point. You may keep it or improve it.
- **Tilting costs lift.** Part of the thrust now pushes sideways: at 16° the
  drone needs about 4 % more throttle, at 20° about 6 %, or it sinks.
  Dividing by cos(tilt) makes up for it.
- `altitude_m` is noisy (about ±0.3 m). Your loop reacts to that noise, so
  filter it or keep the gains gentle; the template estimates the climb rate
  from successive readings.

## Telemetry: what the drone tells you

`telemetry` (a `Telemetry` object):

| field | meaning |
|---|---|
| `altitude_m` | height above the take-off point, from the barometer (usually within 0.3 m of the true height) |
| `heading_deg` | direction the nose points: 0 = north, 90 = east (the start), clockwise |
| `roll_deg` | sideways tilt, + = right side down |
| `pitch_deg` | forward tilt, **+ = nose up** (note: a positive `pitch` command gives a negative `pitch_deg`) |
| `battery_v` | battery voltage |
| `armed` | motors running |
| `ready_to_arm` | the flight controller has finished its start-up calibration |
| `time_s` | when this reading was taken |

There is **no position and no speed.** You can estimate your climb rate
from how `altitude_m` changes (the template does), and your heading
changes from `heading_deg`.

## The camera frame

`frame` (a `Frame` object, or `None` before the first picture):

| field | meaning |
|---|---|
| `image` | NumPy array, shape (720, 1280, 3), `uint8`, **RGB** order |
| `seq` | picture number: the same number means the same picture as last time |
| `time_s` | when it was taken |
| `width`, `height` | 1280, 720 |

The camera makes about 18 pictures per second and `step()` runs 20 times
per second, so often you get the same picture twice. Check `frame.seq`, as
the template does, to skip work you already did. For OpenCV, convert
first: `bgr = cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR)`. OpenCV is in
the dev container; in a local setup install it with
`sudo apt install python3-opencv` (not `pip install opencv-python`, which
brings NumPy 2 and breaks ROS).

More about the camera: [4. Camera and directions](4_camera_and_directions.md).

## Keeping memory between steps

`step()` is called again and again: store anything you want to remember on
`self` (set it up in `__init__`). For example: which balloon you are
chasing, when you last saw it, the last altitude (for the climb rate), or
a state such as `'take off'`, `'search'`, `'approach'`.

## More functions (optional)

The runner uses these; you can also write your own runner. They come from
`from avatic_drone import Drone, Command`:

| | |
|---|---|
| `drone = Drone()` | connect to the simulator |
| `drone.wait_until_ready()` | wait until the flight controller is ready (~4 s) |
| `drone.arm()` | start the motors: **the 25 s clock starts now** |
| `drone.disarm()` | stop the motors at once (the drone falls if flying) |
| `drone.running()` | `False` once the 25 s are over |
| `drone.get_frame()`, `drone.get_telemetry()` | the latest picture and readings |
| `drone.send(command)` or `drone.send_command(roll=..., pitch=..., yaw_rate=..., throttle=...)` | fly |
| `for step in drone.loop(hz=20): ...` | repeat at a steady rate until the time is up |
| `drone.arena()` | `.score`, `.time_remaining_s`, `.finished`, `.events` (the pops so far) |
| `drone.time()`, `drone.sleep(s)` | simulation time |
| `drone.close()` | stop and disconnect |

`drone.arena().score` shows your points while flying. Use it for
printing, not in your algorithm: the real drone has no scoreboard.

Only **one** program may control the drone at a time.

**Next:** [4. Camera and directions](4_camera_and_directions.md)
