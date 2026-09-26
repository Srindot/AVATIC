# Your controller

This folder is yours. Write your balloon-popping code in
**`my_controller.py`**, in the `step()` method of `MyController`.

```text
outerloop_controller/
  my_controller.py          YOUR controller: start here (it climbs to 1 m and hovers)
  examples/hello_drone.py   example: take off, turn, count balloon colours in the camera
  avatic_drone/             the drone interface (do not change it)
```

**New here? Read the [participant guide](../docs/participants/README.md).**

## Quick reference

Run (from the repository folder, after the two `source` lines):

```bash
ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/my_controller.py
```

Your `step()` is called 20 times per second:

```python
def step(self, frame, telemetry, t):   # t = seconds since arming (0 ... 15)
    ...
    return Command(roll=0.0, pitch=0.0, yaw_rate=0.0, throttle=0.76)
```

| Command | range | meaning |
|---|---|---|
| `roll` | −1 … 1 | tilt right (+), so the drone slides right (0.2 ≈ 7°, max 20°) |
| `pitch` | −1 … 1 | nose down (+), so the drone flies forward |
| `yaw_rate` | −1 … 1 | turn clockwise (+), about 77°/s per unit (capped at 0.8 ≈ 62°/s) |
| `throttle` | 0 … 1 | lift; about **0.76 hovers** |

| Input | what it is |
|---|---|
| `frame.image` | camera picture, NumPy (720, 1280, 3), RGB; `frame.seq` = picture number |
| `telemetry.altitude_m` | height from the barometer (m) |
| `telemetry.heading_deg` | 0 = north, 90 = east (start), clockwise |
| `telemetry.roll_deg`, `.pitch_deg` | tilt (+ = right side down / nose up) |

| Balloon | green | blue | yellow | red |
|---|---|---|---|---|
| Points | +100 | +50 | +25 | **−75** |

15 s from arming. The best possible score is 350.

**Remember:** keep the throttle smooth and above about 0.5 in the air,
don't use `altitude_hold`, and keep your files in this folder.

On the real drone: `python3 outerloop_controller/my_controller.py --hardware`
([guide page 7](../docs/participants/7_real_drone.md)).
