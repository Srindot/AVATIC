# Outer-loop controller: your code goes here

You write the algorithm that finds balloons in the camera image and flies
the Pluto X into them. Everything else is already running inside the
simulator: the drone's physics, the real MagisV2 flight-controller firmware
(it stabilises the drone), the camera and the arena with its scoring.

```text
outerloop_controller/
  my_controller.py      <- YOUR controller: start here
  examples/hello_drone.py   take off, hold 1 m, turn, look at the camera
  avatic_drone/         the interface (read it, don't change it)
```

You only use plain Python (NumPy, OpenCV, anything). You never need to
write ROS code.

## Run

Terminal 1, the simulator (Gazebo view plus RViz with the drone's camera):

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch pluto_x_bringup competition.launch.py
```

Terminal 2, your controller:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 outerloop_controller/my_controller.py
```

Every launch places the balloons at new random positions. It prints the
seed; pass `arena_seed:=<seed>` to fly the same layout again. Add
`headless:=true rviz:=false` to run without windows (faster, e.g. for
batch testing).

## The task

| Balloon | Points |
|---|---|
| green | +100 |
| blue | +50 |
| yellow | +25 |
| **red** | **−75, avoid it** |

A balloon pops when the drone touches it. You have **15 s from the moment
you arm**. When the time is up the simulation pauses and the result is
printed in terminal 1.

## The interface (`from avatic_drone import Drone`)

```python
drone = Drone()               # connect to the running simulator
drone.wait_until_ready()      # flight controller calibrated (~4 s after start)
drone.arm()                   # motors on; the 15 s start now

for step in drone.loop(hz=20):     # runs until the time is up
    frame = drone.get_frame()      # camera
    tel = drone.get_telemetry()    # flight-controller estimates
    drone.send_command(roll=0.0, pitch=0.0, yaw_rate=0.0, throttle=0.76)

drone.close()
```

**Camera:** `drone.get_frame()` returns a `Frame` with:
- `.image`: NumPy array, 720 × 1280 × 3, uint8, RGB (for OpenCV, convert
  with `cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR)`);
- `.time_s`: when the frame was taken;
- `.seq`: the frame number. The camera (about 18 fps) is slower than a
  typical control loop, so compare `seq` to skip frames you have already
  processed.

The camera is about 18 frames per second, forward-facing, about 80° field
of view. In the simulator frames arrive without delay; on the real drone
expect 150–400 ms of video delay.

**Telemetry:** `drone.get_telemetry()` returns the flight controller's own
estimates:

| field | meaning |
|---|---|
| `roll_deg` | + = right side down |
| `pitch_deg` | + = nose up |
| `heading_deg` | clockwise from north |
| `altitude_m` | barometric, relative to take-off |
| `battery_v` | battery voltage |
| `armed` | motors armed |

There is no position or velocity: the real drone doesn't have them either.

**Commands:** `drone.send_command(roll, pitch, yaw_rate, throttle)`

| value | range | meaning |
|---|---|---|
| `roll` | −1 … 1 | **bank angle**: + = bank right (about 32° at 1.0) |
| `pitch` | −1 … 1 | **tilt angle**: + = nose down, i.e. move forward |
| `yaw_rate` | −1 … 1 | **turn rate**: + = clockwise (about 77 °/s at 1.0) |
| `throttle` | 0 … 1 | thrust: about 0.76 hovers (it drifts as the battery drains) |

Roll and pitch are angles the firmware holds; only yaw is a rate. This is
exactly what the real Pluto X accepts. If you stop calling `send_command`
for 0.5 s, a failsafe levels the sticks, stops the yaw and sets the
throttle to an estimate of hover (the average of your recent throttle)
until your next call.

Use only one `Drone()` per run: two scripts sending commands at once would
fight over the drone.

**Arena and time:**
- `drone.arena()` returns the `.score`, `.time_remaining_s`, `.finished`
  and `.events` (the pops) of the run.
- `drone.time()` is the simulation time; `drone.sleep(s)` and
  `drone.loop(hz)` use it.

## Tips

- **Take off straight up first.** Tilting or yawing while still on the
  ground presses the drone into the ground instead of lifting it.
- **Turn to centre a balloon, then move towards it.** Slow, smooth
  corrections work better than fast ones, especially once video delay is
  added.
- **Leave the red balloons alone.** Touching one costs 75 points.

## Do not edit

`simulation_engine/` is the simulator (physics, firmware, arena, scoring).
Your submission is `outerloop_controller/my_controller.py` (plus any
modules you add here).
