# Balloon arena: balloons, popping, score, time limit, camera

Status 2026-09-25. Launches: `competition.launch.py` (participant runs: seeded random layouts, clock from arming) and `arena.launch.py` (the fixed default layout below, developer checks).

## What it is

| Piece | Where |
|---|---|
| Balloon mesh: the source `simulation_engine/pluto_x_gazebo/models/balloon/meshes/balloon.stl`, normalised to unit diameter with the origin at its centre; surface profile extracted for touch detection | `pluto_x_gazebo/tools/prepare_balloon_mesh.py` → `models/balloon/{meshes/balloon_unit.stl, profile.yaml}` |
| Arena definition: balloons (colour, position), points per colour, time limit, vehicle contact radius | `pluto_x_gazebo/config/arena_default.yaml` |
| Arena world: field with a 10 m × 10 m boundary, take-off pad, lighting, rendering (Sensors system) | `pluto_x_gazebo/worlds/balloon_arena.sdf.xacro` |
| Arena logic (Gazebo-free, unit-tested): balloon surface geometry, pop rule, score, run clock | `pluto_x_core/arena/{balloon_shape,arena_scoring}` |
| Gazebo world plugin: spawns coloured balloons, pops them (removes the model), publishes score/time/events, pauses the world at the limit | `pluto_x_gazebo/src/arena_system.cpp` (`ArenaSystem`) |
| Forward camera on the Pluto X | vehicle YAML `camera:` section → model xacro |
| RViz: camera image + ground-truth odometry | `pluto_x_bringup/rviz/arena.rviz` |

Points: **green 100, blue 50, yellow 25, and red −75 (a penalty)**. Red
balloons are obstacles: popping one subtracts 75 points. Competition
layouts have 18 balloons (green 3, blue 5, yellow 6, red 4; best 700; see
"Competition rules" below). The default (demo) layout here has two of each
colour; its best achievable score is **350**
(everything except red). In this default (demo) layout the balloons are
1.6–4.0 m from the take-off point horizontally and 0.9–1.9 m high.
Higher-value balloons are farther away and higher, and several are behind
the start heading, so the drone has to scan with yaw. Editing `balloons:`
(or `arena_config:=`) changes the layout for `arena.launch.py` and the
demo. **Competition runs** (`competition.launch.py`) generate the positions
from a seed (`analysis/seed.yaml`, `arena_seed:=`), taking only the rules
and the per-colour counts from this file.

## Rules as implemented

* **Pop**: a balloon pops the first time the vehicle touches its surface.
  The vehicle's outline is six spheres in its body frame (they move and
  rotate with it), fitted to the model's measured envelope of 153 × 153 ×
  47 mm (published: 16 × 16 × 4 cm). There is one r = 30 mm sphere per
  propeller guard at (±47.5, ±47.5) mm, reaching the 77 mm half-width, one
  r = 30 mm sphere on the body, and one r = 15 mm sphere on the camera
  module. The spheres are thicker than the guards (±30 mm against
  ±12 mm). The balloon surface is the actual mesh profile, as a surface of
  revolution, scaled to `balloon.diameter_m`. The popped balloon's model is
  removed and its colour's points are added, once.
* Balloons are **visual-only** (no collision): the vehicle is not deflected
  by a balloon; on first contact the balloon pops, as a real one would.
* **Time limit**: `time_limit_s` = 15 s of simulation time in this file (for
  `arena.launch.py` and the demo; competition runs use 25 s, see below).
  `clock_start: sim_start` (the default, as requested) counts from
  simulation time 0, so the firmware's ~4 s start-up and the take-off count
  against the limit and about 11 s of flight remain.
  `clock_start: armed` starts the clock when the flight controller first
  reports ARMED.
  **The competition uses `clock_start: armed`** (competition.launch.py
  sets it): a participant gets the full run time whatever the
  firmware start-up time, as on the real drone, where the run starts when
  the drone arms.
* **Competition rules: 25 s and 18 balloons** (green 3, blue 5, yellow 6,
  red 4; best score 700; set in the `competition:` block of
  arena_default.yaml, decided 2026-09-26). Chosen to widen the score gap
  between algorithms: in a short run the first search takes a large share
  of the time and luck (is a balloon in view at the start?) decides much;
  over 25 s a controller that searches, re-targets and flies efficiently
  pulls ahead of one that stalls, and layout luck averages out. With 14
  good balloons nobody runs out: a very good controller at one pop every
  ~2 s after the first search would pop about 11. High-value balloons are
  the rarest; red is about a fifth. 30 of 30 test layouts generated with
  the same spacing rules (mean nearest-balloon spacing 0.83 m). Cost:
  evaluation runs take about 40 s each, and the hardware round needs 18
  balloons per layout.
* **Previous rules: 15 s and 12 balloons** (green 2, blue 3, yellow 4,
  red 3; best score 450; 2026-09-26, same day). Every competition layout has
  this composition (so every layout offers the same 450 points); where the
  balloons float and which one has which colour come from the seed. The
  analysis and evaluation notebooks show these rules and flag any run with
  other settings as not official. Why: with 8 balloons (6 good) a strong
  controller could pop every good balloon before the end; with 9 good
  balloons nobody runs out (8 test runs of a camera-only controller popped
  0-3 in 15 s and left at least 6 good ones; a very good controller at one
  pop every ~2 s after the first search reaches about 6). High-value
  balloons are the rarest, and red stays about a quarter of the balloons.
  20 s did not reliably raise the scores in the same test (run-to-run
  noise was larger) and would let top controllers approach running out
  again, so 15 s stays. 40 of 40 test layouts generated with the same
  spacing rules.
* **Earlier reasoning for 15 s and 8 balloons** (2 per colour, 350,
  2026-09-25): every balloon is within 3.5 m of the take-off point and at
  most 2.0 m from a neighbour.
  The shortest horizontal route from take-off through all six scoring
  balloons is 5.2-11.8 m, median 8.4 m (computed over seeds 1-200), so
  350 in 15 s needs only ~0.6 m/s on average, but a controller must also
  find each balloon with the camera, turn to it and avoid the reds, which
  takes most of the time. A crude camera-only controller written to test
  the tools usually found its first balloon 4-8 s after arming and scored
  0-200 over 8 runs (median 50), leaving a wide range above it for better
  controllers. The
  two red balloons (-75 each) make blind forward flight costly. A longer
  limit would let a simple "turn and fly at everything" strategy approach
  the maximum. (Design judgements, not measured optima.)
* **End of run**: at the limit, scoring stops. Contacts at or after the
  limit never score, however quickly the simulator stops. The result is
  published, optionally written to `result_file`, and the world is paused
  (`pause_on_time_limit`). Measured: paused at 15.001 s (the next 1 ms
  physics step).

## Topics

| Topic (ROS) | Type | |
|---|---|---|
| `/pluto/camera/image_raw` | sensor_msgs/Image | 1280×720 rgb8, 18 Hz |
| `/pluto/camera/camera_info` | sensor_msgs/CameraInfo | |
| `/arena/score` | std_msgs/Int32 | total points (10 Hz and on change) |
| `/arena/time_remaining` | std_msgs/Float64 | seconds of run time left |
| `/arena/events` | std_msgs/String | `t=6.910 s: POP yellow +25 (...) - total 25`, start, `TIME UP` |
| `/arena/result` | std_msgs/String | final YAML: score, max, per-balloon popped |

Balloon positions are not published; finding them is the task. In the
simulator, `/sim/pluto/odometry` still gives ground truth for development.

## Camera

A Gazebo camera sensor on the vehicle (vehicle YAML `camera:`), modelling
the Pluto WiFi Camera Module. From the supplier listing: 720p stream
(1280×720) at about 18 FPS live over Wi-Fi, 8 g (added to the vehicle mass:
68 g in flight). Estimated, because the listing does not give them: 80°
horizontal field of view, mounted under the front of the body, facing
ahead (3.5 cm ahead of and 1.8 cm below the centre of mass, no tilt; an
earlier estimate on top of the body put the front prop guards in the image
corners), and the pixel noise. With
the camera fitted, the real drone routes control and telemetry through the
camera's Wi-Fi. The simulator does not model that link's latency, the H.264
compression, motion blur, rolling shutter, or the module's 3.4 V brown-out. Rendering works headless
(`--headless-rendering`, set by `headless:=true`).

## Checks

`simulation_engine/scripts/check_arena.sh` runs the arena headless with a **demo** mission
(`pluto_x_autonomy/config/arena_demo.yaml`: straight to two known balloons,
using ground truth, which participants cannot do). It checks the time limit,
the score arithmetic, the pop events, the camera size and rate, and that
balloon colours are visible in the camera images. Result on 2026-09-24:

| t (sim) | event | total |
|---|---|---|
| 0.001 s | run clock started | 0 |
| 6.91 s | POP yellow +25 | 25 |
| 9.67 s | POP blue +50 | 75 |
| 15.001 s | TIME UP, world paused | 75 (the maximum was 500 then; with red now a −75 penalty it is 350) |

The popped balloons' models were gone from the world afterwards. The
camera ran at 18.0 Hz (1280×720). Arena logic unit tests: `pluto_x_core` `test_arena.cpp`.

## Workshop demo (`balloon_demo.launch.py`)

```bash
ros2 launch pluto_x_demo balloon_demo.launch.py
```

This opens Gazebo (the whole field) and RViz (the drone's camera, the live
score and time left above the field, and the flight path). The demo
controller (`demo/pluto_x_demo/balloon_demo.py`) flies a
hand-planned route through **known** balloon positions using ground truth:

1. it takes off vertically to 0.5 m with level sticks. Commanding tilt or
   yaw on the ground only presses the drone into the ground: the
   firmware's attitude loop saturates and it never lifts off;
2. for each balloon, it turns on the spot to face it, then flies into it
   keeping the nose on the bearing, and moves on once contact is certain.

When the time is up, the world pauses and the scoreboard prints the result
table in the terminal and shows the final score in RViz. **It is not a
solution to the challenge:** teams must find the balloons with the camera.

The demo runs for **35 s** (`time_limit_s`; competition runs use the
official 25 s). It visits the six non-red balloons
(`demo/config/balloon_demo.yaml`) in the shortest order, 17.5 m in total, and
every leg of the planned route passes at least 1.08 m from both red
balloons (the flown path came within 1.05 m, below). Measured in
Gazebo on 2026-09-24:

| # | balloon | popped at (sim) | total |
|---|---|---|---|
| 1 | yellow 0 | 6.80 s | 25 |
| 2 | green 6 | 9.13 s | 125 |
| 3 | blue 2 | 15.00 s | 175 |
| 4 | blue 3 | 17.78 s | 225 |
| 5 | green 7 | 20.04 s | 325 |
| 6 | yellow 1 | 26.36 s | **350 / 350** |

No red balloon was popped. The closest approaches were 2.47 m (red 4) and
1.05 m (red 5); popping needs about 0.23 m. `simulation_engine/scripts/check_arena.sh` with
`FORBID=red` fails the run if a red balloon pops.

Short variant (15 s; route blue 3 → green 7, at least 2.2 m
from red):

```bash
ros2 launch pluto_x_demo balloon_demo.launch.py time_limit_s:=15 route_file:=$(ros2 pkg prefix pluto_x_demo)/share/pluto_x_demo/config/balloon_demo_15s.yaml
```

Result: 150 points (8.30 and 10.51 s), no red popped.

To plan another route, change `route:` (balloon indices are listed in the
file) and pass the file with `route_file:=`.

## Limitations

* The vehicle's contact shape is a sphere, not the real frame, propeller
  and guard geometry.
* Balloons are fixed in place: no drift, bobbing, strings or popping
  debris.
* One vehicle per arena.
