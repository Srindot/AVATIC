# 2. The challenge

Fly the Pluto X into balloons using only its **camera** and its **flight
controller's readings**. Pop the good ones, avoid the red ones.

## Points

| Balloon | Points | How many |
|---|---|---|
| green | **+100** | 3 |
| blue | **+50** | 5 |
| yellow | **+25** | 6 |
| red | **−75**: avoid it | 4 |

- Best possible score: **700** (all green, blue and yellow, no red).
- The rarest balloons are worth the most. There are more good balloons
  than anyone can pop in 25 s: you never run out, so choose well.
- The score can go below zero if you hit red balloons.
- A balloon **pops when any part of the drone touches it.** Balloons do not
  push the drone: it flies straight through, and the balloon disappears.
  Each balloon counts once.

## Time

- You have **25 seconds**, counted **from the moment your controller arms
  the drone** (starts the motors). Waiting before arming costs nothing.
- Pops after the 25 s do not count. At 25 s the simulation pauses and the
  result is printed.
- The template arms as soon as the flight controller is ready (about 4 s
  after the simulator starts).

## The arena

- The drone starts on the ground at the centre, **facing east**.
- There are 18 balloons: 30 cm wide, 45 cm tall, floating 0.8–2.0 m high
  (balloon centre). Which one has which colour changes with the layout.
- Every balloon is 1–3.5 m from the take-off point. Balloons are never
  stacked, and the gap between two balloons is at least 16 cm, about the
  drone's width. That is tight: fly straight through gaps, not diagonally.
- **Balloons are often behind the drone.** You usually have to turn to find
  them.
- A light wind blows, and the drone drifts a little even with centred
  sticks.

## Layouts

The balloon positions come from a number called the **seed**: the same seed
gives the same layout.

- **While developing**, every run uses the same layout (the development seed, 42 at first, stored in
  `analysis/seed.yaml`), so you can see whether a change helped. Change it
  with `python3 analysis/new_seed.py`, or use a random one for a single run
  with `arena_seed:=random`.
- **For testing**, `evaluation/evaluate.py` flies your controller on many
  layouts it has never seen and reports the average score.
- **Judging** uses layouts nobody has seen, like the hidden test cases in a
  programming contest. A controller tuned for one layout will do badly: make
  it find balloons, not remember them.

## What you can and cannot use

Your controller gets exactly what the real drone gives:

- the **camera image** (1280 × 720 colour, about 18 per second);
- the **flight controller's readings**: tilt, heading, barometric
  altitude, battery, armed.

It does **not** get the drone's position or speed, or where the balloons
are. (The analysis notebook shows them afterwards, for you to learn from.)

### Fair play

The simulator knows everything (the true position, every balloon), and
your controller runs on the same computer, so it could reach that
information. **Doing so is cheating.** Your controller may use only the
public methods of the `Drone` from `avatic_drone` (camera frames,
telemetry, time, score), and send only `Command`s. Not allowed:

- any other ROS or Gazebo interface (`rclpy`, `gz`, topics such as
  `/sim/pluto/odometry`, services), including through the `Drone`'s
  internals (`drone._...`);
- reading the simulator's or the run's files (the layout, the world, the
  seed, `analysis/runs/`, `evaluation/sessions/`) or other programs'
  memory or `/proc` files;
- starting other programs, or using the network;
- code that hides what it does (`exec`, `eval`, `__import__`, `getattr`
  with built-up names, encoded text).

Your own files next to your controller (parameters, a colour table) are
fine. It is checked twice: **the code** (`evaluation/check_controller.py`)
and **the running controller** (the simulator watches its ROS
connections, and, when the launch starts it with `controller:=...` as in
judging, the libraries it loads, the files it opens and the programs it
starts; every run's result shows "fair play: clean" or "FLAGGED").
Check your code yourself before you submit:

```bash
python3 evaluation/check_controller.py outerloop_controller/my_controller.py
```

Every line it prints is read by an organiser. If one is harmless (you read
your own parameter file, say), explain it in your report. **A flag is not
a verdict:** the organisers read the code and ask you if unsure. A
controller that reads the ground truth, or tries to hide what it does, is
**disqualified from the simulation round**; the controllers of all
finalists are read in full.

## What you submit

Everything goes in the `output/` folder; the full list, the video and
report requirements, and how to collect it all with one command
(`python3 output/collect.py`) are in **[DELIVERABLES.md](../../DELIVERABLES.md)**.
In short, all **required**:

1. **Your code:** `outerloop_controller/my_controller.py`, plus any Python
   files you add next to it in `outerloop_controller/`. Do not change
   anything in `simulation_engine/`, `outerloop_controller/avatic_drone/`
   or `hitl/`: the judges use their own copies.
2. **Your best run** and its **analysis notebook**, executed.
3. **An evaluation of your final controller** (at least 5 runs) and its
   **notebook**, executed.
4. **A video:** a screen recording of your best run (Gazebo and RViz, from
   arming to the end), showing the pops and how you avoid the red balloons.
5. **A technical report** (PDF, at most 8 pages): your approach
   (perception, decision and red-balloon avoidance, control), results,
   analysis and at least two failure cases.

The submission format and deadline will be announced by the organisers.

**Next:** [3. Writing your controller](3_writing_your_controller.md)
