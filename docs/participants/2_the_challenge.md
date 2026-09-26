# 2. The challenge

Fly the Pluto X into balloons using only its **camera** and its **flight
controller's readings**. Pop the good ones, avoid the red ones.

## Points

| Balloon | Points | How many |
|---|---|---|
| green | **+100** | 2 |
| blue | **+50** | 2 |
| yellow | **+25** | 2 |
| red | **−75**: avoid it | 2 |

- Best possible score: **350** (all green, blue and yellow, no red).
- The score can go below zero if you hit red balloons.
- A balloon **pops when any part of the drone touches it.** Balloons do not
  push the drone: it flies straight through, and the balloon disappears.
  Each balloon counts once.

## Time

- You have **15 seconds**, counted **from the moment your controller arms
  the drone** (starts the motors). Waiting before arming costs nothing.
- Pops after the 15 s do not count. At 15 s the simulation pauses and the
  result is printed.
- The template arms as soon as the flight controller is ready (about 4 s
  after the simulator starts).

## The arena

- The drone starts on the ground at the centre, **facing east**.
- There are 8 balloons: 30 cm wide, 45 cm tall, floating 0.8–2.0 m high
  (balloon centre).
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

## What you submit

All of these are **required**:

1. **Your code:** `outerloop_controller/my_controller.py`, plus any Python
   files you add next to it in `outerloop_controller/`. Do not change
   anything in `simulation_engine/`, `outerloop_controller/avatic_drone/`
   or `hitl/`: the judges use their own copies.
2. **The analysis notebook of your best run** (`analysis/analysis.ipynb`),
   executed and saved with all outputs visible.
3. **The evaluation notebook** (`evaluation/evaluation.ipynb`), executed on
   an evaluation of your final controller (at least 10 runs).
4. **A video:** a screen recording of your best simulation run, showing
   the Gazebo window from arming to the final score.
5. **A report** explaining your outer-loop idea: how you find the balloons
   in the image, how you choose which one to go for, how you fly to it and
   avoid the red ones, and what you tried that did not work.

How to run and save the notebooks: [page 5](5_testing_and_improving.md).
The submission format and deadline will be announced by the organisers.

**Next:** [3. Writing your controller](3_writing_your_controller.md)
