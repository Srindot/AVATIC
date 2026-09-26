# Using the AVATIC simulator

This page explains **where to write your code**, **how to run things**,
and what each tool is for. Every command runs from the repository folder.

- **Dev container:** every terminal is ready.
- **Local setup:** first run
  `source /opt/ros/humble/setup.bash && source install/setup.bash` in each
  new terminal.

## The pieces at a glance

| What | Where | What it is for |
|---|---|---|
| **Your controller** | `outerloop_controller/my_controller.py` | **your code**: the algorithm that flies the drone |
| **Hello drone** | `outerloop_controller/examples/hello_drone.py` | a small example to learn from: take off, turn, count balloon colours |
| **The demo** | `ros2 launch pluto_x_demo balloon_demo.launch.py` | shows the arena: a pre-planned flight that pops every good balloon |
| **Analysis** | `analysis/` | look at **one** flight in detail |
| **Evaluation** | `evaluation/` | test your controller on **many new** layouts, like the judges |

The workflow:

```text
edit my_controller.py  →  fly one run  →  analysis notebook  →  repeat
                           (when it works)  →  evaluation on 10+ new layouts
```

---

## Where to write your code

Open `outerloop_controller/my_controller.py`. It has three sections:

1. **Settings**: numbers you may change (control rate, hover throttle).
2. **`MyController`**: **your code goes here**, in the `step()` method.
3. **Runner**: connects to the drone, arms it and calls `step()`. You do
   not need to change it.

```python
class MyController:
    def step(self, frame, telemetry, t):
        # frame:     the camera picture (frame.image: 720 x 1280 x 3, RGB)
        # telemetry: the drone's readings (altitude_m, heading_deg, ...)
        # t:         seconds since arming (0 ... 15)
        return Command(roll=0.0, pitch=0.0, yaw_rate=0.0, throttle=0.76)
```

As shipped, it takes off to 1 m and hovers. You may add more Python files
next to it in `outerloop_controller/` and import them. Everything you can
use (commands, readings, camera) is explained in
[the participant guide, page 3](../../docs/participants/3_writing_your_controller.md).

---

## Hello drone: the example

```bash
ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/examples/hello_drone.py
```

It climbs to about 1 m, turns slowly on the spot, and every second prints
its height, heading, score and **how many pixels of each balloon colour
the camera sees**. It does **not** pop balloons: it shows how to read the
camera and telemetry and how to send commands. Read its code
(`outerloop_controller/examples/hello_drone.py`, about 60 lines) before
writing your own.

## The demo

```bash
ros2 launch pluto_x_demo balloon_demo.launch.py
```

The drone flies a **pre-planned route** and pops all six good balloons of
its own fixed 8-balloon layout (350 points there; competition layouts have
12 balloons). It **knows where the balloons are** (your controller does
not), it flies a fixed layout, and it gets **35 s** instead of 15 s. It is
there to show what the arena, the balloons and a pop look like: it is not
a benchmark and not a solution.

| Option | What it does | Default |
|---|---|---|
| `headless:=true` | no Gazebo window | `false` |
| `rviz:=false` | no RViz window | `true` |
| `time_limit_s:=15` | the competition's time limit instead of 35 s | `35` |

---

## Run your controller

**One command** (the simulator starts your controller):

```bash
ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/my_controller.py
```

What happens:

1. Gazebo (the 3-D world) and RViz (the drone's camera and the score) open.
2. After about 4 s the drone's flight controller is ready and your
   controller **arms** it. **The 15 s clock starts now.**
3. At 15 s the simulation **pauses** and the score is printed in the
   terminal (and shown in RViz).
4. Press **Ctrl-C** to stop. Lines like `process has died ... exit code -2`
   after Ctrl-C are normal.

**One run per launch:** for the next try, start the command again.

**Two terminals** (optional; your controller's output stays separate):

```bash
ros2 launch pluto_x_bringup competition.launch.py
```

```bash
python3 outerloop_controller/my_controller.py
```

### Launch options (`name:=value`)

Add them to the end of the `ros2 launch ... competition.launch.py`
command.

| Option | What it does | Default |
|---|---|---|
| `controller:=<file>` | the controller to run; a path from the repository folder, or an absolute path | none: then run it yourself in a second terminal |
| `headless:=true` | no Gazebo window (faster, uses less of the computer) | `false` |
| `rviz:=false` | no RViz window | `true` |
| `arena_seed:=random` | a new random balloon layout, for this run only | `analysis`: the development layout (its seed is in `analysis/seed.yaml`, 42 at first) |
| `arena_seed:=7` | a specific layout (any whole number) | |
| `time_limit_s:=30` | a longer run while developing (judging always uses 15) | `15` |
| `record:=false` | do not save this run | `true` |
| `record_dir:=<folder>` | save runs in this folder instead of `analysis/runs/` | `analysis/runs` |
| `run_name:=<name>` | name the run folder yourself instead of the date and time | date and time |

Organisers' options (you do not need them): `msp_bridge:=true` serves the
simulator over the real drone's Wi-Fi protocol (to test the hardware
connection), `result_file:=<file>` writes the final result to a file.

Examples:

```bash
# fast, no windows, a random layout
ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/my_controller.py headless:=true rviz:=false arena_seed:=random
```

```bash
# watch a 30 s run on layout 7
ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/my_controller.py arena_seed:=7 time_limit_s:=30
```

### Options of `my_controller.py`

Only needed when you start it yourself (`python3 outerloop_controller/my_controller.py ...`):

| Option | What it does | Default |
|---|---|---|
| *(none)* | fly the simulator (it must already be running) | |
| `--hardware` | fly the **real** Pluto X over Wi-Fi | |
| `--host <address>` | the drone's address (with `--hardware`) | `192.168.0.1` |
| `--msp-port <port>` | the drone's command port (with `--hardware`) | `9060` |
| `--video <source>` | the camera source (with `--hardware`): `plutocam`, `tcp://host:port` or `none` | `plutocam` |

### The balloon layout while developing

Every development run uses the **same layout**, so you can see whether a
change helped:

| Command | What it does |
|---|---|
| `python3 analysis/new_seed.py` | pick a new random layout for all the next runs |
| `python3 analysis/new_seed.py --seed 7` | use layout 7 for the next runs |
| `python3 analysis/new_seed.py --show` | show which layout is set now |

Runs are **not exactly repeatable** (wind and sensor noise change every
run): judge a change over a few runs, not one.

---

## Analysis: look at one flight

**What it is:** every run is **saved automatically** in
`analysis/runs/<date>_<time>/`. The analysis notebook turns a run into
pictures, so you can see what really happened and why.

```bash
jupyter notebook analysis/analysis.ipynb
```

Run all the cells (VS Code: **Run All**; browser: *Run → Run All Cells*).
It shows the **latest** run; for another one, write its folder name in the
first cell: `RUN = '2026-09-25_14-32-07'`.

It shows, top to bottom:

1. **Your score**, and the balloons of each colour you popped
2. a **summary**: time to the first pop, highest point, fastest speed, distance flown
3. a **map from above**: your flight path over the balloons (× = a pop), the
   height over time, the distance to each balloon over time
4. the **tilt** the flight controller reported, against the truth
5. **your commands** over time
6. **camera pictures** after arming, just before each pop, and at the end

The map uses the drone's **true** position: your controller never gets it,
but it shows you where the drone really went. More:
[analysis/README.md](../../analysis/README.md).

## Evaluation: test like the judges

**What it is:** your controller flies **many random layouts it has never
seen**, one after another, and you get an overall report. Judging works the
same way, with layouts nobody has seen.

```bash
python3 evaluation/evaluate.py --controller outerloop_controller/my_controller.py --runs 10
```

| Option | What it does | Default |
|---|---|---|
| `--controller <file>` | the controller to test | `outerloop_controller/my_controller.py` |
| `--runs <N>` | the number of random layouts | `10` |
| `--seeds 5,17,301` | these layouts instead of random ones (e.g. repeat your worst one) | |
| `--timeout <seconds>` | the longest one run may take, in real time | `120` |

- It runs without windows, about 25 s per run (10 runs: about 5 minutes).
- **Ctrl-C** stops early; the runs already done are kept.
- A run where your controller **never arms** counts as **0 points**.
- The results are saved in `evaluation/sessions/<date>_<time>/`.

Then open the report:

```bash
jupyter notebook evaluation/evaluation.ipynb
```

It starts with your **average score out of 450**, your lowest, middle and
highest scores, red balloons hit and how often you popped each colour.
Then every run, and maps of your best and worst flights. More:
[evaluation/README.md](../../evaluation/README.md).

---

## For your submission

1. your code (`outerloop_controller/`)
2. `analysis/analysis.ipynb`, **executed** on your best run
3. `evaluation/evaluation.ipynb`, **executed** on an evaluation of your
   final controller (at least 10 runs)
4. a **video**: a screen recording of your best run (Gazebo and RViz)
5. a **report** explaining your outer-loop idea

Save an executed notebook from a terminal:

```bash
jupyter nbconvert --to notebook --execute --inplace analysis/analysis.ipynb
```

## The real drone

The same file flies the real Pluto X:

```bash
python3 outerloop_controller/my_controller.py --hardware
```

See [the participant guide, page 7](../../docs/participants/7_real_drone.md).
