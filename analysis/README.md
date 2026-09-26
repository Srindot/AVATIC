# Analysis: look at one flight

Use this folder to understand **what your drone actually did** in a run:
where it flew, what it saw, what your controller commanded, and why it
popped (or missed) a balloon.

## How it works

1. **Every run is saved automatically.** Each time you fly with
   `competition.launch.py`, the run is recorded in a new folder,
   `analysis/runs/<date>_<time>/`. You don't have to do anything.
2. **Open the notebook:**

   ```bash
   jupyter notebook analysis/analysis.ipynb
   ```

3. Run all the cells (VS Code: **Run All**; browser: *Run → Run All
   Cells*). The notebook shows your
   **latest** run.

To look at an older run, write its folder name in the first cell:
`RUN = '2026-09-25_14-32-07'`, then run all cells again.

## What the notebook shows

| Section | What you learn |
|---|---|
| **Score** | your points, and which balloons of each colour you popped |
| **Summary** | time to the first pop, highest point, fastest speed, largest tilt, distance flown |
| **Map from above** | your flight path over the balloons (× = a pop, a dot every 5 s), the height over time, the distance to the nearest balloon of each colour |
| **What the camera could see** | for every balloon, when it was in the camera's view and how big it looked: good balloons in view but never chased are missed chances |
| **Commands** | the roll, pitch, yaw rate and throttle your controller sent, over time, with the safety caps |
| **Camera** | what your drone saw 2 s after arming, just before each pop, and at the end |
| **Diagnostics** | the tilt and heading the flight controller reported, against the truth |

The first lines also say whether the run used the official rules, and the
**fair-play** check of your running controller (page 2 of the guide, "What
you can and cannot use").

The map uses the drone's **true** position from the simulator. Your
controller never gets it (the real drone can't provide it), but it shows
you what really happened.

## The balloon layout (seed)

While you develop, every run uses **the same layout** (the development
seed in `analysis/seed.yaml`, 42 at first), so you can
see whether a change helped:

| Command | What it does |
|---|---|
| `python3 analysis/new_seed.py` | pick a new random layout for the next runs |
| `python3 analysis/new_seed.py --seed 7` | use layout 7 |
| `python3 analysis/new_seed.py --show` | show the current layout number |
| add `arena_seed:=random` to the launch | a random layout for one run only |
| add `record:=false` to the launch | don't save this run |

## For your submission

`python3 output/collect.py` copies your best run into `output/` and runs
the notebook on it (`--run <folder>` for another run). See
[DELIVERABLES.md](../DELIVERABLES.md).

## What is saved in a run folder

| File | Content |
|---|---|
| `meta.yaml` | layout number, controller file, time limit, status, score |
| `layout.yaml` | where the balloons were |
| `trajectory.csv` | the drone's true position, speed and tilt (100 times per second) |
| `telemetry.csv` | what your controller read from the flight controller |
| `commands.csv` | what your controller sent (50 times per second) |
| `events.csv`, `result.yaml` | the pops and the final result |
| `camera.mp4`, `camera_frames.csv` | the camera video (scaled to 640 × 360), and the time of each picture |

A run takes about 0.5 MB. Delete old runs whenever you like.

You can also load a run in your own Python code (from the repository
folder):

```python
import sys; sys.path.insert(0, 'analysis')
import runlog
run = runlog.load_run()          # the latest run
runlog.print_summary(run)
```

Needs Jupyter (`pip install notebook`), matplotlib and ffmpeg. All three
are already in the dev container.
