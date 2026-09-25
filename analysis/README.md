# Analysis: look at your flights

(Participant guide: [5. Testing and improving](../docs/participants/5_testing_and_improving.md).
Needs `pip install notebook` and `sudo apt install ffmpeg python3-matplotlib`.)

Every competition launch **records itself** into `analysis/runs/<date-time>/`,
with or without windows. The notebook `analysis.ipynb` then plots the latest
run.

```bash
ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/my_controller.py
```

```bash
jupyter notebook analysis/analysis.ipynb
```

In the notebook: Kernel → Restart & Run All. It shows the latest run; set
`RUN = '2026-09-25_14-32-07'` in the first cell to look at an older one.

## The balloon layout (seed)

All development runs use the layout from `analysis/seed.yaml` (seed 42 to
start with). It stays the same between runs, so the effect of changing your
controller is easy to see.

| | |
|---|---|
| `python3 analysis/new_seed.py` | new random layout for the next runs |
| `python3 analysis/new_seed.py --seed 7` | a specific layout |
| `python3 analysis/new_seed.py --show` | the current seed |
| `... competition.launch.py arena_seed:=random` | a one-off random layout (the seed file is not changed) |
| `... competition.launch.py record:=false` | don't save this run |

## What the notebook shows

- **First, the result:** the score out of 350, and each colour's balloons
  popped and points (red: −75 each).
- **Summary:** score, balloons popped, red hits, time to the first pop,
  maximum height, speed and tilt, distance flown.
- **Top view:** the flight path over the balloons, with an × where each one
  popped.
- **Altitude:** true height vs the barometric altitude your controller saw.
- **Distance to each balloon over time.** Did it get close? Did it approach
  a red one?
- **Attitude:** what the flight controller reported vs the truth.
- **Your commands:** roll, pitch, yaw rate, throttle, as they reached the
  drone.
- **Camera frames** at key moments: after arming, just before each pop, at
  the end.

## What is saved per run (`analysis/runs/<date-time>/`)

| file | content |
|---|---|
| `meta.yaml` | seed, controller file, time limit, status (complete / incomplete; recording if the recorder was killed), score |
| `layout.yaml` | the balloon layout of this run |
| `trajectory.csv` | the simulator's ground truth, 100 Hz (position, velocity, attitude) |
| `telemetry.csv` | what your controller saw: attitude, heading, baro altitude, battery |
| `commands.csv` | what reached the drone, 50 Hz |
| `events.csv`, `result.yaml` | pops and points, the final result (`result.yaml` is written last; `result_arena.yaml` is the arena's own copy) |
| `camera.mp4`, `camera_frames.csv` | the camera (640 px wide) and the time of each frame |

The trajectory is **ground truth, for analysis only**: your controller never
gets it, and a real drone can't provide it.

A 15 s run takes about 0.5 MB. Delete old runs whenever you like
(`rm -r analysis/runs/2026-09-2*`); nothing else depends on them.

`runlog.py` has the loading and plotting functions the notebook uses. You
can use it in your own scripts too:

```python
import sys; sys.path.insert(0, 'analysis')   # from the repository folder
import runlog
run = runlog.load_run()                      # the latest run
runlog.print_summary(run)
```

Runs started with the two-terminal method record `controller: ''` (the
simulator did not start your script, so it does not know its name).
