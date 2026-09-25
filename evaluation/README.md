# Evaluation: how good is your controller on layouts it has never seen?

(Participant guide: [5. Testing and improving](../docs/participants/5_testing_and_improving.md).)

The analysis tools (`analysis/`) replay one layout again and again, which is
right for tuning. Judging uses layouts your controller has never seen,
so before submitting, test it the same way:

```bash
source install/setup.bash && python3 evaluation/evaluate.py --controller outerloop_controller/my_controller.py --runs 10
```

```bash
jupyter notebook evaluation/evaluation.ipynb
```

Each run is a normal competition run (headless, 15 s from arming) on a new
random layout (never the development seed in `analysis/seed.yaml`). The
runs are one after the other, about 25 s each, so 10 runs take about
5 minutes. Ctrl-C stops early and keeps the runs done so far
(`summary.csv` and `session.yaml` are rewritten after every run).

The notebook shows the latest session (set `SESSION` for an older one):

- **the headline first:** mean score out of 350, min / median / max, the
  mean with never-armed runs as 0, red balloons hit, simulator failures,
  controller errors, and how many balloons of each colour were popped over
  all the runs;
- the score of every run and the distribution;
- balloons per colour and the time to the first pop;
- a table of every run;
- the best and the worst run on a map, and the camera in the worst run.

Any single run can be looked at with all the analysis plots:
`run = evallog.load_run(session, 3)`, then `runlog.plot_overview(run)`.

## Options

| | |
|---|---|
| `--runs N` | number of random layouts (default 10) |
| `--seeds 5,17,301` | these layouts instead (e.g. to rerun a bad one) |
| `--timeout S` | wall-clock limit per run (default 120 s) |

## What is saved

`evaluation/sessions/<date-time>/` (not in git):

| | |
|---|---|
| `session.yaml` | controller path and SHA-256, seeds, times, mean score (and incl. never-armed as 0), min and best run score, best possible per run, red hits, simulator failures |
| `summary.csv` | one row per run: status, score, balloons popped and available per colour, red hits, time to the first pop, distance, tilt, controller error and SHA-256 at that run |
| `run_XX_seed_N/` | the full recording of each run, the same files as `analysis/runs/<run>/` |
| `logs/run_XX.log` | the simulator and controller output of each run (look here if a run failed) |

Run status:

| | |
|---|---|
| `complete` | the run ended normally; its score counts |
| `never_armed` | your controller never armed the drone: **0 points** (included in "mean incl. never-armed runs as 0", as in judging) |
| `crashed` / `timeout` | the simulator stopped before the result, or took longer than `--timeout` with the drone armed: a simulator failure, left out of the means (look at `logs/`) |

A run where your `step()` raised an error still counts: the failsafe takes
over and the run ends normally, with `controller_error = 1` (detected from
the template's message; a controller not built on the template is not
detected).

Each run uses its own Gazebo partition and ROS domain (localhost only), so
it does not interfere with a simulator you have open or other laptops on
the network, but it does share the CPU with it: close other simulations
for results representative of judging.
