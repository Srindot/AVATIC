# Evaluation: test on many new layouts

A controller can work well on the one layout you practise on and still
fail on others. **Judging uses layouts nobody has seen**, like the hidden
test cases of a programming contest. The evaluation checks your controller
the same way before you submit it.

## Run an evaluation

From the repository folder (environment loaded):

```bash
python3 evaluation/evaluate.py --controller outerloop_controller/my_controller.py --runs 5
```

- It flies **5 runs on 5 random layouts**, one after another, without
  windows. Each run takes about 40 s, so 5 runs take about 3 minutes.
- It never uses your practice layout (the seed in `analysis/seed.yaml`).
- It prints each run's score and, at the end, your **average score**.
- **Ctrl-C** stops early. The runs already done are kept.

Then open the report:

```bash
jupyter notebook evaluation/evaluation.ipynb
```

Run all the cells (VS Code: **Run All**; browser: *Run → Run All Cells*).
It shows the **latest**
evaluation. For an older one, set `SESSION = '<folder name>'` in the first
cell.

## What the report shows

| Section | What you learn |
|---|---|
| **Headline** | your **average score out of 700** (runs where your controller never armed count as 0, as in judging), the lowest, middle and highest scores, red balloons hit, failed runs, how often you popped each colour, the rules and the fair-play checks |
| **Points per run** | one bar per layout, split by colour (red hits below 0), with the run's score |
| **Score during the run** | every run's score against the time since arming, and the mean: how fast points come, and whether they stop coming |
| **Every run** | a table with all the numbers |
| **Best and worst run** | a map of each, and what the camera saw in the worst one |

**A run where your controller never armed counts as 0 points**, as in
judging.

## Options

| Option | What it does |
|---|---|
| `--controller <file>` | the controller to test (default `outerloop_controller/my_controller.py`) |
| `--runs 20` | the number of random layouts (default 5) |
| `--seeds 5,17,301` | these layouts instead, for example to repeat your worst one |
| `--timeout 120` | the longest a single run may take, in seconds of real time (default 120) |

Close other simulations first: they slow the computer down and can change
your results.

## For your submission

Run an evaluation of your **final** controller (at least 5 runs), then
`python3 output/collect.py`: it copies the evaluation into `output/` and
runs the notebook on it. See [DELIVERABLES.md](../DELIVERABLES.md).

## Fair play (organisers)

Every judged evaluation is checked twice (rules: guide page 2, "Fair
play"):

- **The code**, at the start of the session (`check_controller.py`; the
  findings are printed and saved in `session.yaml`, `code_check`). It
  reads the code only. Anyone can run it:
  `python3 evaluation/check_controller.py <controller>`.
- **The running controller**, in every run
  (`simulation_engine/pluto_x_bringup/scripts/integrity_monitor.py`): the
  ROS graph (the controller's node may use only the API's topics; no other
  node may read the ground truth or publish `/pluto/rc`), the controller's
  processes (found by a per-run token in their environment: no other
  programs, no Gazebo libraries, none of the run's files open). The
  verdict is in each run's `meta.yaml` (`integrity`) and the
  `fair_play` / `fair_play_flags` columns of `summary.csv`, and in both
  notebooks.

Neither check is proof: a determined cheater can hide from both (a file
read in a millisecond is between two looks), so **a flag starts a review,
and the absence of one does not end it.** What to do:

1. **Before judging:** evaluate on seeds the teams have not seen, on the
   organisers' computer, with nothing else running (a stray ROS tool shows
   up as a note).
2. **A code finding or a flagged run:** read the lines concerned (the
   finding gives file and line; a run flag names the topic, library,
   program or file).
   - Harmless (an unused import, reading the team's own parameter file,
     debugging leftovers): ask the team to remove it or explain it, and
     evaluate the cleaned controller again. No penalty.
   - Reads the ground truth (true position, balloon positions, the layout
     or the seed), or hides what it does: **disqualified from the
     simulation round.** Keep the evidence (the finding, the run's
     `meta.yaml`, the log) and tell the team why.
   - Unsure: ask the team to explain before deciding.
3. **Finalists** (the teams that go on to the hardware round): read every
   controller in full, flagged or not.
4. The hardware round cannot be cheated this way: the real drone has no
   ground truth.

## What is saved

Each evaluation is saved in `evaluation/sessions/<date>_<time>/`:

| File | Content |
|---|---|
| `session.yaml` | your controller file, the layouts, the average score |
| `summary.csv` | one line per run: score, balloons per colour, red hits, status |
| `run_XX_seed_N/` | the full recording of each run (same files as in `analysis/runs/`) |
| `logs/run_XX.log` | the simulator and controller output of each run (look here if a run failed) |

A run's status is `complete` (it counts), `never_armed` (your controller
never armed the drone: 0 points), or `crashed` / `timeout` (the simulator
failed: the run is left out, see its log).
