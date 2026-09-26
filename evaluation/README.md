# Evaluation: test on many new layouts

A controller can work well on the one layout you practise on and still
fail on others. **Judging uses layouts nobody has seen**, like the hidden
test cases of a programming contest. The evaluation checks your controller
the same way before you submit it.

## Run an evaluation

From the repository folder (environment loaded):

```bash
python3 evaluation/evaluate.py --controller outerloop_controller/my_controller.py --runs 10
```

- It flies **10 runs on 10 random layouts**, one after another, without
  windows. Each run takes about 25 s, so 10 runs take about 5 minutes.
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
| **Headline** | your **average score out of 450** (runs where your controller never armed count as 0, as in judging), the lowest, middle and highest scores, red balloons hit, failed runs |
| **Score per run** | one bar per layout, and how the scores are spread |
| **Balloons per colour** | how often you popped each colour, over all runs |
| **Time to the first pop** | how fast your controller finds a balloon |
| **Every run** | a table with all the numbers |
| **Best and worst run** | a map of each, and what the camera saw in the worst one |

**A run where your controller never armed counts as 0 points**, as in
judging.

## Options

| Option | What it does |
|---|---|
| `--controller <file>` | the controller to test (default `outerloop_controller/my_controller.py`) |
| `--runs 20` | the number of random layouts (default 10) |
| `--seeds 5,17,301` | these layouts instead, for example to repeat your worst one |
| `--timeout 120` | the longest a single run may take, in seconds of real time (default 120) |

Close other simulations first: they slow the computer down and can change
your results.

## For your submission

Run an evaluation of your **final** controller (at least 10 runs), run the
notebook, save it with all outputs visible (*File → Save*), and include it
in your submission. From a terminal:

```bash
jupyter nbconvert --to notebook --execute --inplace evaluation/evaluation.ipynb
```

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
