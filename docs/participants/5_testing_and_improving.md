# 5. Testing and improving

The loop you will repeat many times:

```text
  change my_controller.py  →  fly one run  →  look at it in the notebook  →  repeat
                                   (when it works well)  →  evaluate on 10 new layouts
```

## Fly one run

```bash
ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/my_controller.py headless:=true rviz:=false
```

Without windows (`headless:=true rviz:=false`) it is faster and uses less of
your computer. Leave them out when you want to watch. Press Ctrl-C at the
end. Every run is saved in `analysis/runs/<date>_<time>/`.

**Runs are not exactly repeatable.** Wind gusts and sensor noise differ
every run, so the same controller on the same layout can score ±50 points
differently. Judge a change over a few runs, not one.

All development runs use **the same layout** (the development seed in `analysis/seed.yaml`, 42 at first), so you can tell
whether a change helped. Once your controller handles it, try others:

| | |
|---|---|
| `python3 analysis/new_seed.py` | a new random layout for all the next runs |
| `python3 analysis/new_seed.py --seed 7` | a specific layout |
| `python3 analysis/new_seed.py --show` | which layout is set now |
| add `arena_seed:=random` to the launch | a random layout for this run only |

## Look at the run: `analysis/analysis.ipynb`

```bash
jupyter notebook analysis/analysis.ipynb
```

Run all the cells (VS Code: **Run All**; browser: *Run → Run All Cells*). It always shows the **latest** run. For an
older one, set `RUN = '2026-09-25_14-32-07'` (the folder name) in the first
cell.

What you see, top to bottom:

1. **Your score**: points, and which balloons of each colour you popped.
2. **Summary**: time to the first pop, highest point, fastest speed,
   largest tilt, distance flown.
3. **Map from above**: your flight path over the balloons (an × marks each
   pop), the height over time, and the distance to every balloon over time.
   Did you get close to the one you wanted? Did you pass near a red one?
4. **Attitude**: what the flight controller reported against the truth.
5. **Your commands**: what your controller sent, over time.
6. **The camera**: what your controller saw after arming, just before each
   pop, and at the end.

The map shows the drone's **true** position. Your controller never gets
it, but it shows you what really happened.

You can also use the data in your own Python code:

```python
import sys; sys.path.insert(0, 'analysis')     # from the repository folder
import runlog
run = runlog.load_run()                        # the latest run
print(runlog.summary(run)['score'])
run.trajectory['z_enu_m']                      # NumPy arrays of everything recorded
```

## Test on new layouts: `evaluation/evaluate.py`

When your controller works on the development layout, check that it also
works on layouts it has never seen. This is how judging works:

```bash
python3 evaluation/evaluate.py --controller outerloop_controller/my_controller.py --runs 10
```

It runs 10 flights on 10 random layouts, one after another (about 40 s
each, no windows), and prints the average score. Then open the report:

```bash
jupyter notebook evaluation/evaluation.ipynb
```

It starts with your **average score out of 700**, your worst and best runs,
how many red balloons you hit, and how often you popped each colour. Then
it shows every run, and maps of your best and worst flights.

- Ctrl-C stops early; the runs done so far are kept.
- `--seeds 5,17,301` repeats specific layouts (for example your worst one).
- A run where your controller never armed counts as **0 points**.
- The results are saved in `evaluation/sessions/<date>_<time>/`.
- Close other simulations first: they slow the computer down and can
  change your results.

## Save the notebooks for your submission

Your submission needs both notebooks **executed**, with all their outputs
visible:

1. **Analysis:** set `RUN` in the first cell to your **best** run, then
   run all cells, then save.
2. **Evaluation:** evaluate your **final** controller (at least 10 runs),
   open `evaluation/evaluation.ipynb`, run all cells, then save.

Or from a terminal, in the repository folder:

```bash
jupyter nbconvert --to notebook --execute --inplace analysis/analysis.ipynb
```

```bash
jupyter nbconvert --to notebook --execute --inplace evaluation/evaluation.ipynb
```

**Next:** [6. Tips and troubleshooting](6_tips_and_troubleshooting.md)
