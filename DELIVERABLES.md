# Deliverables: what you submit for the simulation round

Everything goes in the **`output/`** folder of this repository. Most of it is
collected for you by one command; you add the **video** and the **report**.
The submission format (where to upload, file size limits) and the deadline
will be announced by the organisers.

## Checklist

| # | Deliverable | File in `output/` | Who makes it |
|---|---|---|---|
| 1 | Your controller code | `code/` | `collect.py` copies it |
| 2 | Your best run: the recording | `best_run/` | `collect.py` copies it |
| 3 | The analysis notebook of that run, executed | `analysis.ipynb` | `collect.py` runs it |
| 4 | An evaluation of your **final** controller (at least **5 runs**) and its notebook, executed | `evaluation/`, `evaluation.ipynb` | you run `evaluate.py`; `collect.py` runs the notebook |
| 5 | The fair-play code check | `code_check.txt` | `collect.py` runs it |
| 6 | **Video** of your best run: balloons popped, red ones avoided | `video.mp4` | **you** |
| 7 | **Technical report** | `report.pdf` | **you** |

## How to put it together

1. **Fly your best run** with your final controller, and **record the
   screen** while it flies (item 6, below). Every run is saved in
   `analysis/runs/`:

   ```bash
   ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/my_controller.py
   ```

2. **Evaluate your final controller** on new layouts (about 3 minutes):

   ```bash
   python3 evaluation/evaluate.py --controller outerloop_controller/my_controller.py --runs 5
   ```

   More runs give a more reliable average; 5 is the minimum. Do not change
   your controller after this: the evaluation records which version it
   tested, and `collect.py` checks it is the one you submit.

3. **Collect everything** into `output/`:

   ```bash
   python3 output/collect.py
   ```

   It takes your **best official run** (the highest score with the
   official rules and a clean fair-play check) and your **latest
   evaluation**, copies them, runs both notebooks on them, runs the
   fair-play code check, and ends with **READY** or a list of what is
   missing. For a particular run or evaluation:
   `python3 output/collect.py --run <folder in analysis/runs> --session <folder in evaluation/sessions>`.
   The run you choose must be the one in your video.

4. **Add** `output/video.mp4` and `output/report.pdf`, run `collect.py`
   once more, and when it says READY, zip the folder:

   ```bash
   zip -r team_<your team name>.zip output
   ```

`output/` is never committed to git (it is in `.gitignore`).

## 6. The video

A **screen recording of your best run**, the same run that is in
`best_run/`. It shows how your drone finds and pops the balloons and how
it **avoids the red balloons**, which are the obstacles of this challenge.

- **What must be visible:** the Gazebo window (the 3-D arena) for the
  whole run, from arming to the end, and RViz (the drone's camera and the
  live score). Side by side is best.
- **Uncut**, at normal speed, one run. You may add a title or narration
  at the start or the end, but not in the middle of the run.
- **Format:** MP4 (H.264), at most 3 minutes and 100 MB, at least 720p.
  Name it `video.mp4`.
- **How to record:** on the computer that shows the windows (with the
  dev container: your own computer, not the container).
  [OBS Studio](https://obsproject.com/) works everywhere (record the whole
  screen). Built-in alternatives:
  - Ubuntu: press Print Screen and choose the video (screencast) option;
  - Windows 11: the Snipping Tool's video mode (the Xbox Game Bar records
    only one window, and Gazebo and RViz are two);
  - macOS: Cmd + Shift + 5.

The run's own camera video is already in `best_run/camera.mp4`; it does
not replace the screen recording.

## 7. The technical report

A PDF, **at most 8 pages** (appendices with extra plots do not count),
named `report.pdf`. Explain what you built, show how well it works and be
honest about where it fails: a clear account of a weak controller is worth
more than a vague account of a strong one. Use these sections:

1. **Team.** Names, institute, programme (B.Tech., M.Tech., M.S., Ph.D.)
   and one contact e-mail.
2. **Approach.** An overview: one diagram from the camera image to the
   `Command`, and the main idea in a few sentences.
3. **Perception.** How you find balloons in the image: colour thresholds
   or other methods, how you separate the colours, how you estimate
   direction and distance (page 4 of the guide has the camera geometry),
   and how you deal with noise, partly hidden balloons and the ground.
4. **Decision and obstacle avoidance.** Which balloon you go for and why
   (points, distance, risk), how you **avoid the red balloons**, and what
   the drone does when it sees nothing (search).
5. **Control.** How you turn the decision into roll, pitch, yaw rate and
   **throttle**: your height control, your gains and how you tuned them,
   and how you stay within the safety limits.
6. **Results.** Your evaluation: the mean score over the runs, the lowest
   and highest, balloons popped per colour, red balloons hit (from
   `evaluation.ipynb`). Your best run: score and what happened.
7. **Analysis.** What the plots show about your controller: use the
   notebooks (the map, "what the camera could see", the commands, the
   score during the run). Where does it lose time? Which balloons does it
   miss, and why?
8. **Failure cases.** At least **two** runs where your controller did
   badly (a red hit, a missed balloon, a crash, lost height, getting stuck
   searching). For each: the run (folder name and seed, so the organisers
   can look at it), what happened (a map or camera frames), why, and what
   you changed or would change.
9. **What did not work.** Ideas you tried and dropped, and why.
10. **Towards the real drone.** What would have to change for the real
    Pluto X (hover throttle, camera latency over Wi-Fi, lighting, wind).
11. **Fair play and sources.** Explain every line in `code_check.txt`, if
    there are any. Name any code, library or AI tool you used that you did
    not write yourself.

## What the organisers do with it

- They **evaluate your code again themselves**, on layouts nobody has
  seen, with the fair-play checks on (guide page 2, "Fair play"). Your own
  evaluation shows how you tested; it is not your judged score.
- They watch the video and read the report and the notebooks.
- They read the code of every team that goes on to the hardware round.
