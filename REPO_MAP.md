# Repository map: where to find what

A map of this repository. **You only write code in `outerloop_controller/`**;
everything else you run, read or leave alone.

## I want to...

| I want to... | Go to |
|---|---|
| know the competition: rules, points, dates, prizes | [README.md](README.md#the-competition) |
| set up on my computer | the manual: [Ubuntu](workshop/manual/setup_ubuntu.md), [Windows](workshop/manual/setup_windows.md), [macOS](workshop/manual/setup_macos.md) |
| learn step by step | the [participant guide](docs/participants/README.md) (7 short pages) |
| see every command and option | [workshop/manual/usage.md](workshop/manual/usage.md) |
| write my controller | [outerloop_controller/my_controller.py](outerloop_controller/my_controller.py), in `step()` |
| see an example controller | [outerloop_controller/examples/hello_drone.py](outerloop_controller/examples/hello_drone.py) |
| know the commands I send and the readings I get | [guide page 3](docs/participants/3_writing_your_controller.md) (formats, units, what reaches the flight controller) |
| understand the camera and directions | [guide page 4](docs/participants/4_camera_and_directions.md) |
| look at one flight | `analysis/analysis.ipynb` ([analysis/README.md](analysis/README.md)); runs are saved in `analysis/runs/` |
| test on many new layouts | `python3 evaluation/evaluate.py` ([evaluation/README.md](evaluation/README.md)); results in `evaluation/sessions/` |
| check my code follows the fair-play rules | `python3 evaluation/check_controller.py outerloop_controller/my_controller.py` ([rules](docs/participants/2_the_challenge.md#fair-play)) |
| fix an error | [guide page 6](docs/participants/6_tips_and_troubleshooting.md) |
| know what to submit, and how I am judged | [DELIVERABLES.md](DELIVERABLES.md) |
| put my submission together | `python3 output/collect.py` ([output/README.md](output/README.md)) |
| fly the real drone | [guide page 7](docs/participants/7_real_drone.md) |
| see the workshop slides | [workshop/presentation.pdf](workshop/presentation.pdf) |
| read related papers | [literature_survey/README.md](literature_survey/README.md) |

## The folders

| Folder | What is in it | You... |
|---|---|---|
| `outerloop_controller/` | your controller, an example, the drone interface (`avatic_drone/`) | **write your code here** (not in `avatic_drone/`) |
| `analysis/` | the notebook for one flight; your recorded runs | run |
| `evaluation/` | the evaluation, its notebook, the fair-play code check | run |
| `output/` | your submission (filled by `collect.py`, plus your video and report) | fill, then zip |
| `docs/participants/` | the participant guide | read |
| `workshop/` | the manual (setup, usage) and the slides | read |
| `docs/` | technical documents: architecture, arena, vehicle parameters | read if curious |
| `demo/` | a demo that pops balloons along a planned route | run to watch |
| `simulation_engine/` | the simulator | leave alone |
| `firmware/magisv2/` | the drone's real flight-controller firmware | leave alone |
| `hitl/` | the connection to the real drone (organisers) | leave alone |
| `.devcontainer/` | the Docker development environment | used by VS Code |
| `images/` | pictures for the README and the manual | |
| `resources/` | Pluto Python tutorials (reference) | |
| `literature_survey/` | related papers (links) | read if curious |
