# AVATIC: Autonomous Vision-Based Aerial Target Interception Challenge

INFINIUM '26. Write a Python program that flies a **Pluto X** drone using
its **camera** and pops the right balloons in **15 seconds**.

| Balloon | green | blue | yellow | red |
|---|---|---|---|---|
| Points | +100 | +50 | +25 | **−75: avoid** |

You develop in a simulator that runs the drone's **real flight-controller
software**, and the same program then flies the real drone.

## Participants: start here

**Read the [participant guide](docs/participants/README.md).** It explains
everything from installing to your first balloon.

The short version, from this folder:

```bash
source /opt/ros/humble/setup.bash && colcon build          # once
source install/setup.bash                                  # in every new terminal
ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/my_controller.py
```

Your code goes in [`outerloop_controller/my_controller.py`](outerloop_controller/my_controller.py).
Look at your flights with `analysis/analysis.ipynb`, and test on new layouts
with `python3 evaluation/evaluate.py --runs 10`.

## What is in this repository

```text
outerloop_controller/   YOUR code: my_controller.py, an example, the drone interface
analysis/               every run is saved here; notebook to look at a run
evaluation/             test your controller on many new layouts; report notebook
docs/participants/      the participant guide
hitl/                   the connection to the real drone (organisers)
demo/                   workshop demo: a pre-planned route popping balloons
simulation_engine/      the simulator (organisers only; do not edit)
firmware/magisv2        the drone's real firmware, unmodified (GPL-3.0-or-later)
docs/                   technical documentation (organisers)
resources/, literature_survey/   reference material (not built)
```

## Organisers

Workshop demo (Gazebo + RViz camera view, result table at the end):

```bash
ros2 launch pluto_x_demo balloon_demo.launch.py
```

Checks:

```bash
colcon test && colcon test-result --all
```

```bash
python3 -m pytest hitl/tests
```

```bash
./simulation_engine/scripts/check_arena.sh
```

The other end-to-end checks are in `simulation_engine/scripts/`
(`check_mission.sh`, `check_yaw.sh`, `check_dynamics.sh`,
`validate_legacy_sim.sh`).

Technical documentation:

- [docs/architecture.md](docs/architecture.md): how it fits together, the
  firmware findings (including the **open altitude-hold question**,
  finding 7), verification
- [docs/arena.md](docs/arena.md): balloons, scoring, arena rules, camera,
  the demo
- [docs/pluto_x_parameters.md](docs/pluto_x_parameters.md): vehicle
  parameters, sources and estimates
- [hitl/README.md](hitl/README.md): the hardware backend, the MSP test
  bridge, and the first-flight checklist
- [simulation_engine/README.md](simulation_engine/README.md): the packages
  and launch files
