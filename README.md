# Autonomous Vision-Based Aerial Target Interception Challenge (AVATIC)

Simulation platform for the INFINIUM '26 Pluto X competition: pop the right
balloons with a camera-guided Pluto X within 15 s.

The simulated Pluto X is flown by its **production MagisV2 firmware**
(compiled unmodified for the PC) on Gazebo physics, with a forward camera.
Teams write the **outer-loop controller** in plain Python: camera and
telemetry in, stick commands out, the same interface as the real drone.

```text
outerloop_controller/   PARTICIPANTS: your controller + the Drone interface  -> outerloop_controller/README.md
demo/                   workshop demo: planned-route balloon popping          -> demo/README.md
simulation_engine/      the simulator (organisers only, do not edit)          -> simulation_engine/README.md
third_party/magisv2     MagisV2 firmware, vendored unmodified (GPL-3.0-or-later)
docs/                   architecture, parameters and sources, arena, legacy port
resources/              reference repositories (not built)
```

## Build

Native; needs ROS 2 Humble, `ros-humble-ros-gzharmonic` and
`ros-humble-xacro`. Run from the repository root:

```bash
source /opt/ros/humble/setup.bash && colcon build
```

## Run

Workshop demo (Gazebo + RViz camera view, results table at the end):

```bash
source install/setup.bash && ros2 launch pluto_x_demo balloon_demo.launch.py
```

A participant run: a random balloon layout that waits for your
controller. Start the simulator:

```bash
source install/setup.bash && ros2 launch pluto_x_bringup competition.launch.py
```

and, in a second terminal:

```bash
source install/setup.bash && python3 outerloop_controller/my_controller.py
```

Without windows: add `headless:=true rviz:=false` to either launch.

## Checks (organisers)

```bash
source install/setup.bash && colcon test && colcon test-result --all
```

```bash
source install/setup.bash && ./simulation_engine/scripts/check_arena.sh
```

The other checks are in `simulation_engine/scripts/`: `check_mission.sh`,
`check_yaw.sh`, `check_dynamics.sh` and `validate_legacy_sim.sh`.

## Documentation

- [docs/architecture.md](docs/architecture.md): how it fits together, the
  MagisV2 findings, verification
- [docs/arena.md](docs/arena.md): balloons, scoring, camera, the demo
- [docs/pluto_x_parameters.md](docs/pluto_x_parameters.md): vehicle
  parameters, sources and estimates
