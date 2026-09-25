# Simulation engine: organisers only

**Participants: do not edit anything in this directory.** Your code goes in
`outerloop_controller/`. Everything here defines the competition: the
drone's physics, the real MagisV2 flight-controller firmware (the inner
loops), the camera, the arena and its scoring. Changing it changes the
simulator, not your controller.

```text
pluto_x_core/        vehicle and sensor models, arena scoring logic, configs (C++, unit-tested)
pluto_x_magisv2/     MagisV2 firmware built for the PC + adapter (GPL-3.0-or-later)
pluto_x_gazebo/      Gazebo plugins (vehicle, arena), models, worlds, arena generator
pluto_x_interfaces/  ROS messages
pluto_x_ros/         fc_link: RC and telemetry link (the MSP link of the real drone)
pluto_x_autonomy/    StickCommand + RC mapping (used by the participant API), outer-loop host
                     and test controllers (used by the checks)
pluto_x_bringup/     launch files (sim, arena, competition), bridges, RViz, checks
scripts/             verification scripts (run from the repository root)
```

Firmware source: `firmware/magisv2` (vendored, never edited).

Launches:

| | |
|---|---|
| `ros2 launch pluto_x_bringup competition.launch.py` | a participant run: layout from `analysis/seed.yaml` (or `arena_seed:=random` / `N`), recorded to `analysis/runs/`, the clock starts on arm, no built-in controller |
| `ros2 launch pluto_x_bringup arena.launch.py` | the arena with the fixed layout and the host-run test controller |
| `ros2 launch pluto_x_bringup sim.launch.py` | the vehicle alone (no arena), host-run controller |

Random arenas: `pluto_x_gazebo/tools/generate_arena.py --seed N` (the
`arena_seed:=N` launch argument). Constraints: balloon surfaces at least
one drone width (16 cm) apart, all balloons within 3.5 m of take-off, and
one connected cluster (each balloon has a neighbour within 2.0 m).

Checks: `simulation_engine/scripts/check_*.sh`, and `colcon test`. Details:
`docs/`.
