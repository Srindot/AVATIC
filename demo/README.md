# Workshop demo ("hero" simulation)

The Pluto X takes off, turns to face each balloon, flies into it, and pops
every non-red balloon, scoring 350 / 350. Red balloons (−75) are avoided
with at least 1 m of clearance. The run lasts 35 s and ends with the
results table.

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch pluto_x_demo balloon_demo.launch.py
```

- **Gazebo** shows the whole field.
- **RViz** shows the drone's camera, the live score and time left, and the
  flight path.

Competition length (15 s: blue → green, 150 points):

```bash
ros2 launch pluto_x_demo balloon_demo.launch.py time_limit_s:=15 route_file:=$(ros2 pkg prefix pluto_x_demo)/share/pluto_x_demo/config/balloon_demo_15s.yaml
```

Without windows: add `headless:=true rviz:=false`.

**This is not a solution to the challenge.** It flies a hand-planned route
through balloon positions it reads from the arena file, using the
simulator's exact drone position. Participants have neither: their
controller (`outerloop_controller/`) must find the balloons with the
camera, and the competition places them from a seed (a fixed development
seed for tuning, unseen random seeds for evaluation and judging).

```text
demo/
  pluto_x_demo/balloon_demo.py   the demo controller
  config/balloon_demo.yaml       35 s route (the six non-red balloons)
  config/balloon_demo_15s.yaml   15 s route
  launch/balloon_demo.launch.py
```

It uses the default arena layout
(`simulation_engine/pluto_x_gazebo/config/arena_default.yaml`). The
balloon indices in the route files refer to that layout.
