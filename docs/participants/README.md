# Participant guide

Welcome to AVATIC. You will write a small Python program that flies a Pluto X
drone with its camera and pops balloons. You do not need to know ROS or
anything about drones to start.

Read the pages in order the first time:

| | Page | What you learn |
|---|---|---|
| 1 | [Getting started](1_getting_started.md) | install, build, first flight in the simulator (15 min) |
| 2 | [The challenge](2_the_challenge.md) | the rules: balloons, points, time, what you submit |
| 3 | [Writing your controller](3_writing_your_controller.md) | the template and every function you can use |
| 4 | [Camera and directions](4_camera_and_directions.md) | what the camera sees, which way is which, distance from pixels |
| 5 | [Testing and improving](5_testing_and_improving.md) | look at your flights, test on many layouts |
| 6 | [Tips and troubleshooting](6_tips_and_troubleshooting.md) | how to start, common errors and fixes, glossary |
| 7 | [The real drone](7_real_drone.md) | flying the same code on the real Pluto X |

**The short version:**

1. Build once: `source /opt/ros/humble/setup.bash && colcon build` (in the repository folder).
2. Write your code in `outerloop_controller/my_controller.py`, in `MyController.step()`.
3. Fly it: `ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/my_controller.py`.
4. Look at the flight: open `analysis/analysis.ipynb`.
5. When it works, test it on 10 new layouts: `python3 evaluation/evaluate.py --runs 10`.
