# 1. Getting started

About 15 minutes. At the end you will have flown the drone in the simulator.

## What you need

- **Ubuntu 22.04** with **ROS 2 Humble** and **Gazebo Harmonic**. On a lab
  machine these are usually installed already. If not:

  ```bash
  sudo apt install ros-humble-desktop ros-humble-ros-gzharmonic ros-humble-xacro
  ```

- **Python tools** for your controller and for looking at your flights:

  ```bash
  sudo apt install ffmpeg python3-numpy python3-yaml python3-matplotlib
  pip install notebook opencv-python
  ```

  (OpenCV is optional. NumPy is enough to start.)

## Build (once)

Open a terminal **in the repository folder** (the folder with this
`README.md`, `outerloop_controller/`, `simulation_engine/` ...) and run:

```bash
source /opt/ros/humble/setup.bash && colcon build
```

It takes a few minutes the first time. You only need to build again if the
organisers update the simulator. **Your own Python code never needs a
build.**

## Every new terminal

Each new terminal needs these two lines once (they tell the terminal where
ROS and the simulator are). Run every command from the repository folder:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
```

Tip: add them to the end of your `~/.bashrc` (with `cd` to the repository
first) so you never forget.

## Your first flight

```bash
ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/my_controller.py
```

What happens:

1. The Gazebo window (the 3-D world) and RViz (the drone's camera view)
   open. Without windows: add `headless:=true rviz:=false` to the command.
2. After about 4 s the drone's flight controller is ready. Your controller
   **arms** the drone (starts the motors). The **15 s clock starts now.**
3. The template climbs to 1 m and hovers. It does not chase balloons: that
   is your job.
4. After 15 s the simulation pauses and the result table is printed in the
   terminal (look for `score`).
5. Press **Ctrl-C** to stop the simulator. Lines like
   `[ERROR] [gz sim-1]: process has died ... exit code -2` are normal after
   Ctrl-C: they only mean "stopped".

**One run per launch.** For the next try, start the command again.

## Two terminals (optional)

Handy when you want your controller's output separate from the simulator's:

Terminal 1 (the simulator; it waits for a controller):

```bash
ros2 launch pluto_x_bringup competition.launch.py
```

Terminal 2 (your controller; remember the two `source` lines):

```bash
python3 outerloop_controller/my_controller.py
```

## See the camera and the balloons

Run the example. It takes off, turns slowly and prints how many pixels of
each balloon colour the camera sees:

```bash
ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/examples/hello_drone.py
```

In the starting layout (seed 42) every balloon is behind or beside the
drone, so the colours appear as it turns.

## Look at what happened

Every run is saved automatically in `analysis/runs/<date and time>/`.
Open the analysis notebook:

```bash
jupyter notebook analysis/analysis.ipynb
```

Then choose *Kernel → Restart & Run All*. The first thing shown is your
score. Below it: a map of your flight over the balloons, your commands, and
camera pictures from key moments.

**Next:** [2. The challenge](2_the_challenge.md)
