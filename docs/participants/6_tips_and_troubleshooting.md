# 6. Tips and troubleshooting

## A plan for a first controller

Build it step by step, and fly after each step:

1. **Take off.** The template already climbs to 1 m. Climb straight up
   first: tilting or turning on the ground pushes the drone into the floor.
2. **Search.** Turn on the spot (`yaw_rate` about 0.3–0.5) until a good
   balloon colour appears in the picture.
3. **Face it.** Steer `yaw_rate` with how far the balloon is from the
   picture centre, for example `yaw_rate = 0.8 * (u - 640) / 640`
   (a *proportional* controller: a big error gives a big correction).
4. **Match its height.** Raise or lower your altitude target so the
   balloon moves towards the middle row of the picture. Remember the
   camera tilts with the drone.
5. **Approach.** Once it is roughly centred, fly forward gently
   (`pitch` about 0.1–0.25). Stop pushing forward if it drifts off
   centre.
6. **Pop and continue.** When the balloon fills a big part of the
   picture and then disappears, it has probably popped. Search for the
   next one.

Write it as a **state machine**: keep the current state in `self.state`
(`'takeoff'`, `'search'`, `'approach'` ...) and switch when a condition is
met.

## Good habits

- **Go for green first.** Green is 100 points, yellow only 25.
- **Stay away from red.** Before flying forward, check that no red balloon
  is in the middle of the picture. If one is, go around it.
- **Smooth beats fast.** Big, sudden commands make the drone wobble and
  lose the balloon from the picture. The real drone's video also arrives
  late, which makes jerky controllers worse.
- **The drone drifts.** Wind and small errors move it even with centred
  sticks. Keep correcting with what the camera shows.
- **Use the time well.** 15 s is short, and a full turn at the maximum
  `yaw_rate` takes about 6 s. Remember where you saw balloons (your heading
  + the balloon's bearing) and turn the short way to the next one after a
  pop, instead of searching again.
- **Estimate your own drift from the camera.** There is no speed reading.
  If a balloon you are facing keeps sliding sideways in the picture (its
  direction in the world, heading + bearing, keeps changing), the drone is
  drifting: correct with a little `roll`. Without this the drone often
  slides past a balloon 0.3–0.5 m away without touching it.
- **Brake before you arrive.** The drone keeps its speed after you level
  it. When a balloon gets close (large in the picture), reduce `pitch`, or
  tilt back briefly after a pop.
- **Red in the way? Pick another target.** Trying to fly around a red
  balloon often ends in circling it for seconds. Choosing a different
  balloon, or approaching from another side, works better.
- **Balloons cut off at the picture edge look far away.** Their size is
  wrong: trust the size only when the whole balloon is in the picture.
- **Change one thing at a time**, and compare runs on the same layout.
- **Print a little, not a lot.** One line per state change is enough, and
  `print(..., flush=True)` shows it at once.
- **Test on many layouts** before you are happy ([page 5](5_testing_and_improving.md)).

## Common problems

| What you see | What it means / what to do |
|---|---|
| `avatic_drone needs the simulator environment` | Run the two `source` lines in this terminal ([page 1](1_getting_started.md#every-new-terminal)). |
| `ros2: command not found` | Same: run `source /opt/ros/humble/setup.bash`. |
| `Package 'pluto_x_...' not found` | Run `source install/setup.bash` again (or open a new terminal). A terminal only knows the packages that existed when it was sourced, so this happens after a new build. |
| `can't open file ... my_controller.py` | Run commands from the repository folder (`cd` into it). |
| `controller file not found: ...` | Give the path from the repository folder: `controller:=outerloop_controller/my_controller.py`. |
| `ModuleNotFoundError: No module named 'avatic_drone'` | Started with `python3` directly from another folder: keep your files in `outerloop_controller/`, or add `import sys; sys.path.insert(0, '<repo>/outerloop_controller')` before the import. (Started by the launch or `evaluate.py`, a controller anywhere works.) |
| `no simulator found: is it running?` | Start the simulator first (terminal 1), or use the one-command way. |
| `the simulator is paused: its run is over` | One run per launch: stop it (Ctrl-C) and start it again. |
| `[ERROR] ... process has died ... exit code -2` after Ctrl-C | Normal: the simulator stopped because you asked it to. |
| `... command beyond the safety cap: clipped` | You asked for more than the limits (page 3). Harmless, but your controller is not getting what it asks for. |
| The drone drops to the ground | `altitude_hold=True` (do not use it), or the throttle was cut suddenly. |
| The altitude reading jumps to about 0 in the air | You cut the throttle hard. The flight controller reset its altitude (page 3, "Two things to avoid"). |
| The drone slides away while "hovering" | Normal: wind and small errors. Correct with the camera. |
| Your code crashed during a run | The traceback is printed above `step() raised an error`. Fix it and fly again. |
| `jupyter: command not found` / `jupyter-notebook not found` | `pip install notebook`; if it is still not found, use `python3 -m notebook analysis/analysis.ipynb` (or log out and in). In the dev container, open the notebook in VS Code. |
| The notebook says `no runs in ... fly one first` | Fly a run first. Runs are saved in `analysis/runs/`. |
| Everything is slow | Close other programs and simulations, and use `headless:=true rviz:=false`. |

## Glossary

| Word | Meaning |
|---|---|
| **arm / disarm** | start / stop the motors |
| **roll, pitch, yaw** | tilting sideways, tilting forwards, turning on the spot |
| **throttle** | how hard the motors push up (0 to 1) |
| **hover** | staying at the same height (throttle about 0.76) |
| **telemetry** | the readings the drone sends back |
| **barometer (baro)** | the air-pressure sensor that measures height |
| **flight controller / firmware** | the small computer on the drone that keeps it stable; the simulator runs the real one (MagisV2) |
| **failsafe** | what the drone does by itself when your commands stop |
| **seed** | a number that picks the balloon layout |
| **ROS 2, Gazebo** | the robotics tools the simulator is built on. You only type the launch command. |
| **launch, `name:=value`** | `ros2 launch` starts the simulator; `name:=value` sets an option |
| **headless** | without windows |
| **Hz** | times per second |
| **MSP** | the protocol the real drone speaks over Wi-Fi (handled for you) |

**Next:** [7. The real drone](7_real_drone.md)
