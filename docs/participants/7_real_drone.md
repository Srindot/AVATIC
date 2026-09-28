# 7. The real drone

The final is on **October 3** in the drone arena, for the teams shortlisted
from the simulation round (submissions close September 30, 11:59 PM;
shortlist on October 1). The same `my_controller.py`
flies the real Pluto X. You change nothing in
your code: the runner connects to the drone over Wi-Fi instead of the
simulator.

> The hardware connection was tested against the simulator only; the
> organisers will check it on the real drone before any participant
> flight. Fly the real drone **only with an organiser present**.

## What you need

- The Pluto X with its camera module, charged.
- On your laptop: Python with NumPy, `ffmpeg` and the camera library:

  ```bash
  sudo apt install ffmpeg
  pip install plutocam numpy
  ```

  ROS is **not** needed for the real drone.

## Fly

1. Turn the drone on, place it on the floor facing a clear space, and
   join its camera module's Wi-Fi network from your laptop.
2. From the repository folder:

   ```bash
   python3 outerloop_controller/my_controller.py --hardware
   ```

3. The drone arms, and your controller flies it for 25 s.
4. At the end it **lands by itself**, then stops the motors.

**Stopping:**

- **Ctrl-C once**: the drone lands.
- **Ctrl-C twice**: the motors stop **immediately**. The drone falls: use it
  only in an emergency.
- Closing the terminal also makes it land.

## What is different from the simulator

- **Video delay.** Pictures arrive late (an estimated 0.15–0.4 s) and fewer per
  second. Slow, smooth control copes with this; fast, jerky control
  overshoots.
- **Hover throttle.** It depends on the drone and its battery. The
  simulator uses 0.76; the real value may be different, so measure it on
  your first flight and change `HOVER_THROTTLE`.
- **Colours** look different under real light: check your colour ranges on
  real pictures.
- **No scoreboard.** `drone.arena()` only counts the 25 s; the judges count
  the balloons.
- The same safety limits apply (page 3).

Organiser details, the drone's network settings and the first-flight
checklist: [`hitl/README.md`](../../hitl/README.md).
