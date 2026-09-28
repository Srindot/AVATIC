# AVATIC manual

Everything you need to set up the AVATIC simulator and use it. Pick the
setup page for your computer, then read the usage page.

**DRONE ARENA @ Infinium:** teams of 2–4, prizes worth ₹20,000. Register
by **September 29, 11:59 PM** ([form](https://forms.gle/6x25JHhfRKqESipF8)).
Simulation submission: **September 30, 11:59 PM**; shortlist: **October 1**. Final in the drone
arena: **October 3**. Details: [README](../README.md#the-competition).

## 1. Set up

| Your computer | Read | The easiest way |
|---|---|---|
| **Ubuntu Linux** | [manual/setup_ubuntu.md](manual/setup_ubuntu.md) | Docker dev container, or local install on Ubuntu 22.04 |
| **Windows** | [manual/setup_windows.md](manual/setup_windows.md) | WSL2 + Docker dev container |
| **macOS** | [manual/setup_macos.md](manual/setup_macos.md) | Docker dev container |

Not sure which way to choose? Use the **Docker dev container**: it is the
same on every computer and installs everything for you. The local install
only works on Ubuntu 22.04 and is harder.

## 2. Use it

[manual/usage.md](manual/usage.md) explains:

- where to write your code;
- how to run the simulator, and every option of every command;
- the **demo**, the **hello drone** example, **analysis** and
  **evaluation**, and what each one is for.

## 3. Learn to write a controller

The [participant guide](../docs/participants/README.md) explains the rules,
every function you can use, the camera and directions, and ideas for a
first controller.

## 4. Submit

[DELIVERABLES.md](../DELIVERABLES.md): what to submit (code, best run,
notebooks, video, technical report) and how to collect it in `output/`
with `python3 output/collect.py`.

## Help

Questions? Contact **Srinath Bhamidipati** at
[srinath.bhamidipati@research.iiit.ac.in](mailto:srinath.bhamidipati@research.iiit.ac.in).
