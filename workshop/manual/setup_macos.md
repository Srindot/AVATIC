# Setup on macOS

ROS 2 and Gazebo do not run directly on macOS, so you use the **Docker dev
container**: a ready-made Ubuntu 22.04 with everything installed, which VS
Code opens for you. It works on Intel Macs and on Apple Silicon Macs (M1,
M2, M3, M4).

**You need:** about **15 GB of free disk** (the image is a 1 GB download
and 5 GB on disk, plus the simulator build), **8 GB of RAM** (16 GB is
better) and internet for the first start.

## 1. Install Docker Desktop

- Download: <https://www.docker.com/products/docker-desktop/>. Choose
  **Apple Silicon** or **Intel** to match your Mac ( → *About This Mac*).
- In Docker Desktop → *Settings → Resources*, give Docker at least
  **8 GB of memory**.
- **Apple Silicon only:** *Settings → General* → turn on **"Use Rosetta
  for x86_64/amd64 emulation on Apple Silicon"**. The AVATIC image is built
  for Intel/AMD computers, and Rosetta runs it much faster than plain
  emulation.

Check, in the Terminal app:

```bash
docker run --rm hello-world
```

## 2. Install XQuartz

XQuartz shows the container's Gazebo and RViz windows on your Mac (it
includes the `xhost` tool).

1. Download and install: <https://www.xquartz.org/>, then **log out and
   in** once.
2. Open XQuartz → *Settings → Security* → tick **Allow connections from
   network clients**.
3. Quit XQuartz and open it again.

Before you open the container, and again **after every XQuartz restart**,
run in the Terminal app:

```bash
xhost +localhost
```

## 3. Install git and VS Code

```bash
xcode-select --install
```

(This installs `git` and other developer tools.)

- VS Code: <https://code.visualstudio.com/>
- In VS Code, *Extensions* (Cmd + Shift + X), search **Dev Containers**
  (`ms-vscode-remote.remote-containers`) and install it.

## 4. Get the code

```bash
git clone --recursive https://github.com/Srindot/AVATIC.git
cd AVATIC
code .
```

(If `code` is not found: in VS Code press Cmd + Shift + P → *Shell Command:
Install 'code' command in PATH*.)

## 5. Open the container

1. In VS Code, press **Cmd + Shift + P** and choose **Dev Containers:
   Reopen in Container**.

   ![Reopen in Container](../../images/guide/reopen.png)

2. Choose **MacOS**.
3. Wait. The first time it downloads the image and builds the simulator
   (longer on Apple Silicon); the log ends with `== done`.
4. Open a terminal: *Terminal → New Terminal*. Everything is ready.

**Test it**, first without windows (this checks the simulator itself):

```bash
ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/examples/hello_drone.py headless:=true rviz:=false
```

Then with windows:

```bash
ros2 launch pluto_x_demo balloon_demo.launch.py
```

Then read [usage.md](usage.md).

## Good to know

- On a Mac, Gazebo draws **without the graphics card**, so the windows are
  slower than on Linux. Runs without windows (`headless:=true rviz:=false`)
  and the evaluation are not affected: use them for most of your work.
- The analysis and evaluation **notebooks** open directly in VS Code: no
  windows needed.

## Problems

| What you see | What to do |
|---|---|
| No window appears (`cannot open display`) | Is XQuartz running? Run `xhost +localhost` again. If it still fails, try `xhost +` (lets any program connect: only on a trusted network). |
| The Gazebo window opens but stays black | Use headless runs and RViz; tell the organisers what you see. |
| Everything is very slow | Apple Silicon: check that Rosetta is on (step 1). Give Docker more memory. Close other programs. |
| You want a clean start | Cmd + Shift + P → **Dev Containers: Rebuild Container**. |
