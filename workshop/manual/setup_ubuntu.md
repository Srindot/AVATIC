# Setup on Ubuntu Linux

There are two ways. **Choose one.**

| | A. Docker dev container (recommended) | B. Local install |
|---|---|---|
| Works on | any Ubuntu (and most other Linux) | **Ubuntu 22.04 only** |
| You install | Docker, VS Code, a few small tools | ROS 2, Gazebo and all tools yourself |
| Difficulty | easy | harder |

Both start the same way: get the code.

```bash
git clone --recursive https://github.com/Srindot/AVATIC.git
cd AVATIC
```

(If `git` is missing: `sudo apt install git`.)

---

## A. Docker dev container (recommended)

A dev container is a ready-made Ubuntu 22.04 with ROS 2, Gazebo and every
tool installed. VS Code opens the repository inside it.

**You need:** about **15 GB of free disk** (the image is a 1 GB download
and 5 GB on disk, plus the simulator build), **8 GB of RAM** (16 GB is
better) and internet for the first start.

### 1. Install Docker

```bash
curl -fsSL https://get.docker.com | sh
```

(Other ways: <https://docs.docker.com/engine/install/ubuntu/>.)

### 2. Use Docker without `sudo`

VS Code runs Docker as you, so your user must be allowed to:

```bash
sudo usermod -aG docker $USER
```

Then **log out and log in again** (or restart). Check:

```bash
docker run --rm hello-world
```

It must print *Hello from Docker!* **without** `sudo`.

### 3. Install xhost

This lets the container open the Gazebo and RViz windows on your screen:

```bash
sudo apt install x11-xserver-utils
```

(Arch Linux: `sudo pacman -S xorg-xhost`.)

### 4. Install VS Code and the Dev Containers extension

- VS Code: <https://code.visualstudio.com/> (or `sudo snap install code --classic`)
- In VS Code, *Extensions* (Ctrl + Shift + X), search **Dev Containers**
  (`ms-vscode-remote.remote-containers`) and install it.

### 5. (Optional) NVIDIA graphics card

The normal container works on every computer: Gazebo draws with your
usual graphics driver. Only if you want Gazebo to use an **NVIDIA** card:

1. The NVIDIA driver must work: `nvidia-smi` must print your card. If not:

   ```bash
   sudo ubuntu-drivers install
   ```

   and restart.
2. Install the NVIDIA Container Toolkit:

   ```bash
   curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
     && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
       sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
       sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
   sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl restart docker
   ```

3. In step 6, choose **Linux + NVIDIA GPU** instead of **Linux**.

### 6. Open the container

1. Open the `AVATIC` folder in VS Code (*File → Open Folder*).
2. Press **Ctrl + Shift + P** and choose **Dev Containers: Reopen in
   Container**.

   ![Reopen in Container](../../images/guide/reopen.png)

3. Choose **Linux** (or **Linux + NVIDIA GPU**).
4. Wait. The first time, it downloads the image and builds the simulator
   (several minutes). The log ends with `== done`. Next time it opens in
   seconds.
5. Open a terminal: *Terminal → New Terminal*. You are in the repository
   folder and everything is ready.

**Test it:**

```bash
ros2 launch pluto_x_demo balloon_demo.launch.py
```

Gazebo and RViz open and the drone pops balloons. Then read
[usage.md](usage.md).

### Problems

| What you see | What to do |
|---|---|
| `permission denied ... docker.sock` | Step 2: add yourself to the `docker` group, then log out and in. |
| `xhost: command not found` | Step 3. |
| `libnvidia-ml.so.1: cannot open shared object file` | The NVIDIA driver is missing: choose plain **Linux**, or install the driver (step 5). |
| No Gazebo window appears | Run `xhost +local:docker` in a normal terminal, then *Rebuild Container*. |
| You want a clean start | Ctrl + Shift + P → **Dev Containers: Rebuild Container**. |

---

## B. Local install (Ubuntu 22.04 only)

> This is the harder way: you install ROS 2 and Gazebo yourself. If
> anything goes wrong, the dev container (A) is usually faster.

Check your version: `lsb_release -a` must say **22.04**.

### 1. Install ROS 2 Humble

Follow the official guide, the "Debian packages" way (the *desktop*
install is fine):
<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

(Do not use the build-from-source guide: the next steps need the packages
in `/opt/ros/humble`.)

### 2. Install Gazebo Harmonic

Add the Gazebo package source as described in
<https://gazebosim.org/docs/harmonic/install_ubuntu/>, then:

```bash
sudo apt install gz-harmonic ros-humble-ros-gzharmonic
```

### 3. Install the other tools

```bash
sudo apt install build-essential cmake pkg-config patch \
    ros-humble-xacro ros-humble-rviz2 ros-humble-ament-cmake-gtest python3-colcon-common-extensions \
    libeigen3-dev libyaml-cpp-dev ffmpeg \
    python3-numpy python3-yaml python3-matplotlib python3-pytest python3-pip python3-opencv
pip install notebook
```

Use Ubuntu's `python3-opencv`, **not** `pip install opencv-python`: the pip
version installs NumPy 2, which breaks ROS and matplotlib.

### 4. Build the simulator

Once, from the `AVATIC` folder (a few minutes):

```bash
source /opt/ros/humble/setup.bash
colcon build
```

### 5. Load the environment

In **every new terminal**, from the `AVATIC` folder:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
```

Tip: add these lines to the end of `~/.bashrc` (with a `cd` to the
repository first), so every terminal is ready.

### 6. Test it

```bash
ros2 launch pluto_x_demo balloon_demo.launch.py
```

Then read [usage.md](usage.md).

### Problems

| What you see | What to do |
|---|---|
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` |
| `Package 'pluto_x_...' not found` | `source install/setup.bash` again (after every build, in every open terminal) |
| `jupyter: command not found` | `python3 -m notebook ...`, or log out and in (pip puts it in `~/.local/bin`) |
| Build error about `gz-sim8` or `gz-plugin2` | Step 2: `gz-harmonic` is missing |
