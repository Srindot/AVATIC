# Setup on Windows

The simulator runs on Linux. On Windows you use **WSL2**: a real Ubuntu
running inside Windows. Windows 11 (or an up-to-date Windows 10) also
shows Linux windows such as Gazebo on your desktop (this is called WSLg).

There are two ways. **Choose one.**

| | A. Docker dev container (recommended) | B. Local install inside WSL2 |
|---|---|---|
| You install | WSL2, Docker Desktop, VS Code | WSL2, then ROS 2, Gazebo and all tools inside Ubuntu |
| Difficulty | easy | harder |

**You need:** about **15 GB of free disk** and **8 GB of RAM** (16 GB is
better).

---

## Step 0 (both ways): install WSL2 with Ubuntu 22.04

1. Open **PowerShell as administrator** (right-click the Start button →
   *Terminal (Admin)*) and run:

   ```powershell
   wsl --install -d Ubuntu-22.04
   ```

2. **Restart** Windows.
3. Open **Ubuntu 22.04** from the Start menu. The first time, it asks for
   a user name and a password (for `sudo` inside Ubuntu).
4. In PowerShell, update WSL (this gives the Linux windows support):

   ```powershell
   wsl --update
   ```

From now on, "the Ubuntu terminal" means that Ubuntu window.

---

## A. Docker dev container (recommended)

### 1. Install Docker Desktop

- Download: <https://www.docker.com/products/docker-desktop/>
- Install it with the **WSL 2 backend** (the default).
- In Docker Desktop: *Settings → Resources → WSL integration* → turn on
  **Ubuntu-22.04** → *Apply & restart*.

Check, in the Ubuntu terminal:

```bash
docker run --rm hello-world
```

### 2. Install git and xhost inside Ubuntu

In the Ubuntu terminal:

```bash
sudo apt update && sudo apt install git x11-xserver-utils
```

### 3. Install VS Code and two extensions

- VS Code for Windows: <https://code.visualstudio.com/>
- Extensions (Ctrl + Shift + X): **WSL** (`ms-vscode-remote.remote-wsl`)
  and **Dev Containers** (`ms-vscode-remote.remote-containers`).

### 4. Get the code, inside Ubuntu

Clone the repository **inside Ubuntu** (not in a Windows folder: it would
be very slow). In the Ubuntu terminal:

```bash
cd ~
git clone --recursive https://github.com/Srindot/AVATIC.git
cd AVATIC
code .
```

`code .` opens VS Code connected to Ubuntu (the first time it installs a
small helper).

### 5. Open the container

1. In VS Code, press **Ctrl + Shift + P** and choose **Dev Containers:
   Reopen in Container**.

   ![Reopen in Container](../../images/guide/reopen.png)

2. Choose **WSL**.
3. Wait. The first time it downloads the image and builds the simulator
   (several minutes); the log ends with `== done`.
4. Open a terminal: *Terminal → New Terminal*. Everything is ready.

An NVIDIA card needs only its normal **Windows** driver: nothing else.

**Test it:**

```bash
ros2 launch pluto_x_demo balloon_demo.launch.py
```

Then read [usage.md](usage.md).

### Problems

| What you see | What to do |
|---|---|
| `docker: command not found` in Ubuntu | Docker Desktop → *Settings → Resources → WSL integration* → turn on Ubuntu-22.04 |
| `xhost: command not found` | Step 2 |
| No windows appear | Run `wsl --update` in PowerShell, restart, and check that Ubuntu apps (e.g. `sudo apt install x11-apps && xeyes`) can open windows |
| Everything is very slow | Keep the repository in Ubuntu's own folders (`~/AVATIC`), not under `/mnt/c/...` |

---

## B. Local install inside WSL2

Do [the local install for Ubuntu](setup_ubuntu.md#b-local-install-ubuntu-2204-only)
inside the Ubuntu terminal: it is the same, because WSL2 is a real Ubuntu
22.04. Keep the repository in `~/AVATIC`. Gazebo and RViz windows appear on
your Windows desktop through WSLg.
