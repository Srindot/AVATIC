# How to Use Each Script — Simple Guide

This is a plain, step-by-step guide for running the three tutorial scripts.
No prior experience needed. Just follow the steps in order.

---

## Before anything: install Python

1. Install **Python 3.8 or newer** from [https://www.python.org/downloads/](https://www.python.org/downloads/).
2. Open a terminal:

   - **Windows:** open *Command Prompt* or *PowerShell*
   - **Mac:** open *Terminal*
3. Check it works:

   ```bash
   python3 --version
   ```

   You should see something like `Python 3.11.x`.

> Note: If `python3` says "not found" on Windows, try just `python` instead.

---

## The one thing to always remember: TWO Wi-Fi networks

The Pluto has two Wi-Fi networks. Which one you join decides which script works.

| Network                | Name looks like | Used for                     |
| ---------------------- | --------------- | ---------------------------- |
| **Drone Wi-Fi**  | `Pluto_XXXX`  | flying the drone from Python |
| **Camera Wi-Fi** | PlutoXCam_      | getting the video feed       |

Enter password for the drone Wi-Fi

You join a Wi-Fi the normal way (click the Wi-Fi icon → pick the network → type the password).

---

## Safety (please read once)

- The **first time** you test arming, **take the propellers off** so nothing spins on your desk.
- Fly only in a **big, open space**, away from people, pets, and faces.
- The **disarm** key is your emergency stop. Know where it is before you fly.

---

## Script 1 — Fly the drone from Python

**File:** `task1_builtin_wifi_control.py`

### What you need to install

```bash
pip install plutocontrol
```

### How to use it

1. Turn on the Pluto.
2. On your computer, join the **drone Wi-Fi** (`PLUTO_XXXX`, password ).
3. Run the script:

   ```bash
   python3 task1_builtin_wifi_control.py
   ```
4. The script reads **single key presses** (no need to press Enter). Just tap
   a key:

   | Key           | Does                          |
   | ------------- | ----------------------------- |
   | `space`     | Take off (arms automatically) |
   | `w` / `s` | Forward / backward            |
   | `a` / `d` | Left / right                  |
   | `x`         | Stop (re-centre the sticks)   |
   | `l`         | Land                          |
   | `q`         | Quit (lands, then exits)      |
5. A good first run: `space` (take off) → try `w` / `a` / `s` / `d` → `x`
   (stop) → `l` (land) → `q` (quit).

> Note: this real-time key capture uses the Unix terminal (macOS / Linux). On
> Windows, run it from WSL or a Unix-like terminal.

---

## Script 2 — Video + control together

**File:** `task2_camera_wifi_vision.py`

This script shows the live video in an **ffplay** window and lets you fly the
drone with **instant single key presses** (no Enter needed). No OpenCV needed.

### What you need to install

```bash
pip install plutocontrol plutocam
```

You also need **ffplay** (it comes with ffmpeg):

- **Mac:** `brew install ffmpeg`
- **Windows:** download from [https://ffmpeg.org/download.html](https://ffmpeg.org/download.html) and add it to PATH
- **Linux:** `sudo apt install ffmpeg`

### How to use it

1. Fit the camera module and switch the drone to **Camera Wi-Fi mode**.
2. On your computer, join the **camera Wi-Fi**.
3. Run the script:

   ```bash
   python3 task2_camera_wifi_vision.py
   ```
4. A video window opens. Keep the **terminal** focused and just tap a key
   (no Enter) — the video window only shows the feed:

   | Key | Does |
   | --- | ---- |
   | `space` | Take off (arms first) |
   | `w` / `s` | Forward / backward |
   | `a` / `d` | Left / right |
   | `j` / `k` | Yaw left / right |
   | `r` / `f` | Up / down |
   | `x` | Stop (re-centre the sticks) |
   | `l` | Land |
   | `0` | Disarm (emergency stop) |
   | `q` | Quit (lands, then exits) |

> Note: the single-key control uses the Unix terminal (macOS / Linux). On
> Windows, run it from WSL or a Unix-like terminal.

---

## Script 3 — Watch the video while flying from the phone

**File:** `task3_camera_feed_mobile_flight.py`

In this one the **phone flies the drone** and the **computer just shows the
video**. This is the simplest script — it only opens the feed in an **ffplay**
window. No OpenCV needed.

### What you need to install

```bash
pip install plutocam
```

(No `plutocontrol` here — Python does not fly the drone in this script.)
You also need **ffplay** (it comes with ffmpeg):

- **Mac:** `brew install ffmpeg`
- **Windows:** download from [https://ffmpeg.org/download.html](https://ffmpeg.org/download.html) and add it to PATH
- **Linux:** `sudo apt install ffmpeg`

### How to use it

1. Switch the drone to **Camera Wi-Fi mode**.
2. **On the phone:** join the **drone Wi-Fi** and open the Pluto Controller app.
3. **On the computer:** join the **camera Wi-Fi**.
4. Run the script:
   ```bash
   python3 task3_camera_feed_mobile_flight.py
   ```
5. A video window opens and shows the live feed. Fly the drone from the phone.
6. To stop: close the video window, or press **Ctrl+C** in the terminal.

---

## If something goes wrong

| Problem                              | Try this                                                                  |
| ------------------------------------ | ------------------------------------------------------------------------- |
| "Could not connect" (Script 1)       | Make sure you joined`PLUTO_XXXX` and no phone/app is already connected. |
| "Connection refused" (Script 2/3)    | You're not on the camera Wi-Fi, or the drone isn't in camera mode.        |
| No video window appears (Script 2/3) | `ffplay` isn't installed or not on PATH — install ffmpeg (see above).  |
| Black/empty video window             | Camera module not powered, wrong Wi-Fi, or a firewall/VPN is blocking it. |
| `ModuleNotFoundError`              | Run the matching`pip install …` line above again.                      |
| Keys do nothing (Script 1 / 2)       | Click the **terminal** window (not the video window) so it has focus.     |
| `pip` not found                    | Try`pip3` instead of `pip`.                                           |

---

## Quick reference

| Script   | Install                   | Also needs      | Computer joins | Phone joins |
| -------- | ------------------------- | --------------- | -------------- | ----------- |
| Script 1 | `plutocontrol`          | —              | Drone Wi-Fi    | —          |
| Script 2 | `plutocontrol plutocam` | ffmpeg (ffplay) | Camera Wi-Fi   | —          |
| Script 3 | `plutocam`              | ffmpeg (ffplay) | Camera Wi-Fi   | Drone Wi-Fi |
