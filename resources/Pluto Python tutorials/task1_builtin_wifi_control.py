"""
Task 1 - Control the Pluto drone with PlutoControl (built-in Wi-Fi)
==================================================================
HARDWARE : Pluto drone only (no camera module required)
LIBRARY  : plutocontrol            ->  pip install plutocontrol

WHAT THIS SCRIPT TEACHES
------------------------
  * Connect to the drone over its built-in Wi-Fi (192.168.4.1 : 23)
  * Arm / Disarm the motors
  * Take off and Land
  * The basic movements (forward / back / left / right / yaw / up / down)

HOW TO RUN
----------
  1. Power on the Pluto. On your computer join the drone's Wi-Fi network,
     usually named  "PLUTO_XXXX"  (password).
  2. pip install plutocontrol
  3. python3 task1_builtin_wifi_control.py

  A simple menu appears. Type the letter/number of an action and press Enter.

SAFETY FIRST
------------
  * The FIRST time you test "Arm", remove the propellers so nothing can
    spin up by accident on your desk.
  * Fly only in a large, open space, away from people, pets and faces.
  * Option 0 / 'x' (DISARM) instantly cuts the motors - your emergency stop.
  * Each movement is a short "nudge" that auto-centres the sticks, so the
    drone never runs away.
"""
from plutocontrol import Pluto
import sys
import tty
import termios

# Function to capture single key press (without pressing Enter)
def get_key():
    fd = sys.stdin.fileno()  # Get file descriptor of terminal
    old_settings = termios.tcgetattr(fd)  # Save current terminal settings
    try:
        tty.setraw(fd)  # Set terminal to raw mode (instant key capture)
        key = sys.stdin.read(1)  # Read one character
    finally:
        # Restore original terminal settings
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

# Initialize and connect to the drone
drone = Pluto()
drone.connect()

# Flag to track whether drone is armed (prevents repeated arming)
armed = False

# Display control instructions
print("Controls: w/a/s/d, space=takeoff, l=land, x=stop, q=quit")

# Main control loop
while True:
    key = get_key()  # Read key input in real-time

    # Takeoff sequence (only if not already armed)
    if key == " " and not armed:
        print("Takeoff")
        drone.arm()        # Arm the drone (enable motors)
        drone.take_off()   # Initiate takeoff
        armed = True

    # Landing sequence
    elif key == "l":
        print("Landing")
        drone.land()       # Land the drone safely
        drone.disarm()     # Disarm motors after landing
        armed = False

    # Forward movement
    elif key == "w":
        print("Forward")
        drone.forward()

    # Backward movement
    elif key == "s":
        print("Backward")
        drone.backward()

    # Left movement
    elif key == "a":
        print("Left")
        drone.left()

    # Right movement
    elif key == "d":
        print("Right")
        drone.right()

    # Stop (reset all movement commands)
    elif key == "x":
        print("Stop")
        drone.reset()

    # Exit program safely
    elif key == "q":
        print("Exit")
        drone.land()       # Ensure drone lands before exit
        drone.disarm()     # Disarm motors
        break