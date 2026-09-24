# Status reported by the Pluto X flight controller: the information the real
# vehicle returns over its MSP link (MSP_STATUS, MSP_ATTITUDE, MSP_ALTITUDE,
# MSP_ANALOG). These are the FLIGHT CONTROLLER'S ESTIMATES, not ground truth.
#
# Conventions: roll + = right side down, pitch + = nose up, heading in
# degrees clockwise from north, [0, 360).

std_msgs/Header header

bool armed
bool ok_to_arm          # calibrated, level, battery ok: arming will succeed
bool calibrated
bool angle_mode
bool altitude_hold
bool healthy            # false: controller rejected input or halted

float32 roll_deg
float32 pitch_deg
float32 heading_deg
float32 altitude_m      # barometric/inertial estimate relative to start
float32 battery_v
