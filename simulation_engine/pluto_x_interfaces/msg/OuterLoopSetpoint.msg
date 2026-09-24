# What the outer-loop controller is currently steering to, published by
# outer_loop_host for evaluation (tracking error). Not a command.
#
# World ENU, origin = simulator/world origin; yaw counter-clockwise from east
# (REP-103), UNWRAPPED (may exceed +/-pi during multi-turn scans).

std_msgs/Header header

bool has_position
float64 x_enu_m
float64 y_enu_m
float64 z_enu_m

bool has_yaw
float64 yaw_rad
