# RC channel values sent to the Pluto X flight controller (MagisV2).
#
# Identical in meaning to the channels of MSP_SET_RAW_RC on the real Pluto X
# Wi-Fi link, so the same message drives the simulator and (through a future
# hardware bridge) the real vehicle. Channel order follows MagisV2's
# "AETR1234" map. Values are pulse widths in microseconds, nominally
# 1000..2000 with 1500 = centre.

std_msgs/Header header

uint16 roll_us      # A: aileron, 1500 = level
uint16 pitch_us     # E: elevator, 1500 = level
uint16 throttle_us  # T: 1000 = minimum
uint16 yaw_us       # R: rudder, 1500 = no yaw rate
uint16 aux1_us      # AUX1..4: mode switches (see MagisV2 RxConfig, e.g. AUX4 = arm)
uint16 aux2_us
uint16 aux3_us
uint16 aux4_us
