#!/usr/bin/env bash
# Yaw-axis checks in Gazebo with the MagisV2 firmware (see
# pluto_x_bringup/scripts/yaw_check.py). Run natively after `colcon build`
# with the workspace sourced:
#
#   simulation_engine/scripts/check_yaw.sh            # both: open-loop stick steps, closed-loop scan
#   simulation_engine/scripts/check_yaw.sh open_loop
#   simulation_engine/scripts/check_yaw.sh closed_loop
#   CONFIG_FILE=/abs/path/vehicle.yaml simulation_engine/scripts/check_yaw.sh
#   WIND=1 simulation_engine/scripts/check_yaw.sh     # keep the configured wind
#
# Wind is switched off by default: these checks isolate the yaw axis (yaw
# dynamics, the firmware's yaw loop, yaw -> position/altitude coupling). With
# wind, the position-hold result measures the outer loop's gust rejection
# instead (see simulation_engine/scripts/check_mission.sh).
set -euo pipefail

# on any exit (error, Ctrl-C) never leave a simulation running
SIM_PGID=""
trap 'if [[ -n "${SIM_PGID}" ]]; then kill -KILL -- "-${SIM_PGID}" 2>/dev/null || true; fi' EXIT

readonly SHUTDOWN_GRACE_S=10
PREFIX="$(ros2 pkg prefix pluto_x_autonomy)"   # fails here (set -e) if not sourced
readonly SHARE="${PREFIX}/share/pluto_x_autonomy"
modes=("${@:-open_loop closed_loop}")
failures=0

source_config="${CONFIG_FILE:-$(ros2 pkg prefix pluto_x_core)/share/pluto_x_core/config/pluto_x_estimated.yaml}"
mkdir -p log/yaw
if [[ "${WIND:-0}" == 1 ]]; then
  config="${source_config}"
else
  config="${PWD}/log/yaw/vehicle_no_wind.yaml"
  python3 - "${source_config}" "${config}" <<'PY'
import sys, yaml
c = yaml.safe_load(open(sys.argv[1]))
c['wind']['enabled'] = False
yaml.safe_dump(c, open(sys.argv[2], 'w'), sort_keys=False)
PY
fi
echo "vehicle config: ${config}"

run_mode() {  # <mode>
  local mode="$1" controller params
  local out="log/yaw/${mode}"
  rm -rf "${out}" && mkdir -p "${out}"  # keeps log/yaw/vehicle_no_wind.yaml
  if [[ "${mode}" == open_loop ]]; then
    controller=pluto_x_autonomy.examples.yaw_open_loop:YawOpenLoopTest
    params="${SHARE}/config/yaw_open_loop.yaml"
  else
    controller=pluto_x_autonomy.examples.waypoint:WaypointController
    params="${SHARE}/config/yaw_scan.yaml"
  fi
  echo "=== ${mode} (${controller})"
  setsid ros2 launch pluto_x_bringup sim.launch.py headless:=true \
    controller:="${controller}" ${params:+controller_params_file:="${params}"} \
    config_file:="${config}" > "${out}/sim.log" 2>&1 &
  local sim_pid=$! status=0
  SIM_PGID="${sim_pid}"
  ros2 run pluto_x_bringup yaw_check.py "${mode}" --out "${out}" 2>&1 \
    | tee "${out}/check.log" || status=$?
  kill -INT -- "-${sim_pid}" 2>/dev/null || true
  for _ in $(seq "${SHUTDOWN_GRACE_S}"); do
    kill -0 -- "-${sim_pid}" 2>/dev/null || break
    sleep 1
  done
  kill -KILL -- "-${sim_pid}" 2>/dev/null || true
  wait "${sim_pid}" 2>/dev/null || true
  [[ "${status}" -eq 0 ]] || failures=$((failures + 1))
}

for mode in ${modes[*]}; do
  run_mode "${mode}"
done
echo
echo "yaw checks: ${failures} failing mode(s)"
exit "${failures}"
