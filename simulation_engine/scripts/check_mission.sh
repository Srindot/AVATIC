#!/usr/bin/env bash
# End-to-end mission check: Gazebo + the MagisV2 firmware + fc_link + the
# outer-loop host flying the example waypoint mission (see
# pluto_x_bringup/scripts/mission_check.py for the checks). Run natively
# after `colcon build` with the workspace sourced:
#
#   simulation_engine/scripts/check_mission.sh                      # example square mission
#   SPAWN_YAW=2.0 simulation_engine/scripts/check_mission.sh        # rotated start
#   CONFIG_FILE=/abs/path/vehicle.yaml simulation_engine/scripts/check_mission.sh
#   CONTROLLER=/path/mine.py:Mine PARAMS=/path/p.yaml simulation_engine/scripts/check_mission.sh
#
# The simulation runs in its own process group (setsid) and the whole group
# is stopped afterwards, so no Gazebo server is left behind.
set -euo pipefail

readonly OUT=log/mission
readonly SHUTDOWN_GRACE_S=10
readonly PARAMS="${PARAMS:-$(ros2 pkg prefix pluto_x_autonomy)/share/pluto_x_autonomy/config/waypoint_square.yaml}"
readonly CONTROLLER="${CONTROLLER:-pluto_x_autonomy.examples.waypoint:WaypointController}"
readonly SPAWN_YAW="${SPAWN_YAW:-0.0}"
rm -rf "${OUT}" && mkdir -p "${OUT}"

echo "controller ${CONTROLLER}, params ${PARAMS}, spawn_yaw ${SPAWN_YAW}"
setsid ros2 launch pluto_x_bringup sim.launch.py headless:=true \
  controller:="${CONTROLLER}" controller_params_file:="${PARAMS}" \
  spawn_yaw:="${SPAWN_YAW}" ${CONFIG_FILE:+config_file:="${CONFIG_FILE}"} > "${OUT}/sim.log" 2>&1 &
sim_pid=$!
status=0
ros2 run pluto_x_bringup mission_check.py "${PARAMS}" \
  --csv "${OUT}/track.csv" 2>&1 | tee "${OUT}/check.log" || status=$?
kill -INT -- "-${sim_pid}" 2>/dev/null || true
for _ in $(seq "${SHUTDOWN_GRACE_S}"); do
  kill -0 -- "-${sim_pid}" 2>/dev/null || break
  sleep 1
done
kill -KILL -- "-${sim_pid}" 2>/dev/null || true
wait "${sim_pid}" 2>/dev/null || true
grep -a "outer_loop_host\]: .*phase\|finished" "${OUT}/sim.log" \
  | sed 's/.*outer_loop_host\]: /  host: /' || true
exit "${status}"
