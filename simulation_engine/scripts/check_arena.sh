#!/usr/bin/env bash
# End-to-end check of the balloon arena: arena world + MagisV2 + outer loop
# flying the DEMO mission (config/arena_demo.yaml: straight to two known
# balloons), camera, popping, score and the time limit (see
# pluto_x_bringup/scripts/arena_check.py). Headless, no RViz.
#
#   simulation_engine/scripts/check_arena.sh
#   CONTROLLER=/path/mine.py:Mine PARAMS=/path/p.yaml MIN_POPS=0 MIN_SCORE=0 simulation_engine/scripts/check_arena.sh
#   TIME_LIMIT=35 simulation_engine/scripts/check_arena.sh    # override the run time limit
#   FORBID=red simulation_engine/scripts/check_arena.sh        # fail if a red (penalty) balloon popped
set -euo pipefail

# on any exit (error, Ctrl-C) never leave a simulation running
SIM_PGID=""
trap '[[ -n "${SIM_PGID}" ]] && kill -KILL -- "-${SIM_PGID}" 2>/dev/null; true' EXIT

readonly OUT=log/arena
readonly SHUTDOWN_GRACE_S=10
PREFIX="$(ros2 pkg prefix pluto_x_autonomy)"   # fails here (set -e) if not sourced
readonly PARAMS="${PARAMS:-${PREFIX}/share/pluto_x_autonomy/config/arena_demo.yaml}"
readonly CONTROLLER="${CONTROLLER:-pluto_x_autonomy.examples.waypoint:WaypointController}"
rm -rf "${OUT}" && mkdir -p "${OUT}"

echo "controller ${CONTROLLER}, params ${PARAMS}"
setsid ros2 launch pluto_x_bringup arena.launch.py headless:=true rviz:=false \
  controller:="${CONTROLLER}" controller_params_file:="${PARAMS}" \
  result_file:="${PWD}/${OUT}/result.yaml" ${TIME_LIMIT:+time_limit_s:="${TIME_LIMIT}"} \
  > "${OUT}/sim.log" 2>&1 &
sim_pid=$!
SIM_PGID="${sim_pid}"
status=0
ros2 run pluto_x_bringup arena_check.py --min-pops "${MIN_POPS:-2}" \
  --min-score "${MIN_SCORE:-75}" ${FORBID:+--forbid-colour "${FORBID}"} --save-frames "${OUT}" --odom-csv "${OUT}/track.csv" 2>&1 | tee "${OUT}/check.log" || status=$?
echo "models left in the world:"
timeout 5 gz model --list 2>/dev/null | grep -E "balloon|pluto" | sed 's/^/    /' || true
kill -INT -- "-${sim_pid}" 2>/dev/null || true
for _ in $(seq "${SHUTDOWN_GRACE_S}"); do
  kill -0 -- "-${sim_pid}" 2>/dev/null || break
  sleep 1
done
kill -KILL -- "-${sim_pid}" 2>/dev/null || true
wait "${sim_pid}" 2>/dev/null || true
exit "${status}"
