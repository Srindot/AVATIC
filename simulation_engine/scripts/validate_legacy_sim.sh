#!/usr/bin/env bash
# End-to-end validation of the legacy-stack Gazebo simulation.
# Run after `colcon build` with the workspace sourced (natively, or inside
# the dev container via simulation_engine/scripts/dev.sh):
#
#   simulation_engine/scripts/validate_legacy_sim.sh                  # estimated Pluto X
#   PLUTO_CONFIG=simulation_engine/pluto_x_core/config/legacy_kwad.yaml \
#     simulation_engine/scripts/validate_legacy_sim.sh                # original kwad.cpp values
#
# Steps (results in log/validation/):
#   1. hold      x2 : Gazebo, position hold from t = 0, ALL configured effects
#                     (motor lag, battery, sensor error, wind); no async input
#   2. determinism  : the two hold runs must be identical sample-for-sample
#   3. reference    : SimulatedVehicle on the offline integrator, same
#                     scenario; compared with the Gazebo run (RMS difference)
#   4. hold, effects off: Gazebo, disturbance-free position hold (battery,
#                     sensor error, wind, latency off; motor lag kept) must
#                     converge exactly
set -euo pipefail

readonly OUT=log/validation
readonly CONFIG="${PLUTO_CONFIG:-simulation_engine/pluto_x_core/config/pluto_x_estimated.yaml}"
readonly HOLD_CONFIG="${OUT}/position_hold_config.yaml"
readonly DURATION_S=30
readonly CHECK_TIMEOUT_S=180
# Face north, like the reference integrator (NED yaw 0), so the two runs are
# comparable.
readonly SPAWN_YAW_NORTH_RAD=1.5707963267948966
rm -rf "${OUT}" && mkdir -p "${OUT}"

readonly EFFECTS_OFF_CONFIG="${OUT}/effects_off_config.yaml"
readonly HOLD_TOLERANCE_M="${PLUTO_HOLD_TOLERANCE_M:-0.3}"
echo "config: ${CONFIG} (hold tolerance ${HOLD_TOLERANCE_M} m)"
python3 - "${CONFIG}" "${HOLD_CONFIG}" "${EFFECTS_OFF_CONFIG}" <<'PY'
import sys, yaml
source, hold_path, off_path = sys.argv[1:4]
config = yaml.safe_load(open(source))
hold = yaml.safe_load(open(source))
hold['pilot']['initial_mode'] = 'position_hold'
yaml.safe_dump(hold, open(hold_path, 'w'), sort_keys=False)
config['pilot']['initial_mode'] = 'position_hold'
config['latency']['command_s'] = 0.0
for section in ('battery', 'state_error', 'wind'):
    config[section]['enabled'] = False
yaml.safe_dump(config, open(off_path, 'w'), sort_keys=False)
PY

readonly SHUTDOWN_GRACE_S=10
sim_pid=""
# The simulation runs in its own process group (setsid) so that the launch
# process, gz sim and the bridge can be stopped together.
stop_sim() {
  if [[ -n "${sim_pid}" ]]; then
    kill -INT -- "-${sim_pid}" 2>/dev/null || true
    for _ in $(seq "${SHUTDOWN_GRACE_S}"); do
      kill -0 -- "-${sim_pid}" 2>/dev/null || break
      sleep 1
    done
    kill -KILL -- "-${sim_pid}" 2>/dev/null || true
    wait "${sim_pid}" 2>/dev/null || true
    sim_pid=""
  fi
}
trap stop_sim EXIT

run_scenario() {  # <name> <config> <scenario> <tolerance_m>
  local name="$1" config="$2" scenario="$3" tolerance="$4"
  echo "=== ${name}"
  setsid ros2 launch pluto_x_bringup legacy_sim.launch.py headless:=true \
    paused:=true config_file:="${PWD}/${config}" \
    spawn_yaw:="${SPAWN_YAW_NORTH_RAD}" \
    > "${OUT}/${name}_sim.log" 2>&1 &
  sim_pid=$!
  local status=0
  timeout "${CHECK_TIMEOUT_S}" ros2 run pluto_x_bringup \
    legacy_flight_check.py "${scenario}" --duration "${DURATION_S}" \
    --output "${OUT}/${name}.csv" --tolerance "${tolerance}" 2>&1 \
    | tee "${OUT}/${name}_check.log" \
    || status=$?
  stop_sim
  grep -E "\[pluto_x\]" "${OUT}/${name}_sim.log" | sed 's/\x1b\[[0-9;]*m//g' \
    | head -20 > "${OUT}/${name}_plugin.log" || true
  return "${status}"
}

failures=0
run_scenario hold_run1 "${HOLD_CONFIG}" hold "${HOLD_TOLERANCE_M}" \
  || failures=$((failures + 1))
run_scenario hold_run2 "${HOLD_CONFIG}" hold "${HOLD_TOLERANCE_M}" \
  || failures=$((failures + 1))

echo "=== determinism (hold_run1 vs hold_run2)"
ros2 run pluto_x_bringup compare_trajectories.py --exact \
  "${OUT}/hold_run1.csv" "${OUT}/hold_run2.csv" || failures=$((failures + 1))

echo "=== reference integrator vs Gazebo"
install/pluto_x_core/lib/pluto_x_core/legacy_reference_sim \
  "${HOLD_CONFIG}" "${DURATION_S}" "${OUT}/reference.csv"
ros2 run pluto_x_bringup compare_trajectories.py \
  "${OUT}/reference.csv" "${OUT}/hold_run1.csv"

run_scenario hold_effects_off "${EFFECTS_OFF_CONFIG}" hold 0.05 \
  || failures=$((failures + 1))

echo
echo "validation: ${failures} failing step(s)"
exit "${failures}"
