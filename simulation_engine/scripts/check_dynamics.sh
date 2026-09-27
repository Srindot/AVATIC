#!/usr/bin/env bash
# Dynamics and coordinate-frame checks in Gazebo (see
# pluto_x_bringup/scripts/dynamics_check.py for what each scenario checks).
# Run natively after `colcon build` with the workspace sourced:
#
#   simulation_engine/scripts/check_dynamics.sh      # estimated Pluto X parameters
#
# Effects that would blur the checks (battery, sensor error, wind, latency)
# are switched off; motor lag, drag and spin-up torque stay on.
set -euo pipefail

# on any exit (error, Ctrl-C) never leave a simulation running
SIM_PGID=""
trap 'if [[ -n "${SIM_PGID}" ]]; then kill -KILL -- "-${SIM_PGID}" 2>/dev/null || true; fi' EXIT

readonly OUT=log/dynamics
readonly CONFIG="${PLUTO_CONFIG:-simulation_engine/pluto_x_core/config/pluto_x_estimated.yaml}"
readonly SHUTDOWN_GRACE_S=10
rm -rf "${OUT}" && mkdir -p "${OUT}"

python3 - "${CONFIG}" "${OUT}" <<'PY'
import copy, sys, yaml
source, out = sys.argv[1:3]
base = yaml.safe_load(open(source))
base['latency']['command_s'] = 0.0
for section in ('battery', 'state_error', 'wind'):
    base[section]['enabled'] = False
def write(name, **edits):
    c = copy.deepcopy(base)
    for path, value in edits.items():
        node = c
        keys = path.split('.')
        for k in keys[:-1]:
            node = node[k]
        node[keys[-1]] = value
    yaml.safe_dump(c, open(f'{out}/{name}.yaml', 'w'), sort_keys=False)
# Motors off: zero thrust at neutral, manual mode.
write('rest', **{'pilot.initial_mode': 'manual_attitude',
                 'pilot.hover_thrust_n': 0.0})
write('freefall', **{'pilot.initial_mode': 'manual_attitude',
                     'pilot.hover_thrust_n': 0.0,
                     'gazebo_model.spawn_height_m': 5.0})
# Start hovering at 1 m; target 1 m ahead / to the right at the same height.
write('forward', **{'pilot.initial_mode': 'position_hold',
                    'pilot.position_hold_target_enu_m': {'x': 1.0, 'y': 0.0, 'z': 1.0},
                    'gazebo_model.spawn_height_m': 1.0})
write('right', **{'pilot.initial_mode': 'position_hold',
                  'pilot.position_hold_target_enu_m': {'x': 0.0, 'y': -1.0, 'z': 1.0},
                  'gazebo_model.spawn_height_m': 1.0})
write('yaw', **{'pilot.initial_mode': 'position_hold',
                'pilot.hold_initial_heading': False,
                'pilot.position_hold_target_enu_m': {'x': 0.0, 'y': 0.0, 'z': 0.5},
                'gazebo_model.spawn_height_m': 0.5})
PY

failures=0
for scenario in rest freefall forward right yaw; do
  echo "=== ${scenario}"
  setsid ros2 launch pluto_x_bringup legacy_sim.launch.py headless:=true \
    paused:=true config_file:="${PWD}/${OUT}/${scenario}.yaml" \
    > "${OUT}/${scenario}_sim.log" 2>&1 &
  sim_pid=$!
  SIM_PGID="${sim_pid}"
  status=0
  timeout 180 ros2 run pluto_x_bringup dynamics_check.py "${scenario}" \
    "${OUT}/${scenario}.yaml" 2>&1 | tee "${OUT}/${scenario}.log" || status=$?
  kill -INT -- "-${sim_pid}" 2>/dev/null || true
  for _ in $(seq "${SHUTDOWN_GRACE_S}"); do
    kill -0 -- "-${sim_pid}" 2>/dev/null || break
    sleep 1
  done
  kill -KILL -- "-${sim_pid}" 2>/dev/null || true
  wait "${sim_pid}" 2>/dev/null || true
  [[ "${status}" -eq 0 ]] || failures=$((failures + 1))
done
echo
echo "dynamics checks: ${failures} failing scenario(s)"
exit "${failures}"
