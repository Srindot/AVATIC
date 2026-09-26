#!/usr/bin/env bash
# Run a command inside the ROS 2 Humble + Gazebo Harmonic dev image with this
# repository mounted at /home/rosusr/avatic (the colcon workspace root).
#
#   simulation_engine/scripts/dev.sh colcon build
#   simulation_engine/scripts/dev.sh colcon test --packages-select pluto_x_core
#   PLUTO_GUI=1 simulation_engine/scripts/dev.sh ros2 launch pluto_x_bringup legacy_sim.launch.py
#
# Environment:
#   PLUTO_DEV_IMAGE  image to use (default: the AVATIC image, .devcontainer/Dockerfile)
#   PLUTO_GUI=1      forward X11 for the Gazebo GUI
set -euo pipefail

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
readonly DEFAULT_IMAGE="ghcr.io/srindot/avatic:latest"
readonly IMAGE="${PLUTO_DEV_IMAGE:-${DEFAULT_IMAGE}}"
readonly WORKSPACE="/home/rosusr/workspace"   # the same path as the dev container

if [[ $# -eq 0 ]]; then
  echo "usage: $0 <command...>" >&2
  exit 2
fi

docker_args=(
  --rm
  --user "$(id -u):$(id -g)"
  --volume "${REPO_ROOT}:${WORKSPACE}"
  --workdir "${WORKSPACE}"
  --env HOME=/home/rosusr
  --env ROS_LOG_DIR="${WORKSPACE}/log/ros"
  --network host
  --entrypoint /bin/bash
)
if [[ -t 0 ]]; then
  docker_args+=(--interactive --tty)
fi
if [[ "${PLUTO_GUI:-0}" == "1" ]]; then
  docker_args+=(--env "DISPLAY=${DISPLAY:-:0}" --env QT_QPA_PLATFORM=xcb
                --volume /tmp/.X11-unix:/tmp/.X11-unix --device /dev/dri)
fi

# shellcheck disable=SC2016  # expanded inside the container
exec docker run "${docker_args[@]}" "${IMAGE}" -c '
  source /opt/ros/humble/setup.bash
  if [[ -f install/setup.bash ]]; then source install/setup.bash; fi
  exec "$@"
' bash "$@"
