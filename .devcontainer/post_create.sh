#!/usr/bin/env bash
# Runs once when the dev container is created (postCreateCommand), in the
# repository folder: installs anything the image lacks, sets up the shell,
# and builds the workspace. The AVATIC image (.devcontainer/Dockerfile) has
# everything already; the older ghcr.io/srindot/rosgzpx4 lacks ffmpeg and
# Jupyter, which are then installed here.
set -euo pipefail
cd "$(dirname "$0")/.."
REPO="$(pwd)"

missing=()
command -v ffmpeg >/dev/null || missing+=(ffmpeg)
dpkg -s ros-humble-ros-gzharmonic >/dev/null 2>&1 || missing+=(ros-humble-ros-gzharmonic)
dpkg -s ros-humble-xacro >/dev/null 2>&1 || missing+=(ros-humble-xacro)
if [ ${#missing[@]} -gt 0 ]; then
  echo "== installing: ${missing[*]}"
  sudo apt-get update -qq
  sudo apt-get install -y -qq "${missing[@]}"
fi
python3 -c "import notebook" 2>/dev/null || { echo "== installing Jupyter"; pip install --quiet notebook; }

# A build/ made outside the container (other absolute paths) cannot be
# reused here: CMake refuses. Then build into *_container folders instead,
# so the host build keeps working too.
BUILD=build INSTALL=install LOG=log
if grep -qsh "^CMAKE_HOME_DIRECTORY" build/*/CMakeCache.txt &&
   ! grep -qsh "^CMAKE_HOME_DIRECTORY:INTERNAL=${REPO}/" build/*/CMakeCache.txt; then
  echo "== build/ was made outside the container: building into build_container/ instead"
  BUILD=build_container INSTALL=install_container LOG=log_container
fi

# every new terminal: ROS + this workspace, starting in the repository
MARK='# >>> AVATIC dev container >>>'
if ! grep -qF "$MARK" ~/.bashrc; then
  cat >> ~/.bashrc <<EOF
$MARK
source /opt/ros/humble/setup.bash
[ -f "${REPO}/${INSTALL}/setup.bash" ] && source "${REPO}/${INSTALL}/setup.bash"
cd "${REPO}"
# <<< AVATIC dev container <<<
EOF
fi

echo "== building the simulator (a few minutes the first time)"
set +u   # ROS and colcon setup scripts read unset variables: must come first
source /opt/ros/humble/setup.bash
colcon --log-base "$LOG" build --build-base "$BUILD" --install-base "$INSTALL"

echo "== done. Open a new terminal and run:"
echo "   ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/my_controller.py"
