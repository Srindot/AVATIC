# AVATIC development image: ROS 2 Humble + Gazebo Harmonic, and only what
# the AVATIC simulator, participant tools and notebooks use.
#
# Lean version of ghcr.io/srindot/rosgzpx4 (same user, entrypoint and
# layout, so the dev containers work with either). Removed: PX4-Autopilot
# and its toolchain, Micro-XRCE-DDS, QGroundControl, the PX4 workspaces,
# Java, doxygen, the full ROS desktop (rqt, demos), and the duplicated
# chown layer.
#
# Build and push (from the repository root):
#     docker build -t ghcr.io/srindot/avatic:latest -f .devcontainer/Dockerfile .devcontainer
#     docker push ghcr.io/srindot/avatic:latest
# For Intel and Apple Silicon (ARM) in one image:
#     docker buildx build --platform linux/amd64,linux/arm64 \
#         -t ghcr.io/srindot/avatic:latest -f .devcontainer/Dockerfile .devcontainer --push
#
# The workspace is not built into the image: the dev container mounts the
# repository and builds it once (post_create.sh).
FROM ubuntu:22.04

LABEL org.opencontainers.image.title="AVATIC" \
      org.opencontainers.image.description="ROS 2 Humble + Gazebo Harmonic for the AVATIC Pluto X simulator" \
      org.opencontainers.image.vendor="Srindot"

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8
SHELL ["/bin/bash", "-c"]

# --- base system, locale, and the ROS 2 + Gazebo (OSRF) package sources ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg lsb-release locales sudo tzdata && \
    locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    rm -rf /var/lib/apt/lists/*

# --- ROS 2 (base + RViz), Gazebo Harmonic, and the AVATIC build/run deps ---
#   ros-base            rclcpp/rclpy, launch, messages, ament
#   rviz2, xacro        camera view, vehicle model
#   ros-gzharmonic      ROS <-> Gazebo bridge and launch (ros_gz_bridge, ros_gz_sim)
#   gz-harmonic         the Gazebo simulator + the gz-sim8 / gz-plugin2 / gz-common5 /
#                       gz-msgs10 / gz-transport13 headers the AVATIC plugins build against
#   build tools         compiler, CMake, patch (firmware host patches), Eigen, yaml-cpp,
#                       gtest, colcon
#   python / media      NumPy, YAML, matplotlib, pytest, pip, OpenCV (Ubuntu's, built
#                       for NumPy 1: the pip wheel would pull NumPy 2); ffmpeg for video
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-humble-rviz2 \
        ros-humble-xacro \
        ros-humble-ros-gzharmonic \
        ros-humble-ament-cmake-gtest \
        gz-harmonic \
        build-essential cmake git patch pkg-config \
        libeigen3-dev libyaml-cpp-dev \
        python3-colcon-common-extensions \
        python3-pip python3-numpy python3-yaml python3-matplotlib python3-pytest \
        python3-opencv \
        ffmpeg && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# --- notebooks (analysis/, evaluation/) ---
RUN pip install --no-cache-dir notebook

# --- the user (same as rosgzpx4: rosusr, UID/GID 1000, passwordless sudo) ---
ARG USERNAME=rosusr
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# --- entrypoint: ROS sourced for every command ---
RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/$ROS_DISTRO/setup.bash\nexec "$@"\n' \
        > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

USER $USERNAME
WORKDIR /home/$USERNAME
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
