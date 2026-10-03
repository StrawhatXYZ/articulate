# AI & Robotics Lab: ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2, with an
# in-container virtual desktop (Xvfb + Openbox + x11vnc + noVNC).
#
# The base image is pinned by digest so builds are reproducible. It is
# published for linux/amd64 only; on Apple Silicon it runs under emulation
# (docker-compose.yml sets the platform explicitly).
# To update: docker pull --platform linux/amd64 osrf/ros:jazzy-desktop, then
#   docker inspect --format '{{index .RepoDigests 0}}' osrf/ros:jazzy-desktop
FROM osrf/ros:jazzy-desktop@sha256:2f520187e84304fffb60d19c9a7d8e2e79d563a6e579a090427e5994eb1b8c45

# ── MoveIt 2, ros2_control, Gazebo bridge, and virtual desktop tooling ─────
RUN apt-get update && apt-get install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-resources \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros-gz \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    vim \
    wget \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    openbox \
    menu \
    && ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Initialise the rosdep database (the base image ships the sources list), and
# put `gz` on PATH (the Gazebo vendor package installs it outside PATH).
RUN rosdep update --rosdistro jazzy \
    && ln -sf /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz /usr/local/bin/gz

COPY --chmod=755 scripts/start-desktop.sh /usr/local/bin/start-desktop.sh

# ── Workspace ──────────────────────────────────────────────────────────────
# Built into the image so it works standalone. In development, docker compose
# bind-mounts ./src over this copy and start-lab.sh rebuilds it.
WORKDIR /workspace
COPY src /workspace/src
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install

# Source ROS 2 and the workspace in interactive shells (docker exec -it ... bash).
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc \
    && echo "source /workspace/install/setup.bash 2>/dev/null || true" >> /etc/bash.bashrc

ENV DISPLAY=:1

CMD ["bash"]
