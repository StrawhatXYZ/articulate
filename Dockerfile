# AI & Robotics Lab: ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2, with an
# in-container virtual desktop (Xvfb + Openbox + x11vnc + noVNC).
#
# The base image is pinned by its multi-architecture index digest, so builds
# are reproducible and run natively on both arm64 (Apple Silicon) and amd64.
# To update: docker buildx imagetools inspect ros:jazzy, then copy the
# "Digest" of the index (MediaType ...image.index...).
FROM ros:jazzy@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca

# ── MoveIt 2, ros2_control, Gazebo bridge, and virtual desktop tooling ─────
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
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

# ros2_control inside Gazebo and the pick_ik solver for the SO-101 (step 2).
# Installed here rather than in the layer above, which holds multi-GB of
# packages that would otherwise be downloaded again.
# Then initialise the rosdep database (the base image ships the sources list),
# and put `gz` on PATH (the Gazebo vendor package installs it outside PATH).
#
# Perception models (M3a) run on CPU PyTorch in their own virtual
# environment, /opt/lab-ai. It sees the system packages (ROS, NumPy), so the
# perception node runs with that interpreter. Model weights are downloaded
# on first use into the hf-cache volume, not into the image.
RUN apt-get update && apt-get install -y \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-pick-ik \
    python3.12-venv \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep update --rosdistro jazzy \
    && ln -sf /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz /usr/local/bin/gz \
    && python3 -m venv --system-site-packages /opt/lab-ai \
    && /opt/lab-ai/bin/pip install --no-cache-dir \
        --index-url https://download.pytorch.org/whl/cpu torch==2.14.1 \
    && /opt/lab-ai/bin/pip install --no-cache-dir \
        transformers==5.19.0 \
        sentencepiece==0.2.2

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
# Where perception model weights are cached (the hf-cache volume).
ENV HF_HOME=/root/.cache/huggingface

CMD ["bash"]
