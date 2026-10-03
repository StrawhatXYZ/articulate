#!/bin/bash
# Build and start the lab container, then build the ROS 2 workspace inside it.
#
# The container runs its own virtual desktop, so Gazebo and RViz are viewed in
# a browser at http://localhost:6080 and no host X server is needed.
set -euo pipefail

cd "$(dirname "$0")"

CONTAINER=ai-robotics-lab
DESKTOP_URL=http://localhost:6080

if ! docker info > /dev/null 2>&1; then
    echo "Docker is not running. Start Docker Desktop and try again." >&2
    exit 1
fi

echo "==> Building image and starting container"
docker compose up -d --build

echo "==> Waiting for the virtual desktop"
for _ in $(seq 1 30); do
    curl -fs -o /dev/null "${DESKTOP_URL}" && break
    sleep 1
done
if ! curl -fsS -o /dev/null "${DESKTOP_URL}"; then
    echo "Virtual desktop did not come up. Check: docker logs ${CONTAINER}" >&2
    exit 1
fi

echo "==> Building the ROS 2 workspace"
docker exec "${CONTAINER}" bash -c \
    "source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --symlink-install"

cat <<EOF

AI & Robotics Lab is ready.

  Desktop (Gazebo, RViz):  ${DESKTOP_URL}

  Terminal 1:  ./enter-lab.sh
               ros2 launch lab_simulation arm_world.launch.py

  Terminal 2:  ./enter-lab.sh
               ros2 run lab_simulation move_arm.py

  Stop:        ./stop-lab.sh
EOF
