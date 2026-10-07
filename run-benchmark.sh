#!/bin/bash
# Run the benchmark in the lab container. Results land in ./results/<run>/.
#
#   ./run-benchmark.sh                          # all tasks, 100 trials each, seed 42
#   TASK=pick_place TRIALS=10 ./run-benchmark.sh
#   TASK=pick_place GRASP_MODE=friction ./run-benchmark.sh   # no grasp latch
#
# Stop any interactive simulation first: two simulations would share topics.
set -euo pipefail

cd "$(dirname "$0")"

CONTAINER="${CONTAINER:-ai-robotics-lab}"
TASK="${TASK:-all}"
TRIALS="${TRIALS:-100}"
SEED="${SEED:-42}"
GRASP_MODE="${GRASP_MODE:-latched}"

if docker exec "${CONTAINER}" pgrep -f "gz sim" > /dev/null; then
    echo "A simulation is already running in ${CONTAINER}. Stop it first." >&2
    exit 1
fi

# ros2 launch stops the gz wrapper process, but the simulation server it
# started can survive as an orphan and would keep publishing on the same
# topics. Stop anything left over, however this script exits.
cleanup() { docker exec "${CONTAINER}" pkill -f "gz sim" > /dev/null 2>&1 || true; }
trap cleanup EXIT

commit="$(git rev-parse HEAD 2>/dev/null || echo unknown)"
if [ -z "$(git status --porcelain 2>/dev/null)" ]; then dirty=false; else dirty=true; fi

# shellcheck disable=SC2016  # expanded inside the container, not here
docker exec \
    -e LAB_GIT_COMMIT="${commit}" -e LAB_GIT_DIRTY="${dirty}" \
    -e TASK="${TASK}" -e TRIALS="${TRIALS}" -e SEED="${SEED}" -e GRASP_MODE="${GRASP_MODE}" \
    "${CONTAINER}" bash -c '
source /opt/ros/jazzy/setup.bash
cd /workspace
colcon build --symlink-install > /dev/null
source install/setup.bash
ros2 launch lab_simulation benchmark.launch.py \
    task:="${TASK}" trials:="${TRIALS}" seed:="${SEED}" grasp_mode:="${GRASP_MODE}" \
    out:=/workspace/results'
