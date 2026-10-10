#!/bin/bash
# Run the benchmark in the lab container. Results land in ./results/<run>/.
#
#   ./run-benchmark.sh                          # all tasks, 100 trials each, seed 42
#   TASK=pick_place TRIALS=10 ./run-benchmark.sh
#   TASK=pick_place GRASP_MODE=friction ./run-benchmark.sh   # no grasp latch
#   TASK=sort PERCEPTION=vision ./run-benchmark.sh           # plan from the camera
#   TASK=instruct TRIALS=50 PERCEPTION=vision ./run-benchmark.sh   # instructions (Ollama)
#   TASK=pick_place SPEED=0.8 ./run-benchmark.sh             # faster arm
#
# Stop any interactive simulation first: two simulations would share topics.
set -euo pipefail

cd "$(dirname "$0")"

CONTAINER="${CONTAINER:-articulate}"
TASK="${TASK:-all}"
TRIALS="${TRIALS:-100}"
SEED="${SEED:-42}"
GRASP_MODE="${GRASP_MODE:-latched}"
PERCEPTION="${PERCEPTION:-ground_truth}"
OLLAMA_URL="${OLLAMA_URL:-http://host.docker.internal:11434}"
MODEL="${MODEL:-gemma4:e2b}"
SPEED="${SPEED:-0.5}"

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
    -e TASK="${TASK}" -e TRIALS="${TRIALS}" -e SEED="${SEED}" -e GRASP_MODE="${GRASP_MODE}" -e PERCEPTION="${PERCEPTION}" \
    -e OLLAMA_URL="${OLLAMA_URL}" -e MODEL="${MODEL}" -e SPEED="${SPEED}" \
    "${CONTAINER}" bash -c '
source /opt/ros/jazzy/setup.bash
cd /workspace
if ! colcon build --symlink-install > /tmp/lab-build.log 2>&1; then
    tail -n 30 /tmp/lab-build.log
    echo "Workspace build failed; not running the benchmark." >&2
    exit 1
fi
source install/setup.bash
ros2 launch lab_simulation benchmark.launch.py \
    task:="${TASK}" trials:="${TRIALS}" seed:="${SEED}" grasp_mode:="${GRASP_MODE}" perception:="${PERCEPTION}" \
    ollama_url:="${OLLAMA_URL}" model:="${MODEL}" speed:="${SPEED}" \
    out:=/workspace/results'
