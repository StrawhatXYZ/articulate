#!/bin/bash
# Open an interactive shell in the running lab container.
set -euo pipefail

CONTAINER=ai-robotics-lab

if [ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null)" != "true" ]; then
    echo "The lab is not running. Start it with ./start-lab.sh" >&2
    exit 1
fi

exec docker exec -it "${CONTAINER}" bash
