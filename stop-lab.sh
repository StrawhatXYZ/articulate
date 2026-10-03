#!/bin/bash
# Stop and remove the lab container. The image and ./src are kept.
set -euo pipefail

cd "$(dirname "$0")"

echo "==> Stopping the lab"
docker compose down
echo "Lab stopped."
