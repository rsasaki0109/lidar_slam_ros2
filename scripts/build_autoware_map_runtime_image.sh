#!/usr/bin/env bash
set -euo pipefail

IMAGE_TAG=${1:-lidarslam_autoware_map_runtime:jazzy}
DOCKERFILE=$(realpath "$(dirname "$0")/autoware_map_runtime.Dockerfile")
CONTEXT_DIR=$(dirname "$DOCKERFILE")

command -v docker >/dev/null 2>&1 || { echo "docker not found" >&2; exit 1; }

echo "Building Autoware map runtime image: $IMAGE_TAG"
docker build -t "$IMAGE_TAG" -f "$DOCKERFILE" "$CONTEXT_DIR"
