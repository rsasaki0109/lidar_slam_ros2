#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  run_default_ci_checks.sh [options]

Options:
  --build-only                 Build the default workflow packages without running tests
  --cmake-build-type <type>    CMake build type (default: Release)
  --help                       Show this help

This script verifies the default permissive-license workflow for this repository:
  - build: ndt_omp_ros2, lidarslam_msgs, scanmatcher, graph_based_slam, lidarslam, rko_lio
  - test:  lidarslam_msgs, scanmatcher, graph_based_slam, lidarslam
EOF
  exit 1
}

BUILD_ONLY=false
CMAKE_BUILD_TYPE="Release"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-only)
      BUILD_ONLY=true
      shift
      ;;
    --cmake-build-type)
      [[ $# -ge 2 ]] || usage
      CMAKE_BUILD_TYPE="$2"
      shift 2
      ;;
    --help|-h)
      usage
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      ;;
  esac
done

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
cd "${REPO_ROOT}"

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon not found in PATH" >&2
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  for candidate in jazzy humble rolling iron; do
    if [[ -f "/opt/ros/${candidate}/setup.bash" ]]; then
      set +u
      # shellcheck source=/dev/null
      source "/opt/ros/${candidate}/setup.bash"
      set -u
      break
    fi
  done
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found in PATH and no /opt/ros/<distro>/setup.bash was detected" >&2
  exit 1
fi

BUILD_TARGETS=(
  lidarslam
  rko_lio
)

TEST_TARGETS=(
  lidarslam_msgs
  scanmatcher
  graph_based_slam
  lidarslam
)

echo "==> Building default workflow packages"
colcon build \
  --event-handlers console_direct+ \
  --packages-up-to "${BUILD_TARGETS[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"

if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "${REPO_ROOT}/install/setup.bash"
  set -u
fi

if [[ "${BUILD_ONLY}" == "true" ]]; then
  echo "==> Build-only mode completed"
  exit 0
fi

echo "==> Running default workflow tests"
colcon test \
  --event-handlers console_direct+ \
  --return-code-on-test-failure \
  --packages-select "${TEST_TARGETS[@]}"

echo "==> Test results"
colcon test-result --verbose
