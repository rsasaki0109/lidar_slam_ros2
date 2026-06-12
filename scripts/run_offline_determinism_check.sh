#!/usr/bin/env bash
set -euo pipefail

# Phase 2 hard gate (docs/roadmap/v0.6.md): run the offline deterministic
# backend runner N times on the same recorded backend-input bag and require
# the loop-edge sets to be byte-identical across runs.
#
# Usage:
#   bash scripts/run_offline_determinism_check.sh \
#     --bag output/backend_replay_x/backend_input \
#     [--params lidarslam/param/lidarslam_mid360_rko_graph.yaml] \
#     [--runs 3] [--output-dir output/offline_determinism_<timestamp>]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BAG=""
PARAMS="${REPO_ROOT}/lidarslam/param/lidarslam_mid360_rko_graph.yaml"
RUNS=3
OUTPUT_DIR="${REPO_ROOT}/output/offline_determinism_$(date +%Y%m%d_%H%M%S)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG="$2"; shift 2 ;;
    --params) PARAMS="$2"; shift 2 ;;
    --runs) RUNS="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

if [[ -z "${BAG}" ]]; then
  echo "--bag <backend_input bag dir> is required" >&2
  exit 2
fi
if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "install/setup.bash not found; build the workspace first" >&2
  exit 2
fi

# shellcheck disable=SC1091
set +u
source "${REPO_ROOT}/install/setup.bash"
set -u

mkdir -p "${OUTPUT_DIR}"
echo "bag:    ${BAG}"
echo "params: ${PARAMS}"
echo "runs:   ${RUNS}"
echo "out:    ${OUTPUT_DIR}"

for i in $(seq 1 "${RUNS}"); do
  run_dir="${OUTPUT_DIR}/run${i}"
  mkdir -p "${run_dir}"
  echo "--- run ${i}/${RUNS}"
  ros2 run graph_based_slam graph_slam_offline_runner --ros-args \
    --params-file "${PARAMS}" \
    -p bag_path:="${BAG}" \
    -p output_dir:="${run_dir}" \
    > "${run_dir}/runner.log" 2>&1
  md5sum "${run_dir}/loop_edges.csv"
done

echo "--- verdict"
reference="${OUTPUT_DIR}/run1/loop_edges.csv"
edge_count=$(($(wc -l < "${reference}") - 1))
status=0
for i in $(seq 2 "${RUNS}"); do
  if ! diff -q "${reference}" "${OUTPUT_DIR}/run${i}/loop_edges.csv" > /dev/null; then
    echo "MISMATCH: run1 vs run${i}"
    diff "${reference}" "${OUTPUT_DIR}/run${i}/loop_edges.csv" | head -10 || true
    status=1
  fi
done
if [[ ${status} -eq 0 ]]; then
  echo "DETERMINISM_OK: ${RUNS} runs produced byte-identical loop_edges.csv (${edge_count} edges)"
else
  echo "DETERMINISM_FAILED"
fi
exit ${status}
