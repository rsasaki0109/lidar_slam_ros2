#!/usr/bin/env bash
set -euo pipefail

# Phase 2 hard gate (docs/roadmap/v0.6.md): run the offline deterministic
# backend runner N times on the same recorded backend-input bag and require
# the loop-edge sets AND the optimized trajectories to be byte-identical
# across runs.
#
# Usage:
#   bash scripts/run_offline_determinism_check.sh \
#     --bag output/backend_replay_x/backend_input \
#     [--params lidarslam/param/lidarslam_mid360_rko_graph.yaml] \
#     [--setup /path/to/workspace/install/setup.bash] \
#     [--runs 3] [--output-dir output/offline_determinism_<timestamp>] \
#     [--ros-domain-base 140] [--resume] \
#     [--reference-tum output/glim_mid360_reference.tum] \
#     [--ape-interpolate] [--ape-max-time-diff 0.05] \
#     [--save-maps] [--param name:=value ...]
#
# --save-maps forwards refine_save_maps:=true to the runner so each run
# writes map_optimized.pcd / map_refined.pcd (used by the release gate to
# run the map-quality profile check on the gate-produced refined map).
# Repeatable --param name:=value arguments are appended after the parameter
# file and make one-knob ablations explicit without copying the full YAML.
#
# When --reference-tum is given, each run's trajectory_optimized.tum is also
# scored with scripts/ape_from_tum.py (report only; the gate is byte
# identity, not an APE threshold). For sparse checkpoint references (e.g.
# total-station checkpoints recorded while the platform is stationary,
# which fall between submap-rate poses) pass --ape-interpolate together
# with a generous --ape-max-time-diff (e.g. 2.0). A markdown summary in the
# same shape as the legacy backend_replay_summary.md is always written.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BAG=""
PARAMS="${REPO_ROOT}/lidarslam/param/lidarslam_mid360_rko_graph.yaml"
SETUP_FILE="${REPO_ROOT}/../install/setup.bash"
if [[ ! -f "${SETUP_FILE}" ]]; then
  SETUP_FILE="${REPO_ROOT}/install/setup.bash"
fi
RUNS=3
OUTPUT_DIR="${REPO_ROOT}/output/offline_determinism_$(date +%Y%m%d_%H%M%S)"
REFERENCE_TUM=""
APE_INTERPOLATE=false
APE_MAX_TIME_DIFF="0.05"
SAVE_MAPS=false
PARAM_OVERRIDES=()
ROS_DOMAIN_BASE=140
RESUME=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG="$2"; shift 2 ;;
    --params) PARAMS="$2"; shift 2 ;;
    --setup) SETUP_FILE="$2"; shift 2 ;;
    --runs) RUNS="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --reference-tum) REFERENCE_TUM="$2"; shift 2 ;;
    --ape-interpolate) APE_INTERPOLATE=true; shift ;;
    --ape-max-time-diff) APE_MAX_TIME_DIFF="$2"; shift 2 ;;
    --save-maps) SAVE_MAPS=true; shift ;;
    --ros-domain-base) ROS_DOMAIN_BASE="$2"; shift 2 ;;
    --resume) RESUME=true; shift ;;
    --param)
      if [[ "${2:-}" != *:=* || "${2%%:=*}" == "" || "${2#*:=}" == "" ]]; then
        echo "--param expects name:=value" >&2
        exit 2
      fi
      PARAM_OVERRIDES+=("$2")
      shift 2
      ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

if (( ROS_DOMAIN_BASE < 0 || ROS_DOMAIN_BASE + RUNS > 233 )); then
  echo "--ros-domain-base must leave one valid ROS domain (0..232) per run" >&2
  exit 2
fi

if [[ -z "${BAG}" ]]; then
  echo "--bag <backend_input bag dir> is required" >&2
  exit 2
fi
PARAMS="$(realpath -m "${PARAMS}")"
SETUP_FILE="$(realpath -m "${SETUP_FILE}")"
BAG="$(realpath -m "${BAG}")"
if [[ ! -f "${SETUP_FILE}" ]]; then
  if [[ -n "${ROS_DISTRO:-}" ]] && command -v ros2 >/dev/null 2>&1; then
    SETUP_FILE="active_environment"
  else
    echo "setup file not found and no active ROS environment is available: ${SETUP_FILE}" >&2
    exit 2
  fi
fi
if [[ ! -f "${PARAMS}" ]]; then
  echo "params file not found: ${PARAMS}" >&2
  exit 2
fi
if [[ ! -d "${BAG}" ]]; then
  echo "bag directory not found: ${BAG}" >&2
  exit 2
fi

if [[ "${SETUP_FILE}" != "active_environment" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "${SETUP_FILE}"
  set -u
fi

GRAPH_PREFIX="$(ros2 pkg prefix graph_based_slam)"
RUNNER_EXECUTABLE="${GRAPH_PREFIX}/lib/graph_based_slam/graph_slam_offline_runner"
if [[ ! -x "${RUNNER_EXECUTABLE}" ]]; then
  echo "graph_slam_offline_runner not found under selected setup: ${RUNNER_EXECUTABLE}" >&2
  exit 2
fi
RUNNER_SHA256="$(sha256sum "${RUNNER_EXECUTABLE}" | awk '{print $1}')"
PARAMS_SHA256="$(sha256sum "${PARAMS}" | awk '{print $1}')"
BAG_METADATA_SHA256="n/a"
if [[ -f "${BAG}/metadata.yaml" ]]; then
  BAG_METADATA_SHA256="$(sha256sum "${BAG}/metadata.yaml" | awk '{print $1}')"
fi
FIXED_LOOP_EDGES_PATH=""
for override in "${PARAM_OVERRIDES[@]}"; do
  if [[ "${override}" == fixed_loop_edges_path:=* ]]; then
    FIXED_LOOP_EDGES_PATH="${override#*:=}"
  fi
done
FIXED_LOOP_EDGES_SHA256=""
if [[ -n "${FIXED_LOOP_EDGES_PATH}" ]]; then
  FIXED_LOOP_EDGES_PATH="$(realpath -m "${FIXED_LOOP_EDGES_PATH}")"
  if [[ ! -f "${FIXED_LOOP_EDGES_PATH}" ]]; then
    echo "fixed loop edge CSV not found: ${FIXED_LOOP_EDGES_PATH}" >&2
    exit 2
  fi
  FIXED_LOOP_EDGES_SHA256="$(sha256sum "${FIXED_LOOP_EDGES_PATH}" | awk '{print $1}')"
fi

mkdir -p "${OUTPUT_DIR}"
echo "bag:    ${BAG}"
echo "params: ${PARAMS}"
echo "setup:  ${SETUP_FILE}"
echo "runner: ${RUNNER_EXECUTABLE} (${RUNNER_SHA256})"
echo "runs:   ${RUNS}"
echo "out:    ${OUTPUT_DIR}"

for i in $(seq 1 "${RUNS}"); do
  run_dir="${OUTPUT_DIR}/run${i}"
  mkdir -p "${run_dir}"
  echo "--- run ${i}/${RUNS}"
  if [[ "${RESUME}" == true && \
        -f "${run_dir}/.complete" && \
        -s "${run_dir}/loop_edges.csv" && \
        -s "${run_dir}/trajectory_optimized.tum" ]]; then
    echo "reuse complete run ${i}: ${run_dir}"
    continue
  fi
  rm -f "${run_dir}/.complete"
  RUNNER_CMD=(
    "${RUNNER_EXECUTABLE}" --ros-args
    --disable-rosout-logs
    --params-file "${PARAMS}"
    -p bag_path:="${BAG}"
    -p output_dir:="${run_dir}"
  )
  if [[ "${SAVE_MAPS}" == "true" ]]; then
    RUNNER_CMD+=(-p refine_save_maps:=true)
  fi
  for override in "${PARAM_OVERRIDES[@]}"; do
    RUNNER_CMD+=(-p "${override}")
  done
  ROS_DOMAIN_ID=$((ROS_DOMAIN_BASE + i - 1)) ROS_LOCALHOST_ONLY=1 \
    "${RUNNER_CMD[@]}" \
    > "${run_dir}/runner.log" 2>&1
  md5sum "${run_dir}/loop_edges.csv" "${run_dir}/trajectory_optimized.tum"
  if [[ -n "${REFERENCE_TUM}" ]]; then
    APE_CMD=(
      python3 "${SCRIPT_DIR}/ape_from_tum.py"
      --ref "${REFERENCE_TUM}"
      --est "${run_dir}/trajectory_optimized.tum"
      --out "${run_dir}/ape.txt"
      --max-time-diff "${APE_MAX_TIME_DIFF}"
    )
    if [[ "${APE_INTERPOLATE}" == "true" ]]; then
      APE_CMD+=(--interpolate)
    fi
    "${APE_CMD[@]}" \
      > "${run_dir}/ape_postprocess.log" 2>&1 \
      || echo "WARN: APE post-processing failed in run ${i}; continuing" >&2
  fi
  touch "${run_dir}/.complete"
done

ape_rmse_for_run() {
  local run_dir="$1"
  if [[ -f "${run_dir}/ape.txt" ]]; then
    awk '/^\s*rmse:/ {print $2; exit}' "${run_dir}/ape.txt"
  fi
}

echo "--- verdict"
edges_ref="${OUTPUT_DIR}/run1/loop_edges.csv"
traj_ref="${OUTPUT_DIR}/run1/trajectory_optimized.tum"
edge_count=$(($(wc -l < "${edges_ref}") - 1))
status=0
for i in $(seq 2 "${RUNS}"); do
  if ! diff -q "${edges_ref}" "${OUTPUT_DIR}/run${i}/loop_edges.csv" > /dev/null; then
    echo "MISMATCH (loop edges): run1 vs run${i}"
    diff "${edges_ref}" "${OUTPUT_DIR}/run${i}/loop_edges.csv" | head -10 || true
    status=1
  fi
  if ! diff -q "${traj_ref}" "${OUTPUT_DIR}/run${i}/trajectory_optimized.tum" > /dev/null; then
    echo "MISMATCH (optimized trajectory): run1 vs run${i}"
    status=1
  fi
  # v0.7 refinement artifacts (present only when the runner ran with
  # refine:=true) join the byte-identity contract.
  for refined_artifact in trajectory_refined.tum map_refinement_report.yaml plane_revisit_report.yaml; do
    if [[ -f "${OUTPUT_DIR}/run1/${refined_artifact}" ]]; then
      if ! diff -q "${OUTPUT_DIR}/run1/${refined_artifact}"         "${OUTPUT_DIR}/run${i}/${refined_artifact}" > /dev/null; then
        echo "MISMATCH (${refined_artifact}): run1 vs run${i}"
        status=1
      fi
    fi
  done
done

summary="${OUTPUT_DIR}/offline_determinism_summary.md"
{
  echo "runner_setup: \`${SETUP_FILE}\`"
  echo "runner_executable: \`${RUNNER_EXECUTABLE}\`"
  echo "runner_sha256: \`${RUNNER_SHA256}\`"
  echo "params_file: \`${PARAMS}\`"
  echo "params_sha256: \`${PARAMS_SHA256}\`"
  echo "bag_metadata_sha256: \`${BAG_METADATA_SHA256}\`"
  if [[ -n "${FIXED_LOOP_EDGES_PATH}" ]]; then
    echo "fixed_loop_edges_path: \`${FIXED_LOOP_EDGES_PATH}\`"
    echo "fixed_loop_edges_sha256: \`${FIXED_LOOP_EDGES_SHA256}\`"
  fi
  echo "parameter_overrides:"
  if [[ ${#PARAM_OVERRIDES[@]} -eq 0 ]]; then
    echo "- none"
  else
    for override in "${PARAM_OVERRIDES[@]}"; do
      echo "- \`${override}\`"
    done
  fi
  echo ""
  echo "| run | ape_rmse | n_loop_edges | loop_edges_md5 | trajectory_md5 |"
  echo "| --- | ---: | ---: | --- | --- |"
  for i in $(seq 1 "${RUNS}"); do
    run_dir="${OUTPUT_DIR}/run${i}"
    n_edges=$(($(wc -l < "${run_dir}/loop_edges.csv") - 1))
    edges_md5=$(md5sum "${run_dir}/loop_edges.csv" | cut -c1-12)
    traj_md5=$(md5sum "${run_dir}/trajectory_optimized.tum" | cut -c1-12)
    rmse=$(ape_rmse_for_run "${run_dir}")
    echo "| run${i} | ${rmse:-n/a} | ${n_edges} | \`${edges_md5}\` | \`${traj_md5}\` |"
  done
  echo ""
  if [[ ${status} -eq 0 ]]; then
    echo "edge_sets_identical: true"
    echo "optimized_trajectories_identical: true"
  else
    echo "edge_sets_or_trajectories_identical: false"
  fi
} | tee "${summary}"

if [[ ${status} -eq 0 ]]; then
  echo "DETERMINISM_OK: ${RUNS} runs produced byte-identical loop_edges.csv and trajectory_optimized.tum (${edge_count} edges)"
else
  echo "DETERMINISM_FAILED"
fi
exit ${status}
