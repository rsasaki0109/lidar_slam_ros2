#!/usr/bin/env bash
# Run one dataset's plane-revisit OFF/ON benchmark and freeze comparable
# public-suite manifests. The resulting two manifests are inputs to
# evaluate_slam_candidate_regression.py.
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
DATASET=""
BAG=""
REFERENCE_TUM=""
DENSE_RAW_TUM=""
FIXED_LOOP_EDGES=""
PARAMS="${REPO_ROOT}/lidarslam/param/lidarslam_mid360_rko_graph.yaml"
SETUP_FILE="${REPO_ROOT}/../install/setup.bash"
if [[ ! -f "${SETUP_FILE}" ]]; then
  SETUP_FILE="${REPO_ROOT}/install/setup.bash"
fi
LOCALIZATION_ZOO="${REPO_ROOT}/../../loc_zoo_ws/localization_zoo"
OUTPUT_DIR="${REPO_ROOT}/output/phase7_plane_revisit_$(date +%Y%m%d_%H%M%S)"
RUNS=3
QUALITY_RUNS=3
DOWNSAMPLE=0.20
ROS_DOMAIN_BASE=170
DRY_RUN=false
CANDIDATE_PARAMS=()

usage() {
  cat <<'EOF' >&2
Usage:
  bash scripts/run_plane_revisit_candidate_benchmark.sh \
    --dataset mid360_public|hilti_exp04 \
    --bag <backend-input-bag> --reference-tum <reference.tum> [options]

Options:
  --fixed-loop-edges <csv>       Replay identical verified loop edges in both arms
  --dense-raw-tum <tum>          Dense frontend poses used to propagate graph corrections
  --params <yaml>                Offline runner parameter file
  --setup <setup.bash>           ROS/workspace setup
  --localization-zoo <dir>       localization_zoo checkout
  --output-dir <dir>             External-SSD output is recommended
  --runs <n>                     Backend repetitions per arm (default: 3)
  --quality-runs <n>             Map-quality repetitions (default: 3)
  --downsample <m>               Map-quality voxel size (default: 0.20)
  --ros-domain-base <id>         OFF uses id..id+n-1; ON uses id+n..id+2n-1
  --candidate-param name:=value  Extra ON-only parameter; repeatable
  --dry-run                      Print commands without executing them
EOF
  exit 2
}

require_value() {
  [[ $# -ge 2 && -n "${2:-}" ]] || usage
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dataset) require_value "$@"; DATASET="$2"; shift 2 ;;
    --bag) require_value "$@"; BAG=$(realpath -m "$2"); shift 2 ;;
    --reference-tum) require_value "$@"; REFERENCE_TUM=$(realpath -m "$2"); shift 2 ;;
    --dense-raw-tum) require_value "$@"; DENSE_RAW_TUM=$(realpath -m "$2"); shift 2 ;;
    --fixed-loop-edges) require_value "$@"; FIXED_LOOP_EDGES=$(realpath -m "$2"); shift 2 ;;
    --params) require_value "$@"; PARAMS=$(realpath -m "$2"); shift 2 ;;
    --setup) require_value "$@"; SETUP_FILE=$(realpath -m "$2"); shift 2 ;;
    --localization-zoo) require_value "$@"; LOCALIZATION_ZOO=$(realpath -m "$2"); shift 2 ;;
    --output-dir) require_value "$@"; OUTPUT_DIR=$(realpath -m "$2"); shift 2 ;;
    --runs) require_value "$@"; RUNS="$2"; shift 2 ;;
    --quality-runs) require_value "$@"; QUALITY_RUNS="$2"; shift 2 ;;
    --downsample) require_value "$@"; DOWNSAMPLE="$2"; shift 2 ;;
    --ros-domain-base) require_value "$@"; ROS_DOMAIN_BASE="$2"; shift 2 ;;
    --candidate-param)
      require_value "$@"
      [[ "$2" == *:=* ]] || usage
      CANDIDATE_PARAMS+=("$2")
      shift 2
      ;;
    --dry-run) DRY_RUN=true; shift ;;
    --help|-h) usage ;;
    *) echo "unknown option: $1" >&2; usage ;;
  esac
done

[[ "${DATASET}" == "mid360_public" || "${DATASET}" == "hilti_exp04" ]] || usage
[[ -d "${BAG}" && -f "${BAG}/metadata.yaml" ]] || {
  echo "backend-input rosbag2 not found: ${BAG}" >&2; exit 2; }
[[ -f "${REFERENCE_TUM}" ]] || { echo "reference TUM not found: ${REFERENCE_TUM}" >&2; exit 2; }
if [[ -n "${DENSE_RAW_TUM}" && ! -f "${DENSE_RAW_TUM}" ]]; then
  echo "dense raw TUM not found: ${DENSE_RAW_TUM}" >&2
  exit 2
fi
[[ -f "${PARAMS}" ]] || { echo "parameter file not found: ${PARAMS}" >&2; exit 2; }
[[ -f "${SETUP_FILE}" ]] || { echo "setup file not found: ${SETUP_FILE}" >&2; exit 2; }
[[ -f "${LOCALIZATION_ZOO}/evaluation/scripts/evaluate_external_tum.py" ]] || {
  echo "localization_zoo evaluator not found: ${LOCALIZATION_ZOO}" >&2; exit 2; }
if [[ -n "${FIXED_LOOP_EDGES}" && ! -f "${FIXED_LOOP_EDGES}" ]]; then
  echo "fixed loop edge CSV not found: ${FIXED_LOOP_EDGES}" >&2
  exit 2
fi
[[ "${RUNS}" =~ ^[1-9][0-9]*$ && "${QUALITY_RUNS}" =~ ^[1-9][0-9]*$ ]] || usage
if ((ROS_DOMAIN_BASE < 0 || ROS_DOMAIN_BASE + 2 * RUNS > 233)); then
  echo "ROS domain range must fit 0..232" >&2
  exit 2
fi

set +u
# shellcheck disable=SC1090
source "${SETUP_FILE}"
set -u
GRAPH_PREFIX=$(ros2 pkg prefix graph_based_slam)
RUNNER_EXECUTABLE="${GRAPH_PREFIX}/lib/graph_based_slam/graph_slam_offline_runner"
[[ -x "${RUNNER_EXECUTABLE}" ]] || {
  echo "offline runner not found under selected setup: ${RUNNER_EXECUTABLE}" >&2
  exit 2
}

run() {
  printf '  $'
  printf ' %q' "$@"
  printf '\n'
  if [[ "${DRY_RUN}" != true ]]; then
    "$@"
  fi
}

mkdir_command=(mkdir -p "${OUTPUT_DIR}")
run "${mkdir_command[@]}"
echo "dataset: ${DATASET}"
echo "bag:     ${BAG}"
echo "runner:  ${RUNNER_EXECUTABLE} ($(sha256sum "${RUNNER_EXECUTABLE}" | awk '{print $1}'))"
echo "output:  ${OUTPUT_DIR}"

for arm in off on; do
  ARM_DIR="${OUTPUT_DIR}/${arm}"
  OFFLINE_DIR="${ARM_DIR}/offline"
  TIME_FILE="${ARM_DIR}/offline.time"
  DOMAIN_BASE="${ROS_DOMAIN_BASE}"
  ENABLED=false
  if [[ "${arm}" == on ]]; then
    DOMAIN_BASE=$((ROS_DOMAIN_BASE + RUNS))
    ENABLED=true
  fi
  OFFLINE_CMD=(
    bash "${SCRIPT_DIR}/run_offline_determinism_check.sh"
    --bag "${BAG}" --params "${PARAMS}" --setup "${SETUP_FILE}"
    --runs "${RUNS}" --output-dir "${OFFLINE_DIR}"
    --ros-domain-base "${DOMAIN_BASE}" --reference-tum "${REFERENCE_TUM}"
    --save-maps --param refine:=true
    --param use_plane_revisit_constraints:="${ENABLED}"
  )
  [[ -n "${FIXED_LOOP_EDGES}" ]] && \
    OFFLINE_CMD+=(--param fixed_loop_edges_path:="${FIXED_LOOP_EDGES}")
  if [[ "${arm}" == on ]]; then
    for value in "${CANDIDATE_PARAMS[@]}"; do
      OFFLINE_CMD+=(--param "${value}")
    done
  fi
  run mkdir -p "${ARM_DIR}"
  run /usr/bin/time -v -o "${TIME_FILE}" "${OFFLINE_CMD[@]}"
  run python3 "${SCRIPT_DIR}/write_runtime_report.py" \
    --time-file "${TIME_FILE}" --bag-metadata "${BAG}/metadata.yaml" \
    --repetitions "${RUNS}" --out "${ARM_DIR}/runtime.json"
  run bash "${SCRIPT_DIR}/run_map_quality_check.sh" \
    --input "${OFFLINE_DIR}/run1/map_optimized.pcd" \
    --output-dir "${ARM_DIR}/map_quality" --runs "${QUALITY_RUNS}" \
    --downsample "${DOWNSAMPLE}" --setup "${SETUP_FILE}"
  RAW_TUM="${OFFLINE_DIR}/run1/trajectory_raw.tum"
  [[ -n "${DENSE_RAW_TUM}" ]] && RAW_TUM="${DENSE_RAW_TUM}"
  run python3 "${SCRIPT_DIR}/run_cross_repo_slam_benchmark.py" \
    --localization-zoo "${LOCALIZATION_ZOO}" --dataset "${DATASET}" \
    --gt-tum "${REFERENCE_TUM}" \
    --raw-tum "${RAW_TUM}" \
    --corrected-tum "${OFFLINE_DIR}/run1/trajectory_optimized.tum" \
    --geometry-report "${ARM_DIR}/map_quality/run1/map_quality_report.yaml" \
    --runtime-report "${ARM_DIR}/runtime.json" \
    --raw-artifact "backend_bag=${BAG}" \
    --raw-artifact "runner_executable=${RUNNER_EXECUTABLE}" \
    --out-dir "${ARM_DIR}/manifest"
done

cat <<EOF

Prepared comparable manifests:
  baseline: ${OUTPUT_DIR}/off/manifest/cross_repo_benchmark.json
  candidate: ${OUTPUT_DIR}/on/manifest/cross_repo_benchmark.json

Combine this dataset pair with the other required Phase 7 pair using:
  python3 scripts/evaluate_slam_candidate_regression.py --baseline ... --candidate ...
EOF
