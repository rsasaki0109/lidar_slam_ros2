#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/run_dynamic_object_filter_benchmark.sh [options]

Options:
  --bag PATH                           Input rosbag2 for the save-time comparison.
                                       Default: demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed
  --param FILE                         Base lidarslam parameter YAML.
                                       Default: lidarslam/param/lidarslam.yaml
  --save-root DIR                      Output root directory.
                                       Default: output/dynamic_object_filter_benchmark_<timestamp>
  --rate FLOAT                         ros2 bag play rate passed to run_open_data_gnss_smoke.sh (default: 1.0)
  --drain-sec SEC                      Extra wait before /map_save (default: 15)
  --skip-verify-map                    Skip verify_autoware_map.py in the underlying smoke runs.
  --filter-voxel-size FLOAT            Override dynamic_object_filter_voxel_size for the filtered run.
  --filter-min-observations INT        Override dynamic_object_filter_min_observations for the filtered run.
  --filter-temporal-window INT         Override dynamic_object_filter_temporal_window for the filtered run.
  --filter-max-range-from-sensor-m M   Override dynamic_object_filter_max_range_from_sensor_m for the filtered run.
EOF
}

timestamp() {
  date +%Y%m%d_%H%M%S
}

die() {
  echo "error: $*" >&2
  exit 1
}

write_dynamic_filter_param() {
  local base_param="$1"
  local out_param="$2"
  local enabled="$3"
  local voxel_size="$4"
  local min_observations="$5"
  local temporal_window="$6"
  local max_range="$7"
  python3 - "$base_param" "$out_param" "$enabled" "$voxel_size" "$min_observations" "$temporal_window" "$max_range" <<'PY'
from pathlib import Path
import sys

src = Path(sys.argv[1])
dst = Path(sys.argv[2])
enabled = sys.argv[3].lower() == 'true'
voxel_size = sys.argv[4]
min_observations = sys.argv[5]
temporal_window = sys.argv[6]
max_range = sys.argv[7]
text = src.read_text(encoding='utf-8')

replacements = {
    'use_dynamic_object_filter': 'true' if enabled else 'false',
    'dynamic_object_filter_voxel_size': voxel_size,
    'dynamic_object_filter_min_observations': min_observations,
    'dynamic_object_filter_temporal_window': temporal_window,
    'dynamic_object_filter_max_range_from_sensor_m': max_range,
}
for key, value in replacements.items():
    marker = f'      {key}:'
    lines = text.splitlines()
    replaced = False
    for idx, line in enumerate(lines):
        if line.startswith(marker):
            lines[idx] = f'{marker} {value}'
            replaced = True
            break
    if not replaced:
        raise SystemExit(f'parameter not found in YAML: {key}')
    text = '\n'.join(lines) + '\n'
dst.write_text(text, encoding='utf-8')
PY
}

BAG_PATH="${REPO_ROOT}/demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed"
PARAM_FILE="${REPO_ROOT}/lidarslam/param/lidarslam.yaml"
SAVE_ROOT=""
RATE="1.0"
DRAIN_SEC="15"
VERIFY_MAP="true"
FILTER_VOXEL_SIZE="0.3"
FILTER_MIN_OBSERVATIONS="2"
FILTER_TEMPORAL_WINDOW="5"
FILTER_MAX_RANGE="30.0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --bag)
      BAG_PATH="$(realpath "${2:-}")"; shift 2 ;;
    --param)
      PARAM_FILE="$(realpath "${2:-}")"; shift 2 ;;
    --save-root)
      SAVE_ROOT="$(realpath -m "${2:-}")"; shift 2 ;;
    --rate)
      RATE="${2:-}"; shift 2 ;;
    --drain-sec)
      DRAIN_SEC="${2:-}"; shift 2 ;;
    --skip-verify-map)
      VERIFY_MAP="false"; shift ;;
    --filter-voxel-size)
      FILTER_VOXEL_SIZE="${2:-}"; shift 2 ;;
    --filter-min-observations)
      FILTER_MIN_OBSERVATIONS="${2:-}"; shift 2 ;;
    --filter-temporal-window)
      FILTER_TEMPORAL_WINDOW="${2:-}"; shift 2 ;;
    --filter-max-range-from-sensor-m)
      FILTER_MAX_RANGE="${2:-}"; shift 2 ;;
    *)
      die "unknown arg: $1"
      ;;
  esac
done

[[ -d "${BAG_PATH}" ]] || die "bag not found: ${BAG_PATH}"
[[ -f "${BAG_PATH}/metadata.yaml" ]] || die "metadata.yaml not found under ${BAG_PATH}"
[[ -f "${PARAM_FILE}" ]] || die "param file not found: ${PARAM_FILE}"

if [[ -z "${SAVE_ROOT}" ]]; then
  SAVE_ROOT="${REPO_ROOT}/output/dynamic_object_filter_benchmark_$(timestamp)"
fi
mkdir -p "${SAVE_ROOT}"

BASELINE_DIR="${SAVE_ROOT}/no_filter"
FILTERED_DIR="${SAVE_ROOT}/dynamic_filter"
BASELINE_PARAM="$(mktemp --suffix=.yaml)"
FILTERED_PARAM="$(mktemp --suffix=.yaml)"
cleanup() {
  rm -f "${BASELINE_PARAM}" "${FILTERED_PARAM}"
}
trap cleanup EXIT INT TERM

write_dynamic_filter_param "${PARAM_FILE}" "${BASELINE_PARAM}" "false" \
  "${FILTER_VOXEL_SIZE}" "${FILTER_MIN_OBSERVATIONS}" "${FILTER_TEMPORAL_WINDOW}" "${FILTER_MAX_RANGE}"
write_dynamic_filter_param "${PARAM_FILE}" "${FILTERED_PARAM}" "true" \
  "${FILTER_VOXEL_SIZE}" "${FILTER_MIN_OBSERVATIONS}" "${FILTER_TEMPORAL_WINDOW}" "${FILTER_MAX_RANGE}"

echo "Running dynamic object filter comparison:"
echo "  bag:             ${BAG_PATH}"
echo "  param:           ${PARAM_FILE}"
echo "  save_root:       ${SAVE_ROOT}"
echo "  verify_map:      ${VERIFY_MAP}"
echo "  filter_voxel:    ${FILTER_VOXEL_SIZE}"
echo "  filter_min_obs:  ${FILTER_MIN_OBSERVATIONS}"
echo "  filter_window:   ${FILTER_TEMPORAL_WINDOW}"
echo "  filter_max_rng:  ${FILTER_MAX_RANGE}"

BASE_CMD=(
  bash "${SCRIPT_DIR}/run_open_data_gnss_smoke.sh"
  --bag "${BAG_PATH}"
  --rate "${RATE}"
  --drain-sec "${DRAIN_SEC}"
)
if [[ "${VERIFY_MAP}" == "true" ]]; then
  BASE_CMD+=(--verify-map)
fi

"${BASE_CMD[@]}" --param "${BASELINE_PARAM}" --save-dir "${BASELINE_DIR}"
"${BASE_CMD[@]}" --param "${FILTERED_PARAM}" --save-dir "${FILTERED_DIR}"

python3 "${SCRIPT_DIR}/generate_dynamic_object_filter_report.py" \
  --baseline-dir "${BASELINE_DIR}" \
  --filtered-dir "${FILTERED_DIR}" \
  --out "${SAVE_ROOT}/dynamic_object_filter_report.md" \
  --write-json "${SAVE_ROOT}/dynamic_object_filter_report.json" \
  --write-svg "${SAVE_ROOT}/dynamic_object_filter_report.svg"

echo "done: ${SAVE_ROOT}"
