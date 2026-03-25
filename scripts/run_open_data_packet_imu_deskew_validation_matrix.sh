#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/run_open_data_packet_imu_deskew_validation_matrix.sh [options]

Options:
  --output-dir DIR                 Output directory
                                   (default: output/open_data_packet_imu_deskew_validation_<timestamp>).
  --benchmark-rate FLOAT           Replay rate used for both no-IMU and IMU runs
                                   (default: 1.0).
  --case SPEC                      Add a case as:
                                   label|/path/to/bag|packet_topic|robot_frame|imu_topic|imu_frame
  --no-default-cases               Do not run the built-in Leo Drive bag1/bag6 front cases.
  --applanix-msg-dir PATH          Forwarded to the benchmark wrapper.
  --verify-map                     Forwarded to the benchmark wrapper.
  --ros-domain-id-base N           Base ROS_DOMAIN_ID used to isolate sequential runs
                                   (default: 40).
  --ros-distro DISTRO              Forwarded to the benchmark wrapper.
  --skip-prepare-overlay           Forwarded to the benchmark wrapper.
  --min-path-coverage FLOAT        Validation threshold (default: 0.95).
  --max-rmse-regression-ratio FLOAT
                                   Validation threshold (default: 1.10).
  --min-matched-pose-ratio FLOAT   Validation threshold (default: 0.80).
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

timestamp() {
  date +%Y%m%d_%H%M%S
}

OUTPUT_DIR=""
APPLANIX_MSG_DIR=""
VERIFY_MAP="false"
ROS_DISTRO_NAME=""
ROS_DOMAIN_ID_BASE="40"
SKIP_PREPARE_OVERLAY="false"
MIN_PATH_COVERAGE="0.95"
MAX_RMSE_REGRESSION_RATIO="1.10"
MIN_MATCHED_POSE_RATIO="0.80"
USE_DEFAULT_CASES="true"
BENCHMARK_RATE="1.0"
declare -a CASES=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --output-dir)
      OUTPUT_DIR="$(realpath -m "${2:-}")"; shift 2 ;;
    --benchmark-rate)
      BENCHMARK_RATE="${2:-}"; shift 2 ;;
    --case)
      CASES+=("${2:-}"); shift 2 ;;
    --no-default-cases)
      USE_DEFAULT_CASES="false"; shift ;;
    --applanix-msg-dir)
      APPLANIX_MSG_DIR="$(realpath "${2:-}")"; shift 2 ;;
    --verify-map)
      VERIFY_MAP="true"; shift ;;
    --ros-domain-id-base)
      ROS_DOMAIN_ID_BASE="${2:-}"; shift 2 ;;
    --ros-distro)
      ROS_DISTRO_NAME="${2:-}"; shift 2 ;;
    --skip-prepare-overlay)
      SKIP_PREPARE_OVERLAY="true"; shift ;;
    --min-path-coverage)
      MIN_PATH_COVERAGE="${2:-}"; shift 2 ;;
    --max-rmse-regression-ratio)
      MAX_RMSE_REGRESSION_RATIO="${2:-}"; shift 2 ;;
    --min-matched-pose-ratio)
      MIN_MATCHED_POSE_RATIO="${2:-}"; shift 2 ;;
    *)
      die "unknown arg: $1"
      ;;
  esac
done

if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${REPO_ROOT}/output/open_data_packet_imu_deskew_validation_$(timestamp)"
fi
mkdir -p "${OUTPUT_DIR}"

if [[ "${USE_DEFAULT_CASES}" == "true" ]]; then
  CASES+=(
    "bag1_front|${REPO_ROOT}/demo_data/autoware_leo_drive_isuzu/all-sensors-bag1_compressed|/sensing/lidar/front/velodyne_packets|velodyne_front|/sensing/imu/imu_data|base_link"
    "bag6_front|${REPO_ROOT}/demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed|/sensing/lidar/front/velodyne_packets|velodyne_front|/sensing/imu/imu_data|base_link"
  )
fi

[[ ${#CASES[@]} -gt 0 ]] || die "no cases configured"

BENCHMARK_COMMON_ARGS=()
if [[ -n "${APPLANIX_MSG_DIR}" ]]; then
  BENCHMARK_COMMON_ARGS+=(--applanix-msg-dir "${APPLANIX_MSG_DIR}")
fi
if [[ "${VERIFY_MAP}" == "true" ]]; then
  BENCHMARK_COMMON_ARGS+=(--verify-map)
fi
if [[ -n "${ROS_DISTRO_NAME}" ]]; then
  BENCHMARK_COMMON_ARGS+=(--ros-distro "${ROS_DISTRO_NAME}")
fi
if [[ "${SKIP_PREPARE_OVERLAY}" == "true" ]]; then
  BENCHMARK_COMMON_ARGS+=(--skip-prepare-overlay)
fi

echo "Running packet IMU deskew validation matrix:"
echo "  output_dir: ${OUTPUT_DIR}"
echo "  cases: ${#CASES[@]}"
echo "  benchmark_rate: ${BENCHMARK_RATE}"

case_index=0
for spec in "${CASES[@]}"; do
  IFS='|' read -r label bag_path packet_topic robot_frame imu_topic imu_frame <<<"${spec}"
  [[ -n "${label}" ]] || die "invalid case spec: ${spec}"
  [[ -d "${bag_path}" ]] || die "bag not found for case ${label}: ${bag_path}"
  case_dir="${OUTPUT_DIR}/${label}"
  no_imu_dir="${case_dir}/no_imu"
  imu_dir="${case_dir}/imu"
  no_imu_domain_id=$((ROS_DOMAIN_ID_BASE + case_index * 2))
  imu_domain_id=$((ROS_DOMAIN_ID_BASE + case_index * 2 + 1))
  mkdir -p "${case_dir}"
  echo "case: ${label}"
  echo "  bag: ${bag_path}"
  echo "  packet_topic: ${packet_topic}"
  echo "  no_imu_ros_domain_id: ${no_imu_domain_id}"
  echo "  imu_ros_domain_id: ${imu_domain_id}"
  bash "${SCRIPT_DIR}/run_open_data_applanix_velodyne_gnss_benchmark.sh" \
    --bag "${bag_path}" \
    --packet-topic "${packet_topic}" \
    --robot-frame-id "${robot_frame}" \
    --rate "${BENCHMARK_RATE}" \
    --use-gnss false \
    --use-imu false \
    --ros-domain-id "${no_imu_domain_id}" \
    --output-dir "${no_imu_dir}" \
    "${BENCHMARK_COMMON_ARGS[@]}"
  bash "${SCRIPT_DIR}/run_open_data_applanix_velodyne_gnss_benchmark.sh" \
    --bag "${bag_path}" \
    --packet-topic "${packet_topic}" \
    --robot-frame-id "${robot_frame}" \
    --rate "${BENCHMARK_RATE}" \
    --use-gnss false \
    --use-imu true \
    --imu-topic "${imu_topic}" \
    --imu-frame-id "${imu_frame}" \
    --ros-domain-id "${imu_domain_id}" \
    --output-dir "${imu_dir}" \
    "${BENCHMARK_COMMON_ARGS[@]}"
  case_index=$((case_index + 1))
done

REPORT_MD="${OUTPUT_DIR}/packet_imu_deskew_validation.md"
REPORT_JSON="${OUTPUT_DIR}/packet_imu_deskew_validation.json"
python3 "${SCRIPT_DIR}/generate_packet_imu_deskew_validation_report.py" \
  --root "${OUTPUT_DIR}" \
  --write-md "${REPORT_MD}" \
  --write-json "${REPORT_JSON}" \
  --min-path-coverage "${MIN_PATH_COVERAGE}" \
  --max-rmse-regression-ratio "${MAX_RMSE_REGRESSION_RATIO}" \
  --min-matched-pose-ratio "${MIN_MATCHED_POSE_RATIO}"

echo "report_md: ${REPORT_MD}"
echo "report_json: ${REPORT_JSON}"
