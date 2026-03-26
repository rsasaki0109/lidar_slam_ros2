#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/run_open_data_classic_path_benchmark_suite.sh [options]

Options:
  --bag PATH                Main driving_30_kmh rosbag2 directory.
                            Default: demo_data/autoware_leo_drive_isuzu/driving_30_kmh_2022_06_10-15_47_42_compressed
  --applanix-msg-dir PATH   Path to applanix_msgs/msg.
                            Default: /tmp/applanix/applanix_msgs/msg
  --tf-bag PATH             TF source bag used for the IMU case.
                            Default: demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed
  --output-dir DIR          Output root directory.
                            Default: output/open_data_classic_path_benchmark_<timestamp>
  --verify-map              Verify each generated map bundle.
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

timestamp() {
  date +%Y%m%d_%H%M%S
}

BAG_PATH="${REPO_ROOT}/demo_data/autoware_leo_drive_isuzu/driving_30_kmh_2022_06_10-15_47_42_compressed"
APPLANIX_MSG_DIR="/tmp/applanix/applanix_msgs/msg"
TF_BAG="${REPO_ROOT}/demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed"
OUTPUT_DIR=""
VERIFY_MAP="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --bag)
      BAG_PATH="$(realpath "${2:-}")"; shift 2 ;;
    --applanix-msg-dir)
      APPLANIX_MSG_DIR="$(realpath "${2:-}")"; shift 2 ;;
    --tf-bag)
      TF_BAG="$(realpath "${2:-}")"; shift 2 ;;
    --output-dir)
      OUTPUT_DIR="$(realpath -m "${2:-}")"; shift 2 ;;
    --verify-map)
      VERIFY_MAP="true"; shift ;;
    *)
      die "unknown arg: $1"
      ;;
  esac
done

[[ -d "${BAG_PATH}" ]] || die "bag not found: ${BAG_PATH}"
[[ -d "${APPLANIX_MSG_DIR}" ]] || die "applanix msg dir not found: ${APPLANIX_MSG_DIR}"
[[ -d "${TF_BAG}" ]] || die "tf bag not found: ${TF_BAG}"

if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${REPO_ROOT}/output/open_data_classic_path_benchmark_$(timestamp)"
fi
mkdir -p "${OUTPUT_DIR}"

COMMON_ARGS=(
  --bag "${BAG_PATH}"
  --applanix-msg-dir "${APPLANIX_MSG_DIR}"
)
if [[ "${VERIFY_MAP}" == "true" ]]; then
  COMMON_ARGS+=(--verify-map)
fi

bash "${SCRIPT_DIR}/run_open_data_applanix_velodyne_gnss_benchmark.sh" \
  "${COMMON_ARGS[@]}" \
  --use-gnss false \
  --output-dir "${OUTPUT_DIR}/no_gnss"

bash "${SCRIPT_DIR}/run_open_data_applanix_velodyne_gnss_benchmark.sh" \
  "${COMMON_ARGS[@]}" \
  --output-dir "${OUTPUT_DIR}/gnss_only"

bash "${SCRIPT_DIR}/run_open_data_applanix_velodyne_gnss_benchmark.sh" \
  "${COMMON_ARGS[@]}" \
  --use-odom-prior true \
  --odom-frame-id odom \
  --odom-prior-planar true \
  --odom-prior-velocity-planar true \
  --odom-prior-translation-only true \
  --odom-prior-weight 1.0 \
  --robot-frame-id velodyne_front \
  --output-dir "${OUTPUT_DIR}/gnss_odom_prior"

bash "${SCRIPT_DIR}/run_open_data_applanix_velodyne_gnss_benchmark.sh" \
  "${COMMON_ARGS[@]}" \
  --use-imu true \
  --tf-bag "${TF_BAG}" \
  --robot-frame-id base_link \
  --imu-frame-id base_link \
  --output-dir "${OUTPUT_DIR}/gnss_imu"

python3 "${SCRIPT_DIR}/generate_classic_path_report.py" \
  --no-gnss-dir "${OUTPUT_DIR}/no_gnss" \
  --gnss-only-dir "${OUTPUT_DIR}/gnss_only" \
  --gnss-odom-dir "${OUTPUT_DIR}/gnss_odom_prior" \
  --gnss-imu-dir "${OUTPUT_DIR}/gnss_imu" \
  --out "${OUTPUT_DIR}/classic_path_report.md" \
  --write-json "${OUTPUT_DIR}/classic_path_report.json" \
  --write-svg "${OUTPUT_DIR}/classic_path_report.svg"

echo "done: ${OUTPUT_DIR}"
