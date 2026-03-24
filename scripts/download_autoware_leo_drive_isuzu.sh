#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
S3_ROOT="s3://autoware-files/collected_data/2022-08-22_leo_drive_isuzu_bags"
GNSS_TOPIC="/applanix/lvx_client/gnss/fix"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/download_autoware_leo_drive_isuzu.sh [options]

Options:
  --dest DIR         Destination root directory
                     (default: ./demo_data/autoware_leo_drive_isuzu)
  --bag NAME         Single bag directory to sync
                     (default: all-sensors-bag1_compressed)
  --full             Download the full dataset tree
  --list             List available bag directories from the official bucket
  --no-inspect       Skip ros2 bag info and GNSS covariance inspection
  -h, --help         Show this help

This script downloads the official Autoware Leo Drive ISUZU open dataset from:
  s3://autoware-files/collected_data/2022-08-22_leo_drive_isuzu_bags/

According to the Autoware dataset documentation, the bag includes:
  - sensor_msgs/msg/NavSatFix on /applanix/lvx_client/gnss/fix
  - sensor_msgs/msg/Imu on /applanix/lvx_client/imu_raw
  - sensor_msgs/msg/PointCloud2 on /pandar_points
  - vehicle status report data
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

DEST_DIR="${REPO_ROOT}/demo_data/autoware_leo_drive_isuzu"
BAG_NAME="all-sensors-bag1_compressed"
DO_FULL="false"
LIST_ONLY="false"
DO_INSPECT="true"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dest)
      DEST_DIR="${2:-}"
      shift 2
      ;;
    --bag)
      BAG_NAME="${2:-}"
      shift 2
      ;;
    --full)
      DO_FULL="true"
      shift
      ;;
    --list)
      LIST_ONLY="true"
      shift
      ;;
    --no-inspect)
      DO_INSPECT="false"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown arg: $1"
      ;;
  esac
done

command -v aws >/dev/null 2>&1 || die "aws not found"
mkdir -p "${DEST_DIR}"

if [[ "${LIST_ONLY}" == "true" ]]; then
  aws s3 ls "${S3_ROOT}/" --no-sign-request
  exit 0
fi

if [[ "${DO_FULL}" == "true" ]]; then
  echo "syncing full dataset from ${S3_ROOT}/"
  aws s3 sync "${S3_ROOT}/" "${DEST_DIR}" --no-sign-request
  TARGET_DIR="${DEST_DIR}"
else
  echo "syncing ${BAG_NAME} from ${S3_ROOT}/${BAG_NAME}/"
  TARGET_DIR="${DEST_DIR}/${BAG_NAME}"
  aws s3 sync "${S3_ROOT}/${BAG_NAME}/" "${TARGET_DIR}" --no-sign-request
fi

echo "downloaded_to: ${TARGET_DIR}"
echo "expected_topics:"
echo "  gnss:  ${GNSS_TOPIC}"
echo "  imu:   /applanix/lvx_client/imu_raw"
echo "  lidar: /pandar_points"

if [[ "${DO_INSPECT}" != "true" ]]; then
  exit 0
fi

METADATA_PATH="$(find "${TARGET_DIR}" -name metadata.yaml | head -n 1 || true)"
if [[ -z "${METADATA_PATH}" ]]; then
  echo "metadata.yaml not found under ${TARGET_DIR}; skipping bag inspection"
  exit 0
fi

BAG_DIR="$(dirname "${METADATA_PATH}")"
echo "bag_dir: ${BAG_DIR}"

if command -v ros2 >/dev/null 2>&1; then
  echo "ros2 bag info:"
  ros2 bag info "${BAG_DIR}"
else
  echo "ros2 not found; skipping ros2 bag info"
fi

if command -v python3 >/dev/null 2>&1; then
  echo "gnss covariance inspection:"
  python3 "${SCRIPT_DIR}/inspect_navsatfix_covariance.py" "${BAG_DIR}" --topic "${GNSS_TOPIC}" || true
else
  echo "python3 not found; skipping GNSS covariance inspection"
fi
