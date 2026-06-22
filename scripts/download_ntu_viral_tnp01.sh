#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat >&2 <<'EOF'
Usage:
  bash scripts/download_ntu_viral_tnp01.sh [options]

Options:
  --dest DIR         Destination root directory (default: ./demo_data/ntu_viral)
  --keep-zip         Keep the downloaded zip file
  --no-convert       Skip rosbag1 -> rosbag2 conversion
  --no-restamp       Skip creation of the default RKO-LIO rosbag2
  -h, --help         Show this help

This script downloads the official NTU VIRAL tnp_01 sequence referenced from
the GLIM supplementary pages, extracts it, and optionally converts the ROS1 bag
to rosbag2 format using rosbags-convert. By default it also writes the
pointcloud+IMU rosbag2 expected by `run_autoware_quickstart.sh` and
`run_rko_lio_graph_benchmark.sh`.
EOF
}

fail() {
  echo "error: $*" >&2
  echo "hint: run 'bash scripts/download_ntu_viral_tnp01.sh --help' for valid options." >&2
  exit 2
}

die() {
  echo "error: $*" >&2
  exit 1
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "${value}" || "${value}" == -* ]]; then
    fail "option requires a value: ${option}"
  fi
  OPTION_VALUE="${value}"
}

require_command() {
  local command_name="$1"
  local install_hint="$2"
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "error: required command not found: ${command_name}" >&2
    echo "hint: ${install_hint}" >&2
    exit 2
  fi
}

DEST_DIR="${REPO_ROOT}/demo_data/ntu_viral"
KEEP_ZIP="false"
DO_CONVERT="true"
DO_RESTAMP="true"
OPTION_VALUE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dest)
      require_value "$1" "${2:-}"
      DEST_DIR="${OPTION_VALUE}"
      shift 2
      ;;
    --keep-zip)
      KEEP_ZIP="true"; shift ;;
    --no-convert)
      DO_CONVERT="false"; shift ;;
    --no-restamp)
      DO_RESTAMP="false"; shift ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      fail "unknown option: $1" ;;
  esac
done

DEST_DIR="$(realpath -m "${DEST_DIR}")"

SEQ_NAME="tnp_01"
ZIP_PATH="${DEST_DIR}/${SEQ_NAME}.zip"
EXTRACT_DIR="${DEST_DIR}/${SEQ_NAME}"
ROS2_DIR="${DEST_DIR}/${SEQ_NAME}_rosbag2"
RESTAMPED_DIR="${DEST_DIR}/${SEQ_NAME}_points_restamped_vn100_rosbag2"
URL="https://researchdata.ntu.edu.sg/api/access/datafile/98195"

if [[ -e "${DEST_DIR}" && ! -d "${DEST_DIR}" ]]; then
  fail "destination path is not a directory: ${DEST_DIR}"
fi

if [[ "${DO_CONVERT}" != "true" && "${DO_RESTAMP}" == "true" && ! -f "${ROS2_DIR}/metadata.yaml" ]]; then
  fail "--no-convert requires existing rosbag2 metadata when restamp is enabled: ${ROS2_DIR}/metadata.yaml"
fi

require_command wget "install wget, for example: sudo apt install wget"
require_command unzip "install unzip, for example: sudo apt install unzip"
if [[ "${DO_CONVERT}" == "true" ]]; then
  require_command rosbags-convert "install rosbags, for example: python3 -m pip install rosbags"
fi
if [[ "${DO_RESTAMP}" == "true" ]]; then
  require_command python3 "install python3, for example: sudo apt install python3"
fi

mkdir -p "${DEST_DIR}" || die "failed to create destination directory: ${DEST_DIR}"

echo "sequence:   ${SEQ_NAME}"
echo "source:     ${URL}"
echo "dest:       ${DEST_DIR}"
echo "extract:    ${EXTRACT_DIR}"
echo "rosbag2:    ${ROS2_DIR}"
echo "restamped:  ${RESTAMPED_DIR}"

if [[ ! -f "${ZIP_PATH}" ]]; then
  echo "downloading zip..."
  if ! wget -c -O "${ZIP_PATH}" "${URL}"; then
    die "download failed from ${URL}"
  fi
else
  echo "zip already exists: ${ZIP_PATH}"
fi

mkdir -p "${EXTRACT_DIR}"
if ! find "${EXTRACT_DIR}" -maxdepth 2 -name '*.bag' | grep -q .; then
  echo "extracting zip..."
  if ! unzip -q -o "${ZIP_PATH}" -d "${EXTRACT_DIR}"; then
    die "failed to extract zip: ${ZIP_PATH}"
  fi
else
  echo "bag already extracted under: ${EXTRACT_DIR}"
fi

BAG_PATH="$(find "${EXTRACT_DIR}" -maxdepth 3 -name '*.bag' | head -n 1 || true)"
[[ -n "${BAG_PATH}" ]] || die "failed to locate extracted .bag file under ${EXTRACT_DIR}"

echo "rosbag1:    ${BAG_PATH}"
echo "topics:"
echo "  points: /os1_cloud_node1/points"
echo "  imu:    /imu/imu"

if [[ "${DO_CONVERT}" == "true" ]]; then
  if [[ ! -e "${ROS2_DIR}/metadata.yaml" ]]; then
    echo "converting rosbag1 -> rosbag2..."
    rm -rf "${ROS2_DIR}"
    if ! rosbags-convert --src "${BAG_PATH}" --dst "${ROS2_DIR}"; then
      die "rosbag1 to rosbag2 conversion failed"
    fi
  else
    echo "rosbag2 already exists: ${ROS2_DIR}"
  fi
  echo "rosbag2 dir: ${ROS2_DIR}"
fi

if [[ "${DO_RESTAMP}" == "true" ]]; then
  [[ -f "${ROS2_DIR}/metadata.yaml" ]] || die "restamp requires rosbag2 at ${ROS2_DIR}"
  if [[ ! -e "${RESTAMPED_DIR}/metadata.yaml" ]]; then
    echo "creating restamped rosbag2 for RKO-LIO..."
    if ! python3 "${SCRIPT_DIR}/restamp_rosbag2_topics.py" \
      --input "${ROS2_DIR}" \
      --output "${RESTAMPED_DIR}" \
      --topic /os1_cloud_node1/points \
      --copy-topic /imu/imu \
      --force; then
      die "failed to create restamped rosbag2: ${RESTAMPED_DIR}"
    fi
  else
    echo "restamped rosbag2 already exists: ${RESTAMPED_DIR}"
  fi
  echo "restamped rosbag2 dir: ${RESTAMPED_DIR}"
fi

if [[ "${KEEP_ZIP}" != "true" ]]; then
  echo "removing zip..."
  rm -f "${ZIP_PATH}"
fi

echo "done"
