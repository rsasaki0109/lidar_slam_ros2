#!/usr/bin/env bash
# Download a HILTI SLAM Challenge 2022 sequence (handheld Phasma platform:
# Hesai PandarXT-32 + Alphasense IMU) and convert it to rosbag2 for the
# RKO-LIO benchmark pipeline.
#
# Ground truth: millimeter-accurate total-station control points, published
# as a sparse TUM-compatible file (identity orientation, 3DOF positions).
# License: the HILTI-Oxford dataset is CC BY-NC-SA 3.0 (non-commercial,
# attribution). It is used here strictly as a local evaluation substrate;
# nothing from the dataset is redistributed with this repository.
#
# Usage:
#   bash scripts/download_hilti2022.sh [--sequence exp01|exp02|exp03|exp04|exp07|exp21] \
#     [--dest datasets/hilti2022] [--no-convert] [--drop-bag1]
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

DEST_DIR="${REPO_ROOT}/datasets/hilti2022"
SEQUENCE="exp01"
DO_CONVERT=true
DROP_BAG1=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sequence) SEQUENCE="$2"; shift 2 ;;
    --dest) DEST_DIR=$(realpath -m "$2"); shift 2 ;;
    --no-convert) DO_CONVERT=false; shift ;;
    --drop-bag1) DROP_BAG1=true; shift ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

# Map a short sequence id to its dataset file slug.
case "${SEQUENCE}" in
  exp01) SLUG="exp01_construction_ground_level"; EXPECTED_BAG_BYTES=0 ;;
  exp02) SLUG="exp02_construction_multilevel"; EXPECTED_BAG_BYTES=35358555915 ;;
  exp03) SLUG="exp03_construction_stairs"; EXPECTED_BAG_BYTES=21915037983 ;;
  exp04) SLUG="exp04_construction_upper_level"; EXPECTED_BAG_BYTES=0 ;;
  exp07) SLUG="exp07_long_corridor"; EXPECTED_BAG_BYTES=0 ;;
  exp21) SLUG="exp21_outside_building"; EXPECTED_BAG_BYTES=12014463331 ;;
  *) echo "unknown sequence: ${SEQUENCE} (known: exp01, exp02, exp03, exp04, exp07, exp21)" >&2; exit 2 ;;
esac

BASE_URL="https://tp-public-facing.s3.eu-north-1.amazonaws.com/Challenges/2022"
GT_URL="https://hilti-challenge.com/assets/2022/ground_truth"
BAG1="${DEST_DIR}/${SLUG}.bag"
ROS2_DIR="${DEST_DIR}/${SEQUENCE}_ros2"
GT_FILE="${DEST_DIR}/${SLUG}_gt.txt"
RAW_BAG_SHA_FILE="${DEST_DIR}/${SEQUENCE}_raw_bag.sha256"

mkdir -p "${DEST_DIR}"

echo "sequence: ${SEQUENCE} (${SLUG})"
echo "dest:     ${DEST_DIR}"
echo "rosbag2:  ${ROS2_DIR}"

if [[ ! -f "${GT_FILE}" ]]; then
  echo "downloading ground truth (control points)..."
  curl -sfL -o "${GT_FILE}" "${GT_URL}/${SLUG}.txt"
fi

if [[ ! -f "${DEST_DIR}/calibration_files.zip" ]]; then
  echo "downloading calibration files..."
  curl -sfL -o "${DEST_DIR}/calibration_files.zip" \
    "${BASE_URL}/2022322_calibration_files.zip"
fi

ACTUAL_BAG_BYTES=0
[[ ! -f "${BAG1}" ]] || ACTUAL_BAG_BYTES=$(stat -c '%s' "${BAG1}")
if [[ ! -e "${ROS2_DIR}/metadata.yaml" ]] && \
   [[ ! -f "${BAG1}" || ("${EXPECTED_BAG_BYTES}" -gt 0 && "${ACTUAL_BAG_BYTES}" -ne "${EXPECTED_BAG_BYTES}") ]]; then
  echo "downloading ${SLUG} rosbag (multi-GB)..."
  curl -fL --retry 3 --retry-all-errors --continue-at - \
    -o "${BAG1}" "${BASE_URL}/${SLUG}.bag"
fi

if [[ -f "${BAG1}" && "${EXPECTED_BAG_BYTES}" -gt 0 ]]; then
  ACTUAL_BAG_BYTES=$(stat -c '%s' "${BAG1}")
  if [[ "${ACTUAL_BAG_BYTES}" -ne "${EXPECTED_BAG_BYTES}" ]]; then
    echo "incomplete rosbag1: expected ${EXPECTED_BAG_BYTES} bytes, got ${ACTUAL_BAG_BYTES}" >&2
    exit 1
  fi
fi

# Keep the original rosbag1 content identity even when --drop-bag1 is used.
# FAST-LIVO2 consumes this representation while the ROS 2 systems consume the
# converted bag, so both identities are part of the same-input audit trail.
if [[ -f "${BAG1}" && ! -s "${RAW_BAG_SHA_FILE}" ]]; then
  echo "hashing original rosbag1..."
  sha256sum "${BAG1}" | cut -d' ' -f1 > "${RAW_BAG_SHA_FILE}"
fi

if [[ "${DO_CONVERT}" == "true" && ! -e "${ROS2_DIR}/metadata.yaml" ]]; then
  command -v rosbags-convert >/dev/null 2>&1 || {
    echo "rosbags-convert not found (pip install rosbags)" >&2
    exit 1
  }
  echo "converting rosbag1 -> rosbag2..."
  rm -rf "${ROS2_DIR}"
  rosbags-convert --src "${BAG1}" --dst "${ROS2_DIR}"
fi

if [[ "${DROP_BAG1}" == "true" && -e "${ROS2_DIR}/metadata.yaml" ]]; then
  rm -f "${BAG1}"
fi

echo "done"
echo "  bag:    ${ROS2_DIR}"
echo "  raw sha: ${RAW_BAG_SHA_FILE}"
echo "  gt:     ${GT_FILE}"
echo "  topics: /hesai/pandar (PointCloud2), /alphasense/imu (Imu)"
echo
echo "Benchmark:"
echo "  bash scripts/run_rko_lio_graph_benchmark.sh \\"
echo "    --bag ${ROS2_DIR} \\"
echo "    --lidar-topic /hesai/pandar --imu-topic /alphasense/imu \\"
echo "    --rko-param configs/hilti2022/rko_lio_hilti2022_pandar.yaml \\"
echo "    --reference-tum ${GT_FILE} \\"
echo "    --skip-reference-gen --reference-source hilti2022_${SEQUENCE}_control_points_gt \\"
echo "    --quiescence-secs 60 --output-dir output/hilti2022_${SEQUENCE}_run"
echo
echo "Scoring (dense raw odometry vs sparse stationary control points):"
echo "  python3 scripts/ape_from_tum.py --interpolate --max-time-diff 3.0 \\"
echo "    --ref ${GT_FILE} \\"
echo "    --est output/hilti2022_${SEQUENCE}_run/traj_raw.tum --out ape.txt"
