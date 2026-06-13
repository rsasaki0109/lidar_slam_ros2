#!/usr/bin/env bash
# Download the HILTI SLAM Challenge 2022 exp01 sequence (construction ground
# level, handheld Phasma platform: Hesai PandarXT-32 + Alphasense IMU) and
# convert it to rosbag2 for the RKO-LIO benchmark pipeline.
#
# Ground truth: millimeter-accurate total-station control points, published
# as a sparse TUM-compatible file (identity orientation, 3DOF positions).
# License: the HILTI-Oxford dataset is CC BY-NC-SA 3.0 (non-commercial,
# attribution). It is used here strictly as a local evaluation substrate;
# nothing from the dataset is redistributed with this repository.
#
# Usage:
#   bash scripts/download_hilti2022_exp01.sh [--dest datasets/hilti2022] \
#     [--no-convert] [--keep-bag1]
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

DEST_DIR="${REPO_ROOT}/datasets/hilti2022"
DO_CONVERT=true
KEEP_BAG1=true

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dest) DEST_DIR=$(realpath -m "$2"); shift 2 ;;
    --no-convert) DO_CONVERT=false; shift ;;
    --keep-bag1) KEEP_BAG1=true; shift ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

BASE_URL="https://tp-public-facing.s3.eu-north-1.amazonaws.com/Challenges/2022"
GT_URL="https://hilti-challenge.com/assets/2022/ground_truth"
BAG1="${DEST_DIR}/exp01_construction_ground_level.bag"
ROS2_DIR="${DEST_DIR}/exp01_ros2"

mkdir -p "${DEST_DIR}"

echo "dest:    ${DEST_DIR}"
echo "rosbag2: ${ROS2_DIR}"

if [[ ! -f "${DEST_DIR}/exp01_construction_ground_level_gt.txt" ]]; then
  echo "downloading ground truth (control points)..."
  curl -sfL -o "${DEST_DIR}/exp01_construction_ground_level_gt.txt" \
    "${GT_URL}/exp01_construction_ground_level.txt"
fi

if [[ ! -f "${DEST_DIR}/calibration_files.zip" ]]; then
  echo "downloading calibration files..."
  curl -sfL -o "${DEST_DIR}/calibration_files.zip" \
    "${BASE_URL}/2022322_calibration_files.zip"
fi

if [[ ! -f "${BAG1}" && ! -e "${ROS2_DIR}/metadata.yaml" ]]; then
  echo "downloading exp01 rosbag (~18 GB)..."
  curl -fL --retry 3 -o "${BAG1}" \
    "${BASE_URL}/exp01_construction_ground_level.bag"
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

if [[ "${KEEP_BAG1}" != "true" && -e "${ROS2_DIR}/metadata.yaml" ]]; then
  rm -f "${BAG1}"
fi

echo "done"
echo "  bag:    ${ROS2_DIR}"
echo "  gt:     ${DEST_DIR}/exp01_construction_ground_level_gt.txt"
echo "  topics: /hesai/pandar (PointCloud2), /alphasense/imu (Imu)"
echo
echo "Benchmark:"
echo "  bash scripts/run_rko_lio_graph_benchmark.sh \\"
echo "    --bag ${ROS2_DIR} \\"
echo "    --lidar-topic /hesai/pandar --imu-topic /alphasense/imu \\"
echo "    --rko-param configs/hilti2022/rko_lio_hilti2022_pandar.yaml \\"
echo "    --reference-tum ${DEST_DIR}/exp01_construction_ground_level_gt.txt \\"
echo "    --skip-reference-gen --reference-source hilti2022_exp01_control_points \\"
echo "    --quiescence-secs 60 --output-dir output/hilti2022_exp01_run"
echo
echo "Scoring (dense raw odometry vs sparse stationary control points):"
echo "  python3 scripts/ape_from_tum.py --interpolate --max-time-diff 2.0 \\"
echo "    --ref ${DEST_DIR}/exp01_construction_ground_level_gt.txt \\"
echo "    --est output/hilti2022_exp01_run/traj_raw.tum --out ape.txt"
