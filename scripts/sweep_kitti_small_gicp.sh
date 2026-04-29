#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  sweep_kitti_small_gicp.sh --dataset <kitti_root> --sequences "00 05 07" [options]

Options:
  --dataset PATH          KITTI odometry root (has sequences/ and poses/)
  --sequences "00 05"     Space-separated list of sequences (training 00-10 recommended)
  --output-root DIR       Output root (default: output/kitti_small_gicp_sweep_<timestamp>)
  --play-rate FLOAT       ros2 bag play rate (default: 5.0)
  --drain-secs SEC        backend drain time (default: 20)
  --force-prepare         overwrite prepared bags
  --help

This script:
  - prepares LiDAR-only rosbag2 + GT TUM for each sequence
  - runs multiple small_gicp configs per sequence
  - writes output/benchmark_summary.md + .csv under output-root

Edit CONFIGS below to add/remove sweeps.
EOF
  exit 1
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

DATASET=""
SEQUENCES=""
OUTPUT_ROOT=""
PLAY_RATE="5.0"
DRAIN_SECS="20"
FORCE_PREPARE=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dataset)
      [[ $# -ge 2 ]] || usage
      DATASET=$(realpath "$2")
      shift 2
      ;;
    --sequences)
      [[ $# -ge 2 ]] || usage
      SEQUENCES="$2"
      shift 2
      ;;
    --output-root)
      [[ $# -ge 2 ]] || usage
      OUTPUT_ROOT=$(realpath -m "$2")
      shift 2
      ;;
    --play-rate)
      [[ $# -ge 2 ]] || usage
      PLAY_RATE="$2"
      shift 2
      ;;
    --drain-secs)
      [[ $# -ge 2 ]] || usage
      DRAIN_SECS="$2"
      shift 2
      ;;
    --force-prepare)
      FORCE_PREPARE=true
      shift
      ;;
    --help|-h)
      usage
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      ;;
  esac
done

[[ -n "$DATASET" ]] || usage
[[ -n "$SEQUENCES" ]] || usage

if [[ -z "$OUTPUT_ROOT" ]]; then
  OUTPUT_ROOT="${REPO_ROOT}/output/kitti_small_gicp_sweep_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$OUTPUT_ROOT"

LIDAR_TOPIC="/kitti/velodyne/points"
LIDARSLAM_PARAM="${REPO_ROOT}/lidarslam/param/lidarslam_kitti_velodyne.yaml"

# name|args... (passed to run_small_gicp_graph_benchmark.sh)
CONFIGS=(
  "icp_fast|--ds 0.7 --voxel 1.0 --corr 3.0 --range 3.0 120.0 --use-gicp false"
  "icp_tight|--ds 0.5 --voxel 1.0 --corr 2.0 --range 3.0 120.0 --use-gicp false"
  "icp_loose_corr|--ds 0.7 --voxel 1.0 --corr 4.0 --range 3.0 120.0 --use-gicp false"
  "voxel_fine|--ds 0.6 --voxel 0.8 --corr 2.5 --range 3.0 120.0 --use-gicp false"
  "range_wide|--ds 0.7 --voxel 1.0 --corr 3.0 --range 2.0 130.0 --use-gicp false"
  "gicp|--ds 0.7 --voxel 1.0 --corr 2.5 --range 3.0 120.0 --use-gicp true"
)

for seq in $SEQUENCES; do
  PREP_DIR="${OUTPUT_ROOT}/prepare_seq${seq}"
  mkdir -p "$PREP_DIR"

  PREP_ARGS=(
    --dataset "$DATASET"
    --sequence "$seq"
    --output-dir "$PREP_DIR"
    --lidar-topic "$LIDAR_TOPIC"
    --lidar-only-bag
  )
  if [[ "$FORCE_PREPARE" == "true" ]]; then
    PREP_ARGS+=(--force)
  fi
  python3 "${SCRIPT_DIR}/kitti_odometry_prepare.py" "${PREP_ARGS[@]}"

  BAG_DIR="${PREP_DIR}/kitti_seq${seq}_rosbag2"
  REF_TUM="${PREP_DIR}/kitti_seq${seq}_gt_velo.tum"
  REF_META="${PREP_DIR}/kitti_seq${seq}_reference.json"

  if [[ ! -f "$REF_TUM" ]]; then
    echo "skip seq ${seq}: missing training GT at ${REF_TUM}" >&2
    continue
  fi

  for item in "${CONFIGS[@]}"; do
    name="${item%%|*}"
    args="${item#*|}"
    OUT_DIR="${OUTPUT_ROOT}/seq${seq}/${name}"
    mkdir -p "$OUT_DIR"

    # shellcheck disable=SC2086
    bash "${SCRIPT_DIR}/run_small_gicp_graph_benchmark.sh" \
      --bag "$BAG_DIR" \
      --reference-tum "$REF_TUM" \
      --reference-meta "$REF_META" \
      --input-cloud "$LIDAR_TOPIC" \
      --lidarslam-param "$LIDARSLAM_PARAM" \
      --robot-frame velodyne \
      --publish-tf true \
      --play-rate "$PLAY_RATE" \
      --drain-secs "$DRAIN_SECS" \
      --reference-source kitti_odometry_gt_velo \
      --output-dir "$OUT_DIR" \
      $args
  done
done

python3 "${SCRIPT_DIR}/benchmark_summary.py" \
  --root "$OUTPUT_ROOT" \
  --write-md "${OUTPUT_ROOT}/benchmark_summary.md" \
  --write-csv "${OUTPUT_ROOT}/benchmark_summary.csv"

echo "done: ${OUTPUT_ROOT}"

