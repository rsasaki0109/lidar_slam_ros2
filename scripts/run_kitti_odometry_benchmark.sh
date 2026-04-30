#!/usr/bin/env bash
set -euo pipefail

# End-to-end: KITTI Odometry sequence -> rosbag2 + GT TUM -> SLAM + APE.
# Default: RKO-LIO + graph.
# - --lo: scanmatcher LiDAR-only + graph (no IMU in bag)
# - --small-gicp: small_gicp ICP/GICP odometry + graph (no IMU in bag; requires small_gicp built)
#
# Requires:
#   - KITTI Odometry dataset root (sequences/ + poses/ for training ids).
#     Fetch with: bash scripts/download_kitti_odometry.sh [--velodyne]
#   - Velodyne bins: same script with --velodyne (~79 GiB), or merge an existing dataset tree.
#   - colcon-built workspace (source install/setup.bash before calling, or place install/ next to repo)
#   - python3 + rosbags, evo (for APE)
#
# Example:
#   bash scripts/run_kitti_odometry_benchmark.sh \
#     --dataset /data/kitti/dataset \
#     --sequence 00 \
#     --prepare-only
#
#   bash scripts/run_kitti_odometry_benchmark.sh \
#     --dataset /data/kitti/dataset \
#     --sequence 00

usage() {
  cat <<'EOF' >&2
Usage:
  run_kitti_odometry_benchmark.sh [--dataset PATH] --sequence ID [options]

Required:
  --sequence ID              KITTI sequence (00, 01, ...)

Options:
  --dataset PATH             KITTI odometry root (default: $KITTI_ODOMETRY_ROOT)
  --prepare-only             Only run kitti_odometry_prepare.py and exit
  --reuse-prepare            Skip prepare step (expect bag + YAML under output-dir)
  --output-dir PATH          Artifact directory (default: output/kitti_bench_<seq>_<timestamp>)
  --force-prepare            Pass --force to prepare (overwrite rosbag2)
  --lo                       LiDAR-only: LiDAR-only rosbag + run_lo_graph_benchmark.sh
  --small-gicp               LiDAR-only: LiDAR-only rosbag + run_small_gicp_graph_benchmark.sh
  --                         Remaining args forwarded to the active benchmark script
  --help                     This help

Environment:
  KITTI_ODOMETRY_ROOT        Default --dataset when set

Compare vs external LIO (e.g. GPL baseline in another workspace; same bag/GT):
  bash scripts/compare_kitti_odometry_two_estimates.sh --help
  bash scripts/emit_kitti_baseline_compare_command.sh --help
EOF
  exit 1
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

DATASET="${KITTI_ODOMETRY_ROOT:-}"
SEQUENCE=""
PREPARE_ONLY=false
REUSE_PREPARE=false
OUTPUT_DIR=""
FORCE_PREPARE=false
USE_LO=false
USE_SMALL_GICP=false
EXTRA_BENCH=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dataset)
      [[ $# -ge 2 ]] || usage
      DATASET=$(realpath "$2")
      shift 2
      ;;
    --sequence)
      [[ $# -ge 2 ]] || usage
      SEQUENCE="$2"
      shift 2
      ;;
    --prepare-only)
      PREPARE_ONLY=true
      shift
      ;;
    --reuse-prepare)
      REUSE_PREPARE=true
      shift
      ;;
    --output-dir)
      [[ $# -ge 2 ]] || usage
      OUTPUT_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --force-prepare)
      FORCE_PREPARE=true
      shift
      ;;
    --lo)
      USE_LO=true
      shift
      ;;
    --small-gicp)
      USE_SMALL_GICP=true
      shift
      ;;
    --)
      shift
      EXTRA_BENCH=("$@")
      break
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

[[ -n "$SEQUENCE" ]] || usage
if [[ -z "$DATASET" || ! -d "$DATASET" ]]; then
  echo "Set --dataset or KITTI_ODOMETRY_ROOT to your KITTI odometry root." >&2
  exit 1
fi

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="${REPO_ROOT}/output/kitti_bench_${SEQUENCE}_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$OUTPUT_DIR"

BAG_DIR="${OUTPUT_DIR}/kitti_seq${SEQUENCE}_rosbag2"
RKO_PARAM="${OUTPUT_DIR}/kitti_seq${SEQUENCE}_rko_lio.yaml"
REF_TUM="${OUTPUT_DIR}/kitti_seq${SEQUENCE}_gt_velo.tum"
REF_META="${OUTPUT_DIR}/kitti_seq${SEQUENCE}_reference.json"
LIDAR_TOPIC="/kitti/velodyne/points"
IMU_TOPIC="/kitti/imu/sample"
LIDARSLAM_PARAM="${REPO_ROOT}/lidarslam/param/lidarslam_kitti_velodyne.yaml"

PREPARE_ARGS=(
  --dataset "$DATASET"
  --sequence "$SEQUENCE"
  --output-dir "$OUTPUT_DIR"
  --lidar-topic "$LIDAR_TOPIC"
  --imu-topic "$IMU_TOPIC"
)
if [[ "$FORCE_PREPARE" == "true" ]]; then
  PREPARE_ARGS+=(--force)
fi
if [[ "$USE_LO" == "true" || "$USE_SMALL_GICP" == "true" ]]; then
  PREPARE_ARGS+=(--lidar-only-bag)
fi

if [[ "$REUSE_PREPARE" == "false" ]]; then
  python3 "${SCRIPT_DIR}/kitti_odometry_prepare.py" "${PREPARE_ARGS[@]}"
else
  [[ -d "$BAG_DIR" ]] || { echo "missing prepared bag: $BAG_DIR" >&2; exit 1; }
  if [[ "$USE_LO" != "true" ]]; then
    [[ -f "$RKO_PARAM" ]] || { echo "missing prepared RKO yaml: $RKO_PARAM" >&2; exit 1; }
  fi
fi

if [[ "$PREPARE_ONLY" == "true" ]]; then
  echo "Prepare outputs under: $OUTPUT_DIR"
  exit 0
fi

if [[ ! -f "$REF_TUM" ]]; then
  echo "No reference TUM at $REF_TUM (poses only for training sequences 00-10)." >&2
  echo "Re-run with a training sequence or provide your own --reference-tum via extra-benchmark-args." >&2
  exit 1
fi

set +u
if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${REPO_ROOT}/install/setup.bash"
elif [[ -f "${REPO_ROOT}/../install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${REPO_ROOT}/../install/setup.bash"
fi
set -u

if [[ "$USE_SMALL_GICP" == "true" ]]; then
  bash "${SCRIPT_DIR}/run_small_gicp_graph_benchmark.sh" \
    --bag "$BAG_DIR" \
    --reference-tum "$REF_TUM" \
    --reference-meta "$REF_META" \
    --input-cloud "$LIDAR_TOPIC" \
    --lidarslam-param "$LIDARSLAM_PARAM" \
    --robot-frame velodyne \
    --publish-tf true \
    --reference-source kitti_odometry_gt_velo \
    --output-dir "${OUTPUT_DIR}/bench_run_small_gicp" \
    "${EXTRA_BENCH[@]}"
  echo "KITTI small_gicp benchmark artifacts: ${OUTPUT_DIR}/bench_run_small_gicp"
elif [[ "$USE_LO" == "true" ]]; then
  bash "${SCRIPT_DIR}/run_lo_graph_benchmark.sh" \
    --bag "$BAG_DIR" \
    --reference-tum "$REF_TUM" \
    --reference-meta "$REF_META" \
    --input-cloud "$LIDAR_TOPIC" \
    --lidarslam-param "$LIDARSLAM_PARAM" \
    --robot-frame velodyne \
    --lidar-frame velodyne \
    --publish-static-tf false \
    --reference-source kitti_odometry_gt_velo \
    --output-dir "${OUTPUT_DIR}/bench_run_lo" \
    "${EXTRA_BENCH[@]}"
  echo "KITTI LO benchmark artifacts: ${OUTPUT_DIR}/bench_run_lo"
else
  bash "${SCRIPT_DIR}/run_rko_lio_graph_benchmark.sh" \
    --bag "$BAG_DIR" \
    --reference-tum "$REF_TUM" \
    --reference-meta "$REF_META" \
    --skip-reference-gen \
    --lidar-topic "$LIDAR_TOPIC" \
    --imu-topic "$IMU_TOPIC" \
    --base-frame velodyne \
    --lidarslam-param "$LIDARSLAM_PARAM" \
    --rko-param "$RKO_PARAM" \
    --output-dir "${OUTPUT_DIR}/bench_run" \
    --publish-static-tf false \
    --reference-source kitti_odometry_gt_velo \
    "${EXTRA_BENCH[@]}"
  echo "KITTI benchmark artifacts: ${OUTPUT_DIR}/bench_run"
fi
