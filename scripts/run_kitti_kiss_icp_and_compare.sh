#!/usr/bin/env bash
# MIT baseline: run KISS-ICP CLI on KITTI Odometry (velodyne bins), then optional
# compare against a lidarslam sweep trajectory via compare_kitti_odometry_two_estimates.sh.
#
# Does not build FAST_LIO (needs livox_ros_driver2 + Livox-SDK2; SDK may fail on newer GCC).
#
# Usage:
#   bash scripts/run_kitti_kiss_icp_and_compare.sh \
#     --dataset /path/to/KITTI_odometry \
#     --sequence 00 \
#     --out-root output/kiss_icp_kitti \
#     [--sweep-root output/kitti_small_gicp_sweep_*]   # if set, prints compare command
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DATASET=""
SEQUENCE="00"
OUT_ROOT="${SCRIPT_DIR}/../output/kiss_icp_kitti"
SWEEP_ROOT=""
RUN_NAME=""

usage() {
  cat <<'EOF' >&2
Usage:
  run_kitti_kiss_icp_and_compare.sh --dataset DIR [--sequence 00] [--out-root DIR] [--sweep-root DIR] [--run NAME]

Requires: kiss_icp_pipeline on PATH (pip install kiss-icp).
EOF
  exit 1
}

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
    --out-root)
      [[ $# -ge 2 ]] || usage
      OUT_ROOT=$(realpath -m "$2")
      shift 2
      ;;
    --sweep-root)
      [[ $# -ge 2 ]] || usage
      SWEEP_ROOT=$(realpath "$2")
      shift 2
      ;;
    --run)
      [[ $# -ge 2 ]] || usage
      RUN_NAME="$2"
      shift 2
      ;;
    -h|--help)
      usage
      ;;
    *)
      echo "Unknown: $1" >&2
      usage
      ;;
  esac
done

[[ -n "$DATASET" && -d "$DATASET/sequences/${SEQUENCE}/velodyne" ]] || usage
command -v kiss_icp_pipeline >/dev/null 2>&1 || {
  echo "kiss_icp_pipeline not found; try: pip install --user kiss-icp" >&2
  exit 1
}

mkdir -p "$OUT_ROOT"
export kiss_icp_out_dir="$OUT_ROOT"

echo "Running KISS-ICP on ${DATASET} sequence ${SEQUENCE} ..."
kiss_icp_pipeline --dataloader kitti --sequence "$SEQUENCE" "$DATASET"

LATEST=$(readlink -f "$OUT_ROOT/latest" 2>/dev/null || true)
KISS_TUM=""
if [[ -n "$LATEST" && -f "$LATEST/${SEQUENCE}_poses_tum.txt" ]]; then
  KISS_TUM="$LATEST/${SEQUENCE}_poses_tum.txt"
elif [[ -n "$LATEST" ]]; then
  KISS_TUM=$(find "$LATEST" -maxdepth 1 -name '*_poses_tum.txt' | head -1)
fi

if [[ -z "$KISS_TUM" || ! -f "$KISS_TUM" ]]; then
  echo "Could not find KISS poses TUM under $OUT_ROOT/latest" >&2
  exit 1
fi

echo "KISS-ICP trajectory: $KISS_TUM"

if [[ -n "$SWEEP_ROOT" ]]; then
  PREP="$SWEEP_ROOT/prepare_seq${SEQUENCE}"
  REF_TUM="$PREP/kitti_seq${SEQUENCE}_gt_velo.tum"
  REF_META="$PREP/kitti_seq${SEQUENCE}_reference.json"
  if [[ ! -f "$REF_TUM" || ! -f "$REF_META" ]]; then
    echo "Sweep prepare dir missing GT or meta: $PREP" >&2
    exit 1
  fi
  if [[ -z "$RUN_NAME" ]]; then
    RUN_NAME=$(python3 - "$SWEEP_ROOT/benchmark_summary.csv" <<'PY'
import csv, sys
from pathlib import Path
p = Path(sys.argv[1])
rows = list(csv.DictReader(p.read_text(encoding='utf-8').splitlines()))
print(min(rows, key=lambda r: float(r.get('ape_rmse_m') or 1e9))['run'])
PY
    )
  fi
  OUR="$SWEEP_ROOT/seq${SEQUENCE}/${RUN_NAME}/traj_raw_prism.tum"
  if [[ ! -f "$OUR" ]]; then
    OUR="$SWEEP_ROOT/seq${SEQUENCE}/${RUN_NAME}/traj_corrected_prism.tum"
  fi
  if [[ ! -f "$OUR" ]]; then
    echo "No lidarslam TUM for run $RUN_NAME under $SWEEP_ROOT/seq${SEQUENCE}/" >&2
    exit 1
  fi
  CMP="$SWEEP_ROOT/seq${SEQUENCE}/compare_lidarslam_${RUN_NAME}_vs_kiss_icp"
  echo ""
  echo "Compare (copy-paste):"
  printf '%q ' bash "$SCRIPT_DIR/compare_kitti_odometry_two_estimates.sh" \
    --reference-tum "$REF_TUM" \
    --reference-meta "$REF_META" \
    --estimate-a-tum "$OUR" \
    --estimate-b-tum "$KISS_TUM" \
    --label-a "lidarslam_${RUN_NAME}" \
    --label-b kiss_icp \
    --out-dir "$CMP"
  echo ""
fi
