#!/usr/bin/env bash
# Side-by-side APE vs KITTI Odometry GT for two estimated TUM trajectories.
# Use this after:
#   (A) lidarslam: run_kitti_odometry_benchmark.sh / run_small_gicp_graph_benchmark.sh → traj*.tum
#   (B) baseline (e.g. FAST-LIO2, GPLv2): run in a *separate* ROS 2 + colcon workspace,
#       play the *same* prepared rosbag2, log a comparable pose topic to TUM.
#
# This repo stays MIT/BSD-only; the baseline is not built or vendored here.
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  compare_kitti_odometry_two_estimates.sh \
    --reference-tum PATH \
    --reference-meta PATH \
    --estimate-a-tum PATH \
    --estimate-b-tum PATH \
    --label-a NAME \
    --label-b NAME \
    [--out-dir DIR]

Computes APE (same logic as ape_from_tum.py) for both estimates vs KITTI training GT,
after applying lidar_to_prism_translation_m from reference-meta (KITTI prepare JSON).

Fair comparison: match trajectory *density* and semantics. Our graph backend may yield a
sparse corrected path (few poses); dense LIO baselines should be compared with our
traj_raw_prism.tum or a downsampled baseline — see emit_kitti_baseline_compare_command.sh --traj.

Baseline workflow example (run outside this repo, GPL stack OK):
  - Prepare bag once: bash scripts/run_kitti_odometry_benchmark.sh --dataset ... --sequence 00 --prepare-only
  - Bag topics (default): /kitti/velodyne/points , /kitti/imu/sample
  - Build FAST-LIO2 in another workspace; remap topics to match; use_sim_time true;
    ros2 bag play <same_rosbag2> --clock ...
  - Log odometry to TUM: python3 scripts/odom_to_tum.py --topic /Odometry --output baseline.tum --use-sim-time true
    (use the topic/frame that matches your comparison intent; both estimates must be comparable to GT framing.)

Outputs under --out-dir:
  <label-a>_prism.tum , <label-b>_prism.tum , ape_<label-a>.txt , ape_<label-b>.txt , comparison_table.txt
EOF
  exit 1
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

REF_TUM=""
REF_META=""
EST_A=""
EST_B=""
LABEL_A="estimate_a"
LABEL_B="estimate_b"
OUT_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --reference-tum)
      [[ $# -ge 2 ]] || usage
      REF_TUM=$(realpath -m "$2")
      shift 2
      ;;
    --reference-meta)
      [[ $# -ge 2 ]] || usage
      REF_META=$(realpath -m "$2")
      shift 2
      ;;
    --estimate-a-tum)
      [[ $# -ge 2 ]] || usage
      EST_A=$(realpath -m "$2")
      shift 2
      ;;
    --estimate-b-tum)
      [[ $# -ge 2 ]] || usage
      EST_B=$(realpath -m "$2")
      shift 2
      ;;
    --label-a)
      [[ $# -ge 2 ]] || usage
      LABEL_A="$2"
      shift 2
      ;;
    --label-b)
      [[ $# -ge 2 ]] || usage
      LABEL_B="$2"
      shift 2
      ;;
    --out-dir)
      [[ $# -ge 2 ]] || usage
      OUT_DIR=$(realpath -m "$2")
      shift 2
      ;;
    -h|--help)
      usage
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      ;;
  esac
done

[[ -n "$REF_TUM" && -f "$REF_TUM" ]] || usage
[[ -n "$REF_META" && -f "$REF_META" ]] || usage
[[ -n "$EST_A" && -f "$EST_A" ]] || usage
[[ -n "$EST_B" && -f "$EST_B" ]] || usage
[[ -n "$OUT_DIR" ]] || OUT_DIR="${SCRIPT_DIR}/../output/kitti_compare_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT_DIR"

sanitize() {
  echo "$1" | tr '/ ' '__'
}

SAFE_A=$(sanitize "$LABEL_A")
SAFE_B=$(sanitize "$LABEL_B")

readarray -t PRISM_OFFSET < <(python3 - "$REF_META" <<'PY'
import json
import sys
from pathlib import Path

meta = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
offset = meta.get('lidar_to_prism_translation_m') or {}
print(offset.get('x', 0.0))
print(offset.get('y', 0.0))
print(offset.get('z', 0.0))
PY
)

A_PRISM="${OUT_DIR}/${SAFE_A}_prism.tum"
B_PRISM="${OUT_DIR}/${SAFE_B}_prism.tum"
APE_A="${OUT_DIR}/ape_${SAFE_A}.txt"
APE_B="${OUT_DIR}/ape_${SAFE_B}.txt"

python3 "${SCRIPT_DIR}/apply_tum_frame_offset.py" \
  --in "$EST_A" \
  --out "$A_PRISM" \
  --tx "${PRISM_OFFSET[0]}" \
  --ty "${PRISM_OFFSET[1]}" \
  --tz "${PRISM_OFFSET[2]}"

python3 "${SCRIPT_DIR}/apply_tum_frame_offset.py" \
  --in "$EST_B" \
  --out "$B_PRISM" \
  --tx "${PRISM_OFFSET[0]}" \
  --ty "${PRISM_OFFSET[1]}" \
  --tz "${PRISM_OFFSET[2]}"

python3 "${SCRIPT_DIR}/ape_from_tum.py" --ref "$REF_TUM" --est "$A_PRISM" --out "$APE_A"
python3 "${SCRIPT_DIR}/ape_from_tum.py" --ref "$REF_TUM" --est "$B_PRISM" --out "$APE_B"

rmse_a=$(awk -F': ' '/^rmse:/{print $2; exit}' "$APE_A")
rmse_b=$(awk -F': ' '/^rmse:/{print $2; exit}' "$APE_B")
mean_a=$(awk -F': ' '/^mean:/{print $2; exit}' "$APE_A")
mean_b=$(awk -F': ' '/^mean:/{print $2; exit}' "$APE_B")
pairs_a=$(awk -F': ' '/^pairs:/{print $2; exit}' "$APE_A")
pairs_b=$(awk -F': ' '/^pairs:/{print $2; exit}' "$APE_B")

TABLE="${OUT_DIR}/comparison_table.txt"
{
  echo "KITTI Odometry APE vs $(basename "$REF_TUM")"
  echo "reference_meta: $REF_META"
  echo ""
  printf "%-24s %12s %12s %10s\n" "label" "rmse_m" "mean_m" "pairs"
  printf "%-24s %12s %12s %10s\n" "$LABEL_A" "$rmse_a" "$mean_a" "$pairs_a"
  printf "%-24s %12s %12s %10s\n" "$LABEL_B" "$rmse_b" "$mean_b" "$pairs_b"
} | tee "$TABLE"

echo ""
echo "Artifacts: $OUT_DIR"
