#!/usr/bin/env bash
# Reproduce the koide LiDAR+camera 3DGS first-light end-to-end:
#   rosbag2 -> lidarslam trajectory -> posed images -> gsplat .ply.
#
# Requires: built workspace (install/), a CUDA GPU with torch + gsplat, and the
# local demo_data/koide_lidar_camera_calib bag. See
# docs/research/3dgs-koide-first-light.md and
# docs/research/3dgs-postprocess-map-design.md.
#
# NOTE: the camera extrinsic used here is an *approximate* frame-convention
# transform (no calibrated lever arm); replace it with a
# direct_visual_lidar_calibration result for quality work.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

BAG="${BAG:-demo_data/koide_lidar_camera_calib/livox/rosbag2_2023_03_09-13_42_46}"
OUT_DIR="${OUT_DIR:-output/koide_3dgs_firstlight}"
EXTRINSIC="${EXTRINSIC:-configs/gaussian_splatting/koide_lidar_camera_extrinsic_approx.yaml}"
ITERS="${ITERS:-1500}"
NUM_INIT="${NUM_INIT:-60000}"
TUM="${OUT_DIR}/lidarslam/traj_map_livox_frame.tum"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

echo "== [1/3] lidarslam frontend -> trajectory =="
rm -rf "${OUT_DIR}/lidarslam"
bash scripts/compare_with_glim.sh \
  --bag "${BAG}" --skip-glim \
  --points-topic /livox/points --imu-topic /livox/imu --no-imu \
  --no-graph-based-slam --param lidarslam/param/lidarslam_mid360_noimu.yaml \
  --robot-frame-id livox_frame --base-frame livox_frame --lidar-frame livox_frame \
  --out-dir "${OUT_DIR}"
test -s "${TUM}" || { echo "ERROR: empty trajectory ${TUM}"; exit 1; }

echo "== [2/3] extract posed images (auto clock alignment) =="
python3 tools/gaussian_splatting/extract_posed_images.py \
  --bag "${BAG}" --traj "${TUM}" \
  --camera-topic /image --camera-info-topic /camera_info \
  --extrinsic "${EXTRINSIC}" \
  --time-offset auto --clock-reference-topic /livox/points \
  --max-extrapolation 0.4 \
  --out "${OUT_DIR}/gsplat"

echo "== [3/3] train gsplat -> .ply =="
python3 tools/gaussian_splatting/train_gsplat.py \
  --transforms "${OUT_DIR}/gsplat/transforms.json" \
  --out "${OUT_DIR}/gsplat/point_cloud.ply" \
  --num-init "${NUM_INIT}" --iters "${ITERS}"

echo "done: ${OUT_DIR}/gsplat/point_cloud.ply"
