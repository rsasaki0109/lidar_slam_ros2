#!/usr/bin/env bash
# One-command SLAM demo: download a public Livox MID-360 driving bag
# (Zenodo 14841855, Koide, CC-BY 4.0, 517 MB zip) and run the flagship
# RKO-LIO + graph_based_slam pipeline headless, saving an Autoware-ready
# map (map.pcd, pointcloud_map/ tiles, map_projector_info.yaml) and the
# corrected trajectory (traj_corrected.tum).
#
# Designed as the default command of the ghcr.io/rsasaki0109/lidar_slam_ros2
# image, but works on any host with the workspace built and sourced:
#
#   bash scripts/run_docker_demo.sh
#
# Environment overrides:
#   DEMO_DATA_DIR    dataset cache directory (default: <repo>/datasets/mid360_public)
#   DEMO_OUTPUT_DIR  output directory        (default: <repo>/output/mid360_demo)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_DIR="${DEMO_DATA_DIR:-${REPO_ROOT}/datasets/mid360_public}"
OUT_DIR="${DEMO_OUTPUT_DIR:-${REPO_ROOT}/output/mid360_demo}"
BAG_NAME="rosbag2_2024_04_16-14_17_01"

echo "== [1/2] demo data: Driving SLAM Test with Livox MID360 =="
echo "   (Koide, Zenodo DOI 10.5281/zenodo.14841855, CC-BY 4.0)"
bag_dir="$(find "${DATA_DIR}" -type d -name "${BAG_NAME}" 2>/dev/null | head -n 1 || true)"
if [[ -z "${bag_dir}" ]]; then
  python3 "${REPO_ROOT}/scripts/download_mid360_robot_public_dataset.py" \
    --dataset driving_slam_mid360 --dataset-root "${DATA_DIR}"
  bag_dir="$(find "${DATA_DIR}" -type d -name "${BAG_NAME}" | head -n 1)"
fi
[[ -n "${bag_dir}" && -f "${bag_dir}/metadata.yaml" ]] || {
  echo "error: demo bag not found under ${DATA_DIR}" >&2
  exit 1
}
echo "   bag: ${bag_dir}"

echo "== [2/2] RKO-LIO + graph_based_slam (headless, offline) =="
bash "${REPO_ROOT}/scripts/run_rko_lio_graph_autoware_dogfood.sh" \
  --bag "${bag_dir}" \
  --lidar-topic /livox/lidar --imu-topic /livox/imu \
  --base-frame livox_frame --lidar-frame livox_frame --imu-frame livox_frame \
  --rko-param "${REPO_ROOT}/lidarslam/param/rko_lio_mid360.yaml" \
  --lidarslam-param "${REPO_ROOT}/lidarslam/param/lidarslam_mid360_rko_graph.yaml" \
  --wait-for-offline-completion --skip-viewer \
  --output-dir "${OUT_DIR}" \
  --run-name mid360_demo

echo
echo "== demo finished =="
echo "outputs under ${OUT_DIR}:"
echo "  map.pcd                  downsampled point-cloud map"
echo "  pointcloud_map/          Autoware map tiles (+ metadata)"
echo "  map_projector_info.yaml  Autoware projector info (local)"
echo "  traj_corrected.tum       loop-closed trajectory (TUM format)"
