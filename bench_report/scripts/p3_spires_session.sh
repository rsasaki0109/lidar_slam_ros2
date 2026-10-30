#!/usr/bin/env bash
# p3_spires_session.sh <sequence> <short> (multi-session GIF, report chapter 38): an Oxford Spires sequence (e.g.
# 2024-03-18-christ-church-02) as one session of a join, as p3_dev_backend.sh makes Keble 05's:
#   1. its LiDAR and IMU into /home/user/data/oxford_<short>/ros2 (oxford_to_ros2.py with WORKERS fetches at a time; no
#      ground truth is fetched),
#   2. the frontend (Phase 1 configuration G12 on the Oxford rig file) publishing its odometry and its deskewed scans
#      thinned to 0.1 m, recorded as the backend input (scripts/record_backend_input.sh),
#   3. graph_slam_offline_runner with the walking preset.
# Outputs: /home/user/bench/spires_ms/<short>/{backend_input,front,run}. DROP_BAG=1 deletes the converted bag afterwards
# (it can be fetched again); CONVERT_ONLY=1 stops after step 1. The sequences share one rig and calibration, so the
# rig file made for Keble 05 holds.
set -euo pipefail
seq=$1; short=$2
REPO=/home/user/lidar_slam_ros2; PY=/home/user/mapshd/venv/bin/python
WS=${WS:-/home/user/ws/rko_g35}; RUNNER=${RUNNER:-/home/user/ws/slam_hi/build/graph_slam_offline_runner}
PRESET=${REPO}/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml
params=/home/user/bench/params/oxford_keble05_gyro002_jerk1000.yaml
D=/home/user/data/oxford_${short}; bag=${D}/ros2
O=/home/user/bench/spires_ms/${short}
G12=(close_scene_max_median_range:=12.0 close_scene_max_correspondence_distance:=0.25
     close_scene_localizability_boost:=10.0 close_scene_localizability_normal_only:=true
     close_scene_min_inlier_fraction:=0.6 min_icp_keypoints:=100 keypoint_shortfall_voxel_multiplier:=1.0)
set +u; source /opt/ros/jazzy/setup.bash; set -u
if [[ ! -e ${bag}/metadata.yaml ]]; then
  U=https://huggingface.co/datasets/ori-drs/oxford_spires_dataset/resolve/main/sequences/${seq}
  mkdir -p "${D}/raw"
  curl -sS -L --fail -o "${D}/raw/imu.csv" "${U}/raw/imu.csv"
  BAG_STORAGE=mcap "${PY}" "${REPO}/bench_report/scripts/oxford_to_ros2.py" "${U}/raw/lidar-clouds.zip" "${D}/raw/imu.csv" \
    "${bag}" "${WORKERS:-8}" > "${D}/convert.log" 2>&1
fi
[[ ${CONVERT_ONLY:-0} == 1 ]] && exit 0
rm -rf "${O}"; mkdir -p "${O}"
set +u; source "${WS}/install/setup.bash"; set -u
export ROS_DOMAIN_ID=$((40 + RANDOM % 50)) ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR="${O}/roslog"
fargs=(); for kv in "${G12[@]}"; do fargs+=(-p "${kv}"); done
BACKEND_CAPTURE_STORAGE_PRESET=zstd_fast bash "${REPO}/scripts/record_backend_input.sh" --output-dir "${O}/backend_input" \
  --setup "${WS}/install/setup.bash" -- \
  "${WS}/install/rko_lio/lib/rko_lio/offline_node" --ros-args --params-file "${params}" \
  -p bag_path:="${bag}" -p lidar_topic:=/hesai/pandar -p imu_topic:=/alphasense_driver_ros/imu -p results_dir:="${O}/front" \
  -p publish_deskewed_scan:=true -p output_subscriber_timeout_sec:=60.0 -p deskewed_scan_voxel_size:=0.1 "${fargs[@]}" \
  > "${O}/record.log" 2>&1
EXT=$("${PY}" -c "import sys, yaml; p = yaml.safe_load(open(sys.argv[1])); p = next(iter(p.values()))['ros__parameters']
print('[' + ','.join(str(v) for v in p['extrinsic_lidar2base_quat_xyzw_xyz']) + ']')" "${params}")
echo "${EXT}" > "${O}/extrinsic.txt"
"${RUNNER}" --ros-args --params-file "${PRESET}" -p bag_path:="${O}/backend_input" -p output_dir:="${O}/run" \
  -p extrinsic_lidar2base_quat_xyzw_xyz:="${EXT}" > "${O}/run.log" 2>&1
grep -h "Offline run complete" "${O}/run.log" || true
[[ ${DROP_BAG:-0} == 1 ]] && rm -rf "${bag}"
rm -rf "${O}/roslog"
