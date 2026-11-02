#!/usr/bin/env bash
# p2_kitti_raw.sh <label> (Phase 2, a long vehicle development sequence; plan in data/kitti/raw05_2026-10-31/plan.txt):
# KITTI Odometry 05 from its raw drive 2011_09_30_drive_0018 (2761 scans, 2.2 km) with the OXTS unit's 100 Hz IMU.
#   1. the bag (bench_report/scripts/kitti_raw_to_ros2.py; since 2026-11-01 with the scans' rotation deskewed from the
#      OXTS gyro and their elevation offset corrected: data/kitti/front_diag_2026-11-01) into
#      /home/user/data/kitti_raw_05/ros2, if not there;
#   2. the frontend with the Phase 1 global configuration G12 (WS, default ws/rko_g35) on the KITTI rig file
#      (data/kitti/raw05_2026-10-31/rig_kitti_raw.yaml: topics, frames, the IMU-Velodyne extrinsic, and deskew off
#      because KITTI's scans are already motion-compensated), its odometry and scans (thinned to 0.1 m) recorded as
#      the backend input;
#   3. graph_slam_offline_runner (RUNNER, default ws/slam_hi's build) with the walking preset of Phase 3 and each
#      variant (ndt2: the preset as it is; sg05: SMALL_GICP loop registration at 0.5 m; VARIANTS to choose);
#   4. each trajectory scored by p2_kitti_score.py against dataset/poses/05.txt in the Velodyne frame.
# Appends label,variant,matched,missing,ape_rmse,ape_max,t_rel_percent,r_rel_deg_per_m,loop_edges,wall_s,front_md5
# to /home/user/bench/kitti_raw/scores.csv. Development data only (reads the ground truth).
set -uo pipefail
label=$1
REPO=/home/user/lidar_slam_ros2; PY=/home/user/mapshd/venv/bin/python
WS=${WS:-/home/user/ws/rko_g35}; RUNNER=${RUNNER:-/home/user/ws/slam_hi/build/graph_slam_offline_runner}
PRESET=${REPO}/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml
VARIANTS=${VARIANTS:-ndt2 sg05}
D=/home/user/data/kitti_raw_05; BAG=${D}/ros2; GT=${D}/gt
RIG=${REPO}/bench_report/data/kitti/raw05_2026-10-31/rig_kitti_raw.yaml
CSV=/home/user/bench/kitti_raw/scores.csv
G12=(close_scene_max_median_range:=12.0 close_scene_max_correspondence_distance:=0.25
     close_scene_localizability_boost:=10.0 close_scene_localizability_normal_only:=true
     close_scene_min_inlier_fraction:=0.6 min_icp_keypoints:=100 keypoint_shortfall_voxel_multiplier:=1.0)
set +u; source /opt/ros/jazzy/setup.bash; set -u
if [[ ! -e ${BAG}/metadata.yaml ]]; then
  mkdir -p "${D}"
  # Rotation deskew from the OXTS gyro and the elevation offset estimated from the scans
  # (data/kitti/front_diag_2026-11-01/plan_rotation_deskew.txt, plan_elevation.txt).
  ROTATION_DESKEW=1 ELEVATION_CORRECTION_DEG=0.1206 \
    /usr/bin/python3 "${REPO}/bench_report/scripts/kitti_raw_to_ros2.py" 2011_09_30 0018 "${BAG}" "${WORKERS:-8}" \
    > "${D}/convert.log" 2>&1 || { echo "${label},convert failed"; exit 1; }
fi
O=/home/user/bench/kitti_raw/${label}
rm -rf "${O}"; mkdir -p "${O}" "$(dirname "${CSV}")"
set +u; source "${WS}/install/setup.bash"; set -u
export ROS_DOMAIN_ID=$((40 + RANDOM % 50)) ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR="${O}/roslog"
fargs=(); for kv in "${G12[@]}"; do fargs+=(-p "${kv}"); done
t0=$(date +%s.%N)
BACKEND_CAPTURE_STORAGE_PRESET=zstd_fast bash "${REPO}/scripts/record_backend_input.sh" --output-dir "${O}/backend_input" \
  --setup "${WS}/install/setup.bash" -- \
  "${WS}/install/rko_lio/lib/rko_lio/offline_node" --ros-args --params-file "${RIG}" \
  -p bag_path:="${BAG}" -p lidar_topic:=/kitti/velodyne/points -p imu_topic:=/kitti/oxts/imu -p results_dir:="${O}/front" \
  -p publish_deskewed_scan:=true -p output_subscriber_timeout_sec:=60.0 -p deskewed_scan_voxel_size:=0.1 "${fargs[@]}" \
  > "${O}/record.log" 2>&1 || { echo "${label},record failed"; exit 1; }
t1=$(date +%s.%N)
FRONT=${O}/front/front_0/front_tum_0.txt
md5=$(md5sum < "${FRONT}" | cut -c1-8)
EXT=$("${PY}" -c "import sys, yaml; p = yaml.safe_load(open(sys.argv[1])); p = next(iter(p.values()))['ros__parameters']
print(','.join(str(v) for v in p['extrinsic_lidar2base_quat_xyzw_xyz']))" "${RIG}")
score() {  # score <trajectory (IMU frame, TUM)> <out json>
  "${PY}" "${REPO}/bench_report/scripts/p2_kitti_score.py" "$1" "${GT}/05.txt" "${GT}/calib_05.txt" "${BAG}.frames.txt" \
    "${EXT}" "$2"
}
echo "${label},front,$(score "${FRONT}" "${O}/front_score.json"),0,$(python3 -c "print(f'{${t1}-${t0}:.1f}')"),${md5}" \
  | tee -a "${CSV}"
for v in ${VARIANTS}; do
  case ${v} in
    ndt2) over=() ;;
    sg05) over=(registration_method:=SMALL_GICP gicp_max_correspondence_distance:=0.5 gicp_transformation_epsilon:=1e-4) ;;
    *) echo "unknown variant ${v}"; continue ;;
  esac
  bargs=(); for kv in "${over[@]}"; do bargs+=(-p "${kv}"); done
  B=${O}/${v}
  t0=$(date +%s.%N)
  "${RUNNER}" --ros-args --params-file "${PRESET}" -p bag_path:="${O}/backend_input" -p output_dir:="${B}" \
    -p "extrinsic_lidar2base_quat_xyzw_xyz:=[${EXT}]" "${bargs[@]}" > "${O}/${v}.log" 2>&1
  t1=$(date +%s.%N)
  loops=$(grep -h "Offline run complete" "${O}/${v}.log" | grep -oP '[0-9]+(?= loop edges)')
  PYTHONPATH=${REPO} python3 "${REPO}/scripts/densify_corrected_trajectory.py" --raw "${FRONT}" \
    --corrected "${B}/trajectory_optimized.tum" --output "${B}/dense.tum" > "${B}/densify.log" 2>&1
  echo "${label},${v},$(score "${B}/dense.tum" "${B}/score.json"),${loops:-NA},$(python3 -c "print(f'{${t1}-${t0}:.1f}')"),${md5}" \
    | tee -a "${CSV}"
done
[[ ${KEEP_BACKEND_INPUT:-0} == 1 ]] || rm -rf "${O}/backend_input"
