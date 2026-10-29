#!/usr/bin/env bash
# p3_dev_backend.sh <sequence> <label> (Phase 3, robust loops on the other development data; plan in
# data/loops_robust_dev_2026-10-29/plan.txt): one sequence (a MapsHD trial name, oxford or exp04) through
#   1. the frontend with the Phase 1 global configuration G12 (WS, default ws/rko_g35), publishing its odometry and
#      deskewed scans (thinned to 0.1 m), recorded as the backend input (scripts/record_backend_input.sh);
#   2. graph_slam_offline_runner (RUNNER, default ws/slam_hi's build) with the walking preset and each variant
#      (ndt2, sg05, sg05_dcs; VARIANTS to choose);
#   3. the optimized submap poses densified onto the frontend's trajectory and scored like the frontend in Phase 1.
# Appends label,sequence,variant,ape_rmse,loop_edges,backend_wall_s,front_md5 to /home/user/bench/dev_loops/scores.csv
# (variant "front" for the frontend alone) and deletes the backend input at the end (KEEP_BACKEND_INPUT=1 keeps it).
# Development data only (reads the ground truth).
set -uo pipefail
seq=$1; label=$2
REPO=/home/user/lidar_slam_ros2; PY=/home/user/mapshd/venv/bin/python
WS=${WS:-/home/user/ws/rko_g35}; RUNNER=${RUNNER:-/home/user/ws/slam_hi/build/graph_slam_offline_runner}
PRESET=${REPO}/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml
VARIANTS=${VARIANTS:-ndt2 sg05 sg05_dcs}
CSV=/home/user/bench/dev_loops/scores.csv
G12=(close_scene_max_median_range:=12.0 close_scene_max_correspondence_distance:=0.25
     close_scene_localizability_boost:=10.0 close_scene_localizability_normal_only:=true
     close_scene_min_inlier_fraction:=0.6 min_icp_keypoints:=100 keypoint_shortfall_voxel_multiplier:=1.0)
case ${seq} in
  oxford) params=/home/user/bench/params/oxford_keble05_gyro002_jerk1000.yaml; bag=/home/user/data/oxford_keble05/ros2
          lidar=/hesai/pandar; imu=/alphasense_driver_ros/imu ;;
  exp04) params=/home/user/bench/params/exp04_gyro002_jerk1000.yaml; bag=/home/user/data/hilti2022/exp04_ros2
         lidar=/hesai/pandar; imu=/alphasense/imu ;;
  *) params=/home/user/bench/params/mapshd_gyro002_jerk1000.yaml; bag=/home/user/data/mapshd/bags/${seq}_c
     lidar=/livox/pointcloud; imu=/livox/imu ;;
esac
O=/home/user/bench/dev_loops/${label}/${seq}
rm -rf "${O}"; mkdir -p "${O}"
set +u; source /opt/ros/jazzy/setup.bash; source "${WS}/install/setup.bash"; set -u
export ROS_DOMAIN_ID=$((40 + RANDOM % 50)) ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR="${O}/roslog"
fargs=(); for kv in "${G12[@]}"; do fargs+=(-p "${kv}"); done
BACKEND_CAPTURE_STORAGE_PRESET=zstd_fast bash "${REPO}/scripts/record_backend_input.sh" --output-dir "${O}/backend_input" \
  --setup "${WS}/install/setup.bash" -- \
  "${WS}/install/rko_lio/lib/rko_lio/offline_node" --ros-args --params-file "${params}" \
  -p bag_path:="${bag}" -p lidar_topic:="${lidar}" -p imu_topic:="${imu}" -p results_dir:="${O}/front" \
  -p publish_deskewed_scan:=true -p output_subscriber_timeout_sec:=60.0 -p deskewed_scan_voxel_size:=0.1 "${fargs[@]}" \
  > "${O}/record.log" 2>&1 || { echo "${label},${seq},record failed"; exit 1; }
FRONT=${O}/front/front_0/front_tum_0.txt
md5=$(md5sum < "${FRONT}" | cut -c1-8)
EXT=$("${PY}" -c "import sys, yaml; p = yaml.safe_load(open(sys.argv[1])); p = next(iter(p.values()))['ros__parameters']
print('[' + ','.join(str(v) for v in p['extrinsic_lidar2base_quat_xyzw_xyz']) + ']')" "${params}")

score() {  # score <trajectory (IMU frame, TUM)> <out dir>; prints the APE RMSE
  local tum=$1 out=$2
  mkdir -p "${out}"
  case ${seq} in
    oxford) "${PY}" /home/user/scripts/ape_oxford.py "${tum}" /home/user/data/oxford_keble05/gt_sealed/gt-tum.txt \
              /home/user/data/oxford_keble05/calib/T_imu_base.npy "${out}/ape" > "${out}/ape.log" 2>&1
            grep -oiP 'rmse[^0-9]*\K[0-9.]+' "${out}/ape.log" | head -1 ;;
    exp04) python3 "${REPO}/scripts/ape_from_tum.py" --ref /home/user/data/hilti2022/exp04_construction_upper_level_gt.txt \
             --est "${tum}" --out "${out}/ape.txt" --interpolate > "${out}/ape.log" 2>&1
           awk '/^rmse:/ {print $2}' "${out}/ape.txt" ;;
    *) "${PY}" /home/user/scripts/front_score.py "${seq}" "${tum}" /home/user/data/mapshd/gt/${seq}/ground_truth.tum \
         2> "${out}/ape.log" | cut -d, -f2 ;;
  esac
}

echo "${label},${seq},front,$(score "${FRONT}" "${O}/front_score"),0,0,${md5}" | tee -a "${CSV}"
for v in ${VARIANTS}; do
  case ${v} in
    ndt2) over=() ;;
    sg05) over=(registration_method:=SMALL_GICP gicp_max_correspondence_distance:=0.5 gicp_transformation_epsilon:=1e-4) ;;
    sg05_dcs) over=(registration_method:=SMALL_GICP gicp_max_correspondence_distance:=0.5 gicp_transformation_epsilon:=1e-4
                    loop_edge_robust_kernel_type:=dcs loop_edge_robust_kernel_delta:=1.0) ;;
    *) echo "unknown variant ${v}"; continue ;;
  esac
  bargs=(); for kv in "${over[@]}"; do bargs+=(-p "${kv}"); done
  B=${O}/${v}
  t0=$(date +%s.%N)
  "${RUNNER}" --ros-args --params-file "${PRESET}" -p bag_path:="${O}/backend_input" -p output_dir:="${B}" \
    -p extrinsic_lidar2base_quat_xyzw_xyz:="${EXT}" "${bargs[@]}" > "${O}/${v}.log" 2>&1
  t1=$(date +%s.%N)
  wall=$(python3 -c "print(f'{${t1}-${t0}:.1f}')")
  loops=$(grep -h "Offline run complete" "${O}/${v}.log" | grep -oP '[0-9]+(?= loop edges)')
  PYTHONPATH=${REPO} python3 "${REPO}/scripts/densify_corrected_trajectory.py" --raw "${FRONT}" \
    --corrected "${B}/trajectory_optimized.tum" --output "${B}/dense.tum" > "${B}/densify.log" 2>&1
  echo "${label},${seq},${v},$(score "${B}/dense.tum" "${B}/score"),${loops:-NA},${wall},${md5}" | tee -a "${CSV}"
done
[[ ${KEEP_BACKEND_INPUT:-0} == 1 ]] || rm -rf "${O}/backend_input"
rm -rf "${O}/roslog"
