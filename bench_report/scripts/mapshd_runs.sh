#!/usr/bin/env bash
# MapsHD HUMANOID-LIO-DATASET: one bag, the pipeline (loop on, NDT 1 thread) with A = NDT 5 m and B = NDT 2 m,
# frontend with the preregistered profile (params/rko_params_mapshd_gyro002.ros.yaml), one run at a time.
# WS overrides the pipeline overlay (default: rko_lio a92dfd4).
# Usage: mapshd_runs.sh <bag_dir> <label> <out_root>
set -uo pipefail
bag=$1; label=$2; root=$3
P=/home/user/bench/params; WS=${WS:-/home/user/ws/pipe_dev_a92dfd4}
mkdir -p "${root}"
dd if="$(ls "${bag}"/*.db3)" of=/dev/null bs=16M 2>&1 | tail -1
declare -A Y=([A]=${P}/lidarslam_competitive_v2_loop_ndt1.yaml [B]=${P}/lidarslam_competitive_v2_loop_ndt1_res2.yaml)
for cfg in A B; do
  bash /home/user/scripts/kill_ros_procs.sh > /dev/null 2>&1
  BAG=${bag} LIDAR=/livox/pointcloud IMU=/livox/imu RKO_PARAMS=${P}/rko_params_mapshd_gyro002.ros.yaml \
    /usr/bin/time -v -o "${root}/${label}_${cfg}.time" \
    bash /home/user/scripts/pipe_pipeline.sh "${root}/rows.csv" "${label}_${cfg}" ${WS} "${Y[$cfg]}" "${root}/${label}_${cfg}" | tail -1
  grep -E "Maximum resident|Elapsed" "${root}/${label}_${cfg}.time"
  grep -E "Moving start|Estimated accel bias|initialized using" "${root}/${label}_${cfg}/slam.launch.log" | head -3
  rm -rf "${root}/${label}_${cfg}/pointcloud_map" "${root}/${label}_${cfg}/map.pcd.tmp"
done
echo "runs done ${label} $(date -u +%T)"
