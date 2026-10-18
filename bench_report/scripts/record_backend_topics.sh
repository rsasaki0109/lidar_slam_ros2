#!/usr/bin/env bash
# Usage: record_backend_topics.sh <ws> <lidarslam_yaml> <out_dir>
# Runs the exp04 pipeline launch once and records the two backend input topics (/rko_lio/frame,
# /rko_lio/odometry) with ros2 bag (sqlite3) into <out_dir>/bag. Used only for offline analysis.
w=$1; yaml=$2; out=$3
rm -rf "${out}"; mkdir -p "${out}"
set +u; source "${w}/install/setup.bash"; set -u
export ROS_DOMAIN_ID=36 ROS_LOG_DIR="${out}/roslog" M6A10_BENCHMARK_NO_MAP_ARTIFACTS=1
cp /home/user/bench/params/rko_params.ros.yaml "${out}/rko_params.ros.yaml"
setsid ros2 bag record -s sqlite3 -o "${out}/bag" /rko_lio/frame /rko_lio/odometry > "${out}/record.log" 2>&1 &
rpid=$!
sleep 3
cd /home/user/rsasaki0109/lidar_slam_ros2
setsid ros2 launch lidarslam rko_lio_slam.launch.py "main_param_dir:=${yaml}" "rko_param_file:=${out}/rko_params.ros.yaml" \
  bag_path:=/home/user/data/hilti2022/exp04_ros2 lidar_topic:=/hesai/pandar imu_topic:=/alphasense/imu \
  base_frame:=base_link publish_static_tf:=true "save_dir:=${out}" "results_dir:=${out}" run_name:=front \
  dump_results:=true use_rviz:=false > "${out}/slam.launch.log" 2>&1 &
lpid=$!
for ((s = 1; s <= 900; s++)); do
  sleep 1
  grep -q "RKO LIO Offline Node took" "${out}/slam.launch.log" && break
done
sleep 10
kill -INT -- -"${lpid}" 2>/dev/null; kill -INT -- -"${rpid}" 2>/dev/null; sleep 8
kill -KILL -- -"${lpid}" 2>/dev/null; kill -KILL -- -"${rpid}" 2>/dev/null
ros2 bag info "${out}/bag" 2>/dev/null | grep -E "Count|Topic" | head -5
md5sum "${out}/front_0/front_tum_0.txt"
