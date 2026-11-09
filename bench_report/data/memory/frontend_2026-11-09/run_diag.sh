#!/usr/bin/env bash
# G12 on HOEB-1 with the local map diagnostic (rko_mem build); md5 and peak RSS
set -uo pipefail
WS=${WS:-/home/user/ws/rko_mem}; NAME=${NAME:-diag}; shift 0
set +u; source /opt/ros/jazzy/setup.bash; set -u
O=/home/user/bench/mem_g/${NAME}; rm -rf "$O"; mkdir -p "$O"
export ROS_LOG_DIR=$O/roslog ROS_DOMAIN_ID=78 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
env ${DIAG:+RKO_LIO_MAP_DIAG=$O/map.txt} /usr/bin/time -v -o "$O/time.txt" "$WS/install/rko_lio/lib/rko_lio/offline_node" --ros-args \
  --params-file /home/user/bench/grandtour/mem_G12/hoeb-1/params.yaml -p bag_path:=/dev/shm/gt_mem/hoeb-1/bag_stim320_imu_hesai_points \
  -p lidar_topic:=/hesai_points -p imu_topic:=/stim320_imu -p results_dir:="$O" "$@" > "$O/stdout.log" 2>&1
echo "$NAME $(md5sum < "$O/front_0/front_tum_0.txt" | cut -c1-8) rss_kb $(grep -oP 'Maximum resident set size \(kbytes\): \K[0-9]+' "$O/time.txt") took $(grep -oP 'took \K[0-9.]+' "$O/stdout.log" | tail -1)"
