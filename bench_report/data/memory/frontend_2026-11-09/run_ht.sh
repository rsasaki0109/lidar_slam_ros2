#!/usr/bin/env bash
# heaptrack + smaps of G12's offline_node on HOEB-1 (memory investigation, outputs compared by md5)
set -uo pipefail
set +u; source /opt/ros/jazzy/setup.bash; source /home/user/lidar_slam_ros2/install/setup.bash; set -u
O=/home/user/bench/mem_g/ht/hoeb-1; rm -rf "$O"; mkdir -p "$O"
export ROS_LOG_DIR=$O/roslog ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 /home/user/lidar_slam_ros2/bench_report/scripts/smaps_sampler.py 'offline_node.*hoeb-1' "$O/smaps.csv" 1.0 &
SP=$!
/usr/bin/time -v -o "$O/time.txt" heaptrack -o "$O/heap" /home/user/lidar_slam_ros2/install/rko_lio/lib/rko_lio/offline_node --ros-args \
  --params-file /home/user/bench/grandtour/mem_G12/hoeb-1/params.yaml -p bag_path:=/dev/shm/gt_mem/hoeb-1/bag_stim320_imu_hesai_points \
  -p lidar_topic:=/hesai_points -p imu_topic:=/stim320_imu -p results_dir:="$O" > "$O/stdout.log" 2>&1
wait $SP
md5sum "$O/front_0/front_tum_0.txt"
grep "Maximum resident" "$O/time.txt"
