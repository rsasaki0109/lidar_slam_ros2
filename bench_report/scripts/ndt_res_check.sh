#!/usr/bin/env bash
# Loop registration NDT resolution 5.0 m (default) vs 2.0 m on exp04 and MID-360: the same pipeline as Oxford
# (lidarslam_competitive_v2_loop_ndt1.yaml, frontend with the 0.02 rad/s moving-start guard), one run at a time.
set -uo pipefail
P=/home/user/bench/params; O=/home/user/bench/ndtres; WS=/home/user/ws/pipe_dev_a92dfd4
M=/home/user/data/mid360_public/driving_slam_mid360/extracted/rosbag2_2024_04_16-14_17_01/rosbag2_2024_04_16-14_17_01
mkdir -p ${O}
declare -A Y=([5]=${P}/lidarslam_competitive_v2_loop_ndt1.yaml [2]=${P}/lidarslam_competitive_v2_loop_ndt1_res2.yaml)
dd if=/home/user/data/hilti2022/exp04_ros2/exp04_ros2.db3 of=/dev/null bs=16M 2>&1 | tail -1
for res in 5 2; do
  bash /home/user/scripts/kill_ros_procs.sh > /dev/null 2>&1
  RKO_PARAMS=${P}/rko_params_gyro002.ros.yaml /usr/bin/time -v -o ${O}/exp04_res${res}.time \
    bash /home/user/scripts/pipe_pipeline.sh ${O}/rows.csv exp04_res${res} ${WS} ${Y[$res]} ${O}/exp04_res${res} | tail -1
  grep -E "Maximum resident" ${O}/exp04_res${res}.time
done
dd if=${M}/rosbag2_2024_04_16-14_17_01_0.db3 of=/dev/null bs=16M 2>&1 | tail -1
for res in 5 2; do
  bash /home/user/scripts/kill_ros_procs.sh > /dev/null 2>&1
  BAG=${M} LIDAR=/livox/lidar IMU=/livox/imu RKO_PARAMS=${P}/rko_params_mid360_gyro002.ros.yaml \
    /usr/bin/time -v -o ${O}/mid360_res${res}.time \
    bash /home/user/scripts/pipe_pipeline.sh ${O}/rows.csv mid360_res${res} ${WS} ${Y[$res]} ${O}/mid360_res${res} | tail -1
  grep -E "Maximum resident" ${O}/mid360_res${res}.time
done
echo "ndt res check done $(date -u +%T)"
