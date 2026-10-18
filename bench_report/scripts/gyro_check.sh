#!/usr/bin/env bash
# Moving-start guard at 0.02 rad/s (the GEODE value): exp04 and MID-360 must keep their a92dfd4 md5,
# Oxford must match the 0.05 rad/s run. Warms each bag into the page cache first (cold random reads are slow).
set -uo pipefail
P=/home/user/bench/params; O=/home/user/bench/oxford; WS=/home/user/ws/cand_a92dfd4
M=/home/user/data/mid360_public/driving_slam_mid360/extracted/rosbag2_2024_04_16-14_17_01/rosbag2_2024_04_16-14_17_01
while pgrep -f "dd if=/home/user/data/hilti2022" > /dev/null; do sleep 5; done
echo "exp04 warm done $(date -u +%T)"
dd if=${M}/rosbag2_2024_04_16-14_17_01_0.db3 of=/dev/null bs=16M 2>&1 | tail -1
export ROS_DOMAIN_ID=27 RUN_TIMEOUT=3000
bash /home/user/scripts/run_front.sh exp04_gyro002 ${WS} ${P}/exp04_gyro002.yaml /home/user/data/hilti2022/exp04_ros2 \
  /hesai/pandar /alphasense/imu ${O}/run_exp04_gyro002 ${O}/runs.csv | tail -1
grep -E "Moving start|Estimated accel bias" ${O}/run_exp04_gyro002/stdout.log
bash /home/user/scripts/run_front.sh mid360_gyro002 ${WS} ${P}/mid360_gyro002.yaml ${M} /livox/lidar /livox/imu \
  ${O}/run_mid360_gyro002 ${O}/runs.csv | tail -1
grep -E "Moving start|Estimated accel bias" ${O}/run_mid360_gyro002/stdout.log
dd if=/home/user/data/oxford_keble05/ros2/ros2_0.db3 of=/dev/null bs=16M 2>&1 | tail -1
bash /home/user/scripts/run_front.sh oxford_gyro002 ${WS} ${P}/oxford_keble05_gyro002.yaml /home/user/data/oxford_keble05/ros2 \
  /hesai/pandar /alphasense_driver_ros/imu ${O}/run_oxford_gyro002 ${O}/runs.csv | tail -1
grep -E "Moving start|Estimated accel bias" ${O}/run_oxford_gyro002/stdout.log
echo "all done $(date -u +%T)"
