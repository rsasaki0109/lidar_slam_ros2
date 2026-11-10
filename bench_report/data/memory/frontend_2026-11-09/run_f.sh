#!/usr/bin/env bash
# HOEB-1: the rko_mem build with the blind run's G12 parameters, default and with local_map_compact_points;
# md5 of the frontend trajectory, peak RSS, ATE against the opened reference (blind_gt, development data now)
set -uo pipefail
S=/home/user/lidar_slam_ros2/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
B=/dev/shm/gt_mem/hoeb-1/bag_stim320_imu_hesai_points; R=/home/user/data/blind_gt/h5/hoeb-1/prism_position
for v in ${VARIANTS:-default compact}; do
  o=/home/user/bench/mem_g/f/${v}${SUFFIX:-}
  RKO_WS=/home/user/ws/rko_mem bash $S/p5_blind_method.sh G /home/user/bench/mem_g/f/rig_${v}.yaml $B $o > /dev/null 2>&1
  echo "$v md5 $(md5sum < $o/front_0/front_tum_0.txt | cut -c1-8) rss_kb $(grep -oP 'Maximum resident set size \(kbytes\): \K[0-9]+' $o/time.txt) took $(grep -oP 'took \K[0-9.]+' $o/stdout.log | tail -1) | $($PY -I $S/p1_grandtour_ate.py $o/traj.tum $R)"
done
