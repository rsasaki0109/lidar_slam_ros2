#!/usr/bin/env bash
# p3_speed_dev_queue.sh <scenario>... (plan: data/backend_speed_dev_2026-10-31/plan.txt): for each MapsHD scenario,
# fetch and convert it (p1_mapshd_get.sh), run every trial through p3_dev_backend.sh with VARIANTS="ndt2 t4v05", then
# delete the scenario's bags. "exp04" runs HILTI 2022 exp04 (its bag stays).
S=/home/user/lidar_slam_ros2/bench_report/scripts
for sc in "$@"; do
  if [[ ${sc} == exp04 ]]; then
    VARIANTS="ndt2 t4v05" bash "$S/p3_dev_backend.sh" exp04 spd
    continue
  fi
  bash "$S/p1_mapshd_get.sh" "${sc}" > "/home/user/bench/dev_loops/get_${sc}.log" 2>&1 || { echo "${sc}: fetch failed"; continue; }
  for b in /home/user/data/mapshd/bags/${sc}_trial-*_c; do
    t=$(basename "$b"); t=${t%_c}
    VARIANTS="ndt2 t4v05" bash "$S/p3_dev_backend.sh" "${t}" spd
  done
  rm -rf /home/user/data/mapshd/bags/${sc}_trial-*_c
  df -h /home/user | tail -1
done
echo QUEUE_DONE
