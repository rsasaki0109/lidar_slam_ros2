#!/usr/bin/env bash
# Oxford Keble 05, same VM, one run at a time: GLIM CPU (hilti2022_cpu with Oxford's T_lidar_imu and IMU topic)
# and ours (lidarslam pipeline, loop on, frontend with the 0.02 rad/s moving-start guard), alternated.
# Usage: oxford_rounds.sh <rounds>
set -uo pipefail
O=/home/user/bench/oxford; P=/home/user/bench/params; BAG=/home/user/data/oxford_keble05/ros2
GT=/home/user/data/oxford_keble05/gt_sealed/gt-tum.txt; X=/home/user/data/oxford_keble05/calib/T_imu_base.npy
for ((r = 1; r <= $1; r++)); do
  bash /home/user/scripts/kill_ros_procs.sh > /dev/null 2>&1
  GLIM_OVERRIDES=${P}/glim_oxford_cpu bash /home/user/scripts/glim_run.sh ${O}/glim_r${r} ${BAG} | sed "s/^/glim_r${r} /"
  python3 /home/user/scripts/ape_oxford.py ${O}/glim_r${r}/dump/traj_imu.txt ${GT} ${X} ${O}/ape_glim_r${r} > ${O}/ape_glim_r${r}.log 2>&1
  grep -E "^rmse|per-axis" ${O}/ape_glim_r${r}.log | sed "s/^/glim_r${r} /"
  du -sh ${O}/glim_r${r}/dump | sed "s/^/glim_r${r} dump /"
  find ${O}/glim_r${r}/dump -mindepth 1 -maxdepth 1 -type d -exec rm -rf {} +
  bash /home/user/scripts/kill_ros_procs.sh > /dev/null 2>&1
  BAG=${BAG} LIDAR=/hesai/pandar IMU=/alphasense_driver_ros/imu RKO_PARAMS=${P}/rko_params_oxford_gyro002.ros.yaml \
    /usr/bin/time -v -o ${O}/pipe_loop_gyro002_r${r}.time bash /home/user/scripts/pipe_pipeline.sh ${O}/pipe_rows.csv \
    oxford_loop_gyro002_r${r} /home/user/ws/pipe_dev_a92dfd4 ${P}/lidarslam_competitive_v2_loop_ndt1.yaml \
    ${O}/pipe_loop_gyro002_r${r} | sed "s/^/ours_r${r} /"
  grep -E "Maximum resident|Elapsed" ${O}/pipe_loop_gyro002_r${r}.time | sed "s/^/ours_r${r} /"
  # Score the frontend and the corrected trajectory densified onto it (as chapter 13 and ape_runs.py do;
  # trajectory_optimized.tum alone has only the submap poses).
  d=${O}/pipe_loop_gyro002_r${r}
  PYTHONPATH=/home/user/lidar_slam_ros2 python3 /home/user/lidar_slam_ros2/scripts/densify_corrected_trajectory.py \
    --raw ${d}/front_0/front_tum_0.txt --corrected ${d}/trajectory_optimized.tum --output ${d}/traj_corrected_dense.tum
  for k in front:front_0/front_tum_0.txt corrected:traj_corrected_dense.tum; do
    python3 /home/user/scripts/ape_oxford.py ${d}/${k#*:} ${GT} ${X} ${O}/ape_pipe_gyro002_r${r}_${k%%:*} \
      > ${O}/ape_pipe_gyro002_r${r}_${k%%:*}.log 2>&1
    grep -E "^rmse|per-axis" ${O}/ape_pipe_gyro002_r${r}_${k%%:*}.log | sed "s|^|ours_r${r} ${k%%:*} |"
  done
done
echo "rounds done $(date -u +%T)"
