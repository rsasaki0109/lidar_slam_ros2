#!/usr/bin/env bash
# Chapter 27 diagnosis: rerun C (chapter 25) on one garage trial with the rebuilt develop workspace (dev_c via
# pipe_c) and record the backend input. A watchdog stops the recorder if the disk falls below 300 MB.
t=$1
P=/home/user/bench/params; OUT=/home/user/bench/loopr; mkdir -p "${OUT}"
bag=/home/user/data/mapshd/bags/underground-garage_${t}
bash /home/user/scripts/kill_ros_procs.sh > /dev/null 2>&1
dd if="$(ls "${bag}"/*.db3)" of=/dev/null bs=16M 2>&1 | tail -1
( while sleep 5; do
    avail=$(df --output=avail -k / | tail -1)
    if (( avail < 300000 )); then echo "WATCHDOG low disk ${avail} kB"; pkill -KILL -f "ros2 bag record"; pkill -KILL -f rosbag2; break; fi
    pgrep -f "save_when_frontend_done.py" > /dev/null || break
  done ) &
env BARRIER=1 BARRIER_EXTRA="min_odom_subscribers:=2 min_deskewed_scan_subscribers:=2" BAG="${bag}" LIDAR=/livox/pointcloud IMU=/livox/imu \
  RKO_PARAMS=${P}/rko_params_mapshd_gyro002_jerk1000_corr025_locn10.ros.yaml \
  bash /home/user/scripts/pipe_record.sh "${OUT}/rows.csv" "underground-garage_${t}_C" /home/user/ws/pipe_c \
  ${P}/lidarslam_competitive_v2_loop_ndt1_res2_rot1e6_drift_adjt1e4_dist60_loopc.yaml \
  "${OUT}/underground-garage_${t}_C"
du -sh "${OUT}/underground-garage_${t}_C.backend_input"; df -h / | tail -1
