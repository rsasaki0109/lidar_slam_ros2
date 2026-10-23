#!/usr/bin/env bash
# Chapter 27 diagnosis: C on one on-disk sequence with backend-input recording (barrier 2 + unpublished-topic
# subscription). Usage: seq_record_run.sh <label> <bag> <lidar_topic> <imu_topic> <rko_params>
label=$1; bag=$2; lidar=$3; imu=$4; rko=$5
P=/home/user/bench/params; OUT=/home/user/bench/loopr; mkdir -p "${OUT}"
bash /home/user/scripts/kill_ros_procs.sh > /dev/null 2>&1
dd if="$(ls "${bag}"/*.db3 | head -1)" of=/dev/null bs=16M 2>&1 | tail -1
( while sleep 5; do
    avail=$(df --output=avail -k / | tail -1)
    if (( avail < 300000 )); then echo "WATCHDOG low disk ${avail} kB"; pkill -KILL -f "ros2 bag record"; pkill -KILL -f rosbag2; break; fi
    pgrep -f "save_when_frontend_done.py" > /dev/null || break
  done ) &
env BARRIER=1 BARRIER_EXTRA="min_odom_subscribers:=2 min_deskewed_scan_subscribers:=2 subscriber_wait_timeout_sec:=90" BAG="${bag}" LIDAR="${lidar}" \
  IMU="${imu}" RKO_PARAMS="${rko}" \
  bash /home/user/scripts/pipe_record.sh "${OUT}/rows.csv" "${label}_C" /home/user/ws/pipe_c \
  ${P}/lidarslam_competitive_v2_loop_ndt1_res2_rot1e6_drift_adjt1e4_dist60_loopc.yaml "${OUT}/${label}_C"
set +u; source /opt/ros/jazzy/setup.bash; set -u
ros2 bag reindex "${OUT}/${label}_C.backend_input" -s sqlite3 > /dev/null 2>&1
du -sh "${OUT}/${label}_C.backend_input"; df -h / | tail -1
