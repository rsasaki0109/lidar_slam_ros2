#!/bin/bash
# rec.sh SEQ: download one RTK-SLAM sequence, record its backend input with GNSS, delete the bag
# (bench_report/data/multisession_gnss_2026-11-07/plan_construction.txt).
set -euo pipefail
SEQ=$1; REPO=/home/user/lidar_slam_ros2; B=/home/user/bench/ms_cons; O=$B/$SEQ; BAG=/home/user/data/rtk_slam/ros2/$SEQ
set +u; source /opt/ros/jazzy/setup.bash; source $REPO/install/setup.bash; set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=$B/roslog ROS_DOMAIN_ID=$((40 + RANDOM % 50))
mkdir -p $O
if [[ ! -s $O/backend_input/metadata.yaml ]]; then
  t0=$(date +%s)
  python3 $REPO/scripts/download_rtk_slam_dataset.py --sequence $SEQ --dest /home/user/data/rtk_slam > $O/download.log 2>&1
  echo "$SEQ downloaded $(( $(date +%s) - t0 )) s; $(du -sh $BAG | cut -f1)"
  grep -E "name: /" $BAG/metadata.yaml | sed 's/^ *//' > $O/topics.txt || true
  rm -rf $O/backend_input $O/front
  t0=$(date +%s)
  BACKEND_CAPTURE_STORAGE_PRESET=zstd_fast bash $REPO/scripts/record_backend_input.sh --output-dir $O/backend_input \
    --gnss-topic /gnss/fix --setup $REPO/install/setup.bash -- \
    $REPO/install/rko_lio/lib/rko_lio/offline_node --ros-args \
    -p bag_path:=$BAG -p lidar_topic:=/livox/points -p imu_topic:=/livox/imu -p gnss_topic:=/gnss/fix \
    -p base_frame:=os_sensor -p odom_frame:=odom -p publish_deskewed_scan:=true \
    -p odom_topic:=/rko_lio/odometry -p deskewed_scan_topic:=/rko_lio/frame -p publisher_queue_depth:=128 \
    -p output_subscriber_timeout_sec:=60.0 -p dump_results:=true -p results_dir:=$O/front -p run_name:=front \
    --params-file /home/user/bench/loops_preset/rko_stadtgarten_seq2.ros.yaml > $O/record.log 2>&1
  echo "$SEQ recorded $(( $(date +%s) - t0 )) s; $(du -sh $O/backend_input | cut -f1); $(tail -3 $O/record.log | tr '\n' ' ')"
fi
rm -rf "${BAG:?}"
echo "$SEQ bag deleted; free $(df -B1 /home/user | tail -1 | awk '{printf "%.2f GB", $4/1e9}')"
