#!/bin/bash
# Stadtgarten 1 and 2 split in two (gap 30 s), as is and with the matches within 2.0 m of the identity removed
# (bench_report/data/multisession_gnss_2026-11-07/plan.txt).
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; O=/home/user/bench/ms_gnss/splits; L=/home/user/bench/loops_preset
PRESET=$REPO/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml
JOIN=$REPO/install/graph_based_slam/lib/graph_based_slam/graph_slam_multisession_join
set +u; source /opt/ros/jazzy/setup.bash; source $REPO/install/setup.bash; set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=$O/roslog ROS_DOMAIN_ID=$((40 + RANDOM % 50))
mkdir -p $O
ID=1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0
for s in 1 2; do
  for mode in asis stress; do
    name=sg${s}_$mode; [[ -s $O/$name/join_report.yaml ]] && { echo "$name present"; continue; }
    ex=(); [[ $mode == stress ]] && ex=(-p "debug_exclude_transform:=[$ID]" -p debug_exclude_radius_m:=2.0)
    t0=$(date +%s)
    $JOIN --ros-args --params-file $PRESET -p "extrinsic_lidar2base_quat_xyzw_xyz:=[0.0,0.0,0.0,1.0,0.0,0.0,0.0]" \
      -p multisession_num_threads:=4 -p "session_bag_paths:=['$L/stadtgarten_seq$s/backend_input']" \
      -p "session_backend_dirs:=['$L/preset_check/stadtgarten_seq$s']" -p debug_split_gap_s:=30.0 "${ex[@]}" \
      -p output_dir:=$O/$name > $O/$name.log 2>&1
    echo "$name exit=$? $(( $(date +%s) - t0 )) s"
  done
done
echo splits done
