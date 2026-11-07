#!/bin/bash
# Runs of bench_report/data/multisession_heldout_2026-11-07/plan.txt.
#   p3_ms_heldout_run.sh replay NAME BACKEND_INPUT EXTRINSIC     the walking-preset replay of a MapsHD trial
#   p3_ms_heldout_run.sh split NAME REF S EXTRINSIC [stress]     split session S (gap 30 s; stress: identity
#                                                                excluded at 0.3 m)                     (part A)
#   p3_ms_heldout_run.sh check NAME A B EXTRINSIC JOIN_REPORT    the whole-map check of B in A at the transform of
#                                                                JOIN_REPORT and its 23 perturbations   (part B)
# REF is multisession_consistency_reference (1e-9: the bar stays 0.5). SKIP: multisession_consistency_skip_neighbors.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; B=/home/user/bench/ms_heldout; O54=/home/user/bench/ms_outdoor
PRESET=$REPO/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml
JOIN=$REPO/install/graph_based_slam/lib/graph_based_slam/graph_slam_multisession_join
set +u; source /opt/ros/jazzy/setup.bash; source $REPO/install/setup.bash; set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=$B/roslog ROS_DOMAIN_ID=$((40 + RANDOM % 50))
input_of() { case $1 in
  con-1|con-2) echo $O54/$1/backend_input ;;
  sg1) echo /home/user/bench/loops_preset/stadtgarten_seq1/backend_input ;;
  sg2) echo /home/user/bench/loops_preset/stadtgarten_seq2/backend_input ;;
  calib2) echo $B/calibration-room_trial-2/backend_input ;;
  garage1) echo $B/underground-garage_trial-1/backend_input ;;
esac; }
replay_of() { case $1 in
  sg1|sg2) echo /home/user/bench/loops_preset/preset_check/stadtgarten_seq${1#sg} ;;
  con-1|con-2) echo $O54/replay/$1 ;;
  *) echo $B/replay/$1 ;;
esac; }
elapsed() { python3 -c "import sys; print(f'{float(sys.argv[2]) - float(sys.argv[1]):.1f}')" "$1" "$(date +%s.%N)"; }
case $1 in
  replay)
    O=$B/replay/$2; [[ -s $O/trajectory_raw.tum ]] && { echo "replay $2 present"; exit 0; }
    rm -rf $O; mkdir -p $B/replay
    $REPO/install/graph_based_slam/lib/graph_based_slam/graph_slam_offline_runner --ros-args --params-file $PRESET \
      -p bag_path:=$3 -p output_dir:=$O -p "extrinsic_lidar2base_quat_xyzw_xyz:=[$4]" > $O.log 2>&1
    echo "replay $2 exit=$?" ;;
  split)
    name=$2 ref=$3 s=$4 ext=$5; ex=()
    [[ ${6:-} == stress ]] && ex=(-p "debug_exclude_transform:=[1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0]" -p debug_exclude_radius_m:=0.3)
    O=$B/runs/$name; [[ -s $O/join_report.yaml ]] && { echo "$name present"; exit 0; }
    rm -rf $O; mkdir -p $B/runs; t0=$(date +%s.%N)
    $JOIN --ros-args --params-file $PRESET -p "extrinsic_lidar2base_quat_xyzw_xyz:=[$ext]" \
      -p multisession_num_threads:=4 -p multisession_consistency_bar:=true \
      -p multisession_consistency_reference:=$ref -p multisession_consistency_skip_neighbors:=${SKIP:-0} \
      -p "session_bag_paths:=['$(input_of $s)']" -p "session_backend_dirs:=['$(replay_of $s)']" \
      -p debug_split_gap_s:=30.0 "${ex[@]}" -p output_dir:=$O > $O.log 2>&1
    echo "$name exit=$? $(elapsed $t0) s" ;;
  check)
    name=$2 a=$3 b=$4 ext=$5 report=$6
    O=$B/checks/$name; [[ -s $O/debug_checks.yaml ]] && { echo "$name present"; exit 0; }
    rm -rf $O; mkdir -p $O; t0=$(date +%s.%N)
    python3 -I $REPO/bench_report/scripts/p3_ms_perturb.py $report $(replay_of $b)/trajectory_raw.tum $O/perturb
    $JOIN --ros-args --params-file $PRESET -p "extrinsic_lidar2base_quat_xyzw_xyz:=[$ext]" \
      -p multisession_num_threads:=4 -p multisession_consistency_bar:=true \
      -p multisession_consistency_reference:=0.880487 -p multisession_consistency_skip_neighbors:=10 \
      -p "session_bag_paths:=['$(input_of $a)','$(input_of $b)']" \
      -p "session_backend_dirs:=['$(replay_of $a)','$(replay_of $b)']" \
      -p "debug_check_transforms:=[$(cat $O/perturb.values)]" -p output_dir:=$O > $O.log 2>&1
    echo "$name exit=$? $(elapsed $t0) s" ;;
esac
