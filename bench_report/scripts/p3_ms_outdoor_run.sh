#!/bin/bash
# Runs of bench_report/data/multisession_outdoor_2026-11-07/plan.txt.
#   run.sh replay NAME BACKEND_INPUT EXTRINSIC         the walking-preset replay (the join's session_backend_dirs)
#   run.sh pair NAME REF A B EXTRINSIC [EXCLUDE_ROW]   join B to A (replays A, B; EXCLUDE_ROW: 12 values, radius 2.0)
#   run.sh split NAME REF S EXTRINSIC [stress]         split session S (gap 30 s; stress: identity excluded, 0.3 m)
# REF is multisession_consistency_reference (1e-9: the bar stays 0.5, consistency only reported).
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; B=/home/user/bench/ms_outdoor
PRESET=$REPO/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml
set +u; source /opt/ros/jazzy/setup.bash; source $REPO/install/setup.bash; set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=$B/roslog ROS_DOMAIN_ID=$((40 + RANDOM % 50))
input_of() { case $1 in
  con-1|con-2) echo $B/$1/backend_input ;;
  sg1) echo /home/user/bench/loops_preset/stadtgarten_seq1/backend_input ;;
  sg2) echo /home/user/bench/loops_preset/stadtgarten_seq2/backend_input ;;
  calib1) echo /home/user/bench/dev_loops/rb/calibration-room_trial-1/backend_input ;;
  exp04) echo /home/user/bench/dev_loops/rb/exp04/backend_input ;;
esac; }
replay_of() { case $1 in
  sg1|sg2) echo /home/user/bench/loops_preset/preset_check/stadtgarten_seq${1#sg} ;;
  *) echo $B/replay/$1 ;;
esac; }
join() {  # join NAME REF EXT session-args...
  local name=$1 ref=$2 ext=$3; shift 3
  local O=$B/runs/$name; [[ -s $O/join_report.yaml ]] && { echo "$name present"; return 0; }
  rm -rf $O; mkdir -p $B/runs $B/cache; local t0=$(date +%s.%N)
  $REPO/install/graph_based_slam/lib/graph_based_slam/graph_slam_multisession_join --ros-args --params-file $PRESET \
    -p "extrinsic_lidar2base_quat_xyzw_xyz:=[$ext]" -p multisession_num_threads:=4 -p multisession_cache_dir:=$B/cache \
    -p multisession_consistency_bar:=true -p multisession_consistency_reference:=$ref -p multisession_consistency_skip_neighbors:=${SKIP:-0} "$@" -p output_dir:=$O > $O.log 2>&1
  echo "$name exit=$? $(python3 -c "print(f'{$(date +%s.%N)-$t0:.1f}')") s"
}
case $1 in
  replay)
    O=$B/replay/$2; [[ -s $O/trajectory_raw.tum ]] && { echo "replay $2 present"; exit 0; }
    rm -rf $O; mkdir -p $B/replay
    $REPO/install/graph_based_slam/lib/graph_based_slam/graph_slam_offline_runner --ros-args --params-file $PRESET \
      -p bag_path:=$3 -p output_dir:=$O -p "extrinsic_lidar2base_quat_xyzw_xyz:=[$4]" > $O.log 2>&1
    echo "replay $2 exit=$?" ;;
  pair)
    name=$2 ref=$3 a=$4 b=$5 ext=$6; ex=()
    [[ -n ${7:-} ]] && ex=(-p "debug_exclude_transform:=[$7]" -p debug_exclude_radius_m:=2.0)
    join $name $ref $ext -p "session_bag_paths:=['$(input_of $a)','$(input_of $b)']" \
      -p "session_backend_dirs:=['$(replay_of $a)','$(replay_of $b)']" "${ex[@]}" ;;
  split)
    name=$2 ref=$3 s=$4 ext=$5; ex=()
    [[ ${6:-} == stress ]] && ex=(-p "debug_exclude_transform:=[1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0]" -p debug_exclude_radius_m:=0.3)
    join $name $ref $ext -p "session_bag_paths:=['$(input_of $s)']" -p "session_backend_dirs:=['$(replay_of $s)']" \
      -p debug_split_gap_s:=30.0 "${ex[@]}" ;;
esac
