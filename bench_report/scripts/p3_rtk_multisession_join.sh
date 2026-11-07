#!/bin/bash
# Joins (REPORT chapter 52) for bench_report/data/multisession_rtk_2026-11-07/plan.txt. usage: run.sh NAME A B [exclude]
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; B=/home/user/bench/ms_rtk; L=/home/user/bench/loops_preset
NAME=$1; SA=$2; SB=$3; EXC=${4:-}
set +u; source /opt/ros/jazzy/setup.bash; source $REPO/install/setup.bash; set -u
export ROS_DOMAIN_ID=$((40 + RANDOM % 50)) ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=$B/roslog
O=$B/$NAME; [[ -s $O/join_report.yaml ]] && { echo "$NAME present"; exit 0; }
rm -rf $O
ex=()
if [[ -n $EXC ]]; then
  T=$(python3 -c "import json; print(','.join(repr(v) for v in json.load(open('$B/alone_A${SA}_B${SB}.json'))['debug_exclude_transform']))")
  ex=(-p "debug_exclude_transform:=[$T]" -p debug_exclude_radius_m:=2.0)
fi
t0=$(date +%s.%N)
$REPO/install/graph_based_slam/lib/graph_based_slam/graph_slam_multisession_join --ros-args \
  --params-file $REPO/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml \
  -p "session_bag_paths:=['$L/stadtgarten_seq$SA/backend_input','$L/stadtgarten_seq$SB/backend_input']" \
  -p "session_backend_dirs:=['$L/preset_check/stadtgarten_seq$SA','$L/preset_check/stadtgarten_seq$SB']" \
  -p "extrinsic_lidar2base_quat_xyzw_xyz:=[0.0,0.0,0.0,1.0,0.0,0.0,0.0]" -p multisession_num_threads:=4 \
  "${ex[@]}" -p output_dir:=$O > $O.log 2>&1
echo "$NAME exit=$? $(python3 -c "print(f'{$(date +%s.%N)-$t0:.1f}')") s joined=$(grep -m2 'joined:' $O/join_report.yaml 2>/dev/null | tr -d ' \n')"
