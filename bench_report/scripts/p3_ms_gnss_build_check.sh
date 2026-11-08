#!/bin/bash
# Check 2 of bench_report/data/multisession_gnss_2026-11-07/plan_build.txt: the joins and checks with the GNSS gate on.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; B=/home/user/bench/ms_build; L=/home/user/bench/loops_preset; C=/home/user/bench/ms_cons
H=/home/user/bench/ms_heldout/checks
PRESET=$REPO/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml
JOIN=$REPO/install/graph_based_slam/lib/graph_based_slam/graph_slam_multisession_join
set +u; source /opt/ros/jazzy/setup.bash; source $REPO/install/setup.bash; set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=$B/roslog
sgin() { echo $L/stadtgarten_seq$1/backend_input; }; sgrep() { echo $L/preset_check/stadtgarten_seq$1; }
cin() { echo $C/construction_seq$1/backend_input; }; crep() { echo $C/replay/construction_seq$1; }
j() { local name=$1; shift; local O=$B/runs/$name; rm -rf $O; mkdir -p $B/runs; local t0=$(date +%s)
  export ROS_DOMAIN_ID=$((40 + RANDOM % 50))
  $JOIN --ros-args --params-file $PRESET -p "extrinsic_lidar2base_quat_xyzw_xyz:=[0.0,0.0,0.0,1.0,0.0,0.0,0.0]" \
    -p multisession_num_threads:=4 -p multisession_gnss_gate:=true -p "multisession_gnss_antenna_xyz:=[0.034,0.0,0.046]" \
    "$@" -p output_dir:=$O > $O.log 2>&1
  echo "$name exit=$? $(( $(date +%s) - t0 )) s"; }
for o in "1 2" "2 1"; do set -- $o
  j sg$1$2 -p "session_bag_paths:=['$(sgin $1)','$(sgin $2)']" -p "session_backend_dirs:=['$(sgrep $1)','$(sgrep $2)']" \
    -p "multisession_gnss_bag_paths:=['$B/sg$1_gnss','$B/sg$2_gnss']"
  j sgcheck$1$2 -p "session_bag_paths:=['$(sgin $1)','$(sgin $2)']" -p "session_backend_dirs:=['$(sgrep $1)','$(sgrep $2)']" \
    -p "multisession_gnss_bag_paths:=['$B/sg$1_gnss','$B/sg$2_gnss']" -p "debug_check_transforms:=[$(cat $H/sg$1$2/perturb.values)]"
  j con$1$2 -p "session_bag_paths:=['$(cin $1)','$(cin $2)']" -p "session_backend_dirs:=['$(crep $1)','$(crep $2)']"
  j concheck$1$2 -p "session_bag_paths:=['$(cin $1)','$(cin $2)']" -p "session_backend_dirs:=['$(crep $1)','$(crep $2)']" \
    -p "debug_check_transforms:=[$(cat $C/checks/perturb$1$2.values)]"
done; set --
j sgsplit1 -p "session_bag_paths:=['$(sgin 1)']" -p "session_backend_dirs:=['$(sgrep 1)']" \
  -p "multisession_gnss_bag_paths:=['$B/sg1_gnss']" -p debug_split_gap_s:=30.0
for s in 1 2; do j consplit$s -p "session_bag_paths:=['$(cin $s)']" -p "session_backend_dirs:=['$(crep $s)']" -p debug_split_gap_s:=30.0; done
echo build checks done
