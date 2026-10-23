#!/usr/bin/env bash
# Usage: offline_c.sh <backend_input_bag> <out_dir> [extra -p overrides...]
# graph_slam_offline_runner (dev_c) on a recorded backend input with chapter 25's C backend parameters.
bag=$1; out=$2; shift 2
rm -rf "${out}"; mkdir -p "${out}"
set +u; source /home/user/ws/dev_c/install/setup.bash; set -u
args=(); for o in "$@"; do args+=(-p "$o"); done
LC_ALL=C /usr/bin/time -v -o "${out}/time_v.txt" env ROS_DOMAIN_ID=37 ROS_LOCALHOST_ONLY=1 \
  /home/user/ws/dev_c/install/graph_based_slam/lib/graph_based_slam/graph_slam_offline_runner --ros-args \
  --disable-rosout-logs \
  --params-file /home/user/bench/params/lidarslam_competitive_v2_loop_ndt1_res2_rot1e6_drift_adjt1e4_dist60_loopc.yaml \
  -p bag_path:="${bag}" -p output_dir:="${out}" -p refine:=false "${args[@]}" > "${out}/runner.log" 2>&1
echo "exit=$? wall=$(grep 'Elapsed (wall' "${out}/time_v.txt" | awk '{print $NF}') loops=$(( $(wc -l < "${out}/loop_edges.csv" 2>/dev/null || echo 1) - 1 )) submaps=$(grep -vc '^#' "${out}/trajectory_optimized.tum" 2>/dev/null)"
