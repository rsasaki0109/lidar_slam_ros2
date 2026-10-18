#!/usr/bin/env bash
# Usage: offline_loop.sh <ws> <bag> <out_dir> [extra -p overrides...]
# Replays a backend-input bag through graph_slam_offline_runner with the loop-on params, refine off.
w=$1; bag=$2; out=$3; shift 3
rm -rf "${out}"; mkdir -p "${out}"
set +u; source "${w}/install/setup.bash"; set -u
args=()
for o in "$@"; do args+=(-p "$o"); done
LC_ALL=C /usr/bin/time -v -o "${out}/time_v.txt" env ROS_DOMAIN_ID=37 ROS_LOCALHOST_ONLY=1 \
  "${w}/install/graph_based_slam/lib/graph_based_slam/graph_slam_offline_runner" --ros-args --disable-rosout-logs \
  --params-file /home/user/bench/params/lidarslam_competitive_v2_loop.yaml \
  -p bag_path:="${bag}" -p output_dir:="${out}" -p refine:=false "${args[@]}" > "${out}/runner.log" 2>&1
echo "exit=$? wall=$(grep 'Elapsed (wall' "${out}/time_v.txt" | awk '{print $NF}') user=$(grep 'User time' "${out}/time_v.txt" | awk '{print $NF}') cands=$(grep -c best_loop_candidate "${out}/runner.log") loops=$(( $(wc -l < "${out}/loop_edges.csv" 2>/dev/null || echo 1) - 1 )) submaps=$(grep -vc '^#' "${out}/trajectory_optimized.tum" 2>/dev/null)"
