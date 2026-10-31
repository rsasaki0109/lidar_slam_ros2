#!/usr/bin/env bash
# p3_spires_join.sh <out dir> <session dir>... (multi-session GIF): graph_slam_multisession_join over Oxford Spires
# sessions made by p3_spires_session.sh, the first one the reference; each joins to the map (the default), and the map is
# re-optimized after each join. Self-similarity results are stored in <out dir>/cache.
set -u
out=$1; shift
J=${JOIN:-/home/user/ws/slam_hi/build/graph_slam_multisession_join}
P=/home/user/lidar_slam_ros2/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml
bags=(); dirs=()
for d in "$@"; do bags+=("'${d}/backend_input'"); dirs+=("'${d}/run'"); done
ext=$(cat "$1/extrinsic.txt")
set +u; source /opt/ros/jazzy/setup.bash; source /home/user/ws/slam_dev/install/setup.bash; set -u
mkdir -p "${out}"
t0=$(date +%s)
ROS_LOG_DIR=${out}/roslog "${J}" --ros-args --params-file "${P}" \
  -p "session_bag_paths:=[$(IFS=,; echo "${bags[*]}")]" -p "session_backend_dirs:=[$(IFS=,; echo "${dirs[*]}")]" \
  -p output_dir:="${out}/join" -p extrinsic_lidar2base_quat_xyzw_xyz:="${ext}" -p multisession_cache_dir:="${out}/cache" \
  > "${out}/join.log" 2>&1
echo "join exit $? in $(( $(date +%s) - t0 )) s"
grep -A30 "^map_joins" "${out}/join/join_report.yaml" 2>/dev/null | head -20
rm -rf "${out}/roslog"
