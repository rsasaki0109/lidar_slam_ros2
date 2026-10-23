#!/usr/bin/env bash
# Usage: pipe_record.sh <csv> <label> <ws> <lidarslam_yaml> <out_dir>
# pipe_pipeline.sh (same env overrides) while `ros2 bag record` (sqlite3, same ROS_DOMAIN_ID 36) keeps the two
# backend input topics, /rko_lio/frame and /rko_lio/odometry, in <out_dir>.backend_input for offline loop studies.
csv=$1; label=$2; w=$3; yaml=$4; out=$5
rec=${out}.backend_input
rm -rf "${rec}"
set +u; source "${w}/install/setup.bash"; set -u
ROS_DOMAIN_ID=36 setsid ros2 bag record -s sqlite3 --include-unpublished-topics -o "${rec}" /rko_lio/frame /rko_lio/odometry \
  > "${out}.record.log" 2>&1 &
rpid=$!
sleep 4
bash /home/user/scripts/pipe_pipeline.sh "${csv}" "${label}" "${w}" "${yaml}" "${out}"
sleep 3
kill -INT -- -"${rpid}" 2>/dev/null; sleep 6; kill -KILL -- -"${rpid}" 2>/dev/null
ros2 bag info "${rec}" 2>/dev/null | grep -E "Count|Topic|Duration" | head -6
