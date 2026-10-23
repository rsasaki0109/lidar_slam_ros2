#!/usr/bin/env bash
# Builds develop (graph_based_slam with loop_edge_info_constant_*, rko_lio ec61a3e = 5207d36 tree) without tests
# into /home/user/ws/dev_c; replaces the deleted slam_lc + cand_5207d36 overlay used by pipe_c.
set -eo pipefail
set +u; source /opt/ros/jazzy/setup.bash; set -u
export CCACHE_DIR=/home/user/.ccache CCACHE_BASEDIR=/home/user CCACHE_MAXSIZE=8G CCACHE_NOHASHDIR=1
cd /home/user/ws/dev_c
start=$(date +%s)
colcon build --base-paths /home/user/lidar_slam_ros2 --packages-up-to lidarslam rko_lio --parallel-workers 1 \
  --event-handlers console_direct- --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
echo "build_seconds=$(( $(date +%s) - start ))"
