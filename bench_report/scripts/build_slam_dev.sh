#!/usr/bin/env bash
# Builds the lidar_slam_ros2 push clone (graph_based_slam, lidarslam, ... and the vendored rko_lio) into /home/user/ws/slam_dev.
set -eo pipefail
set +u; source /opt/ros/jazzy/setup.bash; set -u
export CCACHE_DIR=/home/user/.ccache CCACHE_BASEDIR=/home/user CCACHE_MAXSIZE=8G CCACHE_NOHASHDIR=1
cd /home/user/ws/slam_dev
start=$(date +%s)
colcon build --base-paths /home/user/lidar_slam_ros2 --packages-up-to lidarslam rko_lio --parallel-workers 1 \
  --event-handlers console_direct- --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache "$@"
echo "build_seconds=$(( $(date +%s) - start ))"
