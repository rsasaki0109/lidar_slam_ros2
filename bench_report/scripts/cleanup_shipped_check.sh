#!/usr/bin/env bash
# cleanup_shipped_check.sh <ws> <tag>: runs with settings the product ships (no G keys), product rko_lio vs the cleaned branch.
set -u
ws=$1; tag=$2
P=/home/user/lidar_slam_ros2/bench_report/params; L=/home/user/lidar_slam_ros2/lidarslam/param
OUT=/home/user/bench/cleanup/${tag}; CSV=/home/user/bench/cleanup/runs.csv
run() {
  local name=$1 bag=$2 lidar=$3 imu=$4; shift 4
  local pf=(); for f in "$@"; do pf+=(--params-file "$f"); done
  local o=${OUT}/${name}; rm -rf "$o"; mkdir -p "$o"
  ( set +u; source /opt/ros/jazzy/setup.bash; source "${ws}/install/setup.bash"; set -u
    export ROS_LOG_DIR="$o/roslog" ROS_DOMAIN_ID=$((40 + RANDOM % 50)) ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    timeout -s INT 3600 "${ws}/install/rko_lio/lib/rko_lio/offline_node" --ros-args "${pf[@]}" \
      -p bag_path:="${bag}" -p lidar_topic:="${lidar}" -p imu_topic:="${imu}" -p results_dir:="$o" \
      -p dump_results:=true -p run_name:=front > "$o/stdout.log" 2>&1 )
  local tum=$o/front_0/front_tum_0.txt md5=missing n=0
  [[ -f $tum ]] && md5=$(md5sum < "$tum" | cut -c1-32) && n=$(wc -l < "$tum")
  echo "${tag},${name},${md5},${n}" | tee -a "${CSV}"
}
E=/home/user/data/hilti2022/exp04_ros2; K=/home/user/data/koide/outdoor_hard_02b_c
[[ ${SKIP_EXP04:-0} == 1 ]] || run exp04_rig "$E" /hesai/pandar /alphasense/imu "$P/exp04_gyro002_jerk1000.yaml"
run koide02b_shipped "$K" /livox/points /livox/imu "/home/user/lidar_slam_ros2/bench_report/data/cleanup_2026-10-31/rko_lio_mid360_handheld_outdoor.ros.yaml"
echo DEFAULT_CHECK_DONE
