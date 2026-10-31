#!/usr/bin/env bash
# cleanup_frontend_check.sh <ws> <tag>: frontend-only runs whose trajectories must stay byte-identical through the rko_lio cleanup.
# G12 / G15g: the rig file, then the adopted configuration's file over it (algorithm keys); rig: the rig file alone.
set -u
ws=$1; tag=$2
C=/home/user/lidar_slam_ros2/configs/sota_v7; P=/home/user/lidar_slam_ros2/bench_report/params
OUT=/home/user/bench/cleanup/${tag}; CSV=/home/user/bench/cleanup/runs.csv
mkdir -p /home/user/bench/cleanup
run() {  # run <name> <bag> <lidar> <imu> <params files...>
  local name=$1 bag=$2 lidar=$3 imu=$4; shift 4
  local pf=(); for f in "$@"; do pf+=(--params-file "$f"); done
  local o=${OUT}/${name}; rm -rf "$o"; mkdir -p "$o"
  ( set +u; source /opt/ros/jazzy/setup.bash; source "${ws}/install/setup.bash"; set -u
    export ROS_LOG_DIR="$o/roslog" ROS_DOMAIN_ID=$((40 + RANDOM % 50)) ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    timeout -s INT 3600 "${ws}/install/rko_lio/lib/rko_lio/offline_node" --ros-args "${pf[@]}" \
      -p bag_path:="${bag}" -p lidar_topic:="${lidar}" -p imu_topic:="${imu}" -p results_dir:="$o" > "$o/stdout.log" 2>&1 )
  local tum=$o/front_0/front_tum_0.txt md5=missing n=0
  [[ -f $tum ]] && md5=$(md5sum < "$tum" | cut -c1-32) && n=$(wc -l < "$tum")
  echo "${tag},${name},${md5},${n}" | tee -a "${CSV}"
}
E=/home/user/data/hilti2022/exp04_ros2; K=/home/user/data/koide/outdoor_hard_02b_c
run exp04_G12  "$E" /hesai/pandar /alphasense/imu "$P/exp04_gyro002_jerk1000.yaml" "$C/rko_lio_global_G12_2026-10-26.yaml"
run exp04_G15g "$E" /hesai/pandar /alphasense/imu "$P/exp04_gyro002_jerk1000.yaml" "$C/rko_lio_grandtour_hs_G15g_2026-10-28.yaml"
run exp04_rig  "$E" /hesai/pandar /alphasense/imu "$P/exp04_gyro002_jerk1000.yaml"
run koide02b_G12  "$K" /livox/points /livox/imu "$P/p1_koide_V.yaml" "$C/rko_lio_global_G12_2026-10-26.yaml"
run koide02b_G15g "$K" /livox/points /livox/imu "$P/p1_koide_V.yaml" "$C/rko_lio_grandtour_hs_G15g_2026-10-28.yaml"
echo CLEANUP_CHECK_DONE
