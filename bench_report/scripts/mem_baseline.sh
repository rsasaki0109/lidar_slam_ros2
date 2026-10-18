#!/usr/bin/env bash
# Memory of the current pipeline (rko_lio 40b3a06 + moving-start guard, NDT 2 m, loop on) on exp04, MID-360, Oxford.
# WS, TAG, SEQS, WARM, YAML (backend params) and RKO_SUFFIX (frontend params variant) override the defaults.
set -uo pipefail
P=/home/user/bench/params; O=/home/user/bench/mem; WS=${WS:-/home/user/ws/pipe_dev_40b3a06}; TAG=${TAG:-base}
Y=${YAML:-${P}/lidarslam_competitive_v2_loop_ndt1_res2.yaml}
R=${RKO_SUFFIX:-}
M=/home/user/data/mid360_public/driving_slam_mid360/extracted/rosbag2_2024_04_16-14_17_01/rosbag2_2024_04_16-14_17_01
mkdir -p ${O}
for s in ${SEQS:-exp04 mid360 oxford}; do
  case ${s} in
    exp04) bag=/home/user/data/hilti2022/exp04_ros2; env=(RKO_PARAMS=${P}/rko_params_gyro002${R}.ros.yaml) ;;
    mid360) bag=${M}; env=(BAG=${M} LIDAR=/livox/lidar IMU=/livox/imu RKO_PARAMS=${P}/rko_params_mid360_gyro002${R}.ros.yaml) ;;
    oxford) bag=/home/user/data/oxford_keble05/ros2; env=(BAG=${bag} LIDAR=/hesai/pandar IMU=/alphasense_driver_ros/imu RKO_PARAMS=${P}/rko_params_oxford_gyro002${R}.ros.yaml) ;;
  esac
  rm -rf /home/user/bench/mem/pcd_cache; mkdir -p /home/user/bench/mem/pcd_cache
  [[ ${WARM:-1} == 1 ]] && dd if="$(ls ${bag}/*.db3)" of=/dev/null bs=16M 2>&1 | tail -1
  env "${env[@]}" bash /home/user/scripts/mem_pipeline.sh ${O}/rows.csv ${s}_${TAG} ${WS} ${Y} ${O}/${s}_${TAG}
done
echo "mem done ${TAG} $(date -u +%T)"
