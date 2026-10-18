#!/usr/bin/env bash
# Frontend-only runs of exp04 (7 checkpoints, ape_from_tum --interpolate) and Oxford Keble 05 (ape_oxford.py) per
# variant (ROS parameter overrides "k:=v ..."), one run at a time. Appends seq,variant,ape,md5 to <csv>.
# Usage: front_sweep_other.sh <csv> <out_root> <seqs: exp04,oxford> "<overrides>" ...
set -uo pipefail
csv=$1; root=$2; seqs=$3; shift 3
WS=${WS:-/home/user/ws/cand_77072e4}; P=/home/user/bench/params; REPO=/home/user/lidar_slam_ros2
mkdir -p "${root}"
for seq in ${seqs//,/ }; do
  case ${seq} in
    exp04) yaml=${P}/exp04_gyro002.yaml; bag=/home/user/data/hilti2022/exp04_ros2; lidar=/hesai/pandar; imu=/alphasense/imu ;;
    oxford) yaml=${P}/oxford_keble05_gyro002.yaml; bag=/home/user/data/oxford_keble05/ros2; lidar=/hesai/pandar
            imu=/alphasense_driver_ros/imu ;;
  esac
  dd if="$(ls ${bag}/*.db3)" of=/dev/null bs=16M 2>&1 | tail -1
  for v in "$@"; do
    name=${seq}_$(echo "x${v}" | tr ' =:.' '____')
    args=(); for kv in ${v}; do args+=(-p "${kv}"); done
    bash /home/user/scripts/kill_ros_procs.sh > /dev/null 2>&1
    ROS_DOMAIN_ID=37 bash /home/user/scripts/run_front.sh "${name}" "${WS}" "${yaml}" "${bag}" "${lidar}" "${imu}" \
      "${root}/${name}" "${root}/runs.csv" "${args[@]}" > /dev/null
    t="${root}/${name}/front_0/front_tum_0.txt"
    if [[ ! -f ${t} ]]; then echo "${seq},${v:-default},FAILED" | tee -a "${csv}"; continue; fi
    if [[ ${seq} == exp04 ]]; then
      python3 ${REPO}/scripts/ape_from_tum.py --ref /home/user/data/hilti2022/exp04_construction_upper_level_gt.txt \
        --est "${t}" --out "${root}/${name}/ape.txt" --interpolate > /dev/null 2>&1
      ape=$(awk '/^rmse:/ {print $2}' "${root}/${name}/ape.txt")
    else
      ape=$(python3 ${REPO}/bench_report/scripts/ape_oxford.py "${t}" /home/user/data/oxford_keble05/gt_sealed/gt-tum.txt \
        /home/user/data/oxford_keble05/calib/T_imu_base.npy "${root}/${name}/ape" | awk '/^rmse:/ {print $2}')
    fi
    echo "${seq},${v:-default},${ape},$(md5sum "${t}" | cut -c1-8)" | tee -a "${csv}"
    rm -rf "${root}/${name}/roslog"
  done
done
