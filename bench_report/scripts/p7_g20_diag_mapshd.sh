#!/usr/bin/env bash
# p7_g20_diag_mapshd.sh [scenario ...] (G20 design, development data): G12 on MapsHD's eight Phase 1 trials with
# rko_lio's per-scan attitude diagnostic (RKO_LIO_TILT_DIAG; diagnostic only, the trajectory is unchanged and its md5
# is printed to compare with data/phase1_global/g19_phase1.csv). Sensor data only; no ground truth is read here.
# Outputs: /home/user/bench/g20/diag/G12_mapshd/<trial>.txt (the fields of lio.cpp) and the runs under
# /home/user/bench/g20/diag_runs/. The bags are deleted after each scenario.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts
RIG=${RIG:-/home/user/bench/g19/phase1_cfg/mapshd_G12_rig.yaml}
D=/home/user/bench/g20/diag/G12_mapshd; R=/home/user/bench/g20/diag_runs; mkdir -p "${D}" "${R}"
for sc in ${@:-calibration-room flat-room long-corridor outdoor-urk underground-garage}; do
  bash "${S}/p1_mapshd_get.sh" "${sc}" > "${R}/get_${sc}.log" 2>&1
  for t in /home/user/data/mapshd/bags/${sc}_trial-*_c; do
    tr=$(basename "${t}" _c)
    case ${tr} in
      calibration-room_trial-1|calibration-room_trial-2|flat-room_trial-1|long-corridor_trial-1|outdoor-urk_trial-1| \
      underground-garage_trial-1|underground-garage_trial-2|underground-garage_trial-3) ;;
      *) continue ;;
    esac
    rm -f "${D}/${tr}.txt"
    RKO_LIO_TILT_DIAG="${D}/${tr}.txt" bash "${S}/p5_blind_method.sh" G "${RIG}" "${t}" "${R}/${tr}" > /dev/null 2>&1
    echo "${tr},$(md5sum < "${R}/${tr}/front_0/front_tum_0.txt" | cut -c1-8),$(wc -l < "${D}/${tr}.txt")"
  done
  rm -rf /home/user/data/mapshd/bags/${sc}_trial-*_c
done
echo "diag done"
