#!/usr/bin/env bash
# p7_g20_screen_mapshd.sh [scenario ...] (G20 design, development screen, not a decision): several rko_lio configurations
# on one download of each MapsHD scenario (the eight Phase 1 trials), with the build in RKO_WS (default
# /home/user/ws/rko_g20). CFGS lists name=file pairs (default: G19 and the screen files of
# bench_report/data/phase1_global/screen_g20/). Each file goes on top of MapsHD's rig file, as in p7_g16_phase1.sh.
# One line per run in /home/user/bench/g20/screen_mapshd.csv: cfg,trial,md5,ATE (front_score.py), tilt-limited scans.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
D=${REPO}/bench_report/data/phase1_global; WS=${RKO_WS:-/home/user/ws/rko_g20}
CFGS=${CFGS:-"G19=${D}/rko_lio_candidate_G19_2026-11-09.yaml G17=${D}/rko_lio_candidate_G17_2026-11-08.yaml m10=${D}/screen_g20/m10.yaml
  m15=${D}/screen_g20/m15.yaml m20=${D}/screen_g20/m20.yaml"}
OUT=/home/user/bench/g20/screen_mapshd; CFG=${OUT}/cfg; mkdir -p "${CFG}"
for nc in ${CFGS}; do
  name=${nc%%=*}; file=${nc#*=}
  "${PY}" - "${REPO}/bench_report/params/mapshd_gyro002_jerk1000.yaml" "${file}" "${CFG}/${name}.yaml" "${CFG}/${name}_rig.yaml" <<'PY'
import sys
import yaml
rig = yaml.safe_load(open(sys.argv[1]))['/**']['ros__parameters']
g = yaml.safe_load(open(sys.argv[2]))['/**']['ros__parameters']
yaml.safe_dump({'/**': {'ros__parameters': {**rig, **g}}}, open(sys.argv[3], 'w'), sort_keys=False)
yaml.safe_dump({'lidar_topic': '/livox/pointcloud', 'imu_topic': '/livox/imu',
                'lidar_to_imu_quat_xyzw_xyz': rig.get('extrinsic_lidar2base_quat_xyzw_xyz', [0, 0, 0, 1, 0, 0, 0]),
                'g_params': sys.argv[3]}, open(sys.argv[4], 'w'), sort_keys=False)
PY
done
for sc in ${@:-calibration-room flat-room long-corridor outdoor-urk underground-garage}; do
  bash "${S}/p1_mapshd_get.sh" "${sc}" > "${OUT}/get_${sc}.log" 2>&1
  for t in /home/user/data/mapshd/bags/${sc}_trial-*_c; do
    tr=$(basename "${t}" _c)
    case ${tr} in
      calibration-room_trial-1|calibration-room_trial-2|flat-room_trial-1|long-corridor_trial-1|outdoor-urk_trial-1| \
      underground-garage_trial-1|underground-garage_trial-2|underground-garage_trial-3) ;;
      *) continue ;;
    esac
    for nc in ${CFGS}; do
      name=${nc%%=*}; o=${OUT}/runs/${tr}_${name}
      RKO_WS=${WS} bash "${S}/p5_blind_method.sh" G "${CFG}/${name}_rig.yaml" "${t}" "${o}" > /dev/null 2>&1
      [[ -s ${o}/front_0/front_tum_0.txt ]] || { echo "${name},${tr},FAILED" | tee -a "${OUT}.csv"; continue; }
      sc_=$("${PY}" /home/user/scripts/front_score.py "${tr}" "${o}/front_0/front_tum_0.txt" \
        "/home/user/data/mapshd/gt/${tr}/ground_truth.tum" | cut -d, -f2)
      lim=$(grep -oP 'Tilt correction limited on \K[0-9]+' "${o}/stdout.log" | tail -1)
      echo "${name},${tr},$(md5sum < "${o}/front_0/front_tum_0.txt" | cut -c1-8),${sc_},${lim}" | tee -a "${OUT}.csv"
    done
  done
  rm -rf /home/user/data/mapshd/bags/${sc}_trial-*_c
done
echo "screen done"
