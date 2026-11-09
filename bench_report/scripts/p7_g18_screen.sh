#!/usr/bin/env bash
# p7_g18_screen.sh <rig:short:folder> ... (report chapter 60.4, development screen, not a decision): several rko_lio
# configurations on one download of each GrandTour development mission, with the build in RKO_WS (default
# /home/user/ws/rko_g18). CFGS lists name=file pairs (default: G12 and the three screen files of
# bench_report/data/phase1_global/screen_g18/). rig: hs (Hesai XT-32 + STIM320) or ls (MID-360 x 0.9985 + STIM320).
# One line per run in /home/user/bench/g18/screen.csv: rig,cfg,mission,md5,<ATE line of p1_grandtour_ate.py>, and the
# bias line of the node (SCREEN_CSV: another file). The bags are deleted after the mission.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
WS=${RKO_WS:-/home/user/ws/rko_g18}; D=${REPO}/bench_report/data/phase1_global/screen_g18
CFGS=${CFGS:-"G12=${REPO}/configs/sota_v7/rko_lio_global_G12_2026-10-26.yaml c05b30=${D}/c05b30.yaml
  c05b10=${D}/c05b10.yaml b30=${D}/b30.yaml"}
CSV=${SCREEN_CSV:-/home/user/bench/g18/screen.csv}; mkdir -p /home/user/bench/g18 "${CSV%/*}"
for item in "$@"; do
  IFS=: read -r rig short folder <<< "${item}"
  case ${rig} in
    hs) env=(IMU=stim320_imu LIDAR=hesai_points) ;;
    ls) env=(IMU=stim320_imu RANGE_SCALE=0.9985) ;;
  esac
  for nc in ${CFGS}; do
    name=${nc%%=*}; file=${nc#*=}
    label=g18s_${name}_${rig}; O=/home/user/bench/grandtour/${label}/${short}; mkdir -p "${O}"
    if [[ ! -s ${O}/${short}.tum ]]; then
      env "${env[@]}" G_CONFIG="${file}" bash "${S}/p1_grandtour_run.sh" "${folder}" "${short}" "${label}" "${WS}" \
        > "${O%/*}/${short}.log" 2>&1 || true
    fi
    [[ -s ${O}/${short}.tum ]] || { echo "${rig},${name},${short},FAILED" | tee -a "${CSV}"; continue; }
    ate=$("${PY}" -I "${S}/p1_grandtour_ate.py" "${O}/${short}.tum" \
      "/home/user/data/grandtour/reference_dev/${short}/prism_position")
    bias=$(grep -oP 'Gyro bias tracked on \K.*' "${O}/run/stdout.log" | tail -1)
    echo "${rig},${name},${short},$(md5sum < "${O}/${short}.tum" | cut -c1-8),${ate},${bias}" \
      | tee -a "${CSV}"
  done
  rm -rf /home/user/data/grandtour/${short}/bag_* 2>/dev/null
  find "/home/user/data/grandtour/${short}/z" -type f ! -name '.z*' -delete 2>/dev/null || true
done
echo "screen done"
