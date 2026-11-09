#!/usr/bin/env bash
# p7_g16_grandtour.sh (bench_report/data/phase1_global/decision_rule_G16.txt, criteria 1 and 4): G12 and G16 on the
# GrandTour development missions, the HS rig (Hesai XT-32 + STIM320) on all 16 and the LS rig (Livox MID-360 with range
# scale 0.9985 + STIM320) on the 8 of decision_rule_G14.txt, with p1_grandtour_run.sh (this checkout's build). Each
# trajectory is scored against the mission's prism reference (p1_grandtour_ate.py). One line per run in
# /home/user/bench/g16/grandtour.csv: rig,config,mission,md5,re-levels,node_s,data_s,<ATE line>. The bags are deleted
# after the mission.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
G12=${REPO}/configs/sota_v7/rko_lio_global_G12_2026-10-26.yaml
G16=${CAND_CFG:-${REPO}/bench_report/data/phase1_global/rko_lio_candidate_G16_2026-11-08.yaml}
CN=${CAND_NAME:-G16}  # CAND_CFG / CAND_NAME: another candidate in the place of G16
BN=${BASE_NAME:-G12}; BF=${BASE_CFG:-${G12}}  # BASE_CFG / BASE_NAME: another reference in the place of G12
[[ ${BF} == /* ]] || BF=${REPO}/${BF}
OUT=/home/user/bench/${OUT_NAME:-g16}; mkdir -p "${OUT}"
LS=${LS_MISSIONS-" heap-1 arc-3 arc-4 arc-6 con-1 eig-2 snow-3 spx-1 "}  # LS_MISSIONS= (empty): HS rig only
MR=${MAIN_RIG:-hs}  # MAIN_RIG=hls: the Hesai + Livox merged cloud in the place of the Hesai (HS) rig
DR=${DATA_ROOT:-/home/user/data/grandtour}  # where p1_grandtour_run.sh keeps the data (DATA_ROOT), cleaned per mission
MISSIONS=${MISSIONS:-"heap-1:2024-11-14-13-45-37 arc-3:2024-11-18-13-48-19 arc-4:2024-11-18-15-46-05
  arc-6:2024-11-18-17-13-09 con-1:2024-12-09-09-34-43 eig-2:2024-11-03-13-59-54 snow-3:2024-11-03-08-17-23
  spx-1:2024-11-02-17-10-25 spx-3:2024-11-02-17-43-10 ice-1:2024-11-02-21-12-51 arc-1:2024-11-18-12-05-01
  arc-5:2024-11-18-16-59-23 con-2:2024-12-09-09-41-46 pil-2:2024-11-11-12-42-47 con-3:2024-12-09-11-28-28
  snow-1:2024-11-03-07-52-45"}
for item in ${MISSIONS}; do
  short=${item%%:*}; folder=${item##*:}
  rigs=${MR}; [[ ${LS} == *" ${short} "* ]] && rigs="${MR} ls"
  for rig in ${rigs}; do
    case ${rig} in
      hs) env=(IMU=stim320_imu LIDAR=hesai_points) ;;
      hls) env=(IMU=stim320_imu LIDAR=hesai_points+livox_points) ;;
      ls) env=(IMU=stim320_imu RANGE_SCALE=0.9985) ;;
    esac
    for cfg in ${BN} ${CN}; do
      label=g16_${cfg}_${rig}
      O=/home/user/bench/grandtour/${label}/${short}
      if [[ ! -s ${O}/${short}.tum ]]; then
        c=${BF}; [[ ${cfg} != "${BN}" ]] && c=${G16}; mkdir -p "${O}"
        env "${env[@]}" G_CONFIG="${c}" bash "${S}/p1_grandtour_run.sh" "${folder}" "${short}" "${label}" "${REPO}" \
          > "${O%/*}/${short}.log" 2>&1 || true
      fi
      [[ -s ${O}/${short}.tum ]] || { echo "${rig},${cfg},${short},FAILED" | tee -a "${OUT}/grandtour.csv"; continue; }
      md5=$(md5sum < "${O}/${short}.tum" | cut -c1-8)
      rl=$("${PY}" -c "import json, sys; print(json.load(open(sys.argv[1]))['applied_count'])" \
        "${O}/run/front_0/gravity_alignment_summary.json" 2>/dev/null || echo 0)
      node=$(grep -oP 'took \K[0-9.]+' "${O}/run/stdout.log" | tail -1)
      span=$("${PY}" -c "import numpy as np, sys; t = np.loadtxt(sys.argv[1])[:, 0]; print(f'{t[-1] - t[0]:.1f}')" \
        "${O}/${short}.tum")
      ate=$("${PY}" -I "${S}/p1_grandtour_ate.py" "${O}/${short}.tum" \
        "/home/user/data/grandtour/reference_dev/${short}/prism_position")
      echo "${rig},${cfg},${short},${md5},${rl},${node},${span},${ate}" | tee -a "${OUT}/grandtour.csv"
    done
  done
  rm -rf "${DR:?}/${short:?}"/bag_* 2>/dev/null
  find "${DR}/${short}/z" -type f ! -name '.z*' -delete 2>/dev/null || true
done
echo "grandtour done"
