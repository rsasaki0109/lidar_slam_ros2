#!/usr/bin/env bash
# p7_tilt_diag.sh <rig:short:folder> ... (report chapter 60.4): G12 and G17 on GrandTour development missions with
# rko_lio's per-scan attitude diagnostic (RKO_LIO_TILT_DIAG; diagnostic only, the trajectory is unchanged), to find a
# signal that tells where the registration should keep roll and pitch from the gyro (G17 helped on ARC-3, ARC-5, PIL-2)
# from where it must not (CON-2, EIG-2, LS SPX-1). rig: hs (Hesai XT-32 + STIM320) or ls (MID-360 x 0.9985 + STIM320).
# Also fetches the mission's STIM320 group into /home/user/data/grandtour/imu/<short>/ for p1_grandtour_frame_tilt.py.
# Outputs: /home/user/bench/g18/diag/<cfg>_<rig>/<short>.txt (one line per scan, the fields of lio.cpp) and
# /home/user/bench/grandtour/g18diag_<cfg>_<rig>/<short>/ (the run). The bag is deleted after the mission.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts
declare -A CFG=([G12]=${REPO}/configs/sota_v7/rko_lio_global_G12_2026-10-26.yaml
                [G17]=${REPO}/bench_report/data/phase1_global/rko_lio_candidate_G17_2026-11-08.yaml)
HF=https://huggingface.co/datasets/leggedrobotics/grand_tour_dataset/resolve/main
for item in "$@"; do
  IFS=: read -r rig short folder <<< "${item}"
  I=/home/user/data/grandtour/imu/${short}
  if [[ ! -f ${I}/stim320_imu/.complete ]]; then
    rm -rf "${I}.part"; mkdir -p "${I}.part" "${I}"
    curl -sS -L --fail "${HF}/${folder}/data/stim320_imu.tar" | tar xf - -C "${I}.part" \
      && rm -rf "${I}/stim320_imu" && mv "${I}.part/stim320_imu" "${I}/stim320_imu" && touch "${I}/stim320_imu/.complete"
    rm -rf "${I}.part"
  fi
  case ${rig} in
    hs) env=(IMU=stim320_imu LIDAR=hesai_points) ;;
    ls) env=(IMU=stim320_imu RANGE_SCALE=0.9985) ;;
  esac
  for cfg in ${CFGS:-G12 G17}; do
    D=/home/user/bench/g18/diag/${cfg}_${rig}; mkdir -p "${D}"; rm -f "${D}/${short}.txt"
    label=g18diag_${cfg}_${rig}; O=/home/user/bench/grandtour/${label}/${short}; mkdir -p "${O}"
    env "${env[@]}" G_CONFIG="${CFG[${cfg}]}" RKO_LIO_TILT_DIAG="${D}/${short}.txt" \
      bash "${S}/p1_grandtour_run.sh" "${folder}" "${short}" "${label}" "${REPO}" > "${O%/*}/${short}.log" 2>&1
    echo "${rig},${cfg},${short},$(md5sum < "${O}/${short}.tum" | cut -c1-8),$(wc -l < "${D}/${short}.txt")"
  done
  rm -rf /home/user/data/grandtour/${short}/bag_* 2>/dev/null
  find "/home/user/data/grandtour/${short}/z" -type f ! -name '.z*' -delete 2>/dev/null || true
done
echo "diag done"
