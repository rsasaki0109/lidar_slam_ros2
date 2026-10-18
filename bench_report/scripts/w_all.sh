#!/usr/bin/env bash
# Chapter 21: W (chapter 20's V with max_correspondence_distance: 0.25 in the MapsHD frontend profile) on the eight
# MapsHD trials, one run at a time; the baseline is chapter 20's V (same build, same harness). Both
# with BARRIER=1 (the frontend reads the bag only after graph_based_slam has subscribed) and the build of chapter 18
# (pipe_gate: rko_lio 77072e4 + graph_based_slam with the drift gate, off unless configured).
#   1. MapsHD: every trial.
#      Each scenario archive is downloaded from zenodo record 21318128 (md5 checked) and
#      only its trials are extracted; their ground truth was sealed in chapter 15 and is deleted unread here. One
#      trial at a time is converted (mapshd_to_ros2.py), run, and deleted with its raw data.
set -uo pipefail
D=/home/user/data/mapshd; OUT=/home/user/bench/scale/pipe; VPY=/home/user/mapshd/venv/bin/python
P=/home/user/bench/params; WS=/home/user/ws/pipe_gate; export BARRIER=1
declare -A Y=([W]=${P}/lidarslam_competitive_v2_loop_ndt1_res2_rot1e6_drift.yaml)
declare -A MD5=([calibration-room]=f965681f362701e48b4631e95e54bd10 [flat-room]=b698db851aba5abfc0e5f63bd424bfa9
                [long-corridor]=aebeeab1f6805dd03b821ea33e2d375a [underground-garage]=09da4a85a01d406ae97a3b651d6d8cd2
                [outdoor-urk]=e747e46e39ce18cdec25e818b9dde813)
set +u; source /opt/ros/jazzy/setup.bash; set -u
mkdir -p "${OUT}" "$D/x" "$D/dl"

warm_libs() {
  cat /home/user/ws/slam_gate/install/*/lib/*.so* /home/user/ws/slam_gate/install/graph_based_slam/lib/graph_based_slam/* \
    /home/user/ws/slam_dev/install/*/lib/*.so* /home/user/ws/cand_77072e4/install/*/lib/*.so* \
    /home/user/ws/cand_77072e4/install/rko_lio/lib/rko_lio/* > /dev/null 2>&1
}

# run <label> <cfg> <keep_map> <env...>
run() {
  local label=$1 cfg=$2 keep=$3; shift 3
  bash /home/user/scripts/kill_ros_procs.sh > /dev/null 2>&1
  warm_libs
  env "$@" bash /home/user/scripts/pipe_pipeline.sh "${OUT}/rows.csv" "${label}_${cfg}" ${WS} "${Y[$cfg]}" \
    "${OUT}/${label}_${cfg}" | tail -1
  local d=${OUT}/${label}_${cfg}
  echo "  first submap $(grep -v '^#' "$d/trajectory_optimized.tum" | head -1 | cut -d' ' -f1)," \
       "first frontend pose $(head -1 "$d/front_0/front_tum_0.txt" | cut -d' ' -f1)"
  [[ ${keep} == keep ]] || rm -rf "$d/pointcloud_map" "$d/map.pcd"
}

mapshd_trial() {
  local sc=$1 t=$2 raw=$D/x/$1/$2/hdmapping-raw bag=$D/bags/$1_$2
  rm -rf "${bag}"
  if ! "${VPY}" /home/user/scripts/mapshd_to_ros2.py "${raw}" "${bag}" > "${OUT}/convert_${sc}_${t}.log" 2>&1; then
    echo "CONVERSION FAILED ${sc} ${t}"; rm -rf "${bag}"; return
  fi
  echo "${sc} ${t}: $(tail -1 "${OUT}/convert_${sc}_${t}.log") ($(du -sh "${bag}" | cut -f1))"
  rm -rf "$D/x/${sc}/${t}"
  dd if="$(ls "${bag}"/*.db3)" of=/dev/null bs=16M 2>&1 | tail -1
  local cfgs=(W)
  for cfg in "${cfgs[@]}"; do
    run "${sc}_${t}" "${cfg}" drop BAG="${bag}" LIDAR=/livox/pointcloud IMU=/livox/imu \
      RKO_PARAMS=${P}/rko_params_mapshd_gyro002_jerk1000_corr025.ros.yaml
  done
  rm -rf "${bag}"
  df -h / | tail -1
}

for sc in flat-room calibration-room long-corridor underground-garage outdoor-urk; do
  ok=0
  for wait in 2 4 8 16 0; do
    curl -sS -L --fail -m 3600 -o "$D/dl/${sc}.7z" "https://zenodo.org/records/21318128/files/${sc}.7z?download=1" && { ok=1; break; }
    echo "download attempt failed ${sc}, retry in ${wait} s"; sleep "${wait}"
  done
  [[ ${ok} == 1 ]] || { echo "DOWNLOAD FAILED ${sc}"; continue; }
  got=$(md5sum "$D/dl/${sc}.7z" | cut -d' ' -f1)
  [[ ${got} == "${MD5[$sc]}" ]] || { echo "MD5 MISMATCH ${sc} ${got}"; rm -f "$D/dl/${sc}.7z"; continue; }
  (cd "$D/x" && "${VPY}" -I -c "
import py7zr, sys
with py7zr.SevenZipFile(sys.argv[1]) as z:
    names = [n for n in z.getnames() if '/trial-' in n and '/ground-truth' not in n]
with py7zr.SevenZipFile(sys.argv[1]) as z:
    z.extract(path='.', targets=names)
" "$D/dl/${sc}.7z")
  rm -f "$D/dl/${sc}.7z"
  for tdir in "$D/x/${sc}"/trial-*; do
    mapshd_trial "${sc}" "$(basename "${tdir}")"
  done
  rm -rf "${D:?}/x/${sc}"
done

echo "w all done $(date -u +%T)"
