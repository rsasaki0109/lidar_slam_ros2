#!/usr/bin/env bash
# Frontend-only variants on the MapsHD trials: each scenario archive
# from zenodo record 21318128 (retries, md5 checked), trials extracted without their ground truth (sealed in chapter
# 15), converted one at a time (mapshd_to_ros2.py), every variant run and scored (front_sweep.sh), bag deleted.
# SCENARIOS (default: all four) and TRIALS (default: every trial) limit the run; SWEEP picks the sweep script
# (default front_sweep.sh), OUT the output root, PARAMS (passed through) the frontend parameter file.
# Usage: [SCENARIOS="a b"] [TRIALS="trial-1"] mapshd_front_sweep.sh <csv> "<overrides>" ...
set -uo pipefail
csv=$1; shift
D=/home/user/data/mapshd; VPY=/home/user/mapshd/venv/bin/python; OUT=${OUT:-/home/user/bench/vert/mapshd}
set +u; source /opt/ros/jazzy/setup.bash; set -u
declare -A MD5=([calibration-room]=f965681f362701e48b4631e95e54bd10 [flat-room]=b698db851aba5abfc0e5f63bd424bfa9 [long-corridor]=aebeeab1f6805dd03b821ea33e2d375a
                [underground-garage]=09da4a85a01d406ae97a3b651d6d8cd2 [outdoor-urk]=e747e46e39ce18cdec25e818b9dde813)
mkdir -p "$D/dl" "$D/x" "$D/bags" "${OUT}"
for sc in ${SCENARIOS:-calibration-room flat-room long-corridor underground-garage outdoor-urk}; do
  ok=0
  for wait in 2 4 8 16 0; do
    curl -sS -L --fail -m 3600 -o "$D/dl/${sc}.7z" "https://zenodo.org/records/21318128/files/${sc}.7z?download=1" && { ok=1; break; }
    echo "download attempt failed ${sc}, retry in ${wait} s"; sleep "${wait}"
  done
  [[ ${ok} == 1 ]] || { echo "DOWNLOAD FAILED ${sc}"; continue; }
  [[ $(md5sum "$D/dl/${sc}.7z" | cut -d' ' -f1) == "${MD5[$sc]}" ]] || { echo "MD5 MISMATCH ${sc}"; rm -f "$D/dl/${sc}.7z"; continue; }
  (cd "$D/x" && "${VPY}" -I -c "
import py7zr, sys
with py7zr.SevenZipFile(sys.argv[1]) as z:
    names = [n for n in z.getnames() if '/trial-' in n and '/ground-truth' not in n]
with py7zr.SevenZipFile(sys.argv[1]) as z:
    z.extract(path='.', targets=names)
" "$D/dl/${sc}.7z")
  rm -f "$D/dl/${sc}.7z"
  for tdir in "$D/x/${sc}"/trial-*; do
    t=$(basename "${tdir}"); bag=$D/bags/${sc}_${t}
    if [[ -n ${TRIALS:-} && " ${TRIALS} " != *" ${t} "* ]]; then rm -rf "${tdir:?}"; continue; fi
    rm -rf "${bag}"
    if ! "${VPY}" /home/user/scripts/mapshd_to_ros2.py "${tdir}/hdmapping-raw" "${bag}" > "${OUT}/convert_${sc}_${t}.log" 2>&1; then
      echo "CONVERSION FAILED ${sc} ${t}"; tail -2 "${OUT}/convert_${sc}_${t}.log"; rm -rf "${bag}"; continue
    fi
    rm -rf "${tdir:?}"
    dd if="$(ls "${bag}"/*.db3)" of=/dev/null bs=16M 2>&1 | tail -1
    : > "${OUT}/${sc}_${t}.csv"
    bash /home/user/scripts/${SWEEP:-front_sweep.sh} "${OUT}/${sc}_${t}.csv" "${bag}" "$D/gt_sealed/${sc}/${t}/ground_truth.tum" \
      "${OUT}/${sc}_${t}" "$@" > /dev/null
    sed "s|^|${sc}_${t},|" "${OUT}/${sc}_${t}.csv" | tee -a "${csv}"
    rm -rf "${bag:?}" "${OUT}/${sc}_${t}"/*/front_0/*.ply
    df -h / | tail -1
  done
  rm -rf "${D:?}/x/${sc}"
done
echo "mapshd front sweep done $(date -u +%T)"
