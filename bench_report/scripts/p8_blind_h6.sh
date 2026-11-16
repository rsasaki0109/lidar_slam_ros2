#!/usr/bin/env bash
# p8_blind_h6.sh [seq ...] (bench_report/data/blind_v7_run2_2026-11-10/preregistration.txt): the H6 holdouts
# (FusionPortable V2). For each sequence: stream the 7z archive from Google Drive through p8_7z_http_stream.py (size and
# CRC32 checked against the archive header) into p8_ros1_stream_to_ros2.py (LiDAR + STIM300 only, never stored whole),
# run every method three times (p8_blind_method.sh), then delete the converted bag. No ground-truth file is read.
# The converted bag is written to /dev/shm; one above 9 GB is deleted and the sequence is recorded as too large.
# Outputs: /home/user/bench/blind2/runs/<seq>/<method>_r<k>/, one line per run in /home/user/bench/blind2/runs.csv.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; D=${REPO}/bench_report/data/blind_v7_run2_2026-11-10
PY=/home/user/mapshd/venv/bin/python; PY7=${PY7:-/home/user/venvs/blind2/bin/python}
B=/home/user/bench/blind2; DATA=/dev/shm/blind2
METHODS=${METHODS:-G rko_upstream bievr fastlio glim}
mkdir -p "${DATA}" "${B}/runs"
seqs=("$@")
[[ ${#seqs[@]} -gt 0 ]] || mapfile -t seqs < <(grep -v '^#' "${D}/h6_sequences.txt" | awk '{print $1}')
for seq in "${seqs[@]}"; do
  id=$(awk -v s="${seq}.7z" '$1 == s {print $3}' "${D}/fusionportable_v2_archives.txt")
  [[ -n ${id} ]] || { echo "${seq}: not in the archive list"; exit 1; }
  rig=${D}/rig_fp_${seq%%_*}.yaml
  [[ -f ${rig} ]] || { echo "${seq}: no rig file ${rig}"; exit 1; }
  [[ -f ${B}/runs/${seq}/too_large ]] && continue
  if ! ls "${B}/runs/${seq}"/glim_r3/exit.txt > /dev/null 2>&1 || [[ -n ${FORCE:-} ]]; then
    bag=${DATA}/${seq}/bag
    for k in 1 2 3; do
      [[ -f ${DATA}/${seq}/ok ]] && break
      rm -rf "${DATA:?}/${seq:?}"; mkdir -p "${DATA}/${seq}" "${B}/runs/${seq}"
      echo "$(date -u +%FT%TZ) ${seq}: streaming (attempt ${k})"
      "${PY7}" -I "${S}/p8_7z_http_stream.py" "https://drive.usercontent.google.com/download?id=${id}&export=download&confirm=t" \
        --summary "${DATA}/${seq}/archive.json" 2> "${DATA}/${seq}/archive.err" |
        (cd /tmp && "${PY}" -I "${S}/p8_ros1_stream_to_ros2.py" - "${bag}" --lidar /os_cloud_node/points \
          --imu /stim300/imu/data_raw --drop-zero --keep x,y,z,intensity,t,reflectivity,ring,ambient,range \
          --summary "${DATA}/${seq}/summary.json" > "${DATA}/${seq}/convert.log" 2>&1)
      ok=$("${PY}" -c "
import json, sys
a = json.load(open(sys.argv[1])); s = json.load(open(sys.argv[2]))
print(int(a['complete'] and a['unpacked_bytes'] == a['unpacked_expected'] and a['crc32'] == a['crc32_expected']
          and s['complete'] and s['bytes_read'] == a['unpacked_bytes']))" "${DATA}/${seq}/archive.json" "${DATA}/${seq}/summary.json" 2>/dev/null)
      if [[ ${ok} == 1 ]]; then touch "${DATA}/${seq}/ok"; else echo "${seq}: stream check failed"; tail -n 2 "${DATA}/${seq}/archive.err"; sleep 120; fi
    done
    [[ -f ${DATA}/${seq}/ok ]] || { echo "${seq}: conversion failed three times"; exit 1; }
    cp "${DATA}/${seq}/summary.json" "${B}/runs/${seq}/bag_summary.json"
    cp "${DATA}/${seq}/archive.json" "${B}/runs/${seq}/archive_summary.json"
    kb=$(du -sk "${bag}" | cut -f1)
    if (( kb > 9 * 1024 * 1024 )); then
      echo "${seq}: converted bag ${kb} KiB is above 9 GB, not run" | tee "${B}/runs/${seq}/too_large"
      rm -rf "${DATA:?}/${seq:?}"; continue
    fi
    for m in ${METHODS}; do
      for r in 1 2 3; do
        out=${B}/runs/${seq}/${m}_r${r}
        [[ -f ${out}/exit.txt ]] && continue
        line=$(bash "${S}/p8_blind_method.sh" "${m}" "${rig}" "${bag}" "${out}" 2>>"${B}/runs/${seq}/errors.log" | tail -1)
        echo "${seq},${m},${r},$(date -u +%FT%TZ),${line}" | tee -a "${B}/runs.csv"
      done
    done
  fi
  rm -rf "${DATA:?}/${seq:?}"
done
echo "H6 done"
