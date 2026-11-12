#!/usr/bin/env bash
# p8_blind_h8.sh [seq ...] (bench_report/data/blind_v7_run2_2026-11-10/preregistration_h8.txt): the H8 holdouts (PASTEL,
# Zenodo record 22145476). For each sequence: fetch <n>.bag from Zenodo (kept on disk until its runs are done when it
# fits, so that a VM restart resumes the download; streamed otherwise) into p8_ros1_stream_to_ros2.py (the OS0-128
# cloud and the OS1-16 IMU only; no other topic is deserialised), check the md5 of the bag against the record's, run every method three times (p8_blind_method.sh), then delete the converted
# bag. ground_truth.zip is not downloaded. The converted bag is written to /dev/shm.
# Outputs: /home/user/bench/blind2/runs_h8/<seq>/<method>_r<k>/, one line per run in /home/user/bench/blind2/runs_h8.csv.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; D=${REPO}/bench_report/data/blind_v7_run2_2026-11-10
PY=/home/user/mapshd/venv/bin/python
PY7=${PY7:-/home/user/venvs/blind2/bin/python}
B=/home/user/bench/blind2; DATA=/dev/shm/blind2_h8; RIG=${D}/rig_pastel.yaml
METHODS=${METHODS:-G rko_upstream bievr fastlio glim}
RAW=/home/user/data/blind2/pastel/raw
mkdir -p "${DATA}" "${B}/runs_h8" "${RAW}"
seqs=("$@")
[[ ${#seqs[@]} -gt 0 ]] || mapfile -t seqs < <(grep -v '^#' "${D}/h8_sequences.txt" | awk '{print $1}')
for seq in "${seqs[@]}"; do
  bytes=$(awk -v s="${seq}.bag" '$1 == s {print $2}' "${D}/pastel_files.txt")
  md5=$(awk -v s="${seq}.bag" '$1 == s {sub("md5:", "", $3); print $3}' "${D}/pastel_files.txt")
  [[ -n ${md5} ]] || { echo "${seq}: not in the file list"; exit 1; }
  if ! [[ -f ${B}/runs_h8/${seq}/glim_r3/exit.txt ]]; then
    bag=${DATA}/${seq}/bag
    for k in 1 2 3 4 5; do
      [[ -f ${DATA}/${seq}/ok ]] && break
      rm -rf "${DATA:?}/${seq:?}"; mkdir -p "${DATA}/${seq}" "${B}/runs_h8/${seq}"
      echo "$(date -u +%FT%TZ) ${seq}: streaming (attempt ${k})"
      url="https://zenodo.org/api/records/22145476/files/${seq}.bag/content"
      conv=("${PY}" -I "${S}/p8_ros1_stream_to_ros2.py" - "${bag}" --lidar /os0_128/points_filtered --imu /os1_16/imu
        --drop-zero --keep x,y,z,intensity,t,reflectivity,ring,ambient,range --summary "${DATA}/${seq}/summary.json")
      have=$(stat -c %s "${RAW}/${seq}.bag" 2>/dev/null || echo 0)
      if (( (bytes - have) / 1024 + 512 * 1024 < $(df -k --output=avail /home/user | tail -1) )); then
        # the VM restarts every few hours and Zenodo is slow: keep the download on disk so that it resumes after a restart
        until (( $(stat -c %s "${RAW}/${seq}.bag" 2>/dev/null || echo 0) >= bytes )); do
          curl -sSL --fail -C - -o "${RAW}/${seq}.bag" "${url}" || { echo "${seq}: download interrupted, resuming"; sleep 30; }
        done
        md5sum "${RAW}/${seq}.bag" | cut -d' ' -f1 > "${DATA}/${seq}/md5.txt"
        (cd /tmp && "${conv[@]}" < "${RAW}/${seq}.bag" > "${DATA}/${seq}/convert.log" 2>&1)
      else
        "${PY7}" -I "${S}/p8_http_stream.py" "${url}" 2> "${DATA}/${seq}/stream.err" |
          tee >(md5sum | cut -d' ' -f1 > "${DATA}/${seq}/md5.txt") |
          (cd /tmp && "${conv[@]}" > "${DATA}/${seq}/convert.log" 2>&1)
      fi
      sleep 2
      if [[ $(cat "${DATA}/${seq}/md5.txt" 2>/dev/null) == "${md5}" && -s ${DATA}/${seq}/summary.json ]]; then
        touch "${DATA}/${seq}/ok"
      else
        echo "${seq}: md5 $(cat "${DATA}/${seq}/md5.txt" 2>/dev/null) != ${md5}"; rm -f "${RAW}/${seq}.bag"; sleep 300
      fi
    done
    [[ -f ${DATA}/${seq}/ok ]] || { echo "${seq}: conversion failed five times"; exit 1; }
    cp "${DATA}/${seq}/summary.json" "${B}/runs_h8/${seq}/bag_summary.json"
    for m in ${METHODS}; do
      for r in 1 2 3; do
        out=${B}/runs_h8/${seq}/${m}_r${r}
        [[ -f ${out}/exit.txt ]] && continue
        line=$(bash "${S}/p8_blind_method.sh" "${m}" "${RIG}" "${bag}" "${out}" 2>>"${B}/runs_h8/${seq}/errors.log" | tail -1)
        echo "${seq},${m},${r},$(date -u +%FT%TZ),${line}" | tee -a "${B}/runs_h8.csv"
      done
    done
  fi
  rm -rf "${DATA:?}/${seq:?}"; rm -f "${RAW}/${seq}.bag"
done
echo "H8 done"
