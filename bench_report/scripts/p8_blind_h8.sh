#!/usr/bin/env bash
# p8_blind_h8.sh [seq ...] (bench_report/data/blind_v7_run2_2026-11-10/preregistration_h8.txt): the H8 holdouts (PASTEL,
# Zenodo record 22145476). For each sequence: stream <n>.bag from Zenodo into p8_ros1_stream_to_ros2.py (the OS0-128
# cloud and the OS1-16 IMU only; the bag is never stored and no other topic is deserialised), check the md5 of the
# streamed bytes against the record's, run every method three times (p8_blind_method.sh), then delete the converted
# bag. ground_truth.zip is not downloaded. The converted bag is written to /dev/shm.
# Outputs: /home/user/bench/blind2/runs_h8/<seq>/<method>_r<k>/, one line per run in /home/user/bench/blind2/runs_h8.csv.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; D=${REPO}/bench_report/data/blind_v7_run2_2026-11-10
PY=/home/user/mapshd/venv/bin/python
B=/home/user/bench/blind2; DATA=/dev/shm/blind2_h8; RIG=${D}/rig_pastel.yaml
METHODS=${METHODS:-G rko_upstream bievr fastlio glim}
mkdir -p "${DATA}" "${B}/runs_h8"
seqs=("$@")
[[ ${#seqs[@]} -gt 0 ]] || mapfile -t seqs < <(grep -v '^#' "${D}/h8_sequences.txt" | awk '{print $1}')
for seq in "${seqs[@]}"; do
  md5=$(awk -v s="${seq}.bag" '$1 == s {sub("md5:", "", $3); print $3}' "${D}/pastel_files.txt")
  [[ -n ${md5} ]] || { echo "${seq}: not in the file list"; exit 1; }
  if ! [[ -f ${B}/runs_h8/${seq}/glim_r3/exit.txt ]]; then
    bag=${DATA}/${seq}/bag
    for k in 1 2 3 4 5; do
      [[ -f ${DATA}/${seq}/ok ]] && break
      rm -rf "${DATA:?}/${seq:?}"; mkdir -p "${DATA}/${seq}" "${B}/runs_h8/${seq}"
      echo "$(date -u +%FT%TZ) ${seq}: streaming (attempt ${k})"
      curl -sSL --fail --retry 3 "https://zenodo.org/api/records/22145476/files/${seq}.bag/content" |
        tee >(md5sum | cut -d' ' -f1 > "${DATA}/${seq}/md5.txt") |
        (cd /tmp && "${PY}" -I "${S}/p8_ros1_stream_to_ros2.py" - "${bag}" --lidar /os0_128/points_filtered \
          --imu /os1_16/imu --drop-zero --keep x,y,z,intensity,t,reflectivity,ring,ambient,range \
          --summary "${DATA}/${seq}/summary.json" > "${DATA}/${seq}/convert.log" 2>&1)
      sleep 2
      if [[ $(cat "${DATA}/${seq}/md5.txt" 2>/dev/null) == "${md5}" && -s ${DATA}/${seq}/summary.json ]]; then
        touch "${DATA}/${seq}/ok"
      else
        echo "${seq}: md5 $(cat "${DATA}/${seq}/md5.txt" 2>/dev/null) != ${md5}"; sleep 300
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
  rm -rf "${DATA:?}/${seq:?}"
done
echo "H8 done"
