#!/usr/bin/env bash
# p8_blind_h9.sh [seq ...] (bench_report/data/blind_v7_run2_2026-11-10/preregistration_h9.txt): the H9 holdouts
# (Odyssey). For each sequence: stream <seq>.tar.gz from the dataset's share link into
# p8_odyssey_stream_to_ros2.py (ouster/*.bin and m300/imu.txt only; refsys/ and metadata/ are never read), check that
# the whole archive was read (its length), run every method three times (p8_blind_method.sh), then delete the
# converted bag. A converted bag above 9 GB is not run (selection_h9.txt). The converted bag is written to /dev/shm.
# Outputs: /home/user/bench/blind2/runs_h9/<seq>/<method>_r<k>/, one line per run in /home/user/bench/blind2/runs_h9.csv.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; D=${REPO}/bench_report/data/blind_v7_run2_2026-11-10
PY=/home/user/mapshd/venv/bin/python
PY7=${PY7:-/home/user/venvs/blind2/bin/python}
B=/home/user/bench/blind2; DATA=/dev/shm/blind2_h9; RIG=${D}/rig_odyssey.yaml
METHODS=${METHODS:-G rko_upstream bievr fastlio glim}
mkdir -p "${DATA}" "${B}/runs_h9"
seqs=("$@")
[[ ${#seqs[@]} -gt 0 ]] || mapfile -t seqs < <(grep -v '^#' "${D}/h9_sequences.txt" | awk '{print $1}')
for seq in "${seqs[@]}"; do
  bytes=$(awk -v s="${seq}.tar.gz" '$1 == s {print $2}' "${D}/odyssey_archives.txt")
  url=$(awk -v s="${seq}.tar.gz" '$1 == s {print $3}' "${D}/odyssey_archives.txt")
  [[ -n ${url} ]] || { echo "${seq}: not in the archive list"; exit 1; }
  if ! [[ -f ${B}/runs_h9/${seq}/glim_r3/exit.txt || -f ${B}/runs_h9/${seq}/too_large ]]; then
    bag=${DATA}/${seq}/bag
    for k in 1 2 3; do
      [[ -f ${DATA}/${seq}/ok ]] && break
      rm -rf "${DATA:?}/${seq:?}"; mkdir -p "${DATA}/${seq}" "${B}/runs_h9/${seq}"
      echo "$(date -u +%FT%TZ) ${seq}: streaming (attempt ${k})"
      "${PY7}" -I "${S}/p8_http_stream.py" "${url}" 2> "${DATA}/${seq}/stream.err" |
        (cd /tmp && "${PY}" -I "${S}/p8_odyssey_stream_to_ros2.py" - "${bag}" --summary "${DATA}/${seq}/summary.json" \
           > "${DATA}/${seq}/convert.log" 2>&1)
      got=$("${PY}" -I -c 'import json, sys; d = json.load(open(sys.argv[1])); print(d["bytes_read"] if d["complete"] else 0, d["written"]["/ouster/points"]["messages"])' "${DATA}/${seq}/summary.json" 2>/dev/null)
      if [[ ${got%% *} == "${bytes}" && ${got##* } -gt 0 ]]; then
        touch "${DATA}/${seq}/ok"
      else
        echo "${seq}: conversion failed (${got}; expected ${bytes} bytes): $(tail -n 2 "${DATA}/${seq}/convert.log")"; sleep 60
      fi
    done
    [[ -f ${DATA}/${seq}/ok ]] || { echo "${seq}: conversion failed three times"; exit 1; }
    cp "${DATA}/${seq}/summary.json" "${B}/runs_h9/${seq}/bag_summary.json"
    kb=$(du -sk "${bag}" | cut -f1)
    if (( kb > 9 * 1024 * 1024 )); then
      echo "${seq}: converted bag ${kb} kB, above 9 GB: not run"; echo "${kb}" > "${B}/runs_h9/${seq}/too_large"
    else
      for m in ${METHODS}; do
        for r in 1 2 3; do
          out=${B}/runs_h9/${seq}/${m}_r${r}
          [[ -f ${out}/exit.txt ]] && continue
          line=$(bash "${S}/p8_blind_method.sh" "${m}" "${RIG}" "${bag}" "${out}" 2>>"${B}/runs_h9/${seq}/errors.log" | tail -1)
          echo "${seq},${m},${r},$(date -u +%FT%TZ),${line}" | tee -a "${B}/runs_h9.csv"
        done
      done
    fi
  fi
  rm -rf "${DATA:?}/${seq:?}"
done
echo "H9 done"
