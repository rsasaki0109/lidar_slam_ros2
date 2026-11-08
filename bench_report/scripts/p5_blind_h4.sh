#!/usr/bin/env bash
# p5_blind_h4.sh [seq ...] (bench_report/data/blind_v7_2026-11-08/preregistration.txt): the H4 holdouts (HILTI 2022
# exp14, exp16, exp18 by default). For each sequence: stream the raw bag from the frozen Hugging Face revision into
# p5_ros1_stream_to_ros2.py (LiDAR + IMU only, never stored), check the sha256 of the streamed bytes against the selection
# file, run every method three times (p5_blind_method.sh), then delete the converted bag. No ground-truth file is read.
# Outputs: /home/user/bench/blind_v7/runs/<seq>/<method>_r<k>/, one line per run in /home/user/bench/blind_v7/runs.csv.
set -uo pipefail
S=/home/user/lidar_slam_ros2/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
SEL=/home/user/lidar_slam_ros2/configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml
REV=e62017f907007fdc5ab8c721842e4ae7359d7f49
B=/home/user/bench/blind_v7; RIG=${B}/configs/rig_h4.yaml; DATA=/home/user/data/hilti2022/blind
METHODS=${METHODS:-G rko_upstream bievr fastlio glim}
mkdir -p "${DATA}" "${B}/runs"
seqs=("$@"); [[ ${#seqs[@]} -gt 0 ]] || seqs=(exp14 exp16 exp18)
for seq in "${seqs[@]}"; do
  read -r url sha < <("${PY}" -c "
import sys, yaml
s = yaml.safe_load(open(sys.argv[1]))['selection_decision']['selected_candidates']
c = next(v for v in s.values() if v['sequence'] == sys.argv[2])
print(c['bag']['url'].replace('/blob/', '/resolve/'), c['bag']['lfs_sha256'])" "${SEL}" "${seq}")
  [[ ${url} == *"/${REV}/"* ]] || { echo "${seq}: unexpected revision in ${url}"; exit 1; }
  bag=${DATA}/${seq}/bag
  # the converted bag keeps about 37 % of the raw bytes; wait until that and 1.5 GB more are free
  need=$("${PY}" -c "
import sys, yaml
s = yaml.safe_load(open(sys.argv[1]))['selection_decision']['selected_candidates']
c = next(v for v in s.values() if v['sequence'] == sys.argv[2])
print(int(c['bag']['expected_bytes'] * 0.37 / 1024) + 1536 * 1024)" "${SEL}" "${seq}")
  until [[ -f ${DATA}/${seq}/ok ]] || (( $(df -k --output=avail /home/user | tail -1) > need )); do
    echo "$(date -u +%FT%TZ) ${seq}: waiting for ${need} KiB of free disk"; sleep 300
  done
  for k in 1 2 3; do
    [[ -f ${DATA}/${seq}/ok ]] && break
    rm -rf "${DATA}/${seq}"; mkdir -p "${DATA}/${seq}"
    echo "$(date -u +%FT%TZ) ${seq}: streaming ${url} (attempt ${k})"
    curl -sSL --fail "${url}" | (cd /tmp && "${PY}" -I "${S}/p5_ros1_stream_to_ros2.py" - "${bag}" --lidar /hesai/pandar \
      --imu /alphasense/imu --time-field time --summary "${DATA}/${seq}/summary.json" > "${DATA}/${seq}/convert.log" 2>&1)
    got=$("${PY}" -c "import json, sys; print(json.load(open(sys.argv[1]))['sha256'])" "${DATA}/${seq}/summary.json" 2>/dev/null)
    if [[ ${got} == "${sha}" ]]; then touch "${DATA}/${seq}/ok"; else echo "${seq}: sha256 ${got:-none} != ${sha}"; sleep 60; fi
  done
  [[ -f ${DATA}/${seq}/ok ]] || { echo "${seq}: conversion failed three times"; exit 1; }
  mkdir -p "${B}/runs/${seq}"; cp "${DATA}/${seq}/summary.json" "${B}/runs/${seq}/bag_summary.json"
  for m in ${METHODS}; do
    for r in 1 2 3; do
      out=${B}/runs/${seq}/${m}_r${r}
      [[ -f ${out}/exit.txt ]] && continue
      line=$(bash "${S}/p5_blind_method.sh" "${m}" "${RIG}" "${bag}" "${out}" 2>>"${B}/runs/${seq}/errors.log" | tail -1)
      echo "${seq},${m},${r},$(date -u +%FT%TZ),${line}" | tee -a "${B}/runs.csv"
    done
  done
  rm -rf "${bag}"
done
echo "H4 done"
