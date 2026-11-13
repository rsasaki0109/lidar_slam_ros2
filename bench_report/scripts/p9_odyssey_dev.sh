#!/usr/bin/env bash
# p9_odyssey_dev.sh SEQ VARIANT... (bench_report/data/odyssey_dev_2026-11-13/plan.txt): stream and convert one Odyssey
# drive as in H9 (p8_blind_h9.sh), run the fork once per variant (p9_odyssey_dev_variants.py) and delete the bag.
# Outputs: /home/user/bench/odyssey_dev/<seq>/<variant>/, one line per run in /home/user/bench/odyssey_dev/runs.csv.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; D=${REPO}/bench_report/data/blind_v7_run2_2026-11-10
PY=/home/user/mapshd/venv/bin/python; PY7=${PY7:-/home/user/venvs/blind2/bin/python}
B=/home/user/bench/odyssey_dev; DATA=/dev/shm/odyssey_dev
seq=$1; shift
mkdir -p "${B}/variants" "${B}/${seq}" "${DATA}"
"${PY}" "${S}/p9_odyssey_dev_variants.py" "${D}/g_params_odyssey.yaml" "${D}/rig_odyssey.yaml" "${B}/variants"
todo=(); for v in "$@"; do [[ -f ${B}/${seq}/${v}/exit.txt ]] || todo+=("${v}"); done
[[ ${#todo[@]} -gt 0 ]] || { echo "${seq}: nothing to do"; exit 0; }
bytes=$(awk -v s="${seq}.tar.gz" '$1 == s {print $2}' "${D}/odyssey_archives.txt")
url=$(awk -v s="${seq}.tar.gz" '$1 == s {print $3}' "${D}/odyssey_archives.txt")
bag=${DATA}/${seq}/bag
for k in 1 2 3; do
  [[ -f ${DATA}/${seq}/ok ]] && break
  rm -rf "${DATA:?}/${seq:?}"; mkdir -p "${DATA}/${seq}"
  echo "$(date -u +%FT%TZ) ${seq}: streaming (attempt ${k})"
  "${PY7}" -I "${S}/p8_http_stream.py" "${url}" 2> "${DATA}/${seq}/stream.err" |
    (cd /tmp && "${PY}" -I "${S}/p8_odyssey_stream_to_ros2.py" - "${bag}" --summary "${DATA}/${seq}/summary.json" \
       > "${DATA}/${seq}/convert.log" 2>&1)
  got=$("${PY}" -I -c 'import json, sys; d = json.load(open(sys.argv[1])); print(d["bytes_read"] if d["complete"] else 0)' "${DATA}/${seq}/summary.json" 2>/dev/null)
  [[ ${got} == "${bytes}" ]] && touch "${DATA}/${seq}/ok" || { echo "${seq}: conversion failed (${got})"; sleep 60; }
done
[[ -f ${DATA}/${seq}/ok ]] || exit 1
for v in "${todo[@]}"; do
  line=$(bash "${S}/p8_blind_method.sh" G "${B}/variants/rig_${v}.yaml" "${bag}" "${B}/${seq}/${v}" 2>>"${B}/${seq}/errors.log" | tail -1)
  echo "${seq},${v},$(date -u +%FT%TZ),${line}" | tee -a "${B}/runs.csv"
done
rm -rf "${DATA:?}/${seq:?}"
echo "${seq} done"
