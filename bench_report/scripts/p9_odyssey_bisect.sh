#!/usr/bin/env bash
# p9_odyssey_bisect.sh SEQ THRESHOLD_M GOOD BAD (plan.txt, addendum 3): stream and convert one Odyssey drive as in H9,
# keep the bag in /dev/shm/odyssey_dev/<seq>/, and bisect the fork (worktree /home/user/ws/rko_bisect/src/rko_lio)
# with p9_odyssey_bisect_step.sh. Log: /home/user/bench/odyssey_dev/bisect.log. The bag is deleted at the end.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; D=${REPO}/bench_report/data/blind_v7_run2_2026-11-10
PY=/home/user/mapshd/venv/bin/python; PY7=${PY7:-/home/user/venvs/blind2/bin/python}
B=/home/user/bench/odyssey_dev; DATA=/dev/shm/odyssey_dev; W=/home/user/ws/rko_bisect/src/rko_lio
seq=$1; thr=$2; good=$3; bad=$4
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
cd "${W}" || exit 1
git bisect reset > /dev/null 2>&1
git bisect start "${bad}" "${good}" > /dev/null
git bisect run bash "${S}/p9_odyssey_bisect_step.sh" "${seq}" "${thr}" | tail -n 25
git bisect log > "${B}/bisect_${seq}_git.log"
git bisect reset > /dev/null 2>&1
rm -rf "${DATA:?}/${seq:?}"
echo "${seq} bisect done"
