#!/usr/bin/env bash
# p8_blind_h9_reference.sh [seq ...]: after sealing and the maintainer's approval (preregistration_h9.txt), stream each
# H9 archive again and keep only refsys/lidar_poses_ref.txt and the LiDAR stamps (p8_odyssey_fetch_reference.py), in
# /home/user/data/blind2/odyssey/gt/<seq>/. The archive's length and sha256 must equal the converted bag's summary.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; D=${REPO}/bench_report/data/blind_v7_run2_2026-11-10
PY=/home/user/mapshd/venv/bin/python; PY7=${PY7:-/home/user/venvs/blind2/bin/python}
GT=/home/user/data/blind2/odyssey/gt; B=/home/user/bench/blind2
seqs=("$@")
[[ ${#seqs[@]} -gt 0 ]] || mapfile -t seqs < <(grep -v '^#' "${D}/h9_sequences.txt" | awk '{print $1}')
for seq in "${seqs[@]}"; do
  [[ -f ${GT}/${seq}/ok ]] && continue
  url=$(awk -v s="${seq}.tar.gz" '$1 == s {print $3}' "${D}/odyssey_archives.txt")
  for k in 1 2 3; do
    rm -rf "${GT:?}/${seq:?}"; mkdir -p "${GT}/${seq}"
    echo "$(date -u +%FT%TZ) ${seq}: streaming for the reference (attempt ${k})"
    "${PY7}" -I "${S}/p8_http_stream.py" "${url}" 2> "${GT}/${seq}/stream.err" |
      (cd /tmp && "${PY}" -I "${S}/p8_odyssey_fetch_reference.py" "${GT}/${seq}")
    a=$(grep -o '"sha256": "[0-9a-f]*"' "${GT}/${seq}/fetch.json" 2>/dev/null)
    b=$(grep -o '"sha256": "[0-9a-f]*"' "${B}/runs_h9/${seq}/bag_summary.json")
    if [[ -n ${a} && ${a} == "${b}" ]]; then touch "${GT}/${seq}/ok"; break; fi
    echo "${seq}: archive differs from the converted one or the stream failed (${a} / ${b})"; sleep 60
  done
  [[ -f ${GT}/${seq}/ok ]] || exit 1
done
echo "H9 reference done"
