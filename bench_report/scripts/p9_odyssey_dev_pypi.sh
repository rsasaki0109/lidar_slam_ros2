#!/usr/bin/env bash
# p9_odyssey_dev_pypi.sh SEQ (bench_report/data/odyssey_dev_2026-11-13/plan.txt, addendum 1): stream and convert one
# Odyssey drive as in H9 and run PyPI rko_lio 0.3.2 (the fork's base) and 0.4.0 (the rival) with their own defaults.
# Outputs: /home/user/bench/odyssey_dev/<seq>/up032|up040/traj_imu.tum, one line per run in runs.csv.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; D=${REPO}/bench_report/data/blind_v7_run2_2026-11-10
PY=/home/user/mapshd/venv/bin/python; PY7=${PY7:-/home/user/venvs/blind2/bin/python}
B=/home/user/bench/odyssey_dev; DATA=/dev/shm/odyssey_dev
seq=$1
mkdir -p "${B}/${seq}" "${DATA}"
[[ -f ${B}/${seq}/up032/exit.txt && -f ${B}/${seq}/up040/exit.txt ]] && { echo "${seq}: nothing to do"; exit 0; }
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
for v in up032 up040; do
  out=${B}/${seq}/${v}
  [[ -f ${out}/exit.txt ]] && continue
  rm -rf "${out}"; mkdir -p "${out}"
  case ${v} in
    up032) venv=/home/user/venvs/rko_032; def=${REPO}/bench_report/data/odyssey_dev_2026-11-13/rko_upstream_0.3.2_default.yaml ;;
    up040) venv=/home/user/venvs/rko_upstream; def=${REPO}/bench_report/data/blind_v7_2026-11-08/rko_upstream_default.yaml ;;
  esac
  "${PY}" -c "
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1]))
cfg['extrinsic_imu2base_quat_xyzw_xyz'] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
cfg['extrinsic_lidar2base_quat_xyzw_xyz'] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
yaml.safe_dump(cfg, open(sys.argv[2], 'w'), sort_keys=False)" "${def}" "${out}/config.yaml"
  t0=$(date +%s.%N)
  (cd "${out}" && /usr/bin/time -v -o "${out}/time.txt" timeout -s INT 7200 "${venv}/bin/rko_lio" -d rosbag \
     --imu /m300/imu --lidar /ouster/points -c "${out}/config.yaml" -o "${out}/results" -n run "${bag}" \
     > "${out}/stdout.log" 2>&1)
  rc=$?
  t1=$(date +%s.%N)
  f=$(find "${out}/results" -name '*.txt' -newer "${out}/config.yaml" 2>/dev/null | head -1)
  [[ -n ${f} ]] && grep -v '^#' "${f}" > "${out}/traj_imu.tum"
  echo "wall_seconds $(python3 -c "print(f'{${t1}-${t0}:.1f}')")" > "${out}/run_info.txt"
  echo "${rc}" > "${out}/exit.txt"
  echo "${seq},${v},$(date -u +%FT%TZ),exit ${rc} $(cat "${out}/run_info.txt") poses $(wc -l < "${out}/traj_imu.tum" 2>/dev/null || echo 0)" | tee -a "${B}/runs.csv"
done
rm -rf "${DATA:?}/${seq:?}"
echo "${seq} pypi done"
