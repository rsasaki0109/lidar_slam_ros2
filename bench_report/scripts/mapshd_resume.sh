#!/usr/bin/env bash
# MapsHD HUMANOID-LIO-DATASET, every trial of the remaining scenarios, one at a time: download the scenario's
# archive from zenodo record 21318128 (md5 checked), extract the trials only (no TLS cloud), move each trial's
# ground truth to gt_sealed/ unread (sha256 recorded), then per trial: convert (mapshd_to_ros2.py), run A and B
# (mapshd_runs.sh), delete the bag and the raw data. No ground truth is read here.
# Usage: mapshd_resume.sh
# Resume after the offline-node deadlock fix (rko_lio 40b3a06): underground-garage trial-1 (bag kept), its
# trials 2 and 3 (raw kept), then outdoor-urk, with the overlay pipe_dev_40b3a06.
set -uo pipefail
D=/home/user/data/mapshd; OUT=/home/user/bench/mapshd; VPY=/home/user/mapshd/venv/bin/python
export WS=/home/user/ws/pipe_dev_40b3a06
set +u; source /opt/ros/jazzy/setup.bash; set -u
declare -A MD5=([flat-room]=b698db851aba5abfc0e5f63bd424bfa9 [long-corridor]=aebeeab1f6805dd03b821ea33e2d375a
                [underground-garage]=09da4a85a01d406ae97a3b651d6d8cd2 [outdoor-urk]=e747e46e39ce18cdec25e818b9dde813)

process_trial() {
  local sc=$1 t=$2 raw=$D/x/$1/$2/hdmapping-raw bag=$D/bags/$1_$2
  rm -rf "${bag}"
  if ! /usr/bin/time -v -o "$D/bags/convert_${sc}_${t}.time" "${VPY}" /home/user/scripts/mapshd_to_ros2.py "${raw}" "${bag}" \
      > "$D/bags/convert_${sc}_${t}.log" 2>&1; then
    echo "CONVERSION FAILED ${sc} ${t}"; tail -3 "$D/bags/convert_${sc}_${t}.log"; rm -rf "${bag}"; return
  fi
  echo "${sc} ${t}: $(tail -1 "$D/bags/convert_${sc}_${t}.log") ($(du -sh "${bag}" | cut -f1))"
  rm -rf "$D/x/${sc}/${t}"
  bash /home/user/scripts/mapshd_runs.sh "${bag}" "${sc}_${t}" "${OUT}"
  rm -f "${OUT}/${sc}_${t}_A/map.pcd" "${OUT}/${sc}_${t}_B/map.pcd"
  rm -rf "${bag}"
  df -h / | tail -1
}

# underground-garage trial-1: the bag is still there; skip the conversion.
bag=$D/bags/underground-garage_trial-1
rm -rf "${OUT}/underground-garage_trial-1_A" "${OUT}/underground-garage_trial-1_A.time"
bash /home/user/scripts/mapshd_runs.sh "${bag}" underground-garage_trial-1 "${OUT}"
rm -f "${OUT}/underground-garage_trial-1_A/map.pcd" "${OUT}/underground-garage_trial-1_B/map.pcd"
rm -rf "${bag}"
for t in trial-2 trial-3; do process_trial underground-garage "${t}"; done
rm -rf "$D/x/underground-garage"
for sc in outdoor-urk; do
  mkdir -p "$D/dl"
  if ! curl -sS -L --fail -m 3600 -o "$D/dl/${sc}.7z" "https://zenodo.org/records/21318128/files/${sc}.7z?download=1"; then
    echo "DOWNLOAD FAILED ${sc}"; continue
  fi
  got=$(md5sum "$D/dl/${sc}.7z" | cut -d' ' -f1)
  [[ ${got} == "${MD5[$sc]}" ]] || { echo "MD5 MISMATCH ${sc} ${got}"; rm -f "$D/dl/${sc}.7z"; continue; }
  (cd "$D/x" && "${VPY}" -I -c "
import py7zr, sys
with py7zr.SevenZipFile(sys.argv[1]) as z:
    names = [n for n in z.getnames() if '/trial-' in n]
with py7zr.SevenZipFile(sys.argv[1]) as z:
    z.extract(path='.', targets=names)
" "$D/dl/${sc}.7z")
  rm -f "$D/dl/${sc}.7z"
  for tdir in "$D/x/${sc}"/trial-*; do
    t=$(basename "${tdir}")
    mkdir -p "$D/gt_sealed/${sc}/${t}"
    mv "${tdir}/ground-truth/"* "$D/gt_sealed/${sc}/${t}/" && rmdir "${tdir}/ground-truth"
  done
  sha256sum "$D/gt_sealed/${sc}"/*/* > "$D/gt_sealed/SHA256SUMS_${sc}"
  echo "${sc}: $(ls -d "$D/x/${sc}"/trial-* | wc -l) trials, $(du -sh "$D/x/${sc}" | cut -f1) raw"
  for tdir in "$D/x/${sc}"/trial-*; do
    process_trial "${sc}" "$(basename "${tdir}")"
  done
  rm -rf "$D/x/${sc}"
done
echo "all trials done $(date -u +%T)"
