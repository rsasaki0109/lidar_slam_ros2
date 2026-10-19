#!/usr/bin/env bash
# Garage trial-3 frontend-only variants for the along-corridor scale error: download the scenario archive (retries,
# md5), extract trial-3 without its ground truth, convert, run every variant on top of the adopted frontend
# (max_expected_jerk: 1000), score APE and Sim(3) scale. Usage: scale_sweep_garage3.sh <csv> "<overrides>" ...
set -uo pipefail
csv=$1; shift
D=/home/user/data/mapshd; VPY=/home/user/mapshd/venv/bin/python; OUT=/home/user/bench/scale
set +u; source /opt/ros/jazzy/setup.bash; set -u
mkdir -p "$D/dl" "$D/x" "$D/bags" "${OUT}"
bag=$D/bags/underground-garage_trial-3
if [[ ! -d ${bag} ]]; then
  ok=0
  for wait in 2 4 8 16 0; do
    curl -sS -L --fail -m 3600 -o "$D/dl/underground-garage.7z" \
      "https://zenodo.org/records/21318128/files/underground-garage.7z?download=1" && { ok=1; break; }
    echo "download attempt failed, retry in ${wait} s"; sleep "${wait}"
  done
  [[ ${ok} == 1 ]] || { echo "DOWNLOAD FAILED"; exit 1; }
  [[ $(md5sum "$D/dl/underground-garage.7z" | cut -d' ' -f1) == 09da4a85a01d406ae97a3b651d6d8cd2 ]] || { echo "MD5 MISMATCH"; exit 1; }
  (cd "$D/x" && "${VPY}" -I -c "
import py7zr, sys
with py7zr.SevenZipFile(sys.argv[1]) as z:
    names = [n for n in z.getnames() if '/trial-3' in n and '/ground-truth' not in n]
with py7zr.SevenZipFile(sys.argv[1]) as z:
    z.extract(path='.', targets=names)
" "$D/dl/underground-garage.7z")
  rm -f "$D/dl/underground-garage.7z"
  if ! "${VPY}" /home/user/scripts/mapshd_to_ros2.py "$D/x/underground-garage/trial-3/hdmapping-raw" "${bag}" \
      > "${OUT}/convert_garage3.log" 2>&1; then
    echo "CONVERSION FAILED"; tail -2 "${OUT}/convert_garage3.log"; rm -rf "${bag:?}"; exit 1
  fi
  rm -rf "${D:?}/x/underground-garage"
fi
dd if="$(ls "${bag}"/*.db3)" of=/dev/null bs=16M 2>&1 | tail -1
PARAMS=/home/user/bench/params/mapshd_gyro002_jerk1000.yaml bash /home/user/scripts/front_sweep_scale.sh "${csv}" "${bag}" \
  "$D/gt_sealed/underground-garage/trial-3/ground_truth.tum" "${OUT}/garage3" "$@"
echo "scale sweep done $(date -u +%T)"
