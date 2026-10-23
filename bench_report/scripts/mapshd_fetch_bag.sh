#!/usr/bin/env bash
# Usage: mapshd_fetch_bag.sh <scenario> [trial ...]
# Downloads one MapsHD scenario (zenodo record 21318128, md5 checked), extracts its trials without the ground truth,
# converts each to /home/user/data/mapshd/bags/<scenario>_<trial> and keeps the bag and the raw IMU (chapter 29: several
# frontend variants on the same bag without downloading it again).
set -uo pipefail
sc=$1; shift
D=/home/user/data/mapshd; VPY=/home/user/mapshd/venv/bin/python
declare -A MD5=([calibration-room]=f965681f362701e48b4631e95e54bd10 [flat-room]=b698db851aba5abfc0e5f63bd424bfa9
                [long-corridor]=aebeeab1f6805dd03b821ea33e2d375a [underground-garage]=09da4a85a01d406ae97a3b651d6d8cd2
                [outdoor-urk]=e747e46e39ce18cdec25e818b9dde813)
set +u; source /opt/ros/jazzy/setup.bash; set -u
mkdir -p "$D/dl" "$D/x" "$D/bags"
curl -sS -L --fail -m 3600 -o "$D/dl/${sc}.7z" "https://zenodo.org/records/21318128/files/${sc}.7z?download=1" || exit 1
[[ $(md5sum "$D/dl/${sc}.7z" | cut -d' ' -f1) == "${MD5[$sc]}" ]] || { echo "MD5 MISMATCH ${sc}"; exit 1; }
(cd "$D/x" && "${VPY}" -I -c "
import py7zr, sys
with py7zr.SevenZipFile(sys.argv[1]) as z:
    names = [n for n in z.getnames() if '/trial-' in n and '/ground-truth' not in n]
with py7zr.SevenZipFile(sys.argv[1]) as z:
    z.extract(path='.', targets=names)
" "$D/dl/${sc}.7z")
rm -f "$D/dl/${sc}.7z"
for tdir in "$D/x/${sc}"/trial-*; do
  t=$(basename "${tdir}")
  if [[ $# -gt 0 && " $* " != *" ${t} "* ]]; then rm -rf "${tdir:?}"; continue; fi
  bag=$D/bags/${sc}_${t}; rm -rf "${bag}"
  "${VPY}" /home/user/scripts/mapshd_to_ros2.py "${tdir}/hdmapping-raw" "${bag}" > "$D/bags/convert_${sc}_${t}.log" 2>&1 \
    || { echo "CONVERSION FAILED ${sc} ${t}"; continue; }
  find "${tdir}/hdmapping-raw" -name '*.laz' -delete
  echo "${sc} ${t}: $(du -sh "${bag}" | cut -f1)"
done
df -h / | tail -1
