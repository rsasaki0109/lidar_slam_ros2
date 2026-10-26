#!/usr/bin/env bash
# p1_mapshd_get.sh <scenario> (chapter 30): download one MapsHD scenario (zenodo record 21318128, md5 checked), extract
# its trials (no TLS cloud), copy each trial's ground_truth.tum to gt/<scenario>_<trial>/ and convert each trial to a
# zstd-chunked MCAP bag bags/<scenario>_<trial>_c (BAG_STORAGE=mcap, about 0.75x the sqlite3 size). The archive and the
# raw trial data are removed only when every trial converted.
set -uo pipefail
sc=$1
D=/home/user/data/mapshd; VPY=/home/user/mapshd/venv/bin/python
declare -A MD5=([calibration-room]=f965681f362701e48b4631e95e54bd10 [flat-room]=b698db851aba5abfc0e5f63bd424bfa9
                [long-corridor]=aebeeab1f6805dd03b821ea33e2d375a [underground-garage]=09da4a85a01d406ae97a3b651d6d8cd2
                [outdoor-urk]=e747e46e39ce18cdec25e818b9dde813)
set +u; source /opt/ros/jazzy/setup.bash; set -u
mkdir -p "$D/dl" "$D/x" "$D/bags" "$D/gt"
f="$D/dl/${sc}.7z"
[[ -f $f ]] || curl -sS -L --fail -m 3600 -o "$f" "https://zenodo.org/records/21318128/files/${sc}.7z?download=1" || exit 1
[[ $(md5sum "$f" | cut -d' ' -f1) == "${MD5[$sc]}" ]] || { echo "MD5 MISMATCH ${sc}"; exit 1; }
(cd "$D/x" && "${VPY}" -I -c "
import py7zr, sys
with py7zr.SevenZipFile(sys.argv[1]) as z:
    names = [n for n in z.getnames() if '/trial-' in n]
with py7zr.SevenZipFile(sys.argv[1]) as z:
    z.extract(path='.', targets=names)
" "$f") || exit 1
fail=0
for tdir in "$D/x/${sc}"/trial-*; do
  t=$(basename "${tdir}")
  mkdir -p "$D/gt/${sc}_${t}"; cp "${tdir}/ground-truth/ground_truth.tum" "$D/gt/${sc}_${t}/"
  bag=$D/bags/${sc}_${t}_c; rm -rf "${bag}"
  BAG_STORAGE=mcap "${VPY}" /home/user/scripts/mapshd_to_ros2.py "${tdir}/hdmapping-raw" "${bag}" \
    > "$D/bags/convert_${sc}_${t}.log" 2>&1 || { echo "CONVERSION FAILED ${sc} ${t}"; fail=1; continue; }
  rm -rf "${tdir}"
  echo "${sc} ${t}: $(du -sh "${bag}" | cut -f1)"
done
[[ ${fail} == 0 ]] && { rm -rf "$D/x/${sc}"; rm -f "$f"; }
df -h / | tail -1
