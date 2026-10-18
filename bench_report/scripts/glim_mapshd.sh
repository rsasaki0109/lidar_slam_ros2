#!/usr/bin/env bash
# GLIM CPU on MapsHD calibration-room trial-1 (the trial of the benchmark's README table) with the settings the
# benchmark's GLIM wrapper effectively ran (params/glim_mapshd_benchmark/: CPU odometry, sub-mapping and global
# mapping, /livox topics, acc_scale 9.80665; T_lidar_imu stays at GLIM's default, as the wrapper's sed edits only a
# comment line). Re-downloads the archive (md5 checked), converts trial-1, runs GLIM once, keeps the dump's text files.
# No ground truth is read here (the archive's copy is deleted unread; gt_sealed/ already holds it).
set -uo pipefail
D=/home/user/data/mapshd; OUT=/home/user/bench/mapshd; VPY=/home/user/mapshd/venv/bin/python
set +u; source /opt/ros/jazzy/setup.bash; set -u
mkdir -p "$D/dl" "$D/x"
curl -sS -L --fail -m 3600 -o "$D/dl/calibration-room.7z" "https://zenodo.org/records/21318128/files/calibration-room.7z?download=1"
[[ $(md5sum "$D/dl/calibration-room.7z" | cut -d' ' -f1) == f965681f362701e48b4631e95e54bd10 ]] || { echo "MD5 MISMATCH"; exit 1; }
(cd "$D/x" && "${VPY}" -I -c "
import py7zr, sys
with py7zr.SevenZipFile(sys.argv[1]) as z:
    names = [n for n in z.getnames() if '/trial-1/hdmapping-raw' in n]
with py7zr.SevenZipFile(sys.argv[1]) as z:
    z.extract(path='.', targets=names)
" "$D/dl/calibration-room.7z")
rm -f "$D/dl/calibration-room.7z"
bag=$D/bags/calibration-room_trial-1
rm -rf "${bag}"
"${VPY}" /home/user/scripts/mapshd_to_ros2.py "$D/x/calibration-room/trial-1/hdmapping-raw" "${bag}" | tail -1
rm -rf "$D/x/calibration-room"
dd if="$(ls "${bag}"/*.db3)" of=/dev/null bs=16M 2>&1 | tail -1
bash /home/user/scripts/kill_ros_procs.sh > /dev/null 2>&1
GLIM_OVERRIDES=/home/user/bench/params/glim_mapshd_benchmark bash /home/user/scripts/glim_run.sh "${OUT}/glim_calibration-room_trial-1" "${bag}"
find "${OUT}/glim_calibration-room_trial-1/dump" -mindepth 1 -maxdepth 1 -type d -exec rm -rf {} +
rm -rf "${bag}"
echo "glim done $(date -u +%T)"
