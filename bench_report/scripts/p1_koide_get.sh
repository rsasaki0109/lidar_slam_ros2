#!/usr/bin/env bash
# p1_koide_get.sh <sequence> (chapter 30.7): download a Koide outdoor_hard bag (zenodo 10122133) if missing, unzip it
# and keep only /livox/points and /livox/imu as a zstd-chunked MCAP (<sequence>_c). gt.zip is fetched separately.
set -uo pipefail
s=$1; D=/home/user/data/koide
set +u; source /opt/ros/jazzy/setup.bash; set -u
cd "$D"
[[ -f ${s}.zip || -d ${s}_c ]] || curl -sS -L --fail -o "${s}.zip" "https://zenodo.org/records/10122133/files/${s}.zip?download=1" || exit 1
[[ -d ${s}_c ]] && { echo "${s} exists"; exit 0; }
unzip -q -o "${s}.zip" && rm -f "${s}.zip"
printf "output_bags:\n- uri: %s\n  storage_id: mcap\n  storage_preset_profile: zstd_fast\n  topics: [/livox/points, /livox/imu]\n" "${D}/${s}_c" > "${D}/${s}.yaml"
ros2 bag convert -i "${s}" -o "${D}/${s}.yaml" > /dev/null 2>&1 || { echo "CONVERT FAILED ${s}"; exit 1; }
rm -rf "${s}" "${D}/${s}.yaml"
echo "${s}: $(du -sh "${s}_c" | cut -f1)"
ros2 bag info "${s}_c" 2>/dev/null | grep -E "Topic:|Count" | head -4
