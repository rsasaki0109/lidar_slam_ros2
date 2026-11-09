#!/usr/bin/env bash
# p5_blind_h5.sh (bench_report/data/blind_v7_2026-11-08/preregistration.txt): the H5 holdouts, GrandTour train missions
# whose references were never read. Step 1 picks one mission per site (the name before " - " in missions.json), the
# earliest by date_time whose data folder lists hesai_points.tar, stim320_imu.tar and prism_position.tar (file names
# only; nothing of prism_position is downloaded). Step 2, per mission in that order: download hesai_points and stim320_imu
# only, convert them (p1_grandtour_to_ros2.py, per-point `time`), write the rig file (STIM320 as base, the export's LiDAR
# extrinsic, T_imu_prism) and G's parameters (the HS rig keys plus G12), run every method three times
# (p5_blind_method.sh), then delete the bag and the point arrays (the zarr metadata with the extrinsics stay).
# Outputs: /home/user/bench/blind_v7/runs/<short>/<method>_r<k>/, rows in /home/user/bench/blind_v7/runs.csv,
# the missions picked in /home/user/bench/blind_v7/h5_missions.txt.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
B=/home/user/bench/blind_v7; DATA=/home/user/data/grandtour/blind
HF=https://huggingface.co/datasets/leggedrobotics/grand_tour_dataset
METHODS=${METHODS:-G rko_upstream bievr fastlio glim}
PRISM="0.3088378015503127 -0.038312997211241984 -0.2349441159711169 -0.005724388155828644 -0.0013879431347362404 -0.004742721620119043 0.9999714053840392"
mkdir -p "${DATA}" "${B}/runs"
# the 22 development, validation and test missions (bench_report/data/grandtour/*.txt)
USED="2024-11-02-17-10-25 2024-11-02-17-18-32 2024-11-02-17-43-10 2024-11-02-21-12-51 2024-11-03-07-52-45
2024-11-03-07-57-34 2024-11-03-08-17-23 2024-11-03-13-51-43 2024-11-03-13-59-54 2024-11-11-12-42-47 2024-11-14-13-45-37
2024-11-18-12-05-01 2024-11-18-13-22-14 2024-11-18-13-48-19 2024-11-18-15-46-05 2024-11-18-16-59-23 2024-11-18-17-13-09
2024-11-18-17-31-36 2024-12-09-09-34-43 2024-12-09-09-41-46 2024-12-09-11-28-28 2024-12-09-11-53-11"
if [[ ! -s ${B}/h5_missions.txt ]]; then
  "${PY}" - /home/user/data/grandtour/missions.json "${USED}" "${HF}" > "${B}/h5_missions.txt" <<'PY'
import json
import sys
import time
import urllib.request

missions = json.load(open(sys.argv[1]))
used = set(sys.argv[2].split())
hf = sys.argv[3].replace('/datasets/', '/api/datasets/')
sites = {}
for m in sorted((m for m in missions if m['date_time'] not in used), key=lambda m: m['date_time']):
    sites.setdefault(m['name'].split(' - ')[0], []).append(m)
need = {'hesai_points.tar', 'stim320_imu.tar', 'prism_position.tar'}
for site in sorted(sites, key=lambda s: sites[s][0]['date_time']):
    for m in sites[site]:
        with urllib.request.urlopen(f"{hf}/tree/main/{m['date_time']}/data") as r:
            names = {e['path'].rsplit('/', 1)[-1] for e in json.load(r)}
        time.sleep(1)
        if need <= names:
            short = m['short_name'].replace('Ö', 'OE').replace('Ä', 'AE').replace('Ü', 'UE')
            print(m['date_time'], short.lower(), site, sep='\t')
            break
        print(f"# {m['date_time']} {m['short_name']}: missing {sorted(need - names)}", file=sys.stderr)
PY
fi
cat "${B}/h5_missions.txt"
# the list is read on descriptor 3: the runs (ros2 bag play, curl) must not consume it from standard input
while IFS=$'\t' read -r folder short site <&3; do
  [[ -z ${folder} || ${folder} == \#* ]] && continue
  D=${DATA}/${short}; O=${B}/runs/${short}; BAG=${D}/bag_time
  mkdir -p "${D}/z" "${O}"
  U=${HF}/resolve/main/${folder}/data
  if [[ ! -d ${BAG} ]]; then
    for f in hesai_points stim320_imu; do
      for k in 1 2 3 4 5 6; do
        [[ -f ${D}/z/${f}/.complete ]] && break
        rm -rf "${D}/z.part" "${D}/z/${f}"; mkdir -p "${D}/z.part"
        if curl -sS -L --fail "${U}/${f}.tar" | tar xf - -C "${D}/z.part"; then
          mv "${D}/z.part/${f}" "${D}/z/${f}"; rm -rf "${D}/z.part"; touch "${D}/z/${f}/.complete"
        else
          echo "${short}: download of ${f} failed (attempt ${k})"; sleep $(( k < 5 ? 60 * 2 ** (k - 1) : 900 ))
        fi
      done
    done
    rm -rf "${BAG}.part"
    set +u; source /opt/ros/jazzy/setup.bash; set -u
    "${PY}" "${S}/p1_grandtour_to_ros2.py" --imu stim320_imu --lidar hesai_points --time-field time --consume "${D}/z" \
      "${BAG}.part" > "${D}/convert.log" 2>&1 && mv "${BAG}.part" "${BAG}"
    find "${D}/z" -type f ! -name '.z*' -delete 2>/dev/null || true
  fi
  [[ -d ${BAG} ]] || { echo "${short}: conversion failed"; continue; }
  tail -1 "${D}/convert.log" > "${O}/convert_summary.txt"
  # shellcheck disable=SC2086
  "${PY}" "${S}/p1_grandtour_to_ros2.py" --imu stim320_imu --lidar hesai_points --extrinsics "${D}/z" ${PRISM} \
    > "${O}/extrinsics.txt"
  "${PY}" - "${REPO}/configs/sota_v7/rko_lio_global_G12_2026-10-26.yaml" "${O}" <<'PY'
import sys
import yaml
g = yaml.safe_load(open(sys.argv[1]))['/**']['ros__parameters']
out = sys.argv[2]
ext = yaml.safe_load(open(f'{out}/extrinsics.txt'))
rig = {'base_frame': 'base_link', 'odom_frame': 'odom', 'lidar_frame': '', 'imu_frame': '',
       'odom_topic': '/rko_lio/odometry', 'publish_deskewed_scan': False, 'dump_results': True, 'run_name': 'front',
       'imu_acceleration_unit': 'auto'}
rig.update({k: v for k, v in ext.items() if k.startswith('extrinsic_')})
yaml.safe_dump({'/**': {'ros__parameters': {**rig, **g}}}, open(f'{out}/g_params.yaml', 'w'), sort_keys=False)
yaml.safe_dump({'lidar_topic': '/hesai_points', 'imu_topic': '/stim320_imu',
                'lidar_to_imu_quat_xyzw_xyz': ext['extrinsic_lidar2base_quat_xyzw_xyz'],
                'imu_to_prism_quat_xyzw_xyz': ext['T_imu_prism_quat_xyzw_xyz'],
                'g_params': f'{out}/g_params.yaml'}, open(f'{out}/rig.yaml', 'w'), sort_keys=False)
PY
  for m in ${METHODS}; do
    for r in 1 2 3; do
      out=${O}/${m}_r${r}
      [[ -f ${out}/exit.txt ]] && continue
      line=$(bash "${S}/p5_blind_method.sh" "${m}" "${O}/rig.yaml" "${BAG}" "${out}" 2>>"${O}/errors.log" | tail -1)
      echo "${short},${m},${r},$(date -u +%FT%TZ),${line}" | tee -a "${B}/runs.csv"
    done
  done
  rm -rf "${BAG}"
done 3< "${B}/h5_missions.txt"
echo "H5 done"
