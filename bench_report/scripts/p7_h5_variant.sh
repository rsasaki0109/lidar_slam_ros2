#!/usr/bin/env bash
# p7_h5_variant.sh <label> <rko_lio config> [short ...] (report chapter 60): one run of RKO-LIO with another algorithm
# file on the opened H5 missions of the first blind run (bench_report/data/blind_v7_2026-11-08/h5_missions.txt), the
# same HS rig keys as there, scored at once against the prism reference (opened 2026-11-08). Not a blind run.
# Outputs: /home/user/bench/loss/<label>/<short>/ and one line per mission in /home/user/bench/loss/<label>/scores.csv.
set -uo pipefail
label=$1; cfg=$(realpath "$2"); shift 2
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
B=/home/user/bench/blind_v7; DATA=/home/user/data/grandtour/blind; O=/home/user/bench/loss/${label}
HF=https://huggingface.co/datasets/leggedrobotics/grand_tour_dataset/resolve/main
mkdir -p "${O}"
want=" $* "
while IFS=$'\t' read -r folder short site <&3; do
  [[ $# -gt 0 && ${want} != *" ${short} "* ]] && continue
  D=${DATA}/${short}; BAG=${D}/bag_time
  if [[ ! -d ${BAG} ]]; then
    mkdir -p "${D}/z"
    for f in hesai_points stim320_imu; do
      for k in 1 2 3; do
        [[ -f ${D}/z/${f}/.complete ]] && break
        rm -rf "${D}/z.part" "${D}/z/${f}"; mkdir -p "${D}/z.part"
        curl -sS -L --fail "${HF}/${folder}/data/${f}.tar" | tar xf - -C "${D}/z.part" && mv "${D}/z.part/${f}" "${D}/z/${f}" \
          && touch "${D}/z/${f}/.complete"
        rm -rf "${D}/z.part"
      done
    done
    set +u; source /opt/ros/jazzy/setup.bash; set -u
    "${PY}" "${S}/p1_grandtour_to_ros2.py" --imu stim320_imu --lidar hesai_points --time-field time --consume "${D}/z" \
      "${BAG}.part" > "${D}/convert.log" 2>&1 && mv "${BAG}.part" "${BAG}"
    find "${D}/z" -type f ! -name '.z*' -delete 2>/dev/null || true
  fi
  mkdir -p "${O}/${short}"
  "${PY}" - "${cfg}" "${B}/runs/${short}/g_params.yaml" "${B}/runs/${short}/rig.yaml" "${O}/${short}" <<'PY'
import sys
import yaml
cfg, base, rig, out = sys.argv[1:]
alg = yaml.safe_load(open(cfg))['/**']['ros__parameters']
p = yaml.safe_load(open(base))['/**']['ros__parameters']
g12 = yaml.safe_load(open('/home/user/lidar_slam_ros2/configs/sota_v7/rko_lio_global_G12_2026-10-26.yaml'))['/**']['ros__parameters']
rigkeys = {k: v for k, v in p.items() if k not in g12}  # the rig keys of the blind run
yaml.safe_dump({'/**': {'ros__parameters': {**rigkeys, **alg}}}, open(f'{out}/g_params.yaml', 'w'), sort_keys=False)
r = yaml.safe_load(open(rig))
r['g_params'] = f'{out}/g_params.yaml'
yaml.safe_dump(r, open(f'{out}/rig.yaml', 'w'), sort_keys=False)
PY
  bash "${S}/p5_blind_method.sh" G "${O}/${short}/rig.yaml" "${BAG}" "${O}/${short}/run" | tail -1
  line=$("${PY}" -I "${S}/p1_grandtour_ate.py" "${O}/${short}/run/traj.tum" "/home/user/data/blind_gt/h5/${short}/prism_position")
  echo "${short},$(md5sum < "${O}/${short}/run/traj.tum" | cut -c1-8),${line}" | tee -a "${O}/scores.csv"
  [[ ${KEEP_BAG:-0} == 1 ]] || rm -rf "${BAG}"
done 3< "${B}/h5_missions.txt"
