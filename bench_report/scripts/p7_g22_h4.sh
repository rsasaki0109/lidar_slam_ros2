#!/usr/bin/env bash
# p7_g22_h4.sh <label> <rko_lio config> [seq ...] (bench_report/data/blind_v7_2026-11-08/after/g22_plan.txt): one run of
# RKO-LIO with another algorithm file on the opened H4 sequences of the first blind run (HILTI 2022 exp14, exp16,
# exp18), the blind run's rig keys, the bag streamed from the frozen revision with its sha256 check as in
# p5_blind_h4.sh, scored at once against the 6-DoF ground truth opened on 2026-11-08. Not a blind run.
# Outputs: /home/user/bench/loss/<label>/<seq>/ and one line per sequence in /home/user/bench/loss/<label>/scores_h4.csv.
set -uo pipefail
label=$1; cfg=$(realpath "$2"); shift 2
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
SEL=${REPO}/configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml
B=/home/user/bench/blind_v7; DATA=/home/user/data/hilti2022/g22_h4; O=/home/user/bench/loss/${label}
mkdir -p "${O}" "${DATA}"
seqs=("$@"); [[ ${#seqs[@]} -gt 0 ]] || seqs=(exp14 exp16 exp18)
for seq in "${seqs[@]}"; do
  read -r url sha < <("${PY}" -c "
import sys, yaml
s = yaml.safe_load(open(sys.argv[1]))['selection_decision']['selected_candidates']
c = next(v for v in s.values() if v['sequence'] == sys.argv[2])
print(c['bag']['url'].replace('/blob/', '/resolve/'), c['bag']['lfs_sha256'])" "${SEL}" "${seq}")
  bag=${DATA}/${seq}/bag
  for k in 1 2 3; do
    [[ -f ${DATA}/${seq}/ok ]] && break
    rm -rf "${DATA:?}/${seq:?}"; mkdir -p "${DATA}/${seq}"
    curl -sSL --fail "${url}" | (cd /tmp && "${PY}" -I "${S}/p5_ros1_stream_to_ros2.py" - "${bag}" --lidar /hesai/pandar \
      --imu /alphasense/imu --time-field time --summary "${DATA}/${seq}/summary.json" > "${DATA}/${seq}/convert.log" 2>&1)
    got=$("${PY}" -c "import json, sys; print(json.load(open(sys.argv[1]))['sha256'])" "${DATA}/${seq}/summary.json" 2>/dev/null)
    [[ ${got} == "${sha}" ]] && touch "${DATA}/${seq}/ok" || sleep 60
  done
  [[ -f ${DATA}/${seq}/ok ]] || { echo "${seq},FAILED conversion" | tee -a "${O}/scores_h4.csv"; continue; }
  mkdir -p "${O}/${seq}"
  "${PY}" - "${cfg}" "${B}/configs/g_params_h4.yaml" "${B}/configs/rig_h4.yaml" "${O}/${seq}" <<'PY'
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
  bash "${S}/p5_blind_method.sh" G "${O}/${seq}/rig.yaml" "${bag}" "${O}/${seq}/run" | tail -1
  ref=$(ls /home/user/data/blind_gt/h4/${seq}_*imu*.txt | head -1)
  python3 "${REPO}/scripts/ape_from_tum.py" --ref "${ref}" --est "${O}/${seq}/run/traj.tum" --out "${O}/${seq}/ape.txt" \
    --interpolate > /dev/null 2>&1
  echo "${seq},$(md5sum < "${O}/${seq}/run/traj.tum" | cut -c1-8),$(awk '/^rmse:/ {print $2}' "${O}/${seq}/ape.txt")" \
    | tee -a "${O}/scores_h4.csv"
  rm -rf "${bag}"
done
