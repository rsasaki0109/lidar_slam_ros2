#!/usr/bin/env bash
# p7_g16_phase1.sh [exp04|mapshd|koide|oxford ...] (bench_report/data/phase1_global/decision_rule_G16.txt, criteria 2
# and 3): G12 and G16 side by side on the Phase 1 data, one sequence at a time (fetched, run, deleted):
#   exp04 (streamed from the frozen HILTI revision, p5_ros1_stream_to_ros2.py), MapsHD's eight trials (p1_mapshd_get.sh),
#   Koide outdoor_hard 02a/02b/01b/01a (p1_koide_get.sh), Oxford Keble 05 (p3_spires_session.sh CONVERT_ONLY=1: sensor
#   data only; its ground truth stays sealed and is never read here: Oxford is compared by md5 alone).
# Each run: the rig file of Phase 1 with the G file on top, p5_blind_method.sh G. One line per run in
# /home/user/bench/g16/phase1.csv: family,config,sequence,md5,re-levels,node_s,data_s,score...
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
P=${REPO}/bench_report/params; OUT=/home/user/bench/g16; CFG=${OUT}/phase1_cfg; mkdir -p "${OUT}" "${CFG}"
CN=${CAND_NAME:-G16}  # CAND_CFG / CAND_NAME: another candidate in the place of G16; OUT_NAME: the output directory
OUT=/home/user/bench/${OUT_NAME:-g16}; CFG=${OUT}/phase1_cfg; mkdir -p "${OUT}" "${CFG}"
BN=${BASE_NAME:-G12}  # BASE_CFG / BASE_NAME: another reference in the place of G12
declare -A GF=([${BN}]=${BASE_CFG:-${REPO}/configs/sota_v7/rko_lio_global_G12_2026-10-26.yaml}
               [${CN}]=${CAND_CFG:-${REPO}/bench_report/data/phase1_global/rko_lio_candidate_G16_2026-11-08.yaml})
set +u; source /opt/ros/jazzy/setup.bash; set -u

# rig <name> <rig file> <lidar topic> <imu topic>: the merged parameter files and rig yamls for G12 and G16
rig() {
  local name=$1 file=$2 lidar=$3 imu=$4
  for c in ${BN} ${CN}; do
    "${PY}" - "${file}" "${GF[$c]}" "${CFG}/${name}_${c}.yaml" "${lidar}" "${imu}" "${CFG}/${name}_${c}_rig.yaml" <<'PY'
import sys
import yaml
rig = yaml.safe_load(open(sys.argv[1]))['/**']['ros__parameters']
g = yaml.safe_load(open(sys.argv[2]))['/**']['ros__parameters']
yaml.safe_dump({'/**': {'ros__parameters': {**rig, **g}}}, open(sys.argv[3], 'w'), sort_keys=False)
yaml.safe_dump({'lidar_topic': sys.argv[4], 'imu_topic': sys.argv[5],
                'lidar_to_imu_quat_xyzw_xyz': rig.get('extrinsic_lidar2base_quat_xyzw_xyz', [0, 0, 0, 1, 0, 0, 0]),
                'g_params': sys.argv[3]}, open(sys.argv[6], 'w'), sort_keys=False)
PY
  done
}

# runpair <family> <rig name> <sequence> <bag> <score command prefix ('' = none)>
runpair() {
  local fam=$1 name=$2 seq=$3 bag=$4 score=$5
  # SCAN_STATS=1: GT-free scene statistics of the bag (p10_scan_stats.py), one line per sequence in scan_stats.csv
  if [[ ${SCAN_STATS:-0} == 1 ]] && ! grep -q "^${fam},${seq}," "${OUT}/scan_stats.csv" 2>/dev/null; then
    local lt; lt=$(awk '$1 == "lidar_topic:" {print $2}' "${CFG}/${name}_${BN}_rig.yaml")
    echo "${fam},${seq},$(cd /tmp && "${PY}" -I "${S}/p10_scan_stats.py" "${bag}" "${lt}" 2>/dev/null)" >> "${OUT}/scan_stats.csv"
  fi
  for c in ${BN} ${CN}; do
    local o=${OUT}/phase1/${fam}/${seq}_${c}
    [[ -s ${o}/traj_imu.tum ]] || bash "${S}/p5_blind_method.sh" G "${CFG}/${name}_${c}_rig.yaml" "${bag}" "${o}" > /dev/null 2>&1
    if [[ ! -s ${o}/traj_imu.tum ]]; then echo "${fam},${c},${seq},FAILED" | tee -a "${OUT}/phase1.csv"; continue; fi
    local md5 rl node span sc=
    md5=$(md5sum < "${o}/front_0/front_tum_0.txt" | cut -c1-8)
    rl=$("${PY}" -c "import json, sys; print(json.load(open(sys.argv[1]))['applied_count'])" \
      "${o}/front_0/gravity_alignment_summary.json" 2>/dev/null || echo 0)
    node=$(grep -oP 'took \K[0-9.]+' "${o}/stdout.log" | tail -1)
    span=$("${PY}" -c "import numpy as np, sys; t = np.loadtxt(sys.argv[1])[:, 0]; print(f'{t[-1] - t[0]:.1f}')" "${o}/traj_imu.tum")
    [[ -n ${score} ]] && sc=$(eval "${score} ${o}/front_0/front_tum_0.txt ${o}")
    echo "${fam},${c},${seq},${md5},${rl},${node},${span},${sc}" | tee -a "${OUT}/phase1.csv"
  done
}

for step in ${@:-exp04 mapshd koide oxford}; do
  case ${step} in
    exp04)
      rig exp04 "${P}/exp04_gyro002_jerk1000.yaml" /hesai/pandar /alphasense/imu
      B=/home/user/data/hilti2022/g16_exp04
      if [[ ! -f ${B}/ok ]]; then
        rm -rf "${B}"; mkdir -p "${B}"
        curl -sSL --fail "https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/e62017f907007fdc5ab8c721842e4ae7359d7f49/rosbags/exp04_construction_upper_level.bag" \
          | (cd /tmp && "${PY}" -I "${S}/p5_ros1_stream_to_ros2.py" - "${B}/bag" --lidar /hesai/pandar --imu /alphasense/imu \
             --time-field time --summary "${B}/summary.json" > "${B}/convert.log" 2>&1) && touch "${B}/ok"
      fi
      runpair exp04 exp04 exp04 "${B}/bag" \
        "f() { python3 ${REPO}/scripts/ape_from_tum.py --ref /home/user/data/hilti2022/exp04_construction_upper_level_gt.txt --est \$1 --out \$2/ape.txt --interpolate > /dev/null 2>&1; awk '/^rmse:/ {print \$2}' \$2/ape.txt; }; f"
      rm -rf "${B}" ;;
    mapshd)
      rig mapshd "${P}/mapshd_gyro002_jerk1000.yaml" /livox/pointcloud /livox/imu
      for sc in calibration-room flat-room long-corridor outdoor-urk underground-garage; do
        bash "${S}/p1_mapshd_get.sh" "${sc}" > "${OUT}/get_${sc}.log" 2>&1
        for t in /home/user/data/mapshd/bags/${sc}_trial-*_c; do
          tr=$(basename "${t}" _c)
          case ${tr} in
            calibration-room_trial-1|calibration-room_trial-2|flat-room_trial-1|long-corridor_trial-1|outdoor-urk_trial-1| \
            underground-garage_trial-1|underground-garage_trial-2|underground-garage_trial-3) ;;
            *) continue ;;
          esac
          runpair mapshd mapshd "${tr}" "${t}" \
            "f() { ${PY} /home/user/scripts/front_score.py ${tr} \$1 /home/user/data/mapshd/gt/${tr}/ground_truth.tum | cut -d, -f2; }; f"
        done
        rm -rf /home/user/data/mapshd/bags/${sc}_trial-*_c
      done ;;
    koide)
      rig koide "${P}/p1_koide_V.yaml" /livox/points /livox/imu
      for s in outdoor_hard_02a outdoor_hard_02b outdoor_hard_01b outdoor_hard_01a; do
        bash "${S}/p1_koide_get.sh" "${s}" > "${OUT}/get_${s}.log" 2>&1
        runpair koide koide "${s}" "/home/user/data/koide/${s}_c" \
          "f() { python3 ${REPO}/scripts/ape_from_tum.py --ref /home/user/data/koide/gt/traj_lidar_${s%[ab]}.txt --est \$1 --out \$2/ape.txt > /dev/null 2>&1; awk '/^rmse:/ {print \$2}' \$2/ape.txt; }; f"
        rm -rf "/home/user/data/koide/${s}_c"
      done ;;
    oxford)
      rig oxford "${P}/oxford_keble05_gyro002_jerk1000.yaml" /hesai/pandar /alphasense_driver_ros/imu
      [[ -e /home/user/data/oxford_keble05/ros2/metadata.yaml ]] || CONVERT_ONLY=1 bash "${S}/p3_spires_session.sh" \
        2024-03-12-keble-college-05 keble05 > "${OUT}/get_oxford.log" 2>&1
      # the ground truth was sealed until the maintainer opened it for the Phase 1 gate (decision_rule_G17.txt);
      # OXFORD_SCORE=0 compares by md5 alone, as for G16
      osc=''
      [[ ${OXFORD_SCORE:-0} == 1 ]] && osc="f() { ${PY} /home/user/scripts/ape_oxford.py \$1 /home/user/data/oxford_keble05/gt_sealed/gt-tum.txt /home/user/data/oxford_keble05/calib/T_imu_base.npy \$2/ape > \$2/ape.log 2>&1; grep -oiP 'rmse[^0-9]*\K[0-9.]+' \$2/ape.log | head -1; }; f"
      runpair oxford oxford keble05 /home/user/data/oxford_keble05/ros2 "${osc}"
      rm -rf /home/user/data/oxford_keble05/ros2 ;;
  esac
done
echo "phase1 done"
