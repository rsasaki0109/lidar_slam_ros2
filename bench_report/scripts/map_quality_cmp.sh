#!/usr/bin/env bash
# Usage: map_quality_cmp.sh <out_csv> <run_dir>...
# One evaluator for both systems: GLIM dumps (<run_dir>/dump) are exported to a world-frame PCD with
# scripts/export_glim_dump_map.py (the exporter of docs/comparison.md), ours use <run_dir>/map.pcd from
# /map_save. Each map is scored with scripts/run_map_quality_check.sh --runs 1 --downsample 0.1
# (graph_based_slam map_quality_report), as in docs/comparison.md.
# Columns: run,map_points,map_md5,evaluated_points,thickness_rms_mean_m,thickness_rms_p95_m,planar_coverage,
#          patch_count,mean_map_entropy_nats
csv=$1; shift
REPO=/home/user/lidar_slam_ros2
SETUP=${MAP_QUALITY_SETUP:-/home/user/ws/dev_c/install/setup.bash}
for run in "$@"; do
  name=$(basename "${run}")
  mq=${run}/map_quality
  rm -rf "${mq}"; mkdir -p "${mq}"
  if [[ -d ${run}/dump ]]; then
    python3 "${REPO}/scripts/export_glim_dump_map.py" --dump "${run}/dump" --output "${mq}/glim_map.pcd" > "${mq}/export.log" 2>&1
    map=${mq}/glim_map.pcd
  else
    map=${run}/map.pcd
  fi
  bash "${REPO}/scripts/run_map_quality_check.sh" --input "${map}" --output-dir "${mq}/report" --runs 1 \
    --downsample 0.1 --setup "${SETUP}" > "${mq}/check.log" 2>&1
  python3 - "${mq}/report/run1/map_quality_report.yaml" "${map}" "${name}" <<'PY' | tee -a "${csv}"
import hashlib, sys, yaml
report, cloud, name = sys.argv[1:]
r = yaml.safe_load(open(report))['map_quality_report']
p = r['plane_metrics']
md5 = hashlib.md5(open(cloud, 'rb').read()).hexdigest()
print(f"{name},{r['input_points']},{md5},{r['evaluated_points']},{p['thickness_rms_mean_m']:.6f},"
      f"{p['thickness_rms_p95_m']:.6f},{p['planar_coverage']:.6f},{p['patch_count']},"
      f"{r['mean_map_entropy']['value_nats']:.6f}")
PY
done
