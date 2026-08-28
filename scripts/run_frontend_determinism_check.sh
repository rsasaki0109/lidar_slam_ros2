#!/usr/bin/env bash
set -euo pipefail

# Phase 4 hard gate (docs/roadmap/v0.6.md): run the offline deterministic
# frontend runner N times on the same raw sensor bag and require the
# frontend trajectory AND the submap stream summary to be byte-identical
# across runs.
#
# Usage:
#   bash scripts/run_frontend_determinism_check.sh \
#     --bag demo_data/ntu_viral/tnp_01_points_restamped_vn100_rosbag2 \
#     --cloud-topic /os1_cloud_node1/points \
#     [--imu-topic /imu/imu] \
#     [--params lidarslam/param/lidarslam.yaml] \
#     [--runs 3] [--max-clouds 0] \
#     [--ros-domain-base 120] \
#     [--save-map] \
#     [--registration-method NDT|GICP] \
#     [--registration-plugin-class lidarslam_builtin/NdtOmp] \
#     [--resume] \
#     [--output-dir output/frontend_determinism_<timestamp>] \
#     [--reference-tum demo_data/ntu_viral/tnp_01/leica_pose.tum]
#
# When --reference-tum is given, each run's trajectory_frontend.tum is also
# scored with scripts/ape_from_tum.py (report only; the gate is byte
# identity, not an APE threshold).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BAG=""
CLOUD_TOPIC=""
IMU_TOPIC=""
PARAMS="${REPO_ROOT}/lidarslam/param/lidarslam.yaml"
RUNS=3
MAX_CLOUDS=0
OUTPUT_DIR="${REPO_ROOT}/output/frontend_determinism_$(date +%Y%m%d_%H%M%S)"
REFERENCE_TUM=""
SAVE_MAP=false
REGISTRATION_METHOD=""
REGISTRATION_PLUGIN_CLASS=""
RESUME=false
ROS_DOMAIN_BASE=120

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG="$2"; shift 2 ;;
    --cloud-topic) CLOUD_TOPIC="$2"; shift 2 ;;
    --imu-topic) IMU_TOPIC="$2"; shift 2 ;;
    --params) PARAMS="$2"; shift 2 ;;
    --runs) RUNS="$2"; shift 2 ;;
    --max-clouds) MAX_CLOUDS="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --reference-tum) REFERENCE_TUM="$2"; shift 2 ;;
    --save-map) SAVE_MAP=true; shift ;;
    --registration-method) REGISTRATION_METHOD="$2"; shift 2 ;;
    --registration-plugin-class) REGISTRATION_PLUGIN_CLASS="$2"; shift 2 ;;
    --ros-domain-base) ROS_DOMAIN_BASE="$2"; shift 2 ;;
    --resume) RESUME=true; shift ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

if [[ -z "${BAG}" || -z "${CLOUD_TOPIC}" ]]; then
  echo "--bag <raw bag dir> and --cloud-topic <topic> are required" >&2
  exit 2
fi
if (( ROS_DOMAIN_BASE < 0 || ROS_DOMAIN_BASE + RUNS > 233 )); then
  echo "--ros-domain-base must leave one valid ROS domain (0..232) per run" >&2
  exit 2
fi
if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "install/setup.bash not found; build the workspace first" >&2
  exit 2
fi

# shellcheck disable=SC1091
set +u
source "${REPO_ROOT}/install/setup.bash"
set -u

mkdir -p "${OUTPUT_DIR}"
echo "bag:         ${BAG}"
echo "cloud_topic: ${CLOUD_TOPIC}"
echo "imu_topic:   ${IMU_TOPIC:-<none>}"
echo "params:      ${PARAMS}"
echo "runs:        ${RUNS}"
echo "max_clouds:  ${MAX_CLOUDS}"
echo "save_map:    ${SAVE_MAP}"
echo "registration_method: ${REGISTRATION_METHOD:-<params-file default>}"
echo "plugin:      ${REGISTRATION_PLUGIN_CLASS:-<legacy same-TU>}"
echo "out:         ${OUTPUT_DIR}"

for i in $(seq 1 "${RUNS}"); do
  run_dir="${OUTPUT_DIR}/run${i}"
  mkdir -p "${run_dir}"
  echo "--- run ${i}/${RUNS}"
  if [[ "${RESUME}" == true && \
        -f "${run_dir}/.complete" && \
        -s "${run_dir}/trajectory_frontend.tum" && \
        -s "${run_dir}/submaps_frontend.csv" && \
        ( "${SAVE_MAP}" != true || -s "${run_dir}/map.pcd" ) ]]; then
    echo "reuse complete run ${i}: ${run_dir}"
    continue
  fi
  rm -f "${run_dir}/.complete"
  imu_topic_args=()
  if [[ -n "${IMU_TOPIC}" ]]; then
    imu_topic_args=(-p imu_topic:="${IMU_TOPIC}")
  fi
  map_output_args=()
  if [[ "${SAVE_MAP}" == true ]]; then
    map_output_args=(-p map_output_path:="${run_dir}/map.pcd")
  fi
  registration_plugin_args=()
  if [[ -n "${REGISTRATION_PLUGIN_CLASS}" ]]; then
    registration_plugin_args=(
      -p registration_plugin_enable:=true
      -p registration_plugin_class:="${REGISTRATION_PLUGIN_CLASS}")
  fi
  registration_method_args=()
  if [[ -n "${REGISTRATION_METHOD}" ]]; then
    registration_method_args=(-p registration_method:="${REGISTRATION_METHOD}")
  fi
  # A direct bag callback still constructs ROS publishers. Give every run a
  # private local DDS domain so a stale runner cannot inject wall-clock poses
  # or keep the next runner alive during shutdown.
  ROS_DOMAIN_ID=$((ROS_DOMAIN_BASE + i - 1)) ROS_LOCALHOST_ONLY=1 \
    ros2 run scanmatcher scan_matcher_offline_runner --ros-args \
    --disable-rosout-logs \
    --params-file "${PARAMS}" \
    -p async_map_update:=false \
    -p bag_path:="${BAG}" \
    -p cloud_topic:="${CLOUD_TOPIC}" \
    "${imu_topic_args[@]}" \
    "${map_output_args[@]}" \
    "${registration_plugin_args[@]}" \
    "${registration_method_args[@]}" \
    -p max_clouds:="${MAX_CLOUDS}" \
    -p output_dir:="${run_dir}" \
    > "${run_dir}/runner.log" 2>&1
  touch "${run_dir}/.complete"
  md5sum "${run_dir}/trajectory_frontend.tum" "${run_dir}/submaps_frontend.csv"
  if [[ "${SAVE_MAP}" == true ]]; then
    md5sum "${run_dir}/map.pcd"
  fi
  if [[ -n "${REFERENCE_TUM}" ]]; then
    python3 "${SCRIPT_DIR}/ape_from_tum.py" \
      --ref "${REFERENCE_TUM}" \
      --est "${run_dir}/trajectory_frontend.tum" \
      --out "${run_dir}/ape.txt" \
      > "${run_dir}/ape_postprocess.log" 2>&1 \
      || echo "WARN: APE post-processing failed in run ${i}; continuing" >&2
  fi
done

ape_rmse_for_run() {
  local run_dir="$1"
  if [[ -f "${run_dir}/ape.txt" ]]; then
    awk '/^\s*rmse:/ {print $2; exit}' "${run_dir}/ape.txt"
  fi
}

echo "--- verdict"
traj_ref="${OUTPUT_DIR}/run1/trajectory_frontend.tum"
submaps_ref="${OUTPUT_DIR}/run1/submaps_frontend.csv"
map_ref="${OUTPUT_DIR}/run1/map.pcd"
pose_count=$(wc -l < "${traj_ref}")
submap_count=$(($(wc -l < "${submaps_ref}") - 1))
status=0
for i in $(seq 2 "${RUNS}"); do
  if ! diff -q "${traj_ref}" "${OUTPUT_DIR}/run${i}/trajectory_frontend.tum" > /dev/null; then
    echo "MISMATCH (frontend trajectory): run1 vs run${i}"
    diff "${traj_ref}" "${OUTPUT_DIR}/run${i}/trajectory_frontend.tum" | head -10 || true
    status=1
  fi
  if ! diff -q "${submaps_ref}" "${OUTPUT_DIR}/run${i}/submaps_frontend.csv" > /dev/null; then
    echo "MISMATCH (submap stream): run1 vs run${i}"
    diff "${submaps_ref}" "${OUTPUT_DIR}/run${i}/submaps_frontend.csv" | head -10 || true
    status=1
  fi
  if [[ "${SAVE_MAP}" == true && ! -f "${OUTPUT_DIR}/run${i}/map.pcd" ]]; then
    echo "MISSING (map PCD): run${i}"
    status=1
  elif [[ "${SAVE_MAP}" == true ]] && ! cmp -s "${map_ref}" "${OUTPUT_DIR}/run${i}/map.pcd"; then
    echo "MISMATCH (map PCD): run1 vs run${i}"
    status=1
  fi
done

summary="${OUTPUT_DIR}/frontend_determinism_summary.md"
{
  if [[ "${SAVE_MAP}" == true ]]; then
    echo "| run | ape_rmse | n_poses | n_submaps | trajectory_md5 | submaps_md5 | map_md5 |"
    echo "| --- | ---: | ---: | ---: | --- | --- | --- |"
  else
    echo "| run | ape_rmse | n_poses | n_submaps | trajectory_md5 | submaps_md5 |"
    echo "| --- | ---: | ---: | ---: | --- | --- |"
  fi
  for i in $(seq 1 "${RUNS}"); do
    run_dir="${OUTPUT_DIR}/run${i}"
    n_poses=$(wc -l < "${run_dir}/trajectory_frontend.tum")
    n_submaps=$(($(wc -l < "${run_dir}/submaps_frontend.csv") - 1))
    traj_md5=$(md5sum "${run_dir}/trajectory_frontend.tum" | cut -c1-12)
    submaps_md5=$(md5sum "${run_dir}/submaps_frontend.csv" | cut -c1-12)
    rmse=$(ape_rmse_for_run "${run_dir}")
    if [[ "${SAVE_MAP}" == true ]]; then
      map_md5=$(md5sum "${run_dir}/map.pcd" | cut -c1-12)
      echo "| run${i} | ${rmse:-n/a} | ${n_poses} | ${n_submaps} | \`${traj_md5}\` | \`${submaps_md5}\` | \`${map_md5}\` |"
    else
      echo "| run${i} | ${rmse:-n/a} | ${n_poses} | ${n_submaps} | \`${traj_md5}\` | \`${submaps_md5}\` |"
    fi
  done
  echo ""
  if [[ ${status} -eq 0 ]]; then
    echo "frontend_trajectories_identical: true"
    echo "submap_streams_identical: true"
    if [[ "${SAVE_MAP}" == true ]]; then
      echo "map_pcds_identical: true"
    fi
  else
    echo "frontend_trajectories_submap_streams_or_map_pcds_identical: false"
  fi
} | tee "${summary}"

if [[ ${status} -eq 0 ]]; then
  if [[ "${SAVE_MAP}" == true ]]; then
    echo "FRONTEND_DETERMINISM_OK: ${RUNS} runs produced byte-identical trajectory_frontend.tum, submaps_frontend.csv, and map.pcd (${pose_count} poses, ${submap_count} submaps)"
  else
    echo "FRONTEND_DETERMINISM_OK: ${RUNS} runs produced byte-identical trajectory_frontend.tum and submaps_frontend.csv (${pose_count} poses, ${submap_count} submaps)"
  fi
else
  echo "FRONTEND_DETERMINISM_FAILED"
fi
exit ${status}
