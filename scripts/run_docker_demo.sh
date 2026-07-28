#!/usr/bin/env bash
# One-command SLAM demo: download a public Livox MID-360 driving bag
# (Zenodo 14841855, Koide, CC-BY 4.0, 517 MB zip) and run the installed
# golden-path CLI with the tracked MID-360 profile. The CLI saves and verifies
# an Autoware-ready map and records the versioned run/diagnosis contracts.
#
# Designed as the default command of the ghcr.io/rsasaki0109/lidar_slam_ros2
# image, but works on any host with the workspace built and sourced:
#
#   bash scripts/run_docker_demo.sh
#
# Environment overrides:
#   DEMO_DATA_DIR    dataset cache directory (default: <repo>/datasets/mid360_public)
#   DEMO_OUTPUT_DIR  output directory        (default: <repo>/output/mid360_demo)
#   LIDARSLAM_HOST_UID/GID
#                    optional numeric owner for the output mount; set both
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_DIR="${DEMO_DATA_DIR:-${REPO_ROOT}/datasets/mid360_public}"
OUT_DIR="${DEMO_OUTPUT_DIR:-${REPO_ROOT}/output/mid360_demo}"
BAG_NAME="rosbag2_2024_04_16-14_17_01"
HOST_UID="${LIDARSLAM_HOST_UID:-}"
HOST_GID="${LIDARSLAM_HOST_GID:-}"

if [[ -n "${HOST_UID}" || -n "${HOST_GID}" ]]; then
  if [[ -z "${HOST_UID}" || -z "${HOST_GID}" ]]; then
    echo "error: set both LIDARSLAM_HOST_UID and LIDARSLAM_HOST_GID" >&2
    exit 2
  fi
  if [[ ! "${HOST_UID}" =~ ^[0-9]+$ || ! "${HOST_GID}" =~ ^[0-9]+$ ]]; then
    echo "error: LIDARSLAM_HOST_UID/GID must be numeric" >&2
    exit 2
  fi
fi

restore_output_ownership() {
  local run_status="$1"
  local output_name
  local output_root
  local target
  trap - EXIT

  if [[ -z "${HOST_UID}" ]]; then
    exit "${run_status}"
  fi

  output_root="$(dirname -- "${OUT_DIR}")"
  output_name="$(basename -- "${OUT_DIR}")"
  if [[ -z "${output_root}" || "${output_root}" == "." || "${output_root}" == "/" ]]; then
    echo "error: refusing ownership update for unsafe output root: ${output_root}" >&2
    exit 1
  fi

  if [[ "${EUID}" -eq 0 ]]; then
    if [[ -d "${output_root}" ]]; then
      chown "${HOST_UID}:${HOST_GID}" "${output_root}"
    fi
    for target in \
      "${OUT_DIR}" \
      "${OUT_DIR}.partial" \
      "${output_root}/.${output_name}.postprocess.lock"; do
      if [[ -e "${target}" ]]; then
        chown -R "${HOST_UID}:${HOST_GID}" "${target}"
      fi
    done
    echo "Output ownership: ${HOST_UID}:${HOST_GID}"
  elif [[ "$(id -u)" == "${HOST_UID}" && "$(id -g)" == "${HOST_GID}" ]]; then
    echo "Output ownership already matches ${HOST_UID}:${HOST_GID}"
  else
    echo "error: cannot set output ownership without container root privileges" >&2
    exit 1
  fi
  exit "${run_status}"
}

trap 'restore_output_ownership $?' EXIT

find_demo_bag() {
  local metadata_path
  metadata_path="$(
    find "${DATA_DIR}" -type f \
      -path "*/${BAG_NAME}/metadata.yaml" \
      -print -quit 2>/dev/null
  )"
  [[ -n "${metadata_path}" ]] || return 1
  dirname "${metadata_path}"
}

echo "== [1/2] demo data: Driving SLAM Test with Livox MID360 =="
echo "   (Koide, Zenodo DOI 10.5281/zenodo.14841855, CC-BY 4.0)"
bag_dir="$(find_demo_bag || true)"
if [[ -z "${bag_dir}" ]]; then
  python3 "${REPO_ROOT}/scripts/download_mid360_robot_public_dataset.py" \
    --dataset driving_slam_mid360 --dataset-root "${DATA_DIR}"
  bag_dir="$(find_demo_bag || true)"
fi
[[ -n "${bag_dir}" && -f "${bag_dir}/metadata.yaml" ]] || {
  echo "error: demo bag not found under ${DATA_DIR}" >&2
  exit 1
}
echo "   bag: ${bag_dir}"

echo "== [2/2] verified golden-path map run (headless, offline) =="
lidarslam-map run "${bag_dir}" \
  --profile rko_lio_graph_mid360_preset \
  --output-dir "${OUT_DIR}"

echo
echo "== demo finished =="
echo "outputs under ${OUT_DIR}:"
echo "  map.pcd                  downsampled point-cloud map"
echo "  pointcloud_map/          Autoware map tiles (+ metadata)"
echo "  map_projector_info.yaml  Autoware projector info (local)"
echo "  traj_corrected.tum       loop-closed trajectory (TUM format)"
echo "  verify_autoware_map.log  Autoware compatibility result"
echo "  run_manifest.json        versioned execution evidence"
echo "  autoware_map_diagnosis.* actionable diagnosis"
