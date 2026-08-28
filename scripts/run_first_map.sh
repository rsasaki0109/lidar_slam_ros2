#!/usr/bin/env bash
# One entrypoint for a first verified map. It delegates to the maintained
# Docker demo or source-workspace quickstart instead of duplicating pipelines.
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
PATH_KIND="auto"
DRY_RUN=0

usage() {
  cat <<'EOF'
Usage: bash scripts/run_first_map.sh [--path auto|docker|source] [--dry-run]

  auto    use the sourced ROS 2 workspace when available, otherwise Docker
  docker  run the published one-command MID-360 demo image
  source  download NTU VIRAL tnp_01 if needed, then run the source quickstart

The command runs the read-only environment doctor before downloading or mapping.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --path)
      [[ $# -ge 2 ]] || { echo "error: --path requires a value" >&2; exit 2; }
      PATH_KIND="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "error: unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${PATH_KIND}" in
  auto)
    if [[ -n "${ROS_DISTRO:-}" ]] && command -v ros2 >/dev/null 2>&1; then
      PATH_KIND="source"
    else
      PATH_KIND="docker"
    fi
    ;;
  docker|source) ;;
  *) echo "error: --path must be auto, docker, or source" >&2; exit 2 ;;
esac

doctor=(python3 "${SCRIPT_DIR}/lidarslam_doctor.py" --profile "${PATH_KIND}" --repo-root "${REPO_ROOT}")
if [[ ${DRY_RUN} -eq 1 ]]; then
  echo "+ ${doctor[*]}"
  if [[ "${PATH_KIND}" == "docker" ]]; then
    echo "+ docker run --rm -v \"${PWD}/lidarslam_output:/lidarslam_ws/output\" ghcr.io/rsasaki0109/lidar_slam_ros2:humble"
  else
    echo "+ bash ${SCRIPT_DIR}/download_ntu_viral_tnp01.sh"
    echo "+ bash ${SCRIPT_DIR}/run_autoware_quickstart.sh"
  fi
  exit 0
fi

"${doctor[@]}"
if [[ "${PATH_KIND}" == "docker" ]]; then
  exec docker run --rm \
    -v "${PWD}/lidarslam_output:/lidarslam_ws/output" \
    ghcr.io/rsasaki0109/lidar_slam_ros2:humble
fi

bash "${SCRIPT_DIR}/download_ntu_viral_tnp01.sh"
exec bash "${SCRIPT_DIR}/run_autoware_quickstart.sh"
