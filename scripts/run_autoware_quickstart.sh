#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DEFAULT_AUTO_EXIT_SECS=20

usage() {
  cat <<'EOF' >&2
Usage:
  run_autoware_quickstart.sh
  run_autoware_quickstart.sh dogfood [dogfood options...]
  run_autoware_quickstart.sh existing <graph_slam_output_dir> [viewer options...]
  run_autoware_quickstart.sh <graph_slam_output_dir> [viewer options...]

Default behavior:
  No arguments runs the bundled end-to-end dogfood path with a bounded RViz
  lifetime (`--auto-exit-secs 20`).

If the default NTU VIRAL rosbag2 is not present yet, prepare it with:
  bash scripts/download_ntu_viral_tnp01.sh

Examples:
  bash scripts/run_autoware_quickstart.sh
  bash scripts/run_autoware_quickstart.sh dogfood --viewer-rebuild
  bash scripts/run_autoware_quickstart.sh output/bench_rko_lio_ntu_viral_loopgate_20260324
EOF
  exit 1
}

has_auto_exit_flag() {
  local arg
  for arg in "$@"; do
    if [[ "$arg" == "--auto-exit-secs" || "$arg" == --auto-exit-secs=* ]]; then
      return 0
    fi
  done
  return 1
}

run_dogfood() {
  local cmd=(
    bash "${SCRIPT_DIR}/run_rko_lio_graph_autoware_dogfood.sh"
  )
  if ! has_auto_exit_flag "$@"; then
    cmd+=(--auto-exit-secs "${DEFAULT_AUTO_EXIT_SECS}")
  fi
  cmd+=("$@")
  "${cmd[@]}"
}

run_existing() {
  local source_dir="$1"
  shift

  local cmd=(
    bash "${SCRIPT_DIR}/run_graph_slam_pointcloud_map_in_autoware.sh"
    "${source_dir}"
  )
  if ! has_auto_exit_flag "$@"; then
    cmd+=(--auto-exit-secs "${DEFAULT_AUTO_EXIT_SECS}")
  fi
  cmd+=("$@")
  "${cmd[@]}"
}

case "${1:-}" in
  "")
    run_dogfood
    ;;
  --help|-h)
    usage
    ;;
  dogfood)
    shift
    run_dogfood "$@"
    ;;
  existing)
    [[ $# -ge 2 ]] || usage
    source_dir="$2"
    shift 2
    run_existing "${source_dir}" "$@"
    ;;
  *)
    if [[ -d "$1" ]]; then
      source_dir="$1"
      shift
      run_existing "${source_dir}" "$@"
    else
      run_dogfood "$@"
    fi
    ;;
esac
