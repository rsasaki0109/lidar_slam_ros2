#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  run_graph_slam_pointcloud_map_in_autoware.sh <graph_slam_output_dir> [options]

Options:
  --stage-dir <dir>              Directory to stage the Autoware map bundle
  --autoware-core-dir <dir>      autoware_core checkout used by the viewer
  --work-dir <dir>               Runtime workspace directory used by the viewer
  --run-dir <dir>                Use an existing built Docker workspace run directory
  --rebuild                      Rebuild the minimal Autoware workspace before launching RViz
  --auto-exit-secs <sec>         Auto-close RViz after N seconds
  --help                         Show this help

This stages a graph_based_slam output directory into an Autoware-compatible
pointcloud map bundle, verifies it, and opens the map in the host's rviz2
through Autoware's Dockerized map loaders.
EOF
  exit 1
}

if [[ $# -lt 1 ]]; then
  usage
fi

SOURCE_DIR=$(realpath "$1")
shift

STAGE_DIR=""
AUTOWARE_CORE_DIR=/tmp/autoware_core
WORK_DIR=/tmp/autoware_map_runtime_ws
RUN_DIR=""
REBUILD=false
AUTO_EXIT_SECS=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --stage-dir)
      [[ $# -ge 2 ]] || usage
      STAGE_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --autoware-core-dir)
      [[ $# -ge 2 ]] || usage
      AUTOWARE_CORE_DIR=$(realpath "$2")
      shift 2
      ;;
    --work-dir)
      [[ $# -ge 2 ]] || usage
      WORK_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --run-dir)
      [[ $# -ge 2 ]] || usage
      RUN_DIR=$(realpath "$2")
      shift 2
      ;;
    --rebuild)
      REBUILD=true
      shift
      ;;
    --auto-exit-secs)
      [[ $# -ge 2 ]] || usage
      AUTO_EXIT_SECS="$2"
      shift 2
      ;;
    --help|-h)
      usage
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      ;;
  esac
done

if [[ ! -d "$SOURCE_DIR" ]]; then
  echo "graph_based_slam output directory not found: $SOURCE_DIR" >&2
  exit 1
fi

if [[ -z "$STAGE_DIR" ]]; then
  STAGE_DIR=$(realpath -m "/tmp/autoware_maps/$(basename "$SOURCE_DIR")")
fi

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

echo "Staging graph_based_slam output for Autoware"
echo "  source_dir: $SOURCE_DIR"
echo "  stage_dir:  $STAGE_DIR"

bash "$SCRIPT_DIR/prepare_autoware_map_from_graph_slam.sh" "$SOURCE_DIR" "$STAGE_DIR"

CMD=(
  bash "$SCRIPT_DIR/run_autoware_pointcloud_map_viewer_docker.sh"
  "$STAGE_DIR"
  "$AUTOWARE_CORE_DIR"
  "$WORK_DIR"
)

if [[ -n "$RUN_DIR" ]]; then
  CMD+=(--run-dir "$RUN_DIR")
fi
if [[ "$REBUILD" == "true" ]]; then
  CMD+=(--rebuild)
fi
if [[ -n "$AUTO_EXIT_SECS" ]]; then
  CMD+=(--auto-exit-secs "$AUTO_EXIT_SECS")
fi

"${CMD[@]}"
