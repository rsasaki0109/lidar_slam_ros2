#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  prepare_autoware_map_from_graph_slam.sh <graph_slam_output_dir> <autoware_map_dir> [options]

Options:
  --smoke                         Run Autoware map loader smoke test after staging
  --autoware-core-dir <dir>       autoware_core checkout for the smoke test
  --work-dir <dir>                Runtime workspace directory for the smoke test
  --help                          Show this help

This script copies a graph_based_slam output directory into an Autoware-style
map bundle, verifies the result, and can optionally run the Docker smoke test.
EOF
  exit 1
}

if [[ $# -lt 2 ]]; then
  usage
fi

SOURCE_DIR=$(realpath "$1")
TARGET_DIR=$(realpath -m "$2")
shift 2

RUN_SMOKE=false
AUTOWARE_CORE_DIR=""
WORK_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --smoke)
      RUN_SMOKE=true
      shift
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
    --help|-h)
      usage
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      ;;
  esac
done

if [[ ! -d "$SOURCE_DIR/pointcloud_map" ]]; then
  echo "pointcloud_map directory not found under $SOURCE_DIR" >&2
  exit 1
fi
if [[ ! -f "$SOURCE_DIR/pointcloud_map/pointcloud_map_metadata.yaml" ]]; then
  echo "pointcloud_map_metadata.yaml not found under $SOURCE_DIR/pointcloud_map" >&2
  exit 1
fi
if [[ ! -f "$SOURCE_DIR/map_projector_info.yaml" ]]; then
  echo "map_projector_info.yaml not found under $SOURCE_DIR" >&2
  exit 1
fi
if [[ "$RUN_SMOKE" == "true" && -z "$AUTOWARE_CORE_DIR" ]]; then
  echo "--smoke requires --autoware-core-dir" >&2
  exit 1
fi

mkdir -p "$TARGET_DIR"
rm -rf "$TARGET_DIR/pointcloud_map"
mkdir -p "$TARGET_DIR/pointcloud_map"

cp -a "$SOURCE_DIR/pointcloud_map/." "$TARGET_DIR/pointcloud_map/"
cp -f "$SOURCE_DIR/map_projector_info.yaml" "$TARGET_DIR/map_projector_info.yaml"

if [[ -f "$SOURCE_DIR/map.pcd" ]]; then
  cp -f "$SOURCE_DIR/map.pcd" "$TARGET_DIR/map.pcd"
fi
if [[ -f "$SOURCE_DIR/lanelet2_map.osm" ]]; then
  cp -f "$SOURCE_DIR/lanelet2_map.osm" "$TARGET_DIR/lanelet2_map.osm"
fi

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

python3 "$SCRIPT_DIR/verify_autoware_map.py" "$TARGET_DIR/pointcloud_map"

echo "Staged Autoware map bundle: $TARGET_DIR"
echo "  pointcloud_map: $TARGET_DIR/pointcloud_map"
echo "  map_projector_info: $TARGET_DIR/map_projector_info.yaml"

if [[ "$RUN_SMOKE" == "true" ]]; then
  CMD=("$SCRIPT_DIR/run_autoware_map_loader_smoke_docker.sh" "$TARGET_DIR" "$AUTOWARE_CORE_DIR")
  if [[ -n "$WORK_DIR" ]]; then
    CMD+=("$WORK_DIR")
  fi
  "${CMD[@]}"
fi
