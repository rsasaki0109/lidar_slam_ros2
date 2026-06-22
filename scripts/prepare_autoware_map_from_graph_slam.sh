#!/usr/bin/env bash
set -euo pipefail

usage() {
  local exit_code="${1:-1}"
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
  exit "$exit_code"
}

fail() {
  echo "error: $1" >&2
  if [[ $# -gt 1 ]]; then
    echo "hint: $2" >&2
  fi
  exit 2
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "$value" || "$value" == --* ]]; then
    fail "option requires a value: ${option}" \
      "run this script with --help for valid options."
  fi
}

if [[ $# -eq 1 && ( "$1" == "--help" || "$1" == "-h" ) ]]; then
  usage 0
fi

if [[ $# -lt 2 ]]; then
  fail "graph_slam_output_dir and autoware_map_dir are required." \
    "pass source and target directories before options; run --help for details."
fi

if [[ "$1" == --* ]]; then
  fail "graph_slam_output_dir is required before options." \
    "put <graph_slam_output_dir> before options; run --help for details."
fi

if [[ "$2" == --* ]]; then
  fail "autoware_map_dir is required before options." \
    "put <autoware_map_dir> before options; run --help for details."
fi

if [[ ! -d "$1" ]]; then
  fail "graph_based_slam output directory not found: $1" \
    "pass the output directory that contains pointcloud_map/."
fi

if [[ -e "$2" && ! -d "$2" ]]; then
  fail "Autoware map output path is a file, not a directory: $2" \
    "choose a directory path for the staged map bundle."
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
      require_value "$1" "${2:-}"
      AUTOWARE_CORE_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --work-dir)
      require_value "$1" "${2:-}"
      WORK_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --help|-h)
      usage 0
      ;;
    *)
      fail "unknown option: $1" \
        "run this script with --help for valid options."
      ;;
  esac
done

if [[ ! -d "$SOURCE_DIR/pointcloud_map" ]]; then
  fail "pointcloud_map directory not found under $SOURCE_DIR" \
    "pass the graph_based_slam output directory, not the parent workspace."
fi
if [[ ! -f "$SOURCE_DIR/pointcloud_map/pointcloud_map_metadata.yaml" ]]; then
  fail "pointcloud_map_metadata.yaml not found under $SOURCE_DIR/pointcloud_map" \
    "run the map workflow until /map_save writes pointcloud_map metadata."
fi
if [[ ! -f "$SOURCE_DIR/map_projector_info.yaml" ]]; then
  fail "map_projector_info.yaml not found under $SOURCE_DIR" \
    "stage an output directory produced by the Autoware-compatible map workflow."
fi
if [[ "$RUN_SMOKE" == "true" && -z "$AUTOWARE_CORE_DIR" ]]; then
  fail "--smoke requires --autoware-core-dir <dir>" \
    "pass an autoware_core checkout for the Docker smoke test."
fi
if [[ "$RUN_SMOKE" == "true" && ! -d "$AUTOWARE_CORE_DIR" ]]; then
  fail "autoware_core directory not found: $AUTOWARE_CORE_DIR" \
    "pass an existing autoware_core checkout to --autoware-core-dir."
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
