#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  run_graph_slam_pointcloud_map_in_autoware_foxglove.sh <graph_slam_output_dir> [options]

Options:
  --stage-dir <dir>              Directory to stage the Autoware map bundle
  --autoware-core-dir <dir>      autoware_core checkout used by the map loaders
  --work-dir <dir>               Runtime workspace directory used by the map loaders
  --run-dir <dir>                Use an existing built Docker workspace run directory
  --rebuild                      Rebuild the minimal Autoware workspace before launching
  --foxglove-prefix <dir>        User-writable prefix prepared by prepare_foxglove_bridge_prefix.sh
  --port <port>                  Foxglove Bridge port
  --address <addr>               Foxglove Bridge bind address
  --topic-whitelist <expr>       Foxglove Bridge topic whitelist
  --auto-exit-secs <sec>         Auto-stop the bridge after N seconds
  --help                         Show this help
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
FOXGLOVE_PREFIX=""
PORT=""
ADDRESS=""
TOPIC_WHITELIST=""
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
    --foxglove-prefix)
      [[ $# -ge 2 ]] || usage
      FOXGLOVE_PREFIX=$(realpath "$2")
      shift 2
      ;;
    --port)
      [[ $# -ge 2 ]] || usage
      PORT="$2"
      shift 2
      ;;
    --address)
      [[ $# -ge 2 ]] || usage
      ADDRESS="$2"
      shift 2
      ;;
    --topic-whitelist)
      [[ $# -ge 2 ]] || usage
      TOPIC_WHITELIST="$2"
      shift 2
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

echo "Staging graph_based_slam output for Autoware Foxglove"
echo "  source_dir: $SOURCE_DIR"
echo "  stage_dir:  $STAGE_DIR"

bash "$SCRIPT_DIR/prepare_autoware_map_from_graph_slam.sh" "$SOURCE_DIR" "$STAGE_DIR"

CMD=(
  bash "$SCRIPT_DIR/run_autoware_pointcloud_map_foxglove.sh"
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
if [[ -n "$FOXGLOVE_PREFIX" ]]; then
  CMD+=(--foxglove-prefix "$FOXGLOVE_PREFIX")
fi
if [[ -n "$PORT" ]]; then
  CMD+=(--port "$PORT")
fi
if [[ -n "$ADDRESS" ]]; then
  CMD+=(--address "$ADDRESS")
fi
if [[ -n "$TOPIC_WHITELIST" ]]; then
  CMD+=(--topic-whitelist "$TOPIC_WHITELIST")
fi
if [[ -n "$AUTO_EXIT_SECS" ]]; then
  CMD+=(--auto-exit-secs "$AUTO_EXIT_SECS")
fi

"${CMD[@]}"
