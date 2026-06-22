#!/usr/bin/env bash
set -euo pipefail

usage() {
  local exit_code="${1:-1}"
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

if [[ $# -lt 1 ]]; then
  fail "graph_slam_output_dir is required." \
    "pass the output directory that contains pointcloud_map/ or graph SLAM map artifacts."
fi

if [[ "$1" == --* ]]; then
  fail "graph_slam_output_dir is required before options." \
    "put <graph_slam_output_dir> before options; run --help for details."
fi

if [[ ! -d "$1" ]]; then
  fail "graph_based_slam output directory not found: $1" \
    "pass the saved graph_based_slam output directory, not a pointcloud_map tile or option."
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
      require_value "$1" "${2:-}"
      STAGE_DIR=$(realpath -m "$2")
      shift 2
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
    --run-dir)
      require_value "$1" "${2:-}"
      RUN_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --rebuild)
      REBUILD=true
      shift
      ;;
    --foxglove-prefix)
      require_value "$1" "${2:-}"
      FOXGLOVE_PREFIX=$(realpath -m "$2")
      shift 2
      ;;
    --port)
      require_value "$1" "${2:-}"
      PORT="$2"
      shift 2
      ;;
    --address)
      require_value "$1" "${2:-}"
      ADDRESS="$2"
      shift 2
      ;;
    --topic-whitelist)
      require_value "$1" "${2:-}"
      TOPIC_WHITELIST="$2"
      shift 2
      ;;
    --auto-exit-secs)
      require_value "$1" "${2:-}"
      AUTO_EXIT_SECS="$2"
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
