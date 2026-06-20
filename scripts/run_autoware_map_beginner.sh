#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  run_autoware_map_beginner.sh <rosbag2_dir> [options]

Beginner-friendly wrapper around run_autoware_map_from_bag.py.

Options:
  --preflight-only             Print the bag preflight report and exit
  --foxglove                    Open the saved map in the Foxglove path after the run
  --autoware                    Open the saved map in the Dockerized Autoware viewer after the run
  --no-viewer                   Do not open a viewer after the run (default)
  --dry-run                     Print the selected command without executing it
  --help                        Show this help

Any remaining options are forwarded to run_autoware_map_from_bag.py.

Examples:
  bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2
  bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2 --preflight-only
  bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2 --foxglove
  bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2 --output-dir output/my_map
EOF
  exit 1
}

if [[ $# -lt 1 ]]; then
  usage
fi

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
RUNNER="${SCRIPT_DIR}/run_autoware_map_from_bag.py"
PREFLIGHT="${SCRIPT_DIR}/preflight_autoware_map_bag.py"
BAG_PATH=""
VIEWER=none
PREFLIGHT_ONLY=false
FORWARDED_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --foxglove)
      VIEWER=foxglove
      shift
      ;;
    --autoware)
      VIEWER=autoware
      shift
      ;;
    --no-viewer)
      VIEWER=none
      shift
      ;;
    --preflight-only)
      PREFLIGHT_ONLY=true
      shift
      ;;
    --help|-h)
      usage
      ;;
    --dry-run|--no-verify-map|--viewer-rebuild)
      FORWARDED_ARGS+=("$1")
      shift
      ;;
    --profile|--output-dir|--autoware-core-dir|--work-dir|--viewer-run-dir|--auto-exit-secs)
      [[ $# -ge 2 ]] || usage
      FORWARDED_ARGS+=("$1" "$2")
      shift 2
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage
      ;;
    *)
      if [[ -z "$BAG_PATH" ]]; then
        BAG_PATH="$1"
        shift
      else
        echo "Unexpected positional argument: $1" >&2
        usage
      fi
      ;;
  esac
done

if [[ -z "$BAG_PATH" ]]; then
  usage
fi

if [[ ! -d "$BAG_PATH" ]]; then
  echo "error: rosbag2_dir does not exist or is not a directory: $BAG_PATH" >&2
  echo "hint: pass the rosbag2 directory, not a .db3 file." >&2
  exit 2
fi

if [[ ! -f "$BAG_PATH/metadata.yaml" ]]; then
  echo "error: metadata.yaml not found under: $BAG_PATH" >&2
  echo "hint: pass the rosbag2 directory that contains metadata.yaml." >&2
  exit 2
fi

if [[ "$PREFLIGHT_ONLY" == "true" ]]; then
  python3 "$PREFLIGHT" "$BAG_PATH"
  exit 0
fi

python3 "$RUNNER" "$BAG_PATH" --viewer "$VIEWER" "${FORWARDED_ARGS[@]}"
