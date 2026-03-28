#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  prepare_foxglove_bridge_prefix.sh [out_dir] [--ros-distro <name>]

This downloads the Debian package for `ros-<distro>-foxglove-bridge` and
extracts it into a user-writable prefix. It does not require sudo.

Examples:
  bash scripts/prepare_foxglove_bridge_prefix.sh
  bash scripts/prepare_foxglove_bridge_prefix.sh /tmp/foxglove_bridge_jazzy
EOF
  exit 1
}

ROS_DISTRO_NAME=${ROS_DISTRO:-jazzy}
OUT_DIR="/tmp/foxglove_bridge_${ROS_DISTRO_NAME}"

if [[ $# -gt 0 && "$1" != --* ]]; then
  OUT_DIR=$(realpath -m "$1")
  shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro)
      [[ $# -ge 2 ]] || usage
      ROS_DISTRO_NAME="$2"
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

PACKAGE="ros-${ROS_DISTRO_NAME}-foxglove-bridge"
TMP_DIR=$(mktemp -d)
trap 'rm -rf "$TMP_DIR"' EXIT

echo "Preparing Foxglove Bridge prefix"
echo "  package: $PACKAGE"
echo "  out_dir: $OUT_DIR"

cd "$TMP_DIR"
apt-get download "$PACKAGE" >/dev/null
DEB=$(ls "${PACKAGE}"_*.deb | head -n1)
dpkg-deb -x "$DEB" unpack

PREFIX_SRC="unpack/opt/ros/${ROS_DISTRO_NAME}"
if [[ ! -d "$PREFIX_SRC" ]]; then
  echo "Extracted prefix not found: $PREFIX_SRC" >&2
  exit 1
fi

rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"
cp -a "$PREFIX_SRC"/. "$OUT_DIR"/

if [[ ! -x "$OUT_DIR/lib/foxglove_bridge/foxglove_bridge" ]]; then
  echo "foxglove_bridge executable not found under $OUT_DIR" >&2
  exit 1
fi

echo "$OUT_DIR"
