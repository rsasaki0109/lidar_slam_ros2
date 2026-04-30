#!/usr/bin/env bash
# Download KITTI Odometry archives from the public AWS mirror (no cvlibs login).
# After this script, <dest>/sequences/<id> and <dest>/poses/ match kitti_odometry_prepare.py.
#
#   bash scripts/download_kitti_odometry.sh [--dest DIR] [--velodyne] [--skip-calib] [--skip-poses]
#
# Default: downloads calibration + ground-truth poses (~2 MB). LiDAR requires --velodyne (~79 GB).
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

BASE_URL="https://s3.eu-central-1.amazonaws.com/avg-kitti"
DEST="${REPO_ROOT}/datasets/KITTI_odometry"
DO_VELODYNE=false
DO_CALIB=true
DO_POSES=true

usage() {
  cat <<'EOF' >&2
Usage:
  download_kitti_odometry.sh [--dest DIR] [--velodyne] [--skip-calib] [--skip-poses]

  --dest DIR       Output root (default: <repo>/datasets/KITTI_odometry)
  --velodyne       Also download data_odometry_velodyne.zip (~79 GiB). Uses curl -C - resume.
  --skip-calib     Skip data_odometry_calib.zip
  --skip-poses     Skip data_odometry_poses.zip (training GT 00-10)

Archives extract a top-level "dataset/" folder; contents are merged into --dest.
Set KITTI_ODOMETRY_ROOT to the same path for run_kitti_odometry_benchmark.sh / sweep.
EOF
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dest)
      [[ $# -ge 2 ]] || usage
      DEST=$(realpath -m "$2")
      shift 2
      ;;
    --velodyne)
      DO_VELODYNE=true
      shift
      ;;
    --skip-calib)
      DO_CALIB=false
      shift
      ;;
    --skip-poses)
      DO_POSES=false
      shift
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

if ! "$DO_CALIB" && ! "$DO_POSES" && ! "$DO_VELODYNE"; then
  echo "Nothing to download (all skipped)." >&2
  exit 1
fi

command -v curl >/dev/null 2>&1 || { echo "curl is required" >&2; exit 1; }
command -v unzip >/dev/null 2>&1 || { echo "unzip is required" >&2; exit 1; }

mkdir -p "$DEST"

merge_zip_into_dest() {
  local zip_path="$1"
  local stage
  stage=$(mktemp -d)
  unzip -q -o "$zip_path" -d "$stage"
  if [[ ! -d "$stage/dataset" ]]; then
    echo "unexpected layout in $zip_path (no dataset/ top dir)" >&2
    rm -rf "$stage"
    exit 1
  fi
  mkdir -p "$DEST"
  # Merge sequences/* and poses/* into DEST
  cp -a "$stage/dataset/." "$DEST/"
  rm -rf "$stage"
}

download_and_merge() {
  local name="$1"
  local url="${BASE_URL}/${name}"
  local out="${DEST}/.${name}"
  echo "Downloading ${name} ..."
  # Resume support for large velodyne
  curl -fL --connect-timeout 30 --retry 3 --retry-delay 5 -C - -o "$out" "$url"
  echo "Extracting ${name} into ${DEST} ..."
  merge_zip_into_dest "$out"
  rm -f "$out"
}

if "$DO_VELODYNE"; then
  echo "WARN: data_odometry_velodyne.zip is ~79 GiB. Ensure enough disk space under ${DEST}." >&2
fi

if "$DO_CALIB"; then
  download_and_merge "data_odometry_calib.zip"
fi
if "$DO_POSES"; then
  download_and_merge "data_odometry_poses.zip"
fi
if "$DO_VELODYNE"; then
  download_and_merge "data_odometry_velodyne.zip"
fi

echo "Done. Dataset root: ${DEST}"
if [[ -d "${DEST}/sequences/00/velodyne" ]]; then
  n=$(find "${DEST}/sequences/00/velodyne" -maxdepth 1 -type f -name '*.bin' 2>/dev/null | wc -l)
  echo "  sequences/00/velodyne: ${n} bin files"
else
  echo "  (no velodyne yet — re-run with --velodyne for LiDAR)"
fi
