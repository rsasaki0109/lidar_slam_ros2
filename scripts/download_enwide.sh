#!/usr/bin/env bash
# Download selected ENWIDE sequences from the official ETH Research Collection
# public share. Dataset license: CC BY 4.0.
#
# Usage:
#   bash scripts/download_enwide.sh --sequence tunnel_d --dest datasets/enwide
#   bash scripts/download_enwide.sh --sequence all --convert
#   bash scripts/download_enwide.sh --sequence tunnel_s --metadata-only
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

DEST_DIR="${REPO_ROOT}/datasets/enwide"
SEQUENCE="tunnel_d"
DO_CONVERT=false
DROP_BAG1=false
METADATA_ONLY=false

SHARE_TOKEN="TaWP9QcSnR2Pz9Z"
BASE_URL="https://libdrive.ethz.ch/public.php/webdav"

usage() {
  cat <<'EOF'
Usage: download_enwide.sh [options]

Options:
  --sequence NAME|all               ENWIDE sequence, e.g. tunnel_d (default) or field_d
  --dest PATH                       Destination root (default: datasets/enwide)
  --convert                         Convert rosbag1 to rosbag2 with rosbags-convert
  --drop-bag1                       Remove rosbag1 after successful conversion
  --metadata-only                   Download metadata and ground truth only
  -h, --help                        Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sequence) SEQUENCE="$2"; shift 2 ;;
    --dest) DEST_DIR=$(realpath -m "$2"); shift 2 ;;
    --convert) DO_CONVERT=true; shift ;;
    --drop-bag1) DROP_BAG1=true; shift ;;
    --metadata-only) METADATA_ONLY=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

ALL_SEQUENCES=(tunnel_s tunnel_d katzensee_d katzensee_s field_d field_s intersection_d intersection_s runway_d runway_s)
if [[ "${SEQUENCE}" == "all" ]]; then
  SEQUENCES=("${ALL_SEQUENCES[@]}")
elif [[ " ${ALL_SEQUENCES[*]} " == *" ${SEQUENCE} "* ]]; then
  SEQUENCES=("${SEQUENCE}")
else
  echo "unknown sequence: ${SEQUENCE} (known: ${ALL_SEQUENCES[*]}, all)" >&2
  exit 2
fi

mkdir -p "${DEST_DIR}"

download_file() {
  local remote_path="$1"
  local output_path="$2"
  local expected_bytes="$3"

  mkdir -p "$(dirname "${output_path}")"
  local actual_bytes=0
  [[ ! -f "${output_path}" ]] || actual_bytes=$(stat -c '%s' "${output_path}")
  if [[ "${actual_bytes}" -ne "${expected_bytes}" ]]; then
    curl -fL --retry 8 --retry-all-errors --retry-delay 5 --continue-at - \
      -u "${SHARE_TOKEN}:" \
      -o "${output_path}" \
      "${BASE_URL}/${remote_path}"
  fi

  actual_bytes=$(stat -c '%s' "${output_path}")
  if [[ "${actual_bytes}" -ne "${expected_bytes}" ]]; then
    echo "incomplete file: ${output_path}" >&2
    echo "expected ${expected_bytes} bytes, got ${actual_bytes}" >&2
    exit 1
  fi
}

download_file "readme.md" "${DEST_DIR}/readme.md" 1745
download_file "os_enwide.json" "${DEST_DIR}/os_enwide.json" 10587
download_file "prism_imu_extrinsics.txt" \
  "${DEST_DIR}/prism_imu_extrinsics.txt" 161

for sequence in "${SEQUENCES[@]}"; do
  case "${sequence}" in
    tunnel_s)
      BAG_NAME="2023-08-08-17-12-37-tunnel_s.bag"
      EXPECTED_BAG_BYTES=14983936757
      EXPECTED_BAG_ETAG="672ca9aa6fa170b8c4498974cea6a561"
      EXPECTED_GT_BYTES=9408508
      EXPECTED_GT_ETAG="640b8aa9844eb5db1ebb396ca69144ed"
      ;;
    tunnel_d)
      BAG_NAME="2023-08-08-17-50-31-tunnel_d.bag"
      EXPECTED_BAG_BYTES=7485669675
      EXPECTED_BAG_ETAG="f6afd377894e90a85322e425172f7b89"
      EXPECTED_GT_BYTES=4678234
      EXPECTED_GT_ETAG="f012aef67efd0e14261342fac1ac233f"
      ;;
    katzensee_d)
      BAG_NAME="2023-08-21-10-29-20-katzensee_d.bag"
      EXPECTED_BAG_BYTES=5401781657
      EXPECTED_BAG_ETAG="2005ed64f953012152d183d38ba2531a"
      EXPECTED_GT_BYTES=3250800
      EXPECTED_GT_ETAG="f9f6bbef855b02daee523f8b694a5b3b"
      ;;
    katzensee_s)
      BAG_NAME="2023-08-21-10-20-22-katzensee_s.bag"
      EXPECTED_BAG_BYTES=10199153469
      EXPECTED_BAG_ETAG="d4196d6908cdd9c4573ab67fd718ae5a"
      EXPECTED_GT_BYTES=6134999
      EXPECTED_GT_ETAG="f97d14dca51423868dc54d085924476c"
      ;;
    field_d)
      BAG_NAME="2023-08-09-19-25-45-field_d.bag"
      EXPECTED_BAG_BYTES=9298862315
      EXPECTED_BAG_ETAG="a1b550749a46e50bdcbf9563d0e11323"
      EXPECTED_GT_BYTES=5590799
      EXPECTED_GT_ETAG="2b57f27add0d5eee6b3990222f41d31a"
      ;;
    field_s)
      BAG_NAME="2023-08-09-19-05-05-field_s.bag"
      EXPECTED_BAG_BYTES=10520238663
      EXPECTED_BAG_ETAG="ccb3ce229f64d1962724d99b5e336ba0"
      EXPECTED_GT_BYTES=6226199
      EXPECTED_GT_ETAG="3eb5fc9d6d1151cdf6eac289638b9f5b"
      ;;
    intersection_d)
      BAG_NAME="2023-08-09-17-58-11-intersection_d.bag"
      EXPECTED_BAG_BYTES=11508678061
      EXPECTED_BAG_ETAG="bfb93f2acd77123176a9c1778a48eee9"
      EXPECTED_GT_BYTES=6417600
      EXPECTED_GT_ETAG="c03088cfeb255ce9d659fc5048d2fbb0"
      ;;
    intersection_s)
      BAG_NAME="2023-08-09-16-19-09-intersection_s.bag"
      EXPECTED_BAG_BYTES=12572659507
      EXPECTED_BAG_ETAG="61ac62e62d5ce5b38de5803ce610d40a"
      EXPECTED_GT_BYTES=7799222
      EXPECTED_GT_ETAG="0489b0fd9d29dee7018431dc879c3982"
      ;;
    runway_d)
      BAG_NAME="2023-08-09-18-52-05-runway_d.bag"
      EXPECTED_BAG_BYTES=11974560985
      EXPECTED_BAG_ETAG="4bb739838c8ae98f3d5a58fd3269550d"
      EXPECTED_GT_BYTES=7081547
      EXPECTED_GT_ETAG="3f733cab0fd4a32a177337c5dff2b949"
      ;;
    runway_s)
      BAG_NAME="2023-08-09-18-44-24-runway_s.bag"
      EXPECTED_BAG_BYTES=14089937609
      EXPECTED_BAG_ETAG="9036d54af3ca12836c10efa7cfd11bdd"
      EXPECTED_GT_BYTES=8376373
      EXPECTED_GT_ETAG="c03947a4a285b9b61143e1df38e3e907"
      ;;
  esac

  SEQUENCE_DIR="${DEST_DIR}/${sequence}"
  BAG1="${SEQUENCE_DIR}/${BAG_NAME}"
  GT_FILE="${SEQUENCE_DIR}/gt-${sequence}.csv"
  ROS2_DIR="${SEQUENCE_DIR}/ros2"
  MANIFEST="${SEQUENCE_DIR}/input_manifest.json"

  download_file "${sequence}/gt-${sequence}.csv" \
    "${GT_FILE}" "${EXPECTED_GT_BYTES}"

  if [[ "${METADATA_ONLY}" != "true" ]]; then
    download_file "${sequence}/${BAG_NAME}" \
      "${BAG1}" "${EXPECTED_BAG_BYTES}"
  fi

  if [[ "${DO_CONVERT}" == "true" && "${METADATA_ONLY}" != "true" && \
        ! -e "${ROS2_DIR}/metadata.yaml" ]]; then
    command -v rosbags-convert >/dev/null 2>&1 || {
      echo "rosbags-convert not found (pip install rosbags)" >&2
      exit 1
    }
    CONVERT_PARENT=$(mktemp -d "${SEQUENCE_DIR}/.ros2-convert.XXXXXX")
    if rosbags-convert --src "${BAG1}" --dst "${CONVERT_PARENT}/ros2"; then
      [[ -f "${CONVERT_PARENT}/ros2/metadata.yaml" ]] || {
        echo "conversion completed without metadata.yaml: ${CONVERT_PARENT}" >&2
        exit 1
      }
      mv "${CONVERT_PARENT}/ros2" "${ROS2_DIR}"
      rmdir "${CONVERT_PARENT}"
    else
      echo "conversion failed; partial evidence kept at ${CONVERT_PARENT}" >&2
      exit 1
    fi
  fi

  SEQUENCE="${sequence}" \
  BAG_NAME="${BAG_NAME}" \
  EXPECTED_BAG_BYTES="${EXPECTED_BAG_BYTES}" \
  EXPECTED_BAG_ETAG="${EXPECTED_BAG_ETAG}" \
  EXPECTED_GT_BYTES="${EXPECTED_GT_BYTES}" \
  EXPECTED_GT_ETAG="${EXPECTED_GT_ETAG}" \
  BAG1="${BAG1}" \
  GT_FILE="${GT_FILE}" \
  ROS2_DIR="${ROS2_DIR}" \
  METADATA_ONLY="${METADATA_ONLY}" \
  MANIFEST="${MANIFEST}" \
  python3 - <<'PY'
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def sha256_tree(path: Path) -> str:
    digest = hashlib.sha256()
    for candidate in sorted(item for item in path.rglob('*') if item.is_file()):
        digest.update(candidate.relative_to(path).as_posix().encode())
        digest.update(b'\0')
        with candidate.open('rb') as stream:
            for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
                digest.update(block)
    return digest.hexdigest()


bag = Path(os.environ['BAG1'])
gt = Path(os.environ['GT_FILE'])
ros2 = Path(os.environ['ROS2_DIR'])
metadata_only = os.environ['METADATA_ONLY'] == 'true'
document = {
    'schema_version': 1,
    'dataset': 'ENWIDE',
    'sequence': os.environ['SEQUENCE'],
    'source': 'https://doi.org/10.3929/ethz-c-000787551',
    'license': 'CC-BY-4.0',
    'topics': {'points': '/ouster/points', 'imu': '/ouster/imu'},
    'ground_truth': {
        'path': str(gt),
        'bytes': gt.stat().st_size,
        'sha256': sha256(gt),
        'official_etag': os.environ['EXPECTED_GT_ETAG'],
        'position_only': True,
    },
    'rosbag1': None,
    'rosbag2': None,
}
if not metadata_only:
    document['rosbag1'] = {
        'path': str(bag),
        'bytes': bag.stat().st_size,
        'sha256': sha256(bag),
        'official_etag': os.environ['EXPECTED_BAG_ETAG'],
    }
if (ros2 / 'metadata.yaml').is_file():
    document['rosbag2'] = {
        'path': str(ros2),
        'tree_sha256': sha256_tree(ros2),
        'converter': {
            'name': 'rosbags',
            'version': importlib.metadata.version('rosbags'),
        },
    }
manifest = Path(os.environ['MANIFEST'])
manifest.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n')
PY

  if [[ "${DROP_BAG1}" == "true" && -e "${ROS2_DIR}/metadata.yaml" ]]; then
    rm -f "${BAG1}"
  fi

  echo "ENWIDE ${sequence} ready"
  echo "  manifest: ${MANIFEST}"
  echo "  ground truth: ${GT_FILE}"
  [[ ! -e "${ROS2_DIR}/metadata.yaml" ]] || echo "  rosbag2: ${ROS2_DIR}"
done
