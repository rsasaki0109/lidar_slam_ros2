#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  create_map_authoring_submission_bundle.sh <graph_slam_output_dir> <bundle_dir> [options]

Options:
  --metrics FILE           metrics.json to include. Default: <graph_slam_output_dir>/metrics.json when present
  --report PATH            Focused report file to copy under reports/. Can be specified multiple times.
  --label NAME             Bundle label stored in manifest.json
  --verify-map             Run verify_autoware_map.py and store verify_autoware_map.log in the bundle root
  --tarball                Write <bundle_dir>.tar.gz after staging
  --help                   Show this help

This script standardizes a submission-style bundle around:

- pointcloud_map/
- map_projector_info.yaml
- optional metrics.json
- optional trajectories and logs
- optional focused reports with sibling json/svg assets
- map_qa_summary.md
- manifest.json
EOF
  exit 1
}

if [[ $# -lt 2 ]]; then
  usage
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="$(realpath "$1")"
TARGET_DIR="$(realpath -m "$2")"
shift 2

METRICS_FILE=""
LABEL=""
VERIFY_MAP="false"
WRITE_TARBALL="false"
REPORTS=()
STAGED_REPORTS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --metrics)
      [[ $# -ge 2 ]] || usage
      METRICS_FILE="$(realpath "$2")"
      shift 2
      ;;
    --report)
      [[ $# -ge 2 ]] || usage
      REPORTS+=("$(realpath "$2")")
      shift 2
      ;;
    --label)
      [[ $# -ge 2 ]] || usage
      LABEL="$2"
      shift 2
      ;;
    --verify-map)
      VERIFY_MAP="true"
      shift
      ;;
    --tarball)
      WRITE_TARBALL="true"
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

[[ -d "${SOURCE_DIR}" ]] || {
  echo "source dir not found: ${SOURCE_DIR}" >&2
  exit 1
}
[[ -d "${SOURCE_DIR}/pointcloud_map" ]] || {
  echo "pointcloud_map directory not found under ${SOURCE_DIR}" >&2
  exit 1
}
[[ -f "${SOURCE_DIR}/map_projector_info.yaml" ]] || {
  echo "map_projector_info.yaml not found under ${SOURCE_DIR}" >&2
  exit 1
}

if [[ -z "${METRICS_FILE}" && -f "${SOURCE_DIR}/metrics.json" ]]; then
  METRICS_FILE="${SOURCE_DIR}/metrics.json"
fi
if [[ -n "${METRICS_FILE}" && ! -f "${METRICS_FILE}" ]]; then
  echo "metrics file not found: ${METRICS_FILE}" >&2
  exit 1
fi

for report_path in "${REPORTS[@]}"; do
  [[ -f "${report_path}" ]] || {
    echo "report file not found: ${report_path}" >&2
    exit 1
  }
done

mkdir -p "${TARGET_DIR}"
rm -rf "${TARGET_DIR}/pointcloud_map" "${TARGET_DIR}/reports" "${TARGET_DIR}/logs"
mkdir -p "${TARGET_DIR}/pointcloud_map" "${TARGET_DIR}/reports" "${TARGET_DIR}/logs"

cp -a "${SOURCE_DIR}/pointcloud_map/." "${TARGET_DIR}/pointcloud_map/"
cp -f "${SOURCE_DIR}/map_projector_info.yaml" "${TARGET_DIR}/map_projector_info.yaml"

for optional_file in \
  map.pcd \
  lanelet2_map.osm \
  traj_raw.tum \
  traj_corrected.tum \
  traj_raw_prism.tum \
  traj_corrected_prism.tum \
  ape_raw_vs_gt.txt \
  ape_corrected_vs_gt.txt
do
  if [[ -f "${SOURCE_DIR}/${optional_file}" ]]; then
    cp -f "${SOURCE_DIR}/${optional_file}" "${TARGET_DIR}/${optional_file}"
  fi
done

for optional_log in \
  slam.launch.log \
  graph_slam.log \
  rko_lio.log \
  path_logger.log \
  path_logger_raw.log \
  path_logger_corrected.log \
  map_save.log \
  verify_autoware_map.log
do
  if [[ -f "${SOURCE_DIR}/${optional_log}" ]]; then
    cp -f "${SOURCE_DIR}/${optional_log}" "${TARGET_DIR}/logs/${optional_log}"
  fi
done

if [[ -n "${METRICS_FILE}" ]]; then
  cp -f "${METRICS_FILE}" "${TARGET_DIR}/metrics.json"
fi

for report_path in "${REPORTS[@]}"; do
  report_name="$(basename "${report_path}")"
  cp -f "${report_path}" "${TARGET_DIR}/reports/${report_name}"
  STAGED_REPORTS+=("reports/${report_name}")
  report_stem="${report_path%.*}"
  for sibling_ext in json svg; do
    sibling_path="${report_stem}.${sibling_ext}"
    if [[ -f "${sibling_path}" ]]; then
      sibling_name="$(basename "${sibling_path}")"
      cp -f "${sibling_path}" "${TARGET_DIR}/reports/${sibling_name}"
      STAGED_REPORTS+=("reports/${sibling_name}")
    fi
  done
done

if [[ "${VERIFY_MAP}" == "true" ]]; then
  python3 "${SCRIPT_DIR}/verify_autoware_map.py" "${TARGET_DIR}/pointcloud_map" \
    > "${TARGET_DIR}/verify_autoware_map.log"
fi

export TARGET_DIR SOURCE_DIR
export METRICS_FILE LABEL VERIFY_MAP
export STAGED_REPORTS_TEXT="$(printf '%s\n' "${STAGED_REPORTS[@]}")"
python3 - <<'PY'
from __future__ import annotations

import json
import os
from pathlib import Path

target_dir = Path(os.environ['TARGET_DIR'])
source_dir = Path(os.environ['SOURCE_DIR'])
metrics_file = os.environ.get('METRICS_FILE', '')
label = os.environ.get('LABEL', '')
verify_map = os.environ.get('VERIFY_MAP', 'false').lower() == 'true'
staged_reports = [
    line for line in os.environ.get('STAGED_REPORTS_TEXT', '').splitlines() if line.strip()
]

qa_summary_lines = [
    '# Map QA Summary',
    '',
    f'- bundle label: `{label or target_dir.name}`',
    f'- source dir: `{source_dir}`',
    f'- verify map ran: `{verify_map}`',
    '',
    '## Included QA Reports',
]
if staged_reports:
    qa_summary_lines.extend(f'- `{path}`' for path in staged_reports)
else:
    qa_summary_lines.append('- none')
qa_summary_lines.append('')
(target_dir / 'map_qa_summary.md').write_text(
    '\n'.join(qa_summary_lines) + '\n',
    encoding='utf-8',
)

bundle_files = []
for path in sorted(target_dir.rglob('*')):
    if path.is_file():
        bundle_files.append(str(path.relative_to(target_dir)))

payload = {
    'bundle_label': label or target_dir.name,
    'source_dir': str(source_dir),
    'metrics_source': metrics_file or None,
    'verify_map_ran': verify_map,
    'files': bundle_files,
}
manifest_path = target_dir / 'manifest.json'
manifest_path.write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')
print(manifest_path)
PY

if [[ "${WRITE_TARBALL}" == "true" ]]; then
  tar -C "$(dirname "${TARGET_DIR}")" -czf "${TARGET_DIR}.tar.gz" "$(basename "${TARGET_DIR}")"
  echo "${TARGET_DIR}.tar.gz"
fi

echo "Created submission bundle: ${TARGET_DIR}"
