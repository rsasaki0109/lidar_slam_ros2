#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

usage() {
  cat <<'EOF' >&2
Usage:
  bash scripts/run_hilti_overlap_crossval.sh [options]

Options:
  --sequence exp01|exp02|exp03|exp07|exp21|all
                                Sequence to evaluate (default: all; all keeps the
                                established exp01+exp07 suite)
  --dataset-root <dir>         HILTI dataset root
  --output-root <dir>          Evidence root (default: external SSD)
  --setup <setup.bash>         Workspace setup
  --params <yaml>              graph_based_slam parameters
  --runs <n>                   Determinism runs per gate (default: 2)
  --ros-domain-base <id>       First isolated DDS domain (default: 180)
  --offline-timeout-secs <n>   Frontend capture timeout (default: 1800)
  --quiescence-secs <n>        Frontend completion quiet time (default: 60)
  --record-only                Create frozen backend input only
  --offline-only               Require and reuse frozen backend input
  --resume                     Reuse completed offline runs
  --dry-run                    Validate inputs and print commands only
  --help                       Show this help

The same frozen /rko_lio/odometry + /rko_lio/frame bag is replayed with the
overlap gate disabled and with the correction-adaptive candidate enabled.
Existing captures are never overwritten.
EOF
}

die() {
  echo "error: $*" >&2
  exit 2
}

require_value() {
  [[ $# -ge 2 && -n "$2" && "$2" != -* ]] || die "$1 requires a value"
}

SEQUENCE=all
DATASET_ROOT=${HILTI_DATASET_ROOT:-/media/sasaki/aiueo/datasets/hilti2022}
OUTPUT_ROOT=${LIDARSLAM_BENCHMARK_ROOT:-/media/sasaki/aiueo/benchmarks/phase8/hilti_overlap_crossval_20260714}
SETUP_FILE="${REPO_ROOT}/../install/setup.bash"
PARAMS_FILE="${REPO_ROOT}/lidarslam/param/lidarslam.yaml"
RKO_PARAM="${REPO_ROOT}/configs/hilti2022/rko_lio_hilti2022_pandar.yaml"
RUNS=2
ROS_DOMAIN_BASE=180
OFFLINE_TIMEOUT_SECS=1800
QUIESCENCE_SECS=60
RECORD_ONLY=false
OFFLINE_ONLY=false
RESUME=false
DRY_RUN=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sequence) require_value "$@"; SEQUENCE="$2"; shift 2 ;;
    --dataset-root) require_value "$@"; DATASET_ROOT=$(realpath -m "$2"); shift 2 ;;
    --output-root) require_value "$@"; OUTPUT_ROOT=$(realpath -m "$2"); shift 2 ;;
    --setup) require_value "$@"; SETUP_FILE=$(realpath -m "$2"); shift 2 ;;
    --params) require_value "$@"; PARAMS_FILE=$(realpath -m "$2"); shift 2 ;;
    --runs) require_value "$@"; RUNS="$2"; shift 2 ;;
    --ros-domain-base) require_value "$@"; ROS_DOMAIN_BASE="$2"; shift 2 ;;
    --offline-timeout-secs) require_value "$@"; OFFLINE_TIMEOUT_SECS="$2"; shift 2 ;;
    --quiescence-secs) require_value "$@"; QUIESCENCE_SECS="$2"; shift 2 ;;
    --record-only) RECORD_ONLY=true; shift ;;
    --offline-only) OFFLINE_ONLY=true; shift ;;
    --resume) RESUME=true; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    --help|-h) usage; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
done

case "${SEQUENCE}" in
  exp01) SEQUENCES=(exp01) ;;
  exp02) SEQUENCES=(exp02) ;;
  exp03) SEQUENCES=(exp03) ;;
  exp07) SEQUENCES=(exp07) ;;
  exp21) SEQUENCES=(exp21) ;;
  all) SEQUENCES=(exp01 exp07) ;;
  *) die "unknown sequence: ${SEQUENCE} (known: exp01, exp02, exp03, exp07, exp21, all)" ;;
esac
[[ "${RUNS}" =~ ^[1-9][0-9]*$ ]] || die "--runs must be a positive integer"
[[ "${ROS_DOMAIN_BASE}" =~ ^[0-9]+$ ]] || die "--ros-domain-base must be an integer"
[[ "${OFFLINE_TIMEOUT_SECS}" =~ ^[1-9][0-9]*$ ]] || die "--offline-timeout-secs must be positive"
[[ "${QUIESCENCE_SECS}" =~ ^[1-9][0-9]*$ ]] || die "--quiescence-secs must be positive"
[[ "${RECORD_ONLY}" != true || "${OFFLINE_ONLY}" != true ]] || \
  die "--record-only and --offline-only are mutually exclusive"

max_domain=$((ROS_DOMAIN_BASE + ${#SEQUENCES[@]} * RUNS * 2 - 1))
((max_domain <= 232)) || die "DDS domains would exceed 232 (last: ${max_domain})"
[[ -f "${SETUP_FILE}" ]] || die "setup file not found: ${SETUP_FILE}"
[[ -f "${PARAMS_FILE}" ]] || die "parameter file not found: ${PARAMS_FILE}"
[[ -f "${RKO_PARAM}" ]] || die "RKO-LIO parameter file not found: ${RKO_PARAM}"

print_command() {
  printf 'DRY_RUN'
  printf ' %q' "$@"
  printf '\n'
}

run_command() {
  if [[ "${DRY_RUN}" == true ]]; then
    print_command "$@"
  else
    "$@"
  fi
}

write_comparison() {
  local sequence="$1"
  local gt="$2"
  local backend="$3"
  local off_dir="$4"
  local on_dir="$5"
  local sequence_dir="$6"
  python3 - "${sequence}" "${gt}" "${backend}" "${off_dir}" "${on_dir}" "${sequence_dir}" <<'PY'
import csv
import hashlib
import json
import re
import sys
from pathlib import Path

sequence, gt, backend, off_text, on_text, output_text = sys.argv[1:]
off_dir, on_dir, output_dir = map(Path, (off_text, on_text, output_text))


def digest(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def read_rmse(path: Path):
    if not path.is_file():
        return None
    match = re.search(r'^rmse:\s*(\S+)', path.read_text(), re.MULTILINE)
    return float(match.group(1)) if match else None


def read_gate(label: str, root: Path):
    run = root / 'run1'
    edge_path = run / 'loop_edges.csv'
    trajectory_path = run / 'trajectory_optimized.tum'
    with edge_path.open(newline='') as stream:
        edges = [f"{row['from']}->{row['to']}" for row in csv.DictReader(stream)]
    accepted = []
    log_path = run / 'runner.log'
    if log_path.is_file():
        log = log_path.read_text(errors='replace')
        pattern = re.compile(
            r'id_loop_point 1:(\d+) id_loop_point 2:(\d+).*?'
            r'correction translation\[m\]:(\S+).*?overlap_ratio:(\S+)', re.S)
        accepted = [
            {'edge': f'{match.group(1)}->{match.group(2)}',
             'correction_translation_m': float(match.group(3)),
             'source_overlap': float(match.group(4))}
            for match in pattern.finditer(log)
        ]
    return {
        'gate': label,
        'ape_rmse_m': read_rmse(run / 'ape.txt'),
        'loop_edge_count': len(edges),
        'loop_edges': edges,
        'accepted_loop_metrics': accepted,
        'loop_edges_md5': digest(edge_path),
        'trajectory_md5': digest(trajectory_path),
    }


off = read_gate('off', off_dir)
on = read_gate('adaptive', on_dir)
off_edges, on_edges = set(off['loop_edges']), set(on['loop_edges'])
delta = None
if off['ape_rmse_m'] is not None and on['ape_rmse_m'] is not None:
    delta = on['ape_rmse_m'] - off['ape_rmse_m']
payload = {
    'schema_version': 1,
    'sequence': sequence,
    'reference_tum': gt,
    'backend_input': backend,
    'candidate': {
        'loop_min_overlap_ratio': 0.76,
        'loop_min_overlap_ratio_large_correction': 0.70,
        'loop_overlap_large_correction_translation_m': 1.0,
        'loop_overlap_max_distance_m': 0.5,
    },
    'gate_off': off,
    'gate_adaptive': on,
    'ape_rmse_delta_m': delta,
    'edges_removed_by_gate': sorted(off_edges - on_edges),
    'edges_added_by_gate': sorted(on_edges - off_edges),
}
output_dir.mkdir(parents=True, exist_ok=True)
(output_dir / 'comparison.json').write_text(
    json.dumps(payload, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def fmt(value):
    return 'n/a' if value is None else f'{value:.9f}'


lines = [
    f'# HILTI {sequence} adaptive overlap cross-validation', '',
    '| gate | APE RMSE [m] | loop edges | edge hash | trajectory hash |',
    '| --- | ---: | ---: | --- | --- |',
    f"| off | {fmt(off['ape_rmse_m'])} | {off['loop_edge_count']} | "
    f"`{off['loop_edges_md5'][:12]}` | `{off['trajectory_md5'][:12]}` |",
    f"| adaptive | {fmt(on['ape_rmse_m'])} | {on['loop_edge_count']} | "
    f"`{on['loop_edges_md5'][:12]}` | `{on['trajectory_md5'][:12]}` |",
    '', f"APE RMSE delta (adaptive - off): {fmt(delta)} m", '',
    'Removed edges: ' + (', '.join(payload['edges_removed_by_gate']) or 'none'),
    '', 'Added edges: ' + (', '.join(payload['edges_added_by_gate']) or 'none'),
]
(output_dir / 'comparison.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
print(output_dir / 'comparison.md')
PY
}

domain_cursor=${ROS_DOMAIN_BASE}
for sequence in "${SEQUENCES[@]}"; do
  case "${sequence}" in
    exp01)
      slug=exp01_construction_ground_level
      ;;
    exp02)
      slug=exp02_construction_multilevel
      ;;
    exp03)
      slug=exp03_construction_stairs
      ;;
    exp07)
      slug=exp07_long_corridor
      ;;
    exp21)
      slug=exp21_outside_building
      ;;
  esac
  bag="${DATASET_ROOT}/${sequence}_ros2"
  gt="${DATASET_ROOT}/${slug}_gt.txt"
  sequence_dir="${OUTPUT_ROOT}/${sequence}"
  backend="${sequence_dir}/backend_input"
  source_run="${sequence_dir}/source_run"
  reference_meta="${sequence_dir}/reference_meta.json"
  capture_params="${sequence_dir}/capture_params.yaml"

  [[ -f "${bag}/metadata.yaml" ]] || die "rosbag2 metadata not found: ${bag}/metadata.yaml"
  [[ -f "${gt}" ]] || die "ground truth not found: ${gt}"

  echo "=== HILTI ${sequence} ==="
  if [[ ! -f "${backend}/metadata.yaml" ]]; then
    [[ "${OFFLINE_ONLY}" != true ]] || die "frozen backend input not found: ${backend}"
    [[ ! -e "${backend}" ]] || die "incomplete backend directory exists: ${backend}"
    [[ ! -e "${source_run}" ]] || die "source output exists without backend input: ${source_run}"
    if [[ "${DRY_RUN}" != true ]]; then
      mkdir -p "${sequence_dir}"
      printf '{}\n' >"${reference_meta}"
      python3 - "${PARAMS_FILE}" "${capture_params}" <<'PY'
import sys
from pathlib import Path

import yaml

source, output = map(Path, sys.argv[1:])
document = yaml.safe_load(source.read_text())
graph = document['graph_based_slam']['ros__parameters']
# Backend topics are the capture product. Disable expensive loop registration
# in the concurrently launched graph node without changing submap publication.
graph['distance_loop_closure'] = 1.0e12
graph['use_scan_context'] = False
graph['use_bev_descriptor'] = False
graph['use_solid_descriptor'] = False
graph['use_triangle_descriptor'] = False
graph['debug_flag'] = False
output.write_text(yaml.safe_dump(document, sort_keys=False))
PY
    fi
    capture_command=(
      bash "${SCRIPT_DIR}/record_backend_input.sh"
      --output-dir "${backend}" --setup "${SETUP_FILE}" --
      bash "${SCRIPT_DIR}/run_rko_lio_graph_benchmark.sh"
      --bag "${bag}"
      --lidar-topic /hesai/pandar --imu-topic /alphasense/imu
      --rko-param "${RKO_PARAM}" --lidarslam-param "${capture_params}"
      --reference-tum "${gt}" --reference-meta "${reference_meta}"
      --skip-reference-gen --reference-source "hilti2022_${sequence}_control_points_gt"
      --quiescence-secs "${QUIESCENCE_SECS}"
      --offline-timeout-secs "${OFFLINE_TIMEOUT_SECS}"
      --output-dir "${source_run}" --run-name "hilti_${sequence}_backend_capture"
      --skip-map-save
    )
    run_command "${capture_command[@]}"
  else
    echo "reuse frozen backend input: ${backend}"
  fi

  if [[ "${RECORD_ONLY}" == true ]]; then
    continue
  fi

  off_dir="${sequence_dir}/gate_off"
  on_dir="${sequence_dir}/gate_adaptive"
  common=(
    --bag "${backend}" --params "${PARAMS_FILE}" --setup "${SETUP_FILE}"
    --runs "${RUNS}" --reference-tum "${gt}" --ape-interpolate
    --ape-max-time-diff 3.0 --param refine:=false
  )
  resume_arg=()
  [[ "${RESUME}" != true ]] || resume_arg=(--resume)

  run_command bash "${SCRIPT_DIR}/run_offline_determinism_check.sh" \
    "${common[@]}" --output-dir "${off_dir}" --ros-domain-base "${domain_cursor}" \
    "${resume_arg[@]}" \
    --param loop_min_overlap_ratio:=0.0 \
    --param loop_min_overlap_ratio_large_correction:=0.0 \
    --param loop_overlap_large_correction_translation_m:=0.0 \
    --param loop_overlap_max_distance_m:=0.5
  domain_cursor=$((domain_cursor + RUNS))

  run_command bash "${SCRIPT_DIR}/run_offline_determinism_check.sh" \
    "${common[@]}" --output-dir "${on_dir}" --ros-domain-base "${domain_cursor}" \
    "${resume_arg[@]}" \
    --param loop_min_overlap_ratio:=0.76 \
    --param loop_min_overlap_ratio_large_correction:=0.70 \
    --param loop_overlap_large_correction_translation_m:=1.0 \
    --param loop_overlap_max_distance_m:=0.5
  domain_cursor=$((domain_cursor + RUNS))

  if [[ "${DRY_RUN}" == true ]]; then
    echo "DRY_RUN python3 <write comparison for ${sequence}>"
  else
    write_comparison "${sequence}" "${gt}" "${backend}" "${off_dir}" "${on_dir}" "${sequence_dir}"
  fi
done

echo "HILTI overlap cross-validation complete: ${OUTPUT_ROOT}"
