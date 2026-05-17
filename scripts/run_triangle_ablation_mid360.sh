#!/usr/bin/env bash
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# MID-360 triangle ablation runner: wraps run_rko_lio_mid360_crossval_benchmark.sh
# and compares baseline (use_triangle_descriptor:=false) vs candidate
# (use_triangle_descriptor:=true) on the GLIM MID-360 bag.

set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  run_triangle_ablation_mid360.sh [options]

Required: (none — defaults match the GLIM MID-360 demo bag)

Optional:
  --bag <dir>                rosbag2 dir (default: demo_data/glim_mid360/rosbag2_2024_04_16-14_17_01)
  --reference-tum <file>     TUM reference (default: output/glim_mid360_reference.tum)
  --base-param <file>        base graph_based_slam YAML
                             (default: lidarslam/param/lidarslam_mid360_rko_graph.yaml)
  --rko-param <file>         RKO-LIO YAML (default: lidarslam/param/rko_lio_mid360.yaml)
  --output-dir <dir>         output dir (default: output/triangle_ablation_mid360_<timestamp>)
  --report-out <file>        markdown report (default: <output-dir>/triangle_ablation_report.md)
  --keep-yaml                keep derived baseline / candidate YAMLs
  --help                     show this help and exit
EOF
  exit 1
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

DEFAULT_BAG="${REPO_ROOT}/demo_data/glim_mid360/rosbag2_2024_04_16-14_17_01"
DEFAULT_REFERENCE_TUM="${REPO_ROOT}/output/glim_mid360_reference.tum"
DEFAULT_BASE_PARAM="${REPO_ROOT}/lidarslam/param/lidarslam_mid360_rko_graph.yaml"
DEFAULT_RKO_PARAM="${REPO_ROOT}/lidarslam/param/rko_lio_mid360.yaml"

BAG_PATH="$DEFAULT_BAG"
REFERENCE_TUM="$DEFAULT_REFERENCE_TUM"
BASE_PARAM="$DEFAULT_BASE_PARAM"
RKO_PARAM="$DEFAULT_RKO_PARAM"
OUTPUT_DIR=""
REPORT_OUT=""
KEEP_YAML=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)            BAG_PATH="$2"; shift 2;;
    --reference-tum)  REFERENCE_TUM="$2"; shift 2;;
    --base-param)     BASE_PARAM="$2"; shift 2;;
    --rko-param)      RKO_PARAM="$2"; shift 2;;
    --output-dir)     OUTPUT_DIR="$2"; shift 2;;
    --report-out)     REPORT_OUT="$2"; shift 2;;
    --keep-yaml)      KEEP_YAML=true; shift;;
    --help|-h)        usage;;
    *) echo "Unknown argument: $1" >&2; usage;;
  esac
done

if [[ -z "$OUTPUT_DIR" ]]; then
  TS="$(date +%Y%m%d_%H%M%S)"
  OUTPUT_DIR="${REPO_ROOT}/output/triangle_ablation_mid360_${TS}"
fi

mkdir -p "$OUTPUT_DIR"
BASELINE_DIR="${OUTPUT_DIR}/baseline_no_triangle"
CANDIDATE_DIR="${OUTPUT_DIR}/candidate_with_triangle"
mkdir -p "$BASELINE_DIR" "$CANDIDATE_DIR"

BASELINE_YAML="${OUTPUT_DIR}/.baseline_no_triangle.yaml"
CANDIDATE_YAML="${OUTPUT_DIR}/.candidate_with_triangle.yaml"

python3 - "$BASE_PARAM" "$BASELINE_YAML" false <<'PY'
import sys
from pathlib import Path
import yaml
src = Path(sys.argv[1])
dst = Path(sys.argv[2])
flag = sys.argv[3].lower() == 'true'
data = yaml.safe_load(src.read_text(encoding='utf-8'))
params = data.get('graph_based_slam', {}).get('ros__parameters', {})
params['use_triangle_descriptor'] = flag
dst.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
PY

python3 - "$BASE_PARAM" "$CANDIDATE_YAML" true <<'PY'
import sys
from pathlib import Path
import yaml
src = Path(sys.argv[1])
dst = Path(sys.argv[2])
flag = sys.argv[3].lower() == 'true'
data = yaml.safe_load(src.read_text(encoding='utf-8'))
params = data.get('graph_based_slam', {}).get('ros__parameters', {})
params['use_triangle_descriptor'] = flag
dst.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
PY

run_one() {
  local label="$1"
  local yaml_path="$2"
  local out_dir="$3"
  local run_name="$4"
  echo "=== Running ${label} ==="
  bash "${SCRIPT_DIR}/run_rko_lio_mid360_crossval_benchmark.sh" \
    --bag "$BAG_PATH" \
    --reference-tum "$REFERENCE_TUM" \
    --lidarslam-param "$yaml_path" \
    --rko-param "$RKO_PARAM" \
    --output-dir "$out_dir" \
    --run-name "$run_name"
}

run_one "baseline (use_triangle_descriptor:=false)" \
  "$BASELINE_YAML" "$BASELINE_DIR" "triangle_ablation_mid360_baseline"
run_one "candidate (use_triangle_descriptor:=true)" \
  "$CANDIDATE_YAML" "$CANDIDATE_DIR" "triangle_ablation_mid360_candidate"

BASELINE_METRICS="${BASELINE_DIR}/metrics.json"
CANDIDATE_METRICS="${CANDIDATE_DIR}/metrics.json"

pick_launch_log() {
  local dir="$1"
  if [[ -f "${dir}/slam.launch.log" ]]; then
    echo "${dir}/slam.launch.log"
  else
    find "$dir" -maxdepth 2 -name "*.log" | head -n 1
  fi
}
BASELINE_LOG="$(pick_launch_log "$BASELINE_DIR")"
CANDIDATE_LOG="$(pick_launch_log "$CANDIDATE_DIR")"

if [[ -z "$REPORT_OUT" ]]; then
  REPORT_OUT="${OUTPUT_DIR}/triangle_ablation_report.md"
fi

REPORT_ARGS=(
  --baseline-metrics "$BASELINE_METRICS"
  --candidate-metrics "$CANDIDATE_METRICS"
  --baseline-label "use_triangle:=false"
  --candidate-label "use_triangle:=true"
  --candidate-kind triangle_descriptor
  --out "$REPORT_OUT"
  --write-json "${REPORT_OUT%.md}.json"
  --write-svg "${REPORT_OUT%.md}.svg"
)
if [[ -n "$BASELINE_LOG" ]]; then
  REPORT_ARGS+=(--baseline-log "$BASELINE_LOG")
fi
if [[ -n "$CANDIDATE_LOG" ]]; then
  REPORT_ARGS+=(--candidate-log "$CANDIDATE_LOG")
fi

python3 "${SCRIPT_DIR}/generate_place_recognition_report.py" "${REPORT_ARGS[@]}"

if [[ "$KEEP_YAML" != "true" ]]; then
  rm -f "$BASELINE_YAML" "$CANDIDATE_YAML"
fi

echo "MID-360 triangle ablation complete:"
echo "  baseline metrics:  $BASELINE_METRICS"
echo "  candidate metrics: $CANDIDATE_METRICS"
echo "  report:            $REPORT_OUT"
