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

set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  run_triangle_ablation.sh [options]

Runs the RKO-LIO + graph_based_slam benchmark twice on the same bag (once
with use_triangle_descriptor:=false as baseline, once with
use_triangle_descriptor:=true as candidate) and produces a single
place-recognition report comparing the two.

Required:
  --bag <dir>                rosbag2 directory
  --reference-tum <file>     reference TUM trajectory
  --reference-meta <file>    reference metadata JSON
  --lidar-topic <topic>      LiDAR topic
  --imu-topic <topic>        IMU topic
  --base-param <file>        base graph_based_slam YAML (the script will
                             generate baseline / candidate copies from it)
  --output-dir <dir>         output directory; per-run subdirs are created
                             under it

Optional:
  --rko-param <file>         RKO-LIO parameter YAML (forwarded)
  --reference-bag <dir>      reference bag (forwarded; only required when the
                             NTU VIRAL reference generator is invoked, i.e.
                             when --skip-reference-gen is NOT passed)
  --skip-reference-gen       Reuse the provided --reference-tum / --reference-meta
                             without invoking the NTU-VIRAL-specific reference
                             generator. Required when running on non-NTU bags
                             (Newer College, MID-360, custom).
  --reference-source <label> Label stored in metrics.json (forwarded; e.g.
                             "newer_college_gt", default in benchmark script:
                             "leica_prism_gt")
  --report-out <file>        markdown report path
                             (default: <output-dir>/triangle_ablation_report.md)
  --keep-yaml                keep the derived baseline / candidate YAMLs
  --help                     show this help and exit
EOF
  exit 1
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

BAG_PATH=""
REFERENCE_BAG=""
REFERENCE_TUM=""
REFERENCE_META=""
LIDAR_TOPIC=""
IMU_TOPIC=""
BASE_PARAM=""
RKO_PARAM=""
OUTPUT_DIR=""
REPORT_OUT=""
KEEP_YAML=false
SKIP_REFERENCE_GEN=false
REFERENCE_SOURCE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)            BAG_PATH="$2"; shift 2;;
    --reference-bag)  REFERENCE_BAG="$2"; shift 2;;
    --reference-tum)  REFERENCE_TUM="$2"; shift 2;;
    --reference-meta) REFERENCE_META="$2"; shift 2;;
    --lidar-topic)    LIDAR_TOPIC="$2"; shift 2;;
    --imu-topic)      IMU_TOPIC="$2"; shift 2;;
    --base-param)     BASE_PARAM="$2"; shift 2;;
    --rko-param)      RKO_PARAM="$2"; shift 2;;
    --output-dir)     OUTPUT_DIR="$2"; shift 2;;
    --report-out)     REPORT_OUT="$2"; shift 2;;
    --keep-yaml)      KEEP_YAML=true; shift;;
    --skip-reference-gen) SKIP_REFERENCE_GEN=true; shift;;
    --reference-source)   REFERENCE_SOURCE="$2"; shift 2;;
    --help|-h)        usage;;
    *) echo "Unknown argument: $1" >&2; usage;;
  esac
done

for v in BAG_PATH REFERENCE_TUM REFERENCE_META LIDAR_TOPIC IMU_TOPIC BASE_PARAM OUTPUT_DIR; do
  if [[ -z "${!v}" ]]; then
    echo "Missing required argument for $v" >&2
    usage
  fi
done

if [[ ! -f "$BASE_PARAM" ]]; then
  echo "Base parameter YAML not found: $BASE_PARAM" >&2
  exit 2
fi

mkdir -p "$OUTPUT_DIR"
BASELINE_DIR="${OUTPUT_DIR}/baseline_no_triangle"
CANDIDATE_DIR="${OUTPUT_DIR}/candidate_with_triangle"
mkdir -p "$BASELINE_DIR" "$CANDIDATE_DIR"

BASELINE_YAML="${OUTPUT_DIR}/.baseline_no_triangle.yaml"
CANDIDATE_YAML="${OUTPUT_DIR}/.candidate_with_triangle.yaml"

# Derive the two parameter YAMLs from the base. We only flip
# use_triangle_descriptor; every other knob (BEV / Scan Context / SOLiD)
# stays identical so the report attributes the delta to the new descriptor.
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
  local args=(
    --bag "$BAG_PATH"
    --reference-tum "$REFERENCE_TUM"
    --reference-meta "$REFERENCE_META"
    --lidar-topic "$LIDAR_TOPIC"
    --imu-topic "$IMU_TOPIC"
    --lidarslam-param "$yaml_path"
    --output-dir "$out_dir"
    --run-name "$run_name"
  )
  if [[ -n "$REFERENCE_BAG" ]]; then
    args+=(--reference-bag "$REFERENCE_BAG")
  fi
  if [[ -n "$RKO_PARAM" ]]; then
    args+=(--rko-param "$RKO_PARAM")
  fi
  if [[ "$SKIP_REFERENCE_GEN" == "true" ]]; then
    args+=(--skip-reference-gen)
  fi
  if [[ -n "$REFERENCE_SOURCE" ]]; then
    args+=(--reference-source "$REFERENCE_SOURCE")
  fi
  echo "=== Running ${label} ==="
  bash "${SCRIPT_DIR}/run_rko_lio_graph_benchmark.sh" "${args[@]}"
}

run_one "baseline (use_triangle_descriptor:=false)" \
  "$BASELINE_YAML" "$BASELINE_DIR" "triangle_ablation_baseline"
run_one "candidate (use_triangle_descriptor:=true)" \
  "$CANDIDATE_YAML" "$CANDIDATE_DIR" "triangle_ablation_candidate"

BASELINE_METRICS="${BASELINE_DIR}/metrics.json"
CANDIDATE_METRICS="${CANDIDATE_DIR}/metrics.json"
# Prefer slam.launch.log because that's where the graph_based_slam component
# emits the candidate / loop_candidate_source counters the report parses.
# Fall back to any *.log only when the canonical launch log is missing.
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

echo "Triangle ablation complete:"
echo "  baseline metrics: $BASELINE_METRICS"
echo "  candidate metrics: $CANDIDATE_METRICS"
echo "  report:           $REPORT_OUT"
