#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/run_place_recognition_benchmark.sh [options]

Options:
  --output-dir DIR             Output root directory.
                               Default: output/place_recognition_benchmark_<timestamp>
  --scan-context-threshold F   Candidate Scan Context threshold (default: 0.55)
  --baseline-name NAME         Baseline run-name tag (default: pr_distance)
  --candidate-name NAME        Candidate run-name tag (default: pr_scan_context)
EOF
}

timestamp() {
  date +%Y%m%d_%H%M%S
}

die() {
  echo "error: $*" >&2
  exit 1
}

OUTPUT_DIR=""
SCAN_CONTEXT_THRESHOLD="0.55"
BASELINE_NAME="pr_distance"
CANDIDATE_NAME="pr_scan_context"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --output-dir)
      OUTPUT_DIR="$(realpath -m "${2:-}")"; shift 2 ;;
    --scan-context-threshold)
      SCAN_CONTEXT_THRESHOLD="${2:-}"; shift 2 ;;
    --baseline-name)
      BASELINE_NAME="${2:-}"; shift 2 ;;
    --candidate-name)
      CANDIDATE_NAME="${2:-}"; shift 2 ;;
    *)
      die "unknown arg: $1"
      ;;
  esac
done

if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${REPO_ROOT}/output/place_recognition_benchmark_$(timestamp)"
fi
mkdir -p "${OUTPUT_DIR}"

BASELINE_DIR="${OUTPUT_DIR}/${BASELINE_NAME}"
CANDIDATE_DIR="${OUTPUT_DIR}/${CANDIDATE_NAME}"

bash "${SCRIPT_DIR}/run_rko_lio_mid360_crossval_benchmark.sh" \
  --output-dir "${BASELINE_DIR}" \
  --run-name "${BASELINE_NAME}" \
  --use-scan-context false

bash "${SCRIPT_DIR}/run_rko_lio_mid360_crossval_benchmark.sh" \
  --output-dir "${CANDIDATE_DIR}" \
  --run-name "${CANDIDATE_NAME}" \
  --use-scan-context true \
  --scan-context-threshold "${SCAN_CONTEXT_THRESHOLD}"

python3 "${SCRIPT_DIR}/generate_place_recognition_report.py" \
  --baseline-metrics "${BASELINE_DIR}/metrics.json" \
  --candidate-metrics "${CANDIDATE_DIR}/metrics.json" \
  --out "${OUTPUT_DIR}/place_recognition_report.md" \
  --write-json "${OUTPUT_DIR}/place_recognition_report.json"

echo "done: ${OUTPUT_DIR}"
