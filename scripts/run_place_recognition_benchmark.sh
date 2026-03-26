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
  --candidate-mode MODE        Candidate family: scan_context, bev_rerank, solid_descriptor
                               Default: scan_context
  --scan-context-threshold F   Candidate Scan Context threshold (default: 0.55)
  --bev-descriptor-threshold F Candidate BEV descriptor threshold (default: 0.38)
  --bev-descriptor-sequence-threshold F
                               Candidate BEV sequence threshold (default: 0.42)
  --bev-descriptor-sequence-window N
                               Candidate BEV sequence window (default: 2)
  --bev-descriptor-pose-consistency-threshold-m F
                               Candidate BEV pose consistency threshold in meters (default: 8.0)
  --bev-descriptor-max-euclidean-distance-m F
                               Candidate BEV euclidean gate in meters (default: 50.0)
  --solid-descriptor-min-similarity F
                               Candidate SOLiD similarity threshold (default: 0.70)
  --baseline-name NAME         Baseline run-name tag (default: pr_distance)
  --candidate-name NAME        Candidate run-name tag
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
CANDIDATE_MODE="scan_context"
SCAN_CONTEXT_THRESHOLD="0.55"
BEV_DESCRIPTOR_THRESHOLD="0.38"
BEV_DESCRIPTOR_SEQUENCE_THRESHOLD="0.42"
BEV_DESCRIPTOR_SEQUENCE_WINDOW="2"
BEV_DESCRIPTOR_POSE_CONSISTENCY_THRESHOLD_M="8.0"
BEV_DESCRIPTOR_MAX_EUCLIDEAN_DISTANCE_M="50.0"
SOLID_DESCRIPTOR_MIN_SIMILARITY="0.70"
BASELINE_NAME="pr_distance"
CANDIDATE_NAME=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --output-dir)
      OUTPUT_DIR="$(realpath -m "${2:-}")"; shift 2 ;;
    --candidate-mode)
      CANDIDATE_MODE="${2:-}"; shift 2 ;;
    --scan-context-threshold)
      SCAN_CONTEXT_THRESHOLD="${2:-}"; shift 2 ;;
    --bev-descriptor-threshold)
      BEV_DESCRIPTOR_THRESHOLD="${2:-}"; shift 2 ;;
    --bev-descriptor-sequence-threshold)
      BEV_DESCRIPTOR_SEQUENCE_THRESHOLD="${2:-}"; shift 2 ;;
    --bev-descriptor-sequence-window)
      BEV_DESCRIPTOR_SEQUENCE_WINDOW="${2:-}"; shift 2 ;;
    --bev-descriptor-pose-consistency-threshold-m)
      BEV_DESCRIPTOR_POSE_CONSISTENCY_THRESHOLD_M="${2:-}"; shift 2 ;;
    --bev-descriptor-max-euclidean-distance-m)
      BEV_DESCRIPTOR_MAX_EUCLIDEAN_DISTANCE_M="${2:-}"; shift 2 ;;
    --solid-descriptor-min-similarity)
      SOLID_DESCRIPTOR_MIN_SIMILARITY="${2:-}"; shift 2 ;;
    --baseline-name)
      BASELINE_NAME="${2:-}"; shift 2 ;;
    --candidate-name)
      CANDIDATE_NAME="${2:-}"; shift 2 ;;
    *)
      die "unknown arg: $1"
      ;;
  esac
done

case "${CANDIDATE_MODE}" in
  scan_context|bev_rerank|solid_descriptor)
    ;;
  *)
    die "unsupported candidate mode: ${CANDIDATE_MODE}"
    ;;
esac

if [[ -z "${CANDIDATE_NAME}" ]]; then
  case "${CANDIDATE_MODE}" in
    scan_context) CANDIDATE_NAME="pr_scan_context" ;;
    bev_rerank) CANDIDATE_NAME="pr_bev_rerank" ;;
    solid_descriptor) CANDIDATE_NAME="pr_solid_descriptor" ;;
  esac
fi

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

case "${CANDIDATE_MODE}" in
  scan_context)
    bash "${SCRIPT_DIR}/run_rko_lio_mid360_crossval_benchmark.sh" \
      --output-dir "${CANDIDATE_DIR}" \
      --run-name "${CANDIDATE_NAME}" \
      --use-scan-context true \
      --scan-context-threshold "${SCAN_CONTEXT_THRESHOLD}"
    CANDIDATE_LABEL="Scan Context candidate"
    CANDIDATE_KIND="scan_context"
    ;;
  bev_rerank)
    bash "${SCRIPT_DIR}/run_rko_lio_mid360_crossval_benchmark.sh" \
      --output-dir "${CANDIDATE_DIR}" \
      --run-name "${CANDIDATE_NAME}" \
      --use-bev-descriptor true \
      --bev-descriptor-threshold "${BEV_DESCRIPTOR_THRESHOLD}" \
      --bev-descriptor-sequence-window "${BEV_DESCRIPTOR_SEQUENCE_WINDOW}" \
      --bev-descriptor-sequence-threshold "${BEV_DESCRIPTOR_SEQUENCE_THRESHOLD}" \
      --bev-descriptor-pose-consistency-threshold-m "${BEV_DESCRIPTOR_POSE_CONSISTENCY_THRESHOLD_M}" \
      --bev-descriptor-max-euclidean-distance-m "${BEV_DESCRIPTOR_MAX_EUCLIDEAN_DISTANCE_M}"
    CANDIDATE_LABEL="BEV-assisted rerank"
    CANDIDATE_KIND="bev_rerank"
    ;;
  solid_descriptor)
    bash "${SCRIPT_DIR}/run_rko_lio_mid360_crossval_benchmark.sh" \
      --output-dir "${CANDIDATE_DIR}" \
      --run-name "${CANDIDATE_NAME}" \
      --use-solid-descriptor true \
      --solid-descriptor-min-similarity "${SOLID_DESCRIPTOR_MIN_SIMILARITY}"
    CANDIDATE_LABEL="SOLiD rerank candidate"
    CANDIDATE_KIND="solid_descriptor"
    ;;
esac

python3 "${SCRIPT_DIR}/generate_place_recognition_report.py" \
  --baseline-metrics "${BASELINE_DIR}/metrics.json" \
  --candidate-metrics "${CANDIDATE_DIR}/metrics.json" \
  --baseline-label "distance baseline" \
  --candidate-label "${CANDIDATE_LABEL}" \
  --candidate-kind "${CANDIDATE_KIND}" \
  --out "${OUTPUT_DIR}/place_recognition_report.md" \
  --write-json "${OUTPUT_DIR}/place_recognition_report.json" \
  --write-svg "${OUTPUT_DIR}/place_recognition_report.svg"

echo "done: ${OUTPUT_DIR}"
