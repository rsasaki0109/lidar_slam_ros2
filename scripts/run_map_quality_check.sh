#!/usr/bin/env bash
# Map-quality report with an N-run byte-identity verdict
# (v0.7 Phase 1, docs/roadmap/v0.7.md).
#
# Usage:
#   bash scripts/run_map_quality_check.sh \
#     --input output/rtkslam_cs2_run2/map.pcd \
#     --output-dir output/map_quality_cs2 \
#     [--runs 3] [--downsample 0.1] \
#     [--setup /path/to/workspace/install/setup.bash] \
#     [--profile configs/map_quality_profiles/indoor_construction.yaml] \
#     [--baseline-report baseline/map_quality_report.yaml] \
#     [--max-regression-percent 2.0]
#
# This script always enforces determinism: every run must produce a
# byte-identical map_quality_report.yaml. Without --profile the metric
# VALUES are report-only. With --profile the values are additionally
# compared against the profile's threshold table
# (scripts/check_map_quality_thresholds.py); a `blocking` profile turns
# violations into a non-zero exit, a `report_only` profile prints the
# verdict rows without failing (v0.7 Phase 3 rollout shape).
# With --baseline-report, run1 is also compared by the independent paired
# non-regression checker. That comparison is fail-closed and never relaxes
# the absolute profile above; --max-regression-percent defaults to 2.0 and
# is only valid when a baseline report is supplied. The metric extraction
# profile itself is frozen in MapQualityConfig
# (docs/research/map-quality-baseline.md) and is deliberately not
# exposed here.
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
INPUT=""
OUTPUT_DIR=""
RUNS=3
DOWNSAMPLE=0.1
PROFILE=""
BASELINE_REPORT=""
MAX_REGRESSION_PERCENT="2.0"
MAX_REGRESSION_PERCENT_SET=false
SETUP_FILE="${REPO_ROOT}/../install/setup.bash"
if [[ ! -f "${SETUP_FILE}" ]]; then
  SETUP_FILE="${REPO_ROOT}/install/setup.bash"
fi

usage() {
  grep '^#' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2
  exit 2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --input)
      [[ $# -ge 2 ]] || usage
      INPUT=$(realpath -m "$2")
      shift 2
      ;;
    --output-dir)
      [[ $# -ge 2 ]] || usage
      OUTPUT_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --runs)
      [[ $# -ge 2 ]] || usage
      RUNS="$2"
      shift 2
      ;;
    --downsample)
      [[ $# -ge 2 ]] || usage
      DOWNSAMPLE="$2"
      shift 2
      ;;
    --profile)
      [[ $# -ge 2 ]] || usage
      PROFILE=$(realpath -m "$2")
      shift 2
      ;;
    --baseline-report)
      [[ $# -ge 2 ]] || usage
      BASELINE_REPORT=$(realpath -m "$2")
      shift 2
      ;;
    --max-regression-percent)
      [[ $# -ge 2 ]] || usage
      MAX_REGRESSION_PERCENT="$2"
      MAX_REGRESSION_PERCENT_SET=true
      shift 2
      ;;
    --setup)
      [[ $# -ge 2 ]] || usage
      SETUP_FILE=$(realpath -m "$2")
      shift 2
      ;;
    *)
      usage
      ;;
  esac
done

if [[ -z "${INPUT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--input and --output-dir are required" >&2
  exit 2
fi
if [[ -n "${PROFILE}" && ! -f "${PROFILE}" ]]; then
  echo "profile not found: ${PROFILE}" >&2
  exit 2
fi
if [[ -n "${BASELINE_REPORT}" && ! -f "${BASELINE_REPORT}" ]]; then
  echo "baseline report not found: ${BASELINE_REPORT}" >&2
  exit 2
fi
if [[ -z "${BASELINE_REPORT}" && "${MAX_REGRESSION_PERCENT_SET}" == "true" ]]; then
  echo "--max-regression-percent requires --baseline-report" >&2
  exit 2
fi
if [[ ! -e "${INPUT}" ]]; then
  echo "input not found: ${INPUT}" >&2
  exit 2
fi
if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "install/setup.bash not found in the repository or parent workspace" >&2
  exit 2
fi

# shellcheck disable=SC1091
set +u
source "${SETUP_FILE}"
set -u

mkdir -p "${OUTPUT_DIR}"
echo "input:      ${INPUT}"
echo "runs:       ${RUNS}"
echo "downsample: ${DOWNSAMPLE}"
echo "out:        ${OUTPUT_DIR}"
if [[ -n "${BASELINE_REPORT}" ]]; then
  echo "baseline:   ${BASELINE_REPORT}"
  echo "max regress: ${MAX_REGRESSION_PERCENT}%"
fi

MD5S=()
for ((r = 1; r <= RUNS; r++)); do
  echo "--- run ${r}/${RUNS}"
  RUN_DIR="${OUTPUT_DIR}/run${r}"
  ros2 run graph_based_slam map_quality_report \
    --input "${INPUT}" \
    --output-dir "${RUN_DIR}" \
    --downsample "${DOWNSAMPLE}" \
    > "${OUTPUT_DIR}/run${r}.log" 2>&1
  md5sum "${RUN_DIR}/map_quality_report.yaml"
  MD5S+=("$(md5sum "${RUN_DIR}/map_quality_report.yaml" | cut -d' ' -f1)")
done

IDENTICAL=true
for md5 in "${MD5S[@]}"; do
  if [[ "${md5}" != "${MD5S[0]}" ]]; then
    IDENTICAL=false
  fi
done

THRESHOLD_LOG="${OUTPUT_DIR}/threshold_verdict.txt"
THRESHOLD_STATUS=0
if [[ -n "${PROFILE}" ]]; then
  set +e
  python3 "${REPO_ROOT}/scripts/check_map_quality_thresholds.py" \
    --report "${OUTPUT_DIR}/run1/map_quality_report.yaml" \
    --profile "${PROFILE}" \
    --out "${OUTPUT_DIR}/threshold_verdict.yaml" \
    > "${THRESHOLD_LOG}" 2>&1
  THRESHOLD_STATUS=$?
  set -e
fi

REGRESSION_LOG="${OUTPUT_DIR}/paired_regression_verdict.txt"
REGRESSION_STATUS=0
if [[ -n "${BASELINE_REPORT}" ]]; then
  set +e
  python3 "${REPO_ROOT}/scripts/check_map_quality_regression.py" \
    --baseline-report "${BASELINE_REPORT}" \
    --candidate-report "${OUTPUT_DIR}/run1/map_quality_report.yaml" \
    --max-regression-percent "${MAX_REGRESSION_PERCENT}" \
    --out "${OUTPUT_DIR}/paired_regression_verdict.yaml" \
    --json-out "${OUTPUT_DIR}/paired_regression_verdict.json" \
    > "${REGRESSION_LOG}" 2>&1
  REGRESSION_STATUS=$?
  set -e
fi

SUMMARY="${OUTPUT_DIR}/map_quality_summary.md"
{
  echo "# Map-quality check"
  echo
  echo "- input: \`${INPUT}\`"
  echo "- runner_setup: \`${SETUP_FILE}\`"
  echo "- runs: ${RUNS}, downsample: ${DOWNSAMPLE} m"
  echo "- reports_identical: ${IDENTICAL}"
  echo "- report_md5: \`${MD5S[0]}\`"
  if [[ -n "${PROFILE}" ]]; then
    echo "- threshold_profile: \`${PROFILE}\`"
  fi
  if [[ -n "${BASELINE_REPORT}" ]]; then
    echo "- baseline_report: \`${BASELINE_REPORT}\`"
    echo "- max_regression_percent: ${MAX_REGRESSION_PERCENT}"
    echo "- paired_non_regression_receipt: \`${OUTPUT_DIR}/paired_regression_verdict.yaml\`"
  fi
  echo
  echo '```yaml'
  cat "${OUTPUT_DIR}/run1/map_quality_report.yaml"
  echo '```'
  if [[ -n "${PROFILE}" ]]; then
    echo
    echo '```'
    cat "${THRESHOLD_LOG}"
    echo '```'
  fi
  if [[ -n "${BASELINE_REPORT}" ]]; then
    echo
    echo '```'
    cat "${REGRESSION_LOG}"
    echo '```'
  fi
} > "${SUMMARY}"

echo "--- verdict"
cat "${OUTPUT_DIR}/run1/map_quality_report.yaml"
if [[ -n "${PROFILE}" ]]; then
  cat "${THRESHOLD_LOG}"
fi
if [[ -n "${BASELINE_REPORT}" ]]; then
  cat "${REGRESSION_LOG}"
fi
if [[ "${IDENTICAL}" != "true" ]]; then
  echo "MAP_QUALITY_FAILED: reports differ across runs" >&2
  exit 1
fi
if [[ "${THRESHOLD_STATUS}" -ne 0 ]]; then
  echo "MAP_QUALITY_FAILED: threshold check failed (exit ${THRESHOLD_STATUS})" >&2
  exit "${THRESHOLD_STATUS}"
fi
if [[ "${REGRESSION_STATUS}" -ne 0 ]]; then
  echo "MAP_QUALITY_FAILED: paired non-regression check failed (exit ${REGRESSION_STATUS})" >&2
  exit "${REGRESSION_STATUS}"
fi
echo "MAP_QUALITY_OK: ${RUNS} runs produced byte-identical map_quality_report.yaml"
