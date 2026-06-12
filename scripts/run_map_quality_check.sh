#!/usr/bin/env bash
# Map-quality report with an N-run byte-identity verdict
# (v0.7 Phase 1, docs/roadmap/v0.7.md).
#
# Usage:
#   bash scripts/run_map_quality_check.sh \
#     --input output/rtkslam_cs2_run2/map.pcd \
#     --output-dir output/map_quality_cs2 \
#     [--runs 3] [--downsample 0.1]
#
# Metric VALUES are report-only (the thresholds come in Phase 3, with a
# holdout); what this script *enforces* is determinism: every run must
# produce byte-identical map_quality_report.yaml. The metric extraction
# profile itself is frozen in MapQualityConfig
# (docs/research/map-quality-baseline.md) and is deliberately not
# exposed here.
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
INPUT=""
OUTPUT_DIR=""
RUNS=3
DOWNSAMPLE=0.1

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
    *)
      usage
      ;;
  esac
done

if [[ -z "${INPUT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--input and --output-dir are required" >&2
  exit 2
fi
if [[ ! -e "${INPUT}" ]]; then
  echo "input not found: ${INPUT}" >&2
  exit 2
fi
if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "install/setup.bash not found; build the workspace first" >&2
  exit 2
fi

# shellcheck disable=SC1091
set +u
source "${REPO_ROOT}/install/setup.bash"
set -u

mkdir -p "${OUTPUT_DIR}"
echo "input:      ${INPUT}"
echo "runs:       ${RUNS}"
echo "downsample: ${DOWNSAMPLE}"
echo "out:        ${OUTPUT_DIR}"

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

SUMMARY="${OUTPUT_DIR}/map_quality_summary.md"
{
  echo "# Map-quality check"
  echo
  echo "- input: \`${INPUT}\`"
  echo "- runs: ${RUNS}, downsample: ${DOWNSAMPLE} m"
  echo "- reports_identical: ${IDENTICAL}"
  echo "- report_md5: \`${MD5S[0]}\`"
  echo
  echo '```yaml'
  cat "${OUTPUT_DIR}/run1/map_quality_report.yaml"
  echo '```'
} > "${SUMMARY}"

echo "--- verdict"
cat "${OUTPUT_DIR}/run1/map_quality_report.yaml"
if [[ "${IDENTICAL}" == "true" ]]; then
  echo "MAP_QUALITY_OK: ${RUNS} runs produced byte-identical map_quality_report.yaml"
else
  echo "MAP_QUALITY_FAILED: reports differ across runs" >&2
  exit 1
fi
