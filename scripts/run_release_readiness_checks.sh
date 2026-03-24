#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  run_release_readiness_checks.sh [options]

Options:
  --out-dir <dir>               Output directory for logs and summaries
  --benchmark-root <dir>        Root directory to scan for metrics.json (default: ./output)
  --ape-threshold <m>           Optional APE threshold passed to benchmark_summary.py
  --ape-threshold-reference-kind <kind>
                                Reference kind to gate on (default: ground_truth)
  --skip-default-ci             Skip scripts/run_default_ci_checks.sh
  --skip-benchmark-summary      Skip benchmark summary generation
  --dogfood                     Run the Autoware pointcloud-map dogfood flow
  --autoware-core-dir <dir>     autoware_core checkout for dogfood
  --work-dir <dir>              Runtime workspace directory for dogfood
  --viewer-run-dir <dir>        Reuse an existing viewer run directory for dogfood
  --wait-for-offline-completion Wait for full rosbag completion during dogfood
  --auto-exit-secs <sec>        Auto-close RViz after N seconds during dogfood
  --help                        Show this help

This script is intended as a release/readiness gate for the default workflow.
It can run:
  1. local build/test verification
  2. benchmark summary and HTML report generation from existing metrics.json runs
  3. optional Autoware map dogfood

When --ape-threshold is provided, the benchmark summary becomes a hard gate and
the script exits non-zero if any selected run is missing APE or exceeds the
threshold. By default this gate is scoped to `ground_truth` runs so
cross-validation artifacts can appear in reports without blocking release.
EOF
  exit 1
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

OUT_DIR="${REPO_ROOT}/output/release_readiness_$(date +%Y%m%d_%H%M%S)"
BENCHMARK_ROOT="${REPO_ROOT}/output"
APE_THRESHOLD=""
APE_THRESHOLD_REFERENCE_KIND="ground_truth"
RUN_DEFAULT_CI=true
RUN_BENCHMARK_SUMMARY=true
RUN_DOGFOOD=false

AUTOWARE_CORE_DIR=""
WORK_DIR=""
VIEWER_RUN_DIR=""
WAIT_FOR_OFFLINE_COMPLETION=false
AUTO_EXIT_SECS=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out-dir)
      [[ $# -ge 2 ]] || usage
      OUT_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --benchmark-root)
      [[ $# -ge 2 ]] || usage
      BENCHMARK_ROOT=$(realpath -m "$2")
      shift 2
      ;;
    --ape-threshold)
      [[ $# -ge 2 ]] || usage
      APE_THRESHOLD="$2"
      shift 2
      ;;
    --ape-threshold-reference-kind)
      [[ $# -ge 2 ]] || usage
      APE_THRESHOLD_REFERENCE_KIND="$2"
      shift 2
      ;;
    --skip-default-ci)
      RUN_DEFAULT_CI=false
      shift
      ;;
    --skip-benchmark-summary)
      RUN_BENCHMARK_SUMMARY=false
      shift
      ;;
    --dogfood)
      RUN_DOGFOOD=true
      shift
      ;;
    --autoware-core-dir)
      [[ $# -ge 2 ]] || usage
      AUTOWARE_CORE_DIR=$(realpath "$2")
      shift 2
      ;;
    --work-dir)
      [[ $# -ge 2 ]] || usage
      WORK_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --viewer-run-dir)
      [[ $# -ge 2 ]] || usage
      VIEWER_RUN_DIR=$(realpath "$2")
      shift 2
      ;;
    --wait-for-offline-completion)
      WAIT_FOR_OFFLINE_COMPLETION=true
      shift
      ;;
    --auto-exit-secs)
      [[ $# -ge 2 ]] || usage
      AUTO_EXIT_SECS="$2"
      shift 2
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

mkdir -p "${OUT_DIR}"

echo "Release readiness output: ${OUT_DIR}"

if [[ "${RUN_DEFAULT_CI}" == "true" ]]; then
  echo "==> Running default workflow checks"
  bash "${REPO_ROOT}/scripts/run_default_ci_checks.sh" \
    2>&1 | tee "${OUT_DIR}/default_ci.log"
fi

if [[ "${RUN_BENCHMARK_SUMMARY}" == "true" ]]; then
  METRICS_FOUND="$(find "${BENCHMARK_ROOT}" -name metrics.json -print -quit 2>/dev/null || true)"
  if [[ -n "${METRICS_FOUND}" ]]; then
    echo "==> Generating benchmark summary from ${BENCHMARK_ROOT}"
    SUMMARY_CMD=(
      python3
      "${REPO_ROOT}/scripts/benchmark_summary.py"
      --root "${BENCHMARK_ROOT}"
      --write-md "${OUT_DIR}/benchmark_summary.md"
      --write-csv "${OUT_DIR}/benchmark_summary.csv"
    )
    if [[ -n "${APE_THRESHOLD}" ]]; then
      SUMMARY_CMD+=(
        --ape-threshold "${APE_THRESHOLD}"
        --ape-threshold-reference-kind "${APE_THRESHOLD_REFERENCE_KIND}"
        --fail-on-ape-threshold
      )
    fi
    "${SUMMARY_CMD[@]}" 2>&1 | tee "${OUT_DIR}/benchmark_summary.log"
    echo "==> Generating benchmark HTML report from ${BENCHMARK_ROOT}"
    python3 "${REPO_ROOT}/scripts/generate_html_report.py" \
      --root "${BENCHMARK_ROOT}" \
      --out "${OUT_DIR}/benchmark_report.html" \
      2>&1 | tee "${OUT_DIR}/benchmark_report.log"
  else
    echo "==> No metrics.json found under ${BENCHMARK_ROOT}; skipping benchmark summary" \
      | tee "${OUT_DIR}/benchmark_summary.log"
    echo "==> No metrics.json found under ${BENCHMARK_ROOT}; skipping benchmark HTML report" \
      | tee "${OUT_DIR}/benchmark_report.log"
  fi
fi

if [[ "${RUN_DOGFOOD}" == "true" ]]; then
  echo "==> Running Autoware pointcloud-map dogfood"
  DOGFOOD_CMD=(
    bash
    "${REPO_ROOT}/scripts/run_rko_lio_graph_autoware_dogfood.sh"
  )
  if [[ -n "${AUTOWARE_CORE_DIR}" ]]; then
    DOGFOOD_CMD+=(--autoware-core-dir "${AUTOWARE_CORE_DIR}")
  fi
  if [[ -n "${WORK_DIR}" ]]; then
    DOGFOOD_CMD+=(--work-dir "${WORK_DIR}")
  fi
  if [[ -n "${VIEWER_RUN_DIR}" ]]; then
    DOGFOOD_CMD+=(--viewer-run-dir "${VIEWER_RUN_DIR}")
  fi
  if [[ "${WAIT_FOR_OFFLINE_COMPLETION}" == "true" ]]; then
    DOGFOOD_CMD+=(--wait-for-offline-completion)
  fi
  if [[ -n "${AUTO_EXIT_SECS}" ]]; then
    DOGFOOD_CMD+=(--auto-exit-secs "${AUTO_EXIT_SECS}")
  fi
  "${DOGFOOD_CMD[@]}" 2>&1 | tee "${OUT_DIR}/dogfood.log"
fi

echo "==> Release readiness checks completed"
echo "  output_dir: ${OUT_DIR}"
if [[ -f "${OUT_DIR}/benchmark_summary.md" ]]; then
  echo "  benchmark_summary_md: ${OUT_DIR}/benchmark_summary.md"
fi
if [[ -f "${OUT_DIR}/benchmark_summary.csv" ]]; then
  echo "  benchmark_summary_csv: ${OUT_DIR}/benchmark_summary.csv"
fi
if [[ -f "${OUT_DIR}/benchmark_report.html" ]]; then
  echo "  benchmark_report_html: ${OUT_DIR}/benchmark_report.html"
fi
