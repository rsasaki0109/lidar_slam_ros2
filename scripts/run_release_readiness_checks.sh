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
  --release-profile <path>      Release-profile YAML (per-dataset pass/target)
                                Default: scripts/release_profiles.yaml when omitted
  --no-release-profile          Disable the release-profile gate
  --fail-on-profiles            Exit non-zero if any release-profile FAILs
  --skip-default-ci             Skip scripts/run_default_ci_checks.sh
  --skip-benchmark-summary      Skip benchmark summary generation
  --public-mid360-completion    Run the public MID-360 segment-reset completion gate
  --public-mid360-completion-output-dir <dir>
                                Output directory for the public MID-360 gate
                                (default: <out-dir>/mid360_public_completion_gate)
  --public-mid360-loop-cloud <json>
                                Override public loop-cloud analysis JSON
  --public-mid360-segment-reset-plan <json>
                                Override segment-reset plan JSON
  --public-mid360-start-run-dir <dir>
                                Override start segment RKO run directory
  --public-mid360-end-run-dir <dir>
                                Override end segment RKO run directory
  --public-mid360-segment-map-alignment <json>
                                Override segment map alignment JSON
  --public-mid360-adoption-gate <json>
                                Override public RKO adoption-gate JSON
  --public-mid360-dashboard-html <html>
                                Override segment-reset dashboard HTML
  --public-mid360-min-segment-rko-poses <n>
                                Minimum TUM poses for each reset segment
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
  3. optional public MID-360 segment-reset completion gate
  4. optional Autoware map dogfood

When --ape-threshold is provided, the benchmark summary becomes a hard gate and
the script exits non-zero if any selected run is missing APE or exceeds the
threshold. By default this gate is scoped to `ground_truth` runs so
cross-validation artifacts can appear in reports without blocking release.

The release-profile gate runs in addition to (or instead of) --ape-threshold:
each profile in the YAML scores its own pass/target threshold against the best
matching run, with optional report_only_until semantics so hard datasets
(MID-360, NTU) can be reported without blocking release.
EOF
  exit 1
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

OUT_DIR="${REPO_ROOT}/output/release_readiness_$(date +%Y%m%d_%H%M%S)"
BENCHMARK_ROOT="${REPO_ROOT}/output"
APE_THRESHOLD=""
APE_THRESHOLD_REFERENCE_KIND="ground_truth"
RELEASE_PROFILE="${REPO_ROOT}/scripts/release_profiles.yaml"
FAIL_ON_PROFILES=false
RUN_DEFAULT_CI=true
RUN_BENCHMARK_SUMMARY=true
RUN_PUBLIC_MID360_COMPLETION=false
RUN_DOGFOOD=false

PUBLIC_MID360_COMPLETION_OUTPUT_DIR=""
PUBLIC_MID360_LOOP_CLOUD=""
PUBLIC_MID360_SEGMENT_RESET_PLAN=""
PUBLIC_MID360_START_RUN_DIR=""
PUBLIC_MID360_END_RUN_DIR=""
PUBLIC_MID360_SEGMENT_MAP_ALIGNMENT=""
PUBLIC_MID360_ADOPTION_GATE=""
PUBLIC_MID360_DASHBOARD_HTML=""
PUBLIC_MID360_MIN_SEGMENT_RKO_POSES=""

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
    --release-profile)
      [[ $# -ge 2 ]] || usage
      RELEASE_PROFILE=$(realpath -m "$2")
      shift 2
      ;;
    --no-release-profile)
      RELEASE_PROFILE=""
      shift
      ;;
    --fail-on-profiles)
      FAIL_ON_PROFILES=true
      shift
      ;;
    --skip-default-ci)
      RUN_DEFAULT_CI=false
      shift
      ;;
    --skip-benchmark-summary)
      RUN_BENCHMARK_SUMMARY=false
      shift
      ;;
    --public-mid360-completion)
      RUN_PUBLIC_MID360_COMPLETION=true
      shift
      ;;
    --public-mid360-completion-output-dir)
      [[ $# -ge 2 ]] || usage
      PUBLIC_MID360_COMPLETION_OUTPUT_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-loop-cloud)
      [[ $# -ge 2 ]] || usage
      PUBLIC_MID360_LOOP_CLOUD=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-segment-reset-plan)
      [[ $# -ge 2 ]] || usage
      PUBLIC_MID360_SEGMENT_RESET_PLAN=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-start-run-dir)
      [[ $# -ge 2 ]] || usage
      PUBLIC_MID360_START_RUN_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-end-run-dir)
      [[ $# -ge 2 ]] || usage
      PUBLIC_MID360_END_RUN_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-segment-map-alignment)
      [[ $# -ge 2 ]] || usage
      PUBLIC_MID360_SEGMENT_MAP_ALIGNMENT=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-adoption-gate)
      [[ $# -ge 2 ]] || usage
      PUBLIC_MID360_ADOPTION_GATE=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-dashboard-html)
      [[ $# -ge 2 ]] || usage
      PUBLIC_MID360_DASHBOARD_HTML=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-min-segment-rko-poses)
      [[ $# -ge 2 ]] || usage
      PUBLIC_MID360_MIN_SEGMENT_RKO_POSES="$2"
      shift 2
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
    if [[ -n "${RELEASE_PROFILE}" && -f "${RELEASE_PROFILE}" ]]; then
      SUMMARY_CMD+=(--release-profile "${RELEASE_PROFILE}")
      if [[ "${FAIL_ON_PROFILES}" == "true" ]]; then
        SUMMARY_CMD+=(--fail-on-profiles)
      fi
    elif [[ -n "${RELEASE_PROFILE}" ]]; then
      echo "warning: release profile not found at ${RELEASE_PROFILE}; continuing without profile gate" >&2
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

if [[ "${RUN_PUBLIC_MID360_COMPLETION}" == "true" ]]; then
  if [[ -z "${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}" ]]; then
    PUBLIC_MID360_COMPLETION_OUTPUT_DIR="${OUT_DIR}/mid360_public_completion_gate"
  fi
  echo "==> Running public MID-360 completion gate"
  PUBLIC_MID360_CMD=(
    python3
    "${REPO_ROOT}/scripts/run_mid360_robot_public_completion_gate.py"
    --json
    --output-dir "${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}"
  )
  if [[ -n "${PUBLIC_MID360_LOOP_CLOUD}" ]]; then
    PUBLIC_MID360_CMD+=(--loop-cloud "${PUBLIC_MID360_LOOP_CLOUD}")
  fi
  if [[ -n "${PUBLIC_MID360_SEGMENT_RESET_PLAN}" ]]; then
    PUBLIC_MID360_CMD+=(--segment-reset-plan "${PUBLIC_MID360_SEGMENT_RESET_PLAN}")
  fi
  if [[ -n "${PUBLIC_MID360_START_RUN_DIR}" ]]; then
    PUBLIC_MID360_CMD+=(--start-run-dir "${PUBLIC_MID360_START_RUN_DIR}")
  fi
  if [[ -n "${PUBLIC_MID360_END_RUN_DIR}" ]]; then
    PUBLIC_MID360_CMD+=(--end-run-dir "${PUBLIC_MID360_END_RUN_DIR}")
  fi
  if [[ -n "${PUBLIC_MID360_SEGMENT_MAP_ALIGNMENT}" ]]; then
    PUBLIC_MID360_CMD+=(--segment-map-alignment "${PUBLIC_MID360_SEGMENT_MAP_ALIGNMENT}")
  fi
  if [[ -n "${PUBLIC_MID360_ADOPTION_GATE}" ]]; then
    PUBLIC_MID360_CMD+=(--adoption-gate "${PUBLIC_MID360_ADOPTION_GATE}")
  fi
  if [[ -n "${PUBLIC_MID360_DASHBOARD_HTML}" ]]; then
    PUBLIC_MID360_CMD+=(--dashboard-html "${PUBLIC_MID360_DASHBOARD_HTML}")
  fi
  if [[ -n "${PUBLIC_MID360_MIN_SEGMENT_RKO_POSES}" ]]; then
    PUBLIC_MID360_CMD+=(--min-segment-rko-poses "${PUBLIC_MID360_MIN_SEGMENT_RKO_POSES}")
  fi
  "${PUBLIC_MID360_CMD[@]}" 2>&1 | tee "${OUT_DIR}/public_mid360_completion_gate.log"
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
if [[ -n "${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}" \
  && -f "${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}/mid360_robot_public_completion_gate.json" ]]; then
  echo "  public_mid360_completion_gate_json: ${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}/mid360_robot_public_completion_gate.json"
fi
if [[ -n "${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}" \
  && -f "${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}/mid360_robot_public_completion_gate.md" ]]; then
  echo "  public_mid360_completion_gate_md: ${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}/mid360_robot_public_completion_gate.md"
fi
