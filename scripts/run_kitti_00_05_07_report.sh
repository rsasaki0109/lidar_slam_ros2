#!/usr/bin/env bash
set -euo pipefail

# Run lidarslam on KITTI Odometry 00 / 05 / 07 (the v0.3 dev split) with frozen
# parameters, compute KITTI t_rel / r_rel drift for each, and aggregate into a
# single markdown report.
#
# This wrapper assumes the existing per-sequence benchmark scripts already work
# end-to-end. It only:
#   1. iterates the dev sequences with one shared params file
#   2. invokes scripts/kitti_metrics.py on the resulting traj_corrected.tum
#   3. aggregates the per-sequence JSONs via scripts/aggregate_kitti_metrics.py
#
# Same-params evaluation is what KITTI's official protocol expects. Per-seq
# parameter overrides are intentionally not exposed.
#
# Required:
#   --dataset PATH   KITTI Odometry root (the directory above sequences/poses)
#
# Optional:
#   --sequences "00 05 07"
#   --output-root PATH  (default: output/kitti_dev_<timestamp>)
#   --variant lo|small-gicp|rko-lio   (default: small-gicp)
#   --label NAME        label used in the aggregate report (default: ours)
#   --extra-est-json "label::path,..." comma-separated extra metric JSONs
#                                      to compare against (e.g. KISS-ICP)
#   --reuse              reuse pre-existing per-seq dirs if present

usage() {
  cat <<'EOF' >&2
Usage:
  run_kitti_00_05_07_report.sh --dataset PATH [options]

Options:
  --dataset PATH         KITTI Odometry root (default: $KITTI_ODOMETRY_ROOT)
  --sequences "00 05 07" Whitespace-separated sequence ids
  --output-root PATH     Where to put per-seq + aggregate artifacts
  --variant NAME         lo | small-gicp | rko-lio   (default: small-gicp)
  --label NAME           Label for aggregate report   (default: ours)
  --extra-est-json LIST  Comma-separated "label::path" of extra metric JSONs
  --reuse                Skip prepare/run for sequences whose output exists
  --help
EOF
  exit 1
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

DATASET="${KITTI_ODOMETRY_ROOT:-}"
SEQUENCES="00 05 07"
OUTPUT_ROOT=""
VARIANT="small-gicp"
LABEL="ours"
EXTRA_EST=""
REUSE=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dataset)      [[ $# -ge 2 ]] || usage; DATASET=$(realpath "$2"); shift 2 ;;
    --sequences)    [[ $# -ge 2 ]] || usage; SEQUENCES="$2"; shift 2 ;;
    --output-root)  [[ $# -ge 2 ]] || usage; OUTPUT_ROOT=$(realpath -m "$2"); shift 2 ;;
    --variant)      [[ $# -ge 2 ]] || usage; VARIANT="$2"; shift 2 ;;
    --label)        [[ $# -ge 2 ]] || usage; LABEL="$2"; shift 2 ;;
    --extra-est-json) [[ $# -ge 2 ]] || usage; EXTRA_EST="$2"; shift 2 ;;
    --reuse)        REUSE=true; shift ;;
    --help|-h)      usage ;;
    *) echo "Unknown option: $1" >&2; usage ;;
  esac
done

[[ -n "${DATASET}" ]] || { echo "error: --dataset is required" >&2; usage; }

case "${VARIANT}" in
  lo|small-gicp|rko-lio) ;;
  *) echo "error: invalid --variant '${VARIANT}'" >&2; usage ;;
esac

if [[ -z "${OUTPUT_ROOT}" ]]; then
  OUTPUT_ROOT="${REPO_ROOT}/output/kitti_dev_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "${OUTPUT_ROOT}"

INPUT_ARGS=()

for seq in ${SEQUENCES}; do
  echo "==> Sequence ${seq} (variant=${VARIANT})"
  SEQ_OUT="${OUTPUT_ROOT}/seq${seq}"
  mkdir -p "${SEQ_OUT}"

  BENCH_CMD=(bash "${REPO_ROOT}/scripts/run_kitti_odometry_benchmark.sh"
             --dataset "${DATASET}" --sequence "${seq}"
             --output-dir "${SEQ_OUT}")
  case "${VARIANT}" in
    lo)         BENCH_CMD+=(--lo) ;;
    small-gicp) BENCH_CMD+=(--small-gicp) ;;
    # rko-lio is the default in run_kitti_odometry_benchmark.sh; no extra flag
  esac
  if [[ "${REUSE}" == "true" && -f "${SEQ_OUT}/bench_run_small_gicp/traj_corrected.tum" ]]; then
    echo "  (reusing existing ${SEQ_OUT})"
  else
    "${BENCH_CMD[@]}"
  fi

  # Find canonical trajectory paths produced by the benchmark.
  EST_TUM="${SEQ_OUT}/bench_run_small_gicp/traj_corrected.tum"
  if [[ ! -f "${EST_TUM}" ]]; then
    EST_TUM=$(find "${SEQ_OUT}" -name traj_corrected.tum -print -quit)
  fi
  GT_TUM="${SEQ_OUT}/kitti_seq${seq}_gt_velo.tum"
  if [[ ! -f "${GT_TUM}" ]]; then
    GT_TUM=$(find "${SEQ_OUT}" -name 'kitti_seq*_gt_velo.tum' -print -quit)
  fi

  if [[ ! -f "${EST_TUM}" || ! -f "${GT_TUM}" ]]; then
    echo "  warning: missing GT or estimate for seq ${seq}; skipping metrics" >&2
    continue
  fi

  METRIC_JSON="${SEQ_OUT}/kitti_metrics.json"
  python3 "${REPO_ROOT}/scripts/kitti_metrics.py" \
      --gt "${GT_TUM}" --est "${EST_TUM}" \
      --label "${LABEL}_seq${seq}" \
      --out-json "${METRIC_JSON}"

  INPUT_ARGS+=(--input "${LABEL}::${METRIC_JSON}")
done

if [[ -n "${EXTRA_EST}" ]]; then
  IFS=',' read -r -a EXTRA_LIST <<< "${EXTRA_EST}"
  for entry in "${EXTRA_LIST[@]}"; do
    [[ -n "${entry}" ]] && INPUT_ARGS+=(--input "${entry}")
  done
fi

REPORT_MD="${OUTPUT_ROOT}/kitti_dev_report.md"
python3 "${REPO_ROOT}/scripts/aggregate_kitti_metrics.py" \
    "${INPUT_ARGS[@]}" --out-md "${REPORT_MD}"

echo "==> KITTI dev-split report ready"
echo "  output_root: ${OUTPUT_ROOT}"
echo "  report:      ${REPORT_MD}"
