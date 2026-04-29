#!/usr/bin/env bash
# Print a ready-to-run compare_kitti_odometry_two_estimates.sh invocation for a
# small_gicp sweep output tree (prepare_seq* + seqNN/<config>/).
#
# After you record baseline.tum from FAST-LIO2 (or any external stack) on the
# same prepared rosbag2, plug it into --baseline-tum and run the emitted command.
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  emit_kitti_baseline_compare_command.sh --sweep-root DIR [options]

Options:
  --sweep-root DIR     e.g. output/kitti_small_gicp_sweep_20260407_112957
  --run NAME           Config name under seqNN/ (default: CSV row with min ape_rmse_m)
  --sequence ID        Two digits (default: 00)
  --traj KIND          corrected | raw (default: corrected)
                         benchmark_summary "ape" uses graph-corrected path if present (few poses).
                         For dense LIO baselines (e.g. FAST-LIO2 odom), use --traj raw on our side too.
  --baseline-tum PATH  If set, append to the command; else print PATH_TO_BASELINE.tum
  --label-a LABEL      Default: lidarslam_<run>
  --label-b LABEL      Default: external_lio
  --execute            Run compare_kitti_odometry_two_estimates.sh instead of printing

Requires:
  - sweep-root/prepare_seq{NN}/kitti_seq{NN}_gt_velo.tum
  - sweep-root/prepare_seq{NN}/kitti_seq{NN}_reference.json
  - sweep-root/seq{NN}/{run}/traj_raw_prism.tum (or traj_corrected_prism.tum)
EOF
  exit 1
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SWEEP_ROOT=""
RUN_NAME=""
SEQUENCE="00"
BASELINE_TUM=""
LABEL_A=""
LABEL_B=""
DO_EXECUTE=false
TRAJ_KIND="corrected"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sweep-root)
      [[ $# -ge 2 ]] || usage
      SWEEP_ROOT=$(realpath "$2")
      shift 2
      ;;
    --run)
      [[ $# -ge 2 ]] || usage
      RUN_NAME="$2"
      shift 2
      ;;
    --sequence)
      [[ $# -ge 2 ]] || usage
      SEQUENCE="$2"
      shift 2
      ;;
    --traj)
      [[ $# -ge 2 ]] || usage
      TRAJ_KIND="$2"
      shift 2
      ;;
    --baseline-tum)
      [[ $# -ge 2 ]] || usage
      BASELINE_TUM=$(realpath -m "$2")
      shift 2
      ;;
    --label-a)
      [[ $# -ge 2 ]] || usage
      LABEL_A="$2"
      shift 2
      ;;
    --label-b)
      [[ $# -ge 2 ]] || usage
      LABEL_B="$2"
      shift 2
      ;;
    --execute)
      DO_EXECUTE=true
      shift
      ;;
    -h|--help)
      usage
      ;;
    *)
      echo "Unknown: $1" >&2
      usage
      ;;
  esac
done

[[ -n "$SWEEP_ROOT" && -d "$SWEEP_ROOT" ]] || usage

CSV="$SWEEP_ROOT/benchmark_summary.csv"
if [[ -z "$RUN_NAME" ]]; then
  [[ -f "$CSV" ]] || { echo "missing $CSV (need sweep summary or pass --run)" >&2; exit 1; }
  RUN_NAME=$(python3 - "$CSV" <<'PY'
import csv, sys
from pathlib import Path
path = Path(sys.argv[1])
rows = list(csv.DictReader(path.read_text(encoding='utf-8').splitlines()))
best = min(rows, key=lambda r: float(r.get('ape_rmse_m') or 1e9))
print(best['run'])
PY
  )
fi

PREP="$SWEEP_ROOT/prepare_seq${SEQUENCE}"
REF_TUM="$PREP/kitti_seq${SEQUENCE}_gt_velo.tum"
REF_META="$PREP/kitti_seq${SEQUENCE}_reference.json"
SEQ_DIR="$SWEEP_ROOT/seq${SEQUENCE}/$RUN_NAME"
OUR=""

case "$TRAJ_KIND" in
  corrected)
    if [[ -f "$SEQ_DIR/traj_corrected_prism.tum" && -s "$SEQ_DIR/traj_corrected_prism.tum" ]]; then
      OUR="$SEQ_DIR/traj_corrected_prism.tum"
    fi
    ;;
  raw)
    if [[ -f "$SEQ_DIR/traj_raw_prism.tum" ]]; then
      OUR="$SEQ_DIR/traj_raw_prism.tum"
    fi
    ;;
  *)
    echo "--traj must be corrected or raw" >&2
    exit 1
    ;;
esac

if [[ -z "$OUR" && "$TRAJ_KIND" == "corrected" ]]; then
  if [[ -f "$SEQ_DIR/traj_raw_prism.tum" ]]; then
    OUR="$SEQ_DIR/traj_raw_prism.tum"
    echo "warn: no non-empty traj_corrected_prism.tum; falling back to traj_raw_prism.tum" >&2
  fi
fi

if [[ -z "$OUR" ]]; then
  echo "no trajectory TUM under $SEQ_DIR for --traj $TRAJ_KIND" >&2
  exit 1
fi

[[ -f "$REF_TUM" && -f "$REF_META" ]] || { echo "missing GT or meta under $PREP" >&2; exit 1; }

if [[ -z "$LABEL_A" ]]; then
  LABEL_A="lidarslam_${RUN_NAME}_${TRAJ_KIND}"
fi
if [[ -z "$LABEL_B" ]]; then
  LABEL_B="external_lio"
fi

OUT_DIR="$SWEEP_ROOT/seq${SEQUENCE}/compare_vs_${LABEL_B}_$(date +%Y%m%d_%H%M%S)"
BASELINE_PLACEHOLDER='PATH_TO_BASELINE.tum'
if [[ -n "$BASELINE_TUM" ]]; then
  BLINE="$BASELINE_TUM"
else
  BLINE="$BASELINE_PLACEHOLDER"
fi

CMD=(bash "$SCRIPT_DIR/compare_kitti_odometry_two_estimates.sh" \
  --reference-tum "$REF_TUM" \
  --reference-meta "$REF_META" \
  --estimate-a-tum "$OUR" \
  --estimate-b-tum "$BLINE" \
  --label-a "$LABEL_A" \
  --label-b "$LABEL_B" \
  --out-dir "$OUT_DIR")

printf '# Best ape sweep run (from benchmark_summary.csv unless --run): %s\n' "$RUN_NAME"
printf '# Our trajectory (--traj %s): %s\n' "$TRAJ_KIND" "$OUR"
if [[ "$BLINE" == "$BASELINE_PLACEHOLDER" ]]; then
  printf '# Replace %s with your GPL baseline TUM after odom_to_tum.\n' "$BASELINE_PLACEHOLDER"
fi
printf '\n'
printf '%q ' "${CMD[@]}"
printf '\n'

if "$DO_EXECUTE"; then
  if [[ "$BLINE" == "$BASELINE_PLACEHOLDER" ]]; then
    echo "refusing --execute without real --baseline-tum" >&2
    exit 1
  fi
  exec "${CMD[@]}"
fi
