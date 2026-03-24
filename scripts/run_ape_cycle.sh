#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF_USAGE'
Usage:
  bash scripts/run_ape_cycle.sh --cycles N [options] -- [options passed to compare_with_glim.sh]

Purpose:
  Run compare_with_glim repeatedly and summarize by APE-first priority.

Options:
  --cycles N              Number of repetitions (default: 10)
  --pause-sec SEC         Sleep between cycles (default: 0)
  --run-root DIR          Output root dir (default: ./output/ape_cycle_YYYYMMDD_HHMMSS)
  --summary-md FILE       Write markdown summary (default: <run-root>/summary.md)
  --summary-csv FILE      Write csv summary (default: <run-root>/summary.csv)
  --ape-threshold M       Optional APE rmse threshold (m) for summary label
  --cycle-timeout-sec S    Per-cycle timeout in seconds (default: 900)
  --stall-limit N         Stop if no APE improvement for N consecutive cycles (default: 3)
  --stall-delta M         Minimum APE rmse improvement to reset stall counter (default: 0.001)
  --no-early-stop         Disable stall-based early stopping
  --help                  Show this help

Examples:
  bash scripts/run_ape_cycle.sh --cycles 20 -- \
    --official --variant livox --download \
    --glim-mode lidar-only --auto-static-tf

  bash scripts/run_ape_cycle.sh --cycles 5 --run-root output/ape_cycle_local -- --bag /path/to/bag
EOF_USAGE
}

die() {
  echo "error: $*" >&2
  exit 1
}

arg_present() {
  local target="${1:-}"
  shift
  for arg in "$@"; do
    [[ "${arg}" == "${target}" ]] && return 0
  done
  return 1
}

CYCLES=10
PAUSE_SEC="0"
RUN_ROOT="${REPO_ROOT}/output/ape_cycle_$(date +%Y%m%d_%H%M%S)"
SUMMARY_MD=""
SUMMARY_CSV=""
APE_THRESHOLD=""
CYCLE_TIMEOUT_SEC="900"
STALL_LIMIT="3"
STALL_DELTA="0.001"
EARLY_STOP="true"

OFFICIAL_DETECTED="false"
OFFICIAL_VARIANT="livox"
OFFICIAL_DEST="${REPO_ROOT}/demo_data/koide_lidar_camera_calib"
OFFICIAL_BAG_DIR=""
OFFICIAL_DOWNLOAD="false"
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --cycles)
      CYCLES="${2:-}";
      shift 2
      ;;
    --pause-sec)
      PAUSE_SEC="${2:-}";
      shift 2
      ;;
    --run-root)
      RUN_ROOT="${2:-}";
      shift 2
      ;;
    --summary-md)
      SUMMARY_MD="${2:-}";
      shift 2
      ;;
    --summary-csv)
      SUMMARY_CSV="${2:-}";
      shift 2
      ;;
    --ape-threshold)
      APE_THRESHOLD="${2:-}";
      shift 2
      ;;
    --cycle-timeout-sec)
      CYCLE_TIMEOUT_SEC="${2:-}";
      shift 2
      ;;
    --stall-limit)
      STALL_LIMIT="${2:-}";
      shift 2
      ;;
    --stall-delta)
      STALL_DELTA="${2:-}";
      shift 2
      ;;
    --no-early-stop)
      EARLY_STOP="false"
      shift
      ;;
    --variant)
      OFFICIAL_VARIANT="${2:-}";
      EXTRA_ARGS+=("$1" "$2")
      shift 2
      ;;
    --dest)
      OFFICIAL_DEST="${2:-}";
      EXTRA_ARGS+=("$1" "$2")
      shift 2
      ;;
    --bag-dir)
      OFFICIAL_BAG_DIR="${2:-}";
      EXTRA_ARGS+=("$1" "$2")
      shift 2
      ;;
    --download)
      OFFICIAL_DOWNLOAD="true"
      # keep --download in extras only for the first resolve step
      EXTRA_ARGS+=("$1")
      shift
      ;;
    --)
      shift
      while [[ $# -gt 0 ]]; do
        EXTRA_ARGS+=("$1")
        shift
      done
      ;;
    --official)
      OFFICIAL_DETECTED="true"
      EXTRA_ARGS+=("$1")
      shift
      ;;
    *)
      if [[ "${OFFICIAL_DETECTED}" == "true" ]] && [[ "${1}" == --variant || "${1}" == --dest || "${1}" == --bag-dir ]]; then
        # should be handled above if paired value is missing
        EXTRA_ARGS+=("$1")
        shift
        continue
      fi
      if [[ "${1}" == --bag ]]; then
        has_target="true"
      fi
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

if ! [[ "${CYCLES}" =~ ^[0-9]+$ ]] || [[ "${CYCLES}" -lt 1 ]]; then
  die "--cycles must be an integer >= 1"
fi
if ! [[ "${STALL_LIMIT}" =~ ^[0-9]+$ ]] || [[ "${STALL_LIMIT}" -lt 1 ]]; then
  die "--stall-limit must be an integer >= 1"
fi
if ! [[ "${CYCLE_TIMEOUT_SEC}" =~ ^[0-9]+$ ]] || [[ "${CYCLE_TIMEOUT_SEC}" -lt 1 ]]; then
  die "--cycle-timeout-sec must be an integer >= 1"
fi
if ! python3 - "$STALL_DELTA" <<'PY'
import sys
try:
  float(sys.argv[1])
except Exception:
  raise SystemExit(1)
PY
then die "--stall-delta must be numeric"; fi

has_target="false"
for arg in "${EXTRA_ARGS[@]}"; do
  if [[ "${arg}" == "--bag" || "${arg}" == "--official" ]]; then
    has_target="true"
    break
  fi
done
[[ "${has_target}" == "true" ]] || die "target specifier is required. pass --bag or --official to compare_with_glim"

BASE_ARGS=("${EXTRA_ARGS[@]}")
if [[ "${OFFICIAL_DETECTED}" == "true" ]] && [[ -z "${OFFICIAL_BAG_DIR}" ]]; then
  # Resolve official bag only once so we don't trigger repeated download/reselect every cycle.
  official_args=(--variant "${OFFICIAL_VARIANT}" --dest "${OFFICIAL_DEST}" --print-bag-dir)
  if [[ "${OFFICIAL_DOWNLOAD}" == "true" ]]; then
    official_args+=(--download)
  fi
  OFFICIAL_BAG_DIR="$(bash "${REPO_ROOT}/scripts/run_official_demo.sh" "${official_args[@]}" | tail -n 1 | tr -d '\r' )"
  [[ -n "${OFFICIAL_BAG_DIR}" ]] || die "failed to resolve official bag directory"
fi
if [[ "${OFFICIAL_DETECTED}" == "true" ]] && [[ -n "${OFFICIAL_BAG_DIR}" ]]; then
  # avoid repeated --download and force explicit bag-dir for deterministic selection
  BASE_ARGS=()
  for arg in "${EXTRA_ARGS[@]}"; do
    if [[ "${arg}" == "--download" ]]; then
      continue
    fi
    BASE_ARGS+=("${arg}")
  done
  if [[ -n "${OFFICIAL_BAG_DIR}" ]]; then
    BASE_ARGS+=(--bag-dir "${OFFICIAL_BAG_DIR}")
  fi
fi
if [[ "${OFFICIAL_DETECTED}" == "true" && "${OFFICIAL_VARIANT}" == "livox" ]]; then
  if ! arg_present --points-frame-id "${BASE_ARGS[@]}"; then
    BASE_ARGS+=(--points-frame-id "livox_frame")
  fi
  if ! arg_present --robot-frame-id "${BASE_ARGS[@]}"; then
    BASE_ARGS+=(--robot-frame-id "livox_frame")
  fi
  if ! arg_present --base-frame "${BASE_ARGS[@]}"; then
    BASE_ARGS+=(--base-frame "livox_frame")
  fi
  if ! arg_present --lidar-frame "${BASE_ARGS[@]}"; then
    BASE_ARGS+=(--lidar-frame "livox_frame")
  fi
  if ! arg_present --auto-static-tf "${BASE_ARGS[@]}" && ! arg_present --no-auto-static-tf "${BASE_ARGS[@]}"; then
    BASE_ARGS+=(--no-auto-static-tf)
  fi
  if ! arg_present --glim-timeout-sec "${BASE_ARGS[@]}"; then
    BASE_ARGS+=(--glim-timeout-sec "30")
  fi
fi

if ! arg_present --no-glim-viewer "${BASE_ARGS[@]}" && ! arg_present --glim-viewer "${BASE_ARGS[@]}"; then
  BASE_ARGS+=(--no-glim-viewer)
fi

mkdir -p "${RUN_ROOT}"

if [[ -z "${SUMMARY_MD}" ]]; then
  SUMMARY_MD="${RUN_ROOT}/summary.md"
fi
if [[ -z "${SUMMARY_CSV}" ]]; then
  SUMMARY_CSV="${RUN_ROOT}/summary.csv"
fi

BEST_APE=""
NO_IMPROVE_COUNT=0

for ((i=1; i<=CYCLES; i++)); do
  out_dir="${RUN_ROOT}/compare_cycle_$(printf '%03d' "${i}")"
  mkdir -p "${out_dir}"
  echo "[$(date -Iseconds)] start cycle ${i}/${CYCLES} -> ${out_dir}" | tee -a "${RUN_ROOT}/cycle.log"

  set +e
  if command -v timeout >/dev/null 2>&1; then
    timeout "${CYCLE_TIMEOUT_SEC}" bash "${REPO_ROOT}/scripts/compare_with_glim.sh" \
      "${BASE_ARGS[@]}" \
      --out-dir "${out_dir}" \
      2>&1 | tee "${out_dir}/compare.log"
  else
    bash "${REPO_ROOT}/scripts/compare_with_glim.sh" \
      "${BASE_ARGS[@]}" \
      --out-dir "${out_dir}" \
      2>&1 | tee "${out_dir}/compare.log"
  fi
  rc=${PIPESTATUS[0]}
  set -e

  echo "cycle=${i} rc=${rc} out=${out_dir}" | tee -a "${RUN_ROOT}/cycle.log"
  if [[ "${rc}" -ne 0 ]]; then
    echo "warn: compare_with_glim returned rc=${rc} at cycle ${i}" | tee -a "${RUN_ROOT}/cycle.log"
  fi

  if [[ -f "${out_dir}/metrics.json" ]]; then
    read -r APE_VALUE LIDARSLAM_SUCCESS GLIM_SUCCESS < <(
    python3 - "${out_dir}/metrics.json" <<'PY'
import json
import sys

path = sys.argv[1]
ape_value = "NA"
ls = 0
gs = 0

try:
  with open(path, "r", encoding="utf-8") as f:
    data = json.load(f)
except Exception:
  print("NA 0 0")
  sys.exit(0)

try:
  metrics = data.get("evo", {}).get("ape", {})
  if metrics:
    rmse = metrics.get("rmse")
    if rmse is not None:
      ape_value = str(rmse)
except Exception:
  pass

try:
  ls = 1 if data.get("lidarslam", {}).get("success") is True else 0
except Exception:
  ls = 0
try:
  gs = 1 if data.get("glim", {}).get("success") is True else 0
except Exception:
  gs = 0

print(ape_value, ls, gs)
PY
    ) || true
  else
    APE_VALUE="NA"
    LIDARSLAM_SUCCESS="0"
    GLIM_SUCCESS="0"
  fi
  APE_VALUE="${APE_VALUE:-NA}"
  LIDARSLAM_SUCCESS="${LIDARSLAM_SUCCESS:-0}"
  GLIM_SUCCESS="${GLIM_SUCCESS:-0}"

  if [[ "${APE_VALUE}" == "NA" ]]; then
    echo "cycle=${i} rc=${rc} ape=NA best=${BEST_APE:-NA} no_improve=${NO_IMPROVE_COUNT} lid_ok=${LIDARSLAM_SUCCESS} glim_ok=${GLIM_SUCCESS}" | tee -a "${RUN_ROOT}/cycle.log"
    NO_IMPROVE_COUNT=$((NO_IMPROVE_COUNT + 1))
  else
    if [[ -z "${BEST_APE}" ]]; then
      BEST_APE="${APE_VALUE}"
      NO_IMPROVE_COUNT=0
      echo "cycle=${i} rc=${rc} ape=${APE_VALUE} best=${BEST_APE} no_improve=0 lid_ok=${LIDARSLAM_SUCCESS} glim_ok=${GLIM_SUCCESS}" | tee -a "${RUN_ROOT}/cycle.log"
    else
      IMPROVED="$(BEST_APE="${BEST_APE}" APE_VALUE="${APE_VALUE}" STALL_DELTA="${STALL_DELTA}" python3 - <<'PY'
import os
prev = float(os.environ["BEST_APE"])
cur = float(os.environ["APE_VALUE"])
delta = float(os.environ["STALL_DELTA"])
print(1 if cur < (prev - delta) else 0)
PY
      )"
      if [[ "${IMPROVED}" == "1" ]]; then
        BEST_APE="${APE_VALUE}"
        NO_IMPROVE_COUNT=0
      else
        NO_IMPROVE_COUNT=$((NO_IMPROVE_COUNT + 1))
      fi
      echo "cycle=${i} rc=${rc} ape=${APE_VALUE} best=${BEST_APE} no_improve=${NO_IMPROVE_COUNT} lid_ok=${LIDARSLAM_SUCCESS} glim_ok=${GLIM_SUCCESS}" | tee -a "${RUN_ROOT}/cycle.log"
    fi
  fi

  if [[ "${EARLY_STOP}" == "true" && "${NO_IMPROVE_COUNT}" -ge "${STALL_LIMIT}" ]]; then
    echo "warn: APE has not improved by ${STALL_DELTA} for ${NO_IMPROVE_COUNT} cycles. Please review parameters and decide next action." | tee -a "${RUN_ROOT}/cycle.log"
    echo "      best_ape=${BEST_APE:-NA} run_root=${RUN_ROOT}" | tee -a "${RUN_ROOT}/cycle.log"
    break
  fi

  if [[ "${PAUSE_SEC}" != "0" ]]; then
    sleep "${PAUSE_SEC}"
  fi

done

if [[ -n "${APE_THRESHOLD}" ]]; then
  python3 "${REPO_ROOT}/scripts/benchmark_summary.py" \
    --root "${RUN_ROOT}" \
    --primary ape \
    --ape-threshold "${APE_THRESHOLD}" \
    --write-md "${SUMMARY_MD}" \
    --write-csv "${SUMMARY_CSV}"
else
  python3 "${REPO_ROOT}/scripts/benchmark_summary.py" \
    --root "${RUN_ROOT}" \
    --primary ape \
    --write-md "${SUMMARY_MD}" \
    --write-csv "${SUMMARY_CSV}"
fi

echo "summary written: ${SUMMARY_MD} / ${SUMMARY_CSV}"
