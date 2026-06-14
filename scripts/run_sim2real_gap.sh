#!/usr/bin/env bash
# Measure the sim2real gap of a trained LiDAR-primed 3DGS scene (Phase 0 of the
# 3DGS-as-sim2real track). Renders each training view at the recorded pose and
# at a sweep of lateral camera offsets, then scores:
#   - reconstruction fidelity at offset 0 (render vs the real image: PSNR/SSIM,
#     plus optional object-detector agreement), and
#   - extrapolation stability across offsets (SSIM vs the offset-0 render, a
#     sharpness ratio, a bright-floater fraction, and detector retention).
# The offset where stability falls off is the valid viewpoint range for using
# the scene as a closed-loop simulator.
#
# Requires: a CUDA GPU with torch + gsplat; an ultralytics install if --detector
# is passed. The PLY and TRANSFORMS must be a matched pair (the model trained
# from exactly that transforms.json) or recon PSNR will be meaningless.
#
# Usage:
#   PLY=output/koide_3dgs_firstlight/gsplat/pc_sh1_15k.ply \
#   TRANSFORMS=output/koide_3dgs_firstlight/gsplat/transforms.json \
#   OUT_DIR=output/sim2real_gap/koide \
#   bash scripts/run_sim2real_gap.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

PLY="${PLY:?set PLY to the trained 3DGS .ply}"
TRANSFORMS="${TRANSFORMS:?set TRANSFORMS to the transforms.json the model trained from}"
OUT_DIR="${OUT_DIR:-output/sim2real_gap/run}"
OFFSETS="${OFFSETS:--1.0,-0.5,-0.25,0.25,0.5,1.0}"
AXIS="${AXIS:-x}"
SCALE="${SCALE:-0.5}"
VIEWS="${VIEWS:-12}"
DETECTOR="${DETECTOR:-}"   # e.g. yolov8n.pt; empty disables the detector gap

DET_ARG=()
if [[ -n "${DETECTOR}" ]]; then
  DET_ARG=(--detector "${DETECTOR}")
fi

python3 tools/gaussian_splatting/sim2real_gap.py \
  --ply "${PLY}" \
  --transforms "${TRANSFORMS}" \
  --out "${OUT_DIR}" \
  --offsets="${OFFSETS}" \
  --axis "${AXIS}" \
  --scale "${SCALE}" \
  --views "${VIEWS}" \
  "${DET_ARG[@]}"
