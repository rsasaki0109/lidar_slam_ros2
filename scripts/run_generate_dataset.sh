#!/usr/bin/env bash
# Generate an open-loop RGB-D perception dataset from a trained LiDAR-primed 3DGS
# scene (Phase 1 of the 3DGS-as-sim2real track). For every reference view it
# renders the recorded pose plus AUG jittered poses sampled inside the Phase 0
# valid viewpoint range, writing:
#   - rgb/<stem>.png    8-bit RGB renders
#   - depth/<stem>.png  16-bit metric depth (millimetres by default)
#   - transforms.json   nerfstudio-style intrinsics + per-frame camera-to-world
# Because the model is LiDAR-primed the depth is metric, so the dataset carries
# true range labels usable for offline perception training.
#
# Keep MAX_LATERAL / MAX_VERTICAL within the scene's valid range (measured by
# scripts/run_sim2real_gap.sh): ~0.2 m for close indoor walks, up to ~1.0 m for
# open driving-scale scenes.
#
# Requires: a CUDA GPU with torch + gsplat. PLY and TRANSFORMS must be a matched
# pair (the model trained from exactly that transforms.json).
#
# Usage:
#   PLY=output/rtkslam_3dgs_clean/gsplat/point_cloud_good.ply \
#   TRANSFORMS=output/rtkslam_3dgs_clean/gsplat/transforms_good.json \
#   OUT_DIR=output/phase1_dataset \
#   bash scripts/run_generate_dataset.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

PLY="${PLY:?set PLY to the trained 3DGS .ply}"
TRANSFORMS="${TRANSFORMS:?set TRANSFORMS to the transforms.json the model trained from}"
OUT_DIR="${OUT_DIR:-output/phase1_dataset}"
VIEWS="${VIEWS:-0}"            # 0 = all reference views
AUG="${AUG:-4}"               # jittered poses per view (within valid range)
MAX_LATERAL="${MAX_LATERAL:-0.2}"
MAX_VERTICAL="${MAX_VERTICAL:-0.05}"
SCALE="${SCALE:-0.5}"
DEPTH_SCALE="${DEPTH_SCALE:-0.001}"   # metres per uint16 unit (0.001 = mm)
MAX_DEPTH="${MAX_DEPTH:-0.0}"         # 0 = no clip
SEED="${SEED:-0}"

python3 tools/gaussian_splatting/generate_dataset.py \
  --ply "${PLY}" \
  --transforms "${TRANSFORMS}" \
  --out "${OUT_DIR}" \
  --views "${VIEWS}" \
  --aug "${AUG}" \
  --max-lateral "${MAX_LATERAL}" \
  --max-vertical "${MAX_VERTICAL}" \
  --scale "${SCALE}" \
  --depth-scale "${DEPTH_SCALE}" \
  --max-depth "${MAX_DEPTH}" \
  --seed "${SEED}"
