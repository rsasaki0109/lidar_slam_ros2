#!/usr/bin/env bash
# Render the koide 3DGS flythrough video (mp4 + README GIF) from a trained
# .ply. Runs scripts/run_koide_3dgs_firstlight.sh first if no checkpoint
# exists yet.
#
# Requires: a CUDA GPU with torch + gsplat (same as the trainer) and ffmpeg
# (imageio-ffmpeg) for the mp4. See docs/3dgs-map-tutorial.md.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

OUT_DIR="${OUT_DIR:-output/koide_3dgs_firstlight}"
PLY="${PLY:-${OUT_DIR}/gsplat/point_cloud.ply}"
TRANSFORMS="${TRANSFORMS:-${OUT_DIR}/gsplat/transforms.json}"
FRAMES="${FRAMES:-240}"
SCALE="${SCALE:-0.25}"     # render resolution vs the 2448x2048 training images
ROTATE="${ROTATE:-90}"     # the koide camera is mounted sideways
GIF_SCALE="${GIF_SCALE:-0.6}"
GIF_FPS="${GIF_FPS:-8}"

if [[ ! -f "${PLY}" ]]; then
  echo "== no checkpoint at ${PLY}; training first =="
  bash scripts/run_koide_3dgs_firstlight.sh
fi

python3 tools/gaussian_splatting/render_path.py \
  --ply "${PLY}" --transforms "${TRANSFORMS}" \
  --frames "${FRAMES}" --fps 30 --ping-pong --smooth-window 5 \
  --scale "${SCALE}" --rotate "${ROTATE}" \
  --mp4 "${OUT_DIR}/gsplat/flythrough.mp4" \
  --gif "${OUT_DIR}/gsplat/flythrough.gif" \
  --gif-scale "${GIF_SCALE}" --gif-fps "${GIF_FPS}"

echo "done: ${OUT_DIR}/gsplat/flythrough.mp4 / flythrough.gif"
