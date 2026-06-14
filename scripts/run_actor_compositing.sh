#!/usr/bin/env bash
# Composite a dynamic actor into a trained 3DGS scene with correct occlusion
# (Phase 3 of the 3DGS-as-sim2real track). The actor is staged in front of a
# scene camera and swept across the field of view; the scene render and the
# actor are merged by a per-pixel depth test (the actor is drawn only where it
# is nearer than the scene), and a per-frame ground-truth 2D box is exported.
#
# MODE=box     a synthetic Gaussian solid (geometry/occlusion demo, no asset).
# MODE=sprite  a real-photo RGBA cutout billboarded in -- a COCO detector can
#              fire on it, so pass DETECTOR to measure the detection gap (how
#              reliably a real object is detected once embedded in a 3DGS render).
#
# A sprite can be made from any image with a detector, e.g. the tallest portrait
# person in ultralytics' bundled bus.jpg:
#   python3 - <<'PY'
#   import numpy as np, imageio.v2 as imageio, os
#   from ultralytics import YOLO
#   m = YOLO('yolov8n.pt')
#   src = os.path.expanduser('~/.local/lib/python3.12/site-packages/ultralytics/assets/bus.jpg')
#   img = np.asarray(imageio.imread(src))[..., :3]
#   res = m.predict(img[..., ::-1], conf=0.4, verbose=False)[0]
#   best = None
#   for b in res.boxes:
#       if int(b.cls.item()) == 0:
#           x1, y1, x2, y2 = (int(v) for v in b.xyxy[0].tolist())
#           if y2 - y1 > x2 - x1 and (best is None or y2 - y1 > best[1]):
#               best = ((x1, y1, x2, y2), y2 - y1)
#   (x1, y1, x2, y2), _ = best
#   crop = img[y1:y2, x1:x2]
#   rgba = np.dstack([crop, np.full(crop.shape[:2], 255, np.uint8)])
#   os.makedirs('output/assets', exist_ok=True)
#   imageio.imwrite('output/assets/person_sprite.png', rgba)
#   PY
#
# Requires: a CUDA GPU with torch + gsplat; ultralytics if DETECTOR is set.
#
# Usage:
#   PLY=output/rtkslam_3dgs_clean/gsplat/point_cloud_good.ply \
#   TRANSFORMS=output/rtkslam_3dgs_clean/gsplat/transforms_good.json \
#   OUT_DIR=output/phase3_actor MODE=sprite \
#   SPRITE=output/assets/person_sprite.png DETECTOR=yolov8n.pt \
#   bash scripts/run_actor_compositing.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

PLY="${PLY:?set PLY to the trained 3DGS scene .ply}"
TRANSFORMS="${TRANSFORMS:?set TRANSFORMS to the transforms.json the scene trained from}"
OUT_DIR="${OUT_DIR:-output/phase3_actor}"
VIEW="${VIEW:-40}"
FRAMES="${FRAMES:-36}"
MODE="${MODE:-box}"
BOX_SIZE="${BOX_SIZE:-0.6,0.6,1.7}"
SPRITE="${SPRITE:-}"
DISTANCE="${DISTANCE:-3.5}"
LATERAL="${LATERAL:-1.5}"
DROP="${DROP:-0.7}"
SPRITE_HEIGHT_M="${SPRITE_HEIGHT_M:-1.7}"
SCALE="${SCALE:-1.0}"
DETECTOR="${DETECTOR:-}"
FPS="${FPS:-12}"

ARGS=(--ply "${PLY}" --transforms "${TRANSFORMS}" --out "${OUT_DIR}"
      --view "${VIEW}" --frames "${FRAMES}" --mode "${MODE}"
      --box-size "${BOX_SIZE}" --distance "${DISTANCE}" --lateral "${LATERAL}"
      --drop "${DROP}" --sprite-height-m "${SPRITE_HEIGHT_M}" --scale "${SCALE}"
      --fps "${FPS}")
[[ -n "${SPRITE}" ]] && ARGS+=(--sprite "${SPRITE}")
[[ -n "${DETECTOR}" ]] && ARGS+=(--detector "${DETECTOR}")

python3 tools/gaussian_splatting/actor_compositing.py "${ARGS[@]}"
