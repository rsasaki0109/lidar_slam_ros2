#!/usr/bin/env python3
"""Cut a tight RGBA actor sprite from a real photo via instance segmentation.

Phase 3 of the 3DGS-as-sim2real track used a *rectangular* crop as the sprite,
so the composited actor carried a halo of source background and the exported
ground-truth box was loose (detector IoU capped ~0.28). This tool replaces that
crop with a **segmentation alpha matte**: it runs an instance-segmentation model
(ultralytics ``*-seg``), takes the largest portrait instance of the requested
class, and writes an RGBA PNG whose alpha is the object silhouette, tightly
cropped to it. Fed to ``actor_compositing.py --mode sprite`` it composites with
no background halo and yields a tight per-frame label, so the detection-gap
numbers measure the object, not the crop.

The pure helpers (RGBA assembly from a soft mask, tight crop to the alpha
bounding box, portrait-instance selection) are numpy-only and unit tested on
CPU; extracting from a photo needs ultralytics + a segmentation model.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Sequence

import numpy as np


# --------------------------------------------------------------------------- #
# Pure helpers (no torch/detector)
# --------------------------------------------------------------------------- #
def compose_rgba(rgb: np.ndarray, mask: np.ndarray, *,
                 mask_thresh: float = 0.5) -> np.ndarray:
    """Stack an ``(H, W, 3)`` image and an ``(H, W)`` soft mask into RGBA uint8.

    Alpha is 255 where the mask meets ``mask_thresh`` and 0 elsewhere, so the
    sprite carries the object silhouette rather than a rectangle.
    """
    rgb = np.asarray(rgb)
    mask = np.asarray(mask, dtype=float)
    if mask.shape != rgb.shape[:2]:
        raise ValueError('mask shape must match the image height/width')
    alpha = np.where(mask >= mask_thresh, 255, 0).astype(np.uint8)
    return np.dstack([rgb.astype(np.uint8), alpha])


def tight_crop_rgba(rgba: np.ndarray, *, alpha_thresh: int = 1) -> np.ndarray:
    """Crop an RGBA image to the bounding box of pixels with alpha >= threshold."""
    a = np.asarray(rgba)
    ys, xs = np.where(a[..., 3] >= alpha_thresh)
    if ys.size == 0:
        raise ValueError('sprite has no opaque pixels')
    return np.ascontiguousarray(a[ys.min():ys.max() + 1, xs.min():xs.max() + 1])


def alpha_coverage(rgba: np.ndarray, *, alpha_thresh: int = 1) -> float:
    """Fraction of the RGBA tile that is opaque -- a halo/tightness proxy."""
    a = np.asarray(rgba)[..., 3]
    return float(np.mean(a >= alpha_thresh))


def pick_portrait_instance(boxes: Sequence[Sequence[float]],
                           classes: Sequence[int], target_cls: int
                           ) -> Optional[int]:
    """Index of the tallest taller-than-wide instance of ``target_cls`` (or None).

    Restricting to portrait (height > width) boxes rejects the wide,
    multi-subject detections that do not look like a single standing object --
    the trap that produced a non-detectable sprite in the first Phase 3 pass.
    """
    best, best_h = None, -1.0
    for i, (box, cls) in enumerate(zip(boxes, classes)):
        if int(cls) != target_cls:
            continue
        w = float(box[2]) - float(box[0])
        h = float(box[3]) - float(box[1])
        if h > w and h > best_h:
            best, best_h = i, h
    return best


# --------------------------------------------------------------------------- #
# Extraction (ultralytics; imported lazily)
# --------------------------------------------------------------------------- #
def extract_sprite(image: Path, out: Path, *, weights: str, target_cls: int,
                   conf: float, mask_thresh: float) -> dict:
    """Segment the largest portrait instance of a class and write a tight RGBA PNG."""
    import imageio.v2 as imageio
    from ultralytics import YOLO

    img = np.asarray(imageio.imread(image))[..., :3]
    res = YOLO(weights).predict(img[..., ::-1], conf=conf, verbose=False,
                                retina_masks=True)[0]
    if res.masks is None:
        raise SystemExit('segmentation model returned no masks')
    boxes = [b.xyxy[0].tolist() for b in res.boxes]
    classes = [int(b.cls.item()) for b in res.boxes]
    idx = pick_portrait_instance(boxes, classes, target_cls)
    if idx is None:
        raise SystemExit(f'no portrait instance of class {target_cls} found')
    mask = res.masks.data[idx].cpu().numpy()
    rgba = tight_crop_rgba(compose_rgba(img, mask, mask_thresh=mask_thresh))
    out.parent.mkdir(parents=True, exist_ok=True)
    imageio.imwrite(out, rgba)
    return {'out': str(out), 'size_hw': [int(rgba.shape[0]), int(rgba.shape[1])],
            'alpha_coverage': round(alpha_coverage(rgba), 3),
            'class': target_cls, 'conf': round(float(res.boxes[idx].conf.item()), 3)}


def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--image', required=True, help='source photo')
    p.add_argument('--out', required=True, help='output RGBA .png sprite')
    p.add_argument('--weights', default='yolov8n-seg.pt',
                   help='ultralytics segmentation weights')
    p.add_argument('--class-id', type=int, default=0,
                   help='COCO class to extract (0 = person)')
    p.add_argument('--conf', type=float, default=0.4,
                   help='detection confidence threshold')
    p.add_argument('--mask-thresh', type=float, default=0.5,
                   help='soft-mask cutoff for the alpha matte')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    info = extract_sprite(Path(args.image), Path(args.out), weights=args.weights,
                          target_cls=args.class_id, conf=args.conf,
                          mask_thresh=args.mask_thresh)
    print(f"wrote {info['out']} {info['size_hw'][1]}x{info['size_hw'][0]} "
          f"(class {info['class']} conf {info['conf']}, "
          f"alpha coverage {info['alpha_coverage']})")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
