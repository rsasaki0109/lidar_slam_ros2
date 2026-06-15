#!/usr/bin/env python3
"""Measure an object detector's gap on a real object across novel 3DGS views (gsplat).

The closing question of the sim2real track's perception thread: *does a model
trained on real images fire on a real object as re-rendered by 3DGS, and how
does that hold up across novel viewpoints?* Phase 0 could not answer it -- the
LiDAR-primed scenes on hand contain no COCO-class objects -- and Phase 3's
compositing answered an adjacent question (a pasted/placed actor) but adds
compositing artefacts.

This tool answers it directly, with no compositing: point it at a 3DGS scene
that already contains a real object (e.g. the Apache-2.0 Tanks&Temples *Truck*),
give the object's axis-aligned box, and it orbits a synthesised camera around
the object, renders each novel view, runs the detector, and scores detections
against the projected box. The recall/IoU over the orbit is the detector's
sim2real gap on a genuinely real object seen through 3DGS.

The pure helpers (orbit camera synthesis, AABB corner projection, class-aware
detection scoring) are numpy-only and unit tested on CPU; rendering needs
CUDA + torch + gsplat and the detector needs ultralytics.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from actor_compositing import (crop_gaussians, points_to_bbox, project_points,
                               rasterize_rgbda)
from render_path import load_gaussian_ply

_UP = {'x': 0, 'y': 1, 'z': 2}


# --------------------------------------------------------------------------- #
# Pure helpers (no torch/CUDA/detector)
# --------------------------------------------------------------------------- #
def look_at_viewmat(eye: Sequence[float], target: Sequence[float],
                    up: Sequence[float]) -> np.ndarray:
    """World->camera (OpenCV) matrix for a camera at ``eye`` looking at ``target``."""
    eye = np.asarray(eye, dtype=float)
    fwd = np.asarray(target, dtype=float) - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(np.asarray(up, dtype=float), fwd)
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    c2w = np.eye(4)
    c2w[:3, 0], c2w[:3, 1], c2w[:3, 2], c2w[:3, 3] = right, down, fwd, eye
    return np.linalg.inv(c2w)


def orbit_viewmats(target: Sequence[float], radius: float, elevation: float,
                   count: int, *, up_axis: str = 'y', arc_deg: float = 360.0,
                   start_deg: float = 0.0) -> np.ndarray:
    """Synthesise ``count`` world->camera views orbiting ``target``.

    Cameras ride a circle of ``radius`` in the plane perpendicular to ``up_axis``
    (offset ``elevation`` along the up axis), each looking at ``target`` with the
    up axis as up. A full ``arc_deg`` of 360 drops the duplicate end sample.
    """
    if count < 1:
        raise ValueError('count must be positive')
    if up_axis not in _UP:
        raise ValueError(f'up_axis must be one of x/y/z, got {up_axis!r}')
    target = np.asarray(target, dtype=float)
    ui = _UP[up_axis]
    up = np.zeros(3)
    up[ui] = 1.0
    h0, h1 = (i for i in range(3) if i != ui)
    endpoint = abs(arc_deg) < 360.0 or count == 1
    angles = np.radians(np.linspace(start_deg, start_deg + arc_deg, count,
                                    endpoint=endpoint))
    out = []
    for a in angles:
        eye = target.astype(float).copy()
        eye[h0] += radius * np.cos(a)
        eye[h1] += radius * np.sin(a)
        eye[ui] += elevation
        out.append(look_at_viewmat(eye, target, up))
    return np.stack(out)


def subsample(points: np.ndarray, limit: int) -> np.ndarray:
    """Evenly stride ``points`` down to at most ``limit`` rows (deterministic)."""
    points = np.asarray(points)
    if limit <= 0 or len(points) <= limit:
        return points
    idx = np.linspace(0, len(points) - 1, limit).astype(int)
    return points[idx]


def score_detection(dets: Sequence[dict], gt_box: Optional[Sequence[float]],
                    target_cls: int, *, iou_thresh: float = 0.5):
    """Best same-class IoU against ``gt_box`` and whether it clears ``iou_thresh``.

    Returns ``(best_iou, hit)``. ``hit`` is False when there is no ground-truth
    box or no detection of ``target_cls`` overlaps it enough.
    """
    from sim2real_gap import box_iou

    if gt_box is None:
        return 0.0, False
    best = max((box_iou(gt_box, d['box']) for d in dets
               if d['cls'] == target_cls), default=0.0)
    return float(best), bool(best >= iou_thresh)


# --------------------------------------------------------------------------- #
# Driver (torch + gsplat + detector; imported lazily)
# --------------------------------------------------------------------------- #
def run(ply: Path, out_dir: Path, *, center_xy: Sequence[float],
        half_extent: float, z_range: Sequence[float], target_cls: int,
        radius: float, elevation: float, frames: int, up_axis: str,
        arc_deg: float, fx: float, width: int, height: int, detector,
        out_fps: int, device: str = 'cuda') -> dict:
    """Orbit the object, render + detect each novel view, score against the box."""
    import imageio.v2 as imageio

    scene = load_gaussian_ply(ply)
    target = np.array([center_xy[0], center_xy[1],
                       0.5 * (z_range[0] + z_range[1])], dtype=float)
    K = np.array([[fx, 0.0, width / 2.0], [0.0, fx, height / 2.0],
                  [0.0, 0.0, 1.0]])
    vms = orbit_viewmats(target, radius, elevation, frames, up_axis=up_axis,
                         arc_deg=arc_deg)
    # Tight ground truth: project the object's own Gaussian means (subsampled),
    # not the loose AABB corners, so the box hugs the object's silhouette.
    obj_pts = subsample(crop_gaussians(scene, center_xy, half_extent,
                                       z_range)['means'], 20000)

    out_dir.mkdir(parents=True, exist_ok=True)
    frames_rgb, per_view = [], []
    for i, vm in enumerate(vms):
        rgb, _, _ = rasterize_rgbda(scene, vm, K, width, height, device=device)
        uv, z = project_points(obj_pts, vm, K)
        gt = points_to_bbox(uv, z, width, height)
        dets = detector(rgb) if detector is not None else []
        best, hit = score_detection(dets, gt, target_cls)
        frames_rgb.append(rgb)
        per_view.append({'view': i, 'gt_bbox': gt, 'n_dets': len(dets),
                         'best_iou': round(best, 3), 'hit': hit,
                         'classes': sorted({d['cls'] for d in dets})})

    mp4 = out_dir / 'orbit.mp4'
    with imageio.get_writer(mp4, fps=out_fps, codec='libx264', quality=8,
                            macro_block_size=2) as wr:
        for f in frames_rgb:
            wr.append_data(f)

    scored = [v for v in per_view if v['gt_bbox'] is not None]
    cls_present = sum(1 for v in scored if target_cls in v['classes'])
    report = {
        'ply': str(ply), 'target_cls': target_cls, 'frames': frames,
        'render_size': [width, height], 'orbit_radius': radius,
        'recall_iou50': (sum(v['hit'] for v in scored) / len(scored)
                         if scored else 0.0),
        'class_present_rate': cls_present / len(scored) if scored else 0.0,
        'mean_best_iou': (float(np.mean([v['best_iou'] for v in scored]))
                          if scored else 0.0),
        'per_view': per_view,
    }
    (out_dir / 'detect_in_scene.json').write_text(json.dumps(report, indent=2))
    report['mp4'] = str(mp4)
    return report


def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--ply', required=True, help='trained 3DGS scene .ply')
    p.add_argument('--out', required=True, help='output directory')
    p.add_argument('--center', required=True,
                   help='object centre x,y in the scene frame (metres)')
    p.add_argument('--half-extent', type=float, default=4.0,
                   help='half-width of the object box in x and y (metres)')
    p.add_argument('--z-range', default='-4,3', help='object z_min,z_max (metres)')
    p.add_argument('--class-id', type=int, default=7,
                   help='COCO class to score (7 = truck, 2 = car, 0 = person)')
    p.add_argument('--radius', type=float, default=9.0,
                   help='orbit radius around the object (metres)')
    p.add_argument('--elevation', type=float, default=-2.0,
                   help='camera offset along the up axis (metres)')
    p.add_argument('--frames', type=int, default=36)
    p.add_argument('--up-axis', default='y', choices=('x', 'y', 'z'),
                   help='scene up axis (y for Tanks&Temples)')
    p.add_argument('--arc', type=float, default=360.0,
                   help='orbit arc in degrees')
    p.add_argument('--fx', type=float, default=480.0, help='focal length (px)')
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--detector', default='yolov8n.pt',
                   help='ultralytics weights')
    p.add_argument('--det-conf', type=float, default=0.25)
    p.add_argument('--fps', type=int, default=12)
    p.add_argument('--device', default='cuda')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    from sim2real_gap import Detector

    detector = Detector(args.detector, conf=args.det_conf)
    report = run(Path(args.ply), Path(args.out),
                 center_xy=[float(v) for v in args.center.split(',')],
                 half_extent=args.half_extent,
                 z_range=[float(v) for v in args.z_range.split(',')],
                 target_cls=args.class_id, radius=args.radius,
                 elevation=args.elevation, frames=args.frames,
                 up_axis=args.up_axis, arc_deg=args.arc, fx=args.fx,
                 width=args.width, height=args.height, detector=detector,
                 out_fps=args.fps, device=args.device)
    print(f"wrote {report['mp4']} ({report['frames']} orbit views)")
    print(f"  class-present rate {report['class_present_rate']:.2f}, "
          f"recall@IoU0.5 {report['recall_iou50']:.2f}, "
          f"mean best IoU {report['mean_best_iou']:.2f}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
