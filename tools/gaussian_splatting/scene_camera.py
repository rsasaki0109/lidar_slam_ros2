#!/usr/bin/env python3
"""Map a planar ego pose to a 3DGS camera view (pixel-observation RL bridge).

Phase 4 (pixel observation) of the 3DGS-as-sim2real track. ``drive_env`` drives a
planar unicycle ``(x, y, yaw)``; to let the agent *see* the scene we need to turn
that pose into a ``GaussianRenderer`` view. The recorded camera trajectory
(``transforms.json``) defines a **driving plane**: its cameras lie on a near-flat
ground patch, so we recover a world-up axis and an in-plane 2D frame from them
(PCA), express the corridor in that frame, and place the ego camera at the
recorded eye height looking along its heading.

This keeps the env's 2D corridor (Phase 0 valid-viewpoint range) and the 3DGS
render in one consistent frame: ``(x, y)`` are metres in the ground plane,
``yaw=0`` points along the travelled direction, and renders stay trustworthy
while the ego is near the recorded path.

The geometry (``load_cam_c2w``, ``derive_ground_frame``, ``corridor_xy``,
``eye_target_up``, ``look_at_viewmat``) is pure numpy and unit tested on CPU. Only
:func:`make_scene_render_fn` touches the GPU renderer.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Callable

import numpy as np


def load_cam_c2w(transforms_path: str | Path) -> np.ndarray:
    """Read camera-to-world matrices (OpenGL/nerfstudio) from a transforms.json.

    Returns an ``(N, 4, 4)`` array of the frames' ``transform_matrix`` entries.
    """
    doc = json.loads(Path(transforms_path).read_text())
    return np.array([np.asarray(f['transform_matrix'], dtype=float)
                     for f in doc['frames']], dtype=float)


def derive_ground_frame(c2w: np.ndarray) -> dict:
    """Recover a driving frame from recorded OpenGL camera-to-world matrices.

    The cameras' up columns average to the world-up axis; projecting their
    positions off that axis gives a ground patch whose principal direction is the
    travel axis ``e1`` (``e2 = up x e1`` completes a right-handed in-plane basis).
    Returns ``{origin, e1, e2, up, height}``: ``origin`` is the ground centroid,
    ``height`` the mean camera elevation above it, so an ego at plane ``(x, y)``
    sits at ``origin + x*e1 + y*e2 + height*up``.
    """
    c2w = np.asarray(c2w, dtype=float)
    pos = c2w[:, :3, 3]
    up = c2w[:, :3, 1].mean(axis=0)
    up = up / np.linalg.norm(up)
    elev = pos @ up
    ground = pos - np.outer(elev, up)
    origin = ground.mean(axis=0)
    centred = ground - origin
    _, _, vt = np.linalg.svd(centred, full_matrices=False)
    e1 = vt[0]
    e1 = e1 / np.linalg.norm(e1)
    if (pos[-1] - pos[0]) @ e1 < 0.0:  # orient e1 along the travel direction
        e1 = -e1
    e2 = np.cross(up, e1)
    e2 = e2 / np.linalg.norm(e2)
    return {'origin': origin, 'e1': e1, 'e2': e2, 'up': up,
            'height': float(elev.mean())}


def corridor_xy(c2w: np.ndarray, frame: dict) -> np.ndarray:
    """Express recorded camera positions as ``(N, 2)`` plane coords in ``frame``.

    The path runs in the ``+x`` (``e1``) direction (oriented in the frame); the
    returned points are the env's corridor anchors.
    """
    pos = np.asarray(c2w, dtype=float)[:, :3, 3]
    rel = pos - frame['origin']
    return np.stack([rel @ frame['e1'], rel @ frame['e2']], axis=1)


def eye_target_up(x: float, y: float, yaw: float,
                  frame: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """World-space ``(eye, target, up)`` for an ego at plane ``(x, y)`` heading ``yaw``.

    ``yaw=0`` looks along ``+e1`` (the travel direction); the camera sits at the
    recorded eye height above the ground plane.
    """
    e1, e2, up = frame['e1'], frame['e2'], frame['up']
    eye = frame['origin'] + float(x) * e1 + float(y) * e2 + frame['height'] * up
    direction = np.cos(float(yaw)) * e1 + np.sin(float(yaw)) * e2
    return eye, eye + direction, up


def look_at_viewmat(eye: np.ndarray, target: np.ndarray,
                    up: np.ndarray) -> np.ndarray:
    """World->camera (OpenCV optical) matrix for a camera looking at ``target``."""
    eye = np.asarray(eye, dtype=float)
    fwd = np.asarray(target, dtype=float) - eye
    fwd = fwd / np.linalg.norm(fwd)
    right = np.cross(np.asarray(up, dtype=float), fwd)
    right = right / np.linalg.norm(right)
    down = np.cross(fwd, right)
    c2w = np.eye(4)
    c2w[:3, 0], c2w[:3, 1], c2w[:3, 2], c2w[:3, 3] = right, down, fwd, eye
    return np.linalg.inv(c2w)


def make_scene_render_fn(renderer, frame: dict, *, fx: float = 60.0,
                         width: int = 84, height: int = 84
                         ) -> Callable[[np.ndarray], np.ndarray]:
    """Build ``pose (x, y, yaw) -> uint8 (H, W, 3)`` over a resident renderer.

    The returned closure is the env's ``render_fn`` for ``obs_mode='camera'``: it
    converts the ego pose to a look-at viewmat in the model frame and rasterises.
    """
    k = np.array([[fx, 0.0, width / 2.0], [0.0, fx, height / 2.0],
                  [0.0, 0.0, 1.0]], dtype=float)

    def render_fn(pose: np.ndarray) -> np.ndarray:
        eye, target, up = eye_target_up(pose[0], pose[1], pose[2], frame)
        vm = look_at_viewmat(eye, target, up)
        return renderer.render(vm, k, width, height)

    return render_fn
