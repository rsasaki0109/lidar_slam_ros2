#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Plane-prior patch warping for direct camera alignment (numpy only)."""

from __future__ import annotations

import numpy as np


def plane_homography(K: np.ndarray, target_T_reference: np.ndarray,
                     normal_reference: np.ndarray,
                     point_reference: np.ndarray) -> np.ndarray:
    """Return the reference-pixel to target-pixel plane homography.

    Implements FAST-LIVO2 equation (13):
    ``K (R + t n^T / (n^T p)) K^-1``. Inputs use OpenCV camera coordinates
    (+z forward). The normal sign is irrelevant because it appears in both the
    numerator and denominator.
    """
    intr = np.asarray(K, dtype=np.float64)
    transform = np.asarray(target_T_reference, dtype=np.float64)
    normal = np.asarray(normal_reference, dtype=np.float64).reshape(3)
    point = np.asarray(point_reference, dtype=np.float64).reshape(3)
    if intr.shape != (3, 3):
        raise ValueError(f'K must be 3x3, got {intr.shape}')
    if transform.shape != (4, 4):
        raise ValueError(f'target_T_reference must be 4x4, got {transform.shape}')
    if not np.all(np.isfinite(intr)) or not np.all(np.isfinite(transform)):
        raise ValueError('K and transform must be finite')
    normal_norm = float(np.linalg.norm(normal))
    if normal_norm <= 1.0e-12:
        raise ValueError('plane normal must be non-zero')
    normal = normal / normal_norm
    denominator = float(normal @ point)
    if abs(denominator) <= 1.0e-9:
        raise ValueError('plane passes through the reference camera centre')
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    affine = rotation + np.outer(translation, normal) / denominator
    return intr @ affine @ np.linalg.inv(intr)


def warp_pixels(homography: np.ndarray, pixels: np.ndarray
                ) -> tuple[np.ndarray, np.ndarray]:
    """Warp Nx2 pixels and return coordinates plus a finite/front mask."""
    matrix = np.asarray(homography, dtype=np.float64)
    uv = np.asarray(pixels, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError(f'homography must be 3x3, got {matrix.shape}')
    if uv.ndim != 2 or uv.shape[1] != 2:
        raise ValueError(f'pixels must be Nx2, got {uv.shape}')
    homogeneous = np.column_stack((uv, np.ones(len(uv)))) @ matrix.T
    valid = np.isfinite(homogeneous).all(axis=1) & (homogeneous[:, 2] > 1.0e-9)
    result = np.full((len(uv), 2), np.nan, dtype=np.float64)
    result[valid] = homogeneous[valid, :2] / homogeneous[valid, 2, None]
    return result, valid


def bilinear_sample(image: np.ndarray, pixels: np.ndarray
                    ) -> tuple[np.ndarray, np.ndarray]:
    """Sample a mono image at Nx2 pixels, returning values and in-frame mask."""
    source = np.asarray(image, dtype=np.float32)
    uv = np.asarray(pixels, dtype=np.float64)
    if source.ndim != 2:
        raise ValueError(f'image must be HxW mono, got {source.shape}')
    if uv.ndim != 2 or uv.shape[1] != 2:
        raise ValueError(f'pixels must be Nx2, got {uv.shape}')
    x, y = uv[:, 0], uv[:, 1]
    valid = (np.isfinite(uv).all(axis=1) & (x >= 0.0) & (y >= 0.0) &
             (x <= source.shape[1] - 1) & (y <= source.shape[0] - 1))
    values = np.full(len(uv), np.nan, dtype=np.float32)
    if not valid.any():
        return values, valid
    x0 = np.floor(x[valid]).astype(np.int64)
    y0 = np.floor(y[valid]).astype(np.int64)
    x1 = np.minimum(x0 + 1, source.shape[1] - 1)
    y1 = np.minimum(y0 + 1, source.shape[0] - 1)
    wx = (x[valid] - x0).astype(np.float32)
    wy = (y[valid] - y0).astype(np.float32)
    top = source[y0, x0] * (1.0 - wx) + source[y0, x1] * wx
    bottom = source[y1, x0] * (1.0 - wx) + source[y1, x1] * wx
    values[valid] = top * (1.0 - wy) + bottom * wy
    return values, valid


def zero_mean_ncc(reference: np.ndarray, target: np.ndarray,
                  valid: np.ndarray | None = None) -> float | None:
    """Return zero-mean NCC, or None for too few/textureless samples."""
    left = np.asarray(reference, dtype=np.float64).reshape(-1)
    right = np.asarray(target, dtype=np.float64).reshape(-1)
    if left.shape != right.shape:
        raise ValueError('reference and target patches must have equal shape')
    keep = np.isfinite(left) & np.isfinite(right)
    if valid is not None:
        mask = np.asarray(valid, dtype=bool).reshape(-1)
        if mask.shape != keep.shape:
            raise ValueError('valid mask must match patches')
        keep &= mask
    if keep.sum() < 4:
        return None
    left = left[keep] - left[keep].mean()
    right = right[keep] - right[keep].mean()
    denominator = float(np.linalg.norm(left) * np.linalg.norm(right))
    if denominator <= 1.0e-9:
        return None
    return float(np.clip(left @ right / denominator, -1.0, 1.0))
