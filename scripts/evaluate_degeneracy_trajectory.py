#!/usr/bin/env python3
"""Compute accuracy-oriented, no-GT metrics for degeneracy trajectories."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def load_tum(path: Path) -> np.ndarray:
    trajectory = np.loadtxt(path, dtype=float)
    if trajectory.ndim == 1:
        trajectory = trajectory.reshape(1, -1)
    if trajectory.shape[1] < 8:
        raise ValueError(f"{path}: expected TUM rows with at least 8 columns")
    if len(trajectory) < 2:
        raise ValueError(f"{path}: expected at least two poses")
    if np.any(np.diff(trajectory[:, 0]) <= 0.0):
        raise ValueError(f"{path}: timestamps must be strictly increasing")
    return trajectory


def evaluate(path: Path, expected_endpoint_distance: float | None) -> dict:
    trajectory = load_tum(path)
    xyz = trajectory[:, 1:4]
    deltas = np.diff(xyz, axis=0)
    steps = np.linalg.norm(deltas, axis=1)
    centered = xyz - xyz.mean(axis=0)
    _, singular_values, principal_axes = np.linalg.svd(centered, full_matrices=False)
    along = centered @ principal_axes[0]
    transverse = centered - np.outer(along, principal_axes[0])
    transverse_norm = np.linalg.norm(transverse, axis=1)
    endpoint_distance = float(np.linalg.norm(xyz[-1] - xyz[0]))

    result = {
        "trajectory": str(path.resolve()),
        "pose_count": int(len(trajectory)),
        "duration_sec": float(trajectory[-1, 0] - trajectory[0, 0]),
        "path_length_m": float(steps.sum()),
        "endpoint_distance_m": endpoint_distance,
        "max_step_m": float(steps.max()),
        "p99_step_m": float(np.quantile(steps, 0.99)),
        "principal_axis_span_m": float(along.max() - along.min()),
        "transverse_rms_m": float(np.sqrt(np.mean(np.square(transverse_norm)))),
        "transverse_max_m": float(transverse_norm.max()),
        "principal_variance_fraction": float(
            singular_values[0] ** 2 / np.square(singular_values).sum()
        ),
        "start_xyz_m": xyz[0].tolist(),
        "end_xyz_m": xyz[-1].tolist(),
    }
    if expected_endpoint_distance is not None:
        result["expected_endpoint_distance_m"] = expected_endpoint_distance
        result["endpoint_distance_error_m"] = endpoint_distance - expected_endpoint_distance
        result["endpoint_distance_error_percent"] = (
            100.0 * (endpoint_distance - expected_endpoint_distance)
            / expected_endpoint_distance
        )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--expected-endpoint-distance", type=float)
    args = parser.parse_args()
    if args.expected_endpoint_distance is not None and args.expected_endpoint_distance <= 0:
        parser.error("--expected-endpoint-distance must be positive")
    print(json.dumps(
        evaluate(args.trajectory, args.expected_endpoint_distance),
        indent=2,
        sort_keys=True,
    ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
