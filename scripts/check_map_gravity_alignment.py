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
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
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

"""Measure how far a map frame is tilted from gravity, using the bag's IMU.

Each IMU acceleration is rotated into the map frame with the run's trajectory
and averaged. Over a whole run the motion terms cancel (the mean equals gravity
plus the velocity change divided by the duration), so the mean points up in a
gravity-aligned map. The angle between it and +z is the map tilt.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

DEFAULT_MAX_TILT_DEG = 3.0


def _quat_rotate(quat_xyzw: np.ndarray, vectors: np.ndarray) -> np.ndarray:
    """Rotate vectors (N, 3) by unit quaternions (N, 4) or (4,) in x, y, z, w order."""
    q = np.broadcast_to(quat_xyzw, (len(vectors), 4))
    u, w = q[:, :3], q[:, 3:4]
    t = 2.0 * np.cross(u, vectors)
    return vectors + w * t + np.cross(u, t)


def _slerp(times: np.ndarray, quats: np.ndarray, query: np.ndarray) -> np.ndarray:
    """Interpolate unit quaternions (x, y, z, w) at query times inside [times]."""
    index = np.clip(np.searchsorted(times, query) - 1, 0, len(times) - 2)
    t0, t1 = times[index], times[index + 1]
    fraction = np.where(t1 > t0, (query - t0) / np.where(t1 > t0, t1 - t0, 1.0), 0.0)
    q0, q1 = quats[index], quats[index + 1].copy()
    dot = np.sum(q0 * q1, axis=1)
    q1[dot < 0.0] *= -1.0
    dot = np.abs(dot)
    angle = np.arccos(np.clip(dot, -1.0, 1.0))
    sin_angle = np.sin(angle)
    small = sin_angle < 1e-6
    safe_sin = np.where(small, 1.0, sin_angle)
    w0 = np.where(small, 1.0 - fraction, np.sin((1.0 - fraction) * angle) / safe_sin)
    w1 = np.where(small, fraction, np.sin(fraction * angle) / safe_sin)
    out = w0[:, None] * q0 + w1[:, None] * q1
    return out / np.linalg.norm(out, axis=1, keepdims=True)


def estimate_map_tilt(
    trajectory: np.ndarray,
    imu_times: np.ndarray,
    imu_accelerations: np.ndarray,
    imu_to_base_xyzw: np.ndarray,
) -> dict[str, float | int]:
    """Return the map tilt from gravity for a TUM trajectory and IMU samples."""
    inside = (imu_times >= trajectory[0, 0]) & (imu_times <= trajectory[-1, 0])
    if trajectory.shape[0] < 2 or not np.any(inside):
        raise ValueError('no IMU samples inside the trajectory time span')
    base_accel = _quat_rotate(np.asarray(imu_to_base_xyzw, dtype=float), imu_accelerations[inside])
    map_from_base = _slerp(trajectory[:, 0], trajectory[:, 4:8], imu_times[inside])
    mean = _quat_rotate(map_from_base, base_accel).mean(axis=0)
    norm = float(np.linalg.norm(mean))
    return {
        'tilt_deg': math.degrees(math.acos(max(-1.0, min(1.0, mean[2] / norm)))),
        'imu_samples': int(np.count_nonzero(inside)),
        'span_sec': float(trajectory[-1, 0] - trajectory[0, 0]),
        'mean_acceleration_norm': norm,
    }


def _read_imu_extrinsic(run_dir: Path) -> np.ndarray:
    import yaml

    params_path = run_dir / 'rko_params.ros.yaml'
    if not params_path.is_file():
        return np.array([0.0, 0.0, 0.0, 1.0])
    data = yaml.safe_load(params_path.read_text(encoding='utf-8')) or {}
    for node_params in data.values():
        values = (node_params or {}).get('ros__parameters', {})
        extrinsic = values.get('extrinsic_imu2base_quat_xyzw_xyz')
        if extrinsic:
            quat = np.asarray(extrinsic[:4], dtype=float)
            return quat / np.linalg.norm(quat)
    return np.array([0.0, 0.0, 0.0, 1.0])


def _bag_path_from_manifest(run_dir: Path) -> Path | None:
    manifest_path = run_dir / 'run_manifest.json'
    if not manifest_path.is_file():
        return None
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    bag_path = (manifest.get('input') or {}).get('bag_path')
    return Path(bag_path) if bag_path else None


def _read_imu(bag_path: Path, topic: str | None) -> tuple[np.ndarray, np.ndarray, str]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=''),
        rosbag2_py.ConverterOptions('', ''),
    )
    imu_topics = [
        item.name for item in reader.get_all_topics_and_types()
        if item.type == 'sensor_msgs/msg/Imu'
    ]
    if topic is None:
        if len(imu_topics) != 1:
            raise ValueError(f'choose one IMU topic with --imu-topic: {imu_topics}')
        topic = imu_topics[0]
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    times, accelerations = [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, Imu)
        times.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        acceleration = msg.linear_acceleration
        accelerations.append((acceleration.x, acceleration.y, acceleration.z))
    return np.asarray(times), np.asarray(accelerations, dtype=float).reshape(-1, 3), topic


def check_run(
    run_dir: Path,
    bag_path: Path | None = None,
    imu_topic: str | None = None,
    max_tilt_deg: float = DEFAULT_MAX_TILT_DEG,
) -> dict[str, object]:
    """Measure the tilt of a run's map frame; raises ValueError when it cannot."""
    trajectory_path = run_dir / 'traj_corrected.tum'
    if not trajectory_path.is_file():
        raise ValueError(f'missing {trajectory_path}')
    bag_path = bag_path or _bag_path_from_manifest(run_dir)
    if bag_path is None:
        raise ValueError('no bag path: pass --bag')
    trajectory = np.loadtxt(trajectory_path, ndmin=2)
    imu_times, accelerations, topic = _read_imu(bag_path, imu_topic)
    result = estimate_map_tilt(trajectory, imu_times, accelerations, _read_imu_extrinsic(run_dir))
    result.update({
        'imu_topic': topic,
        'max_tilt_deg': max_tilt_deg,
        'level': result['tilt_deg'] <= max_tilt_deg,
    })
    return result


def tilt_hint(result: dict[str, object]) -> str | None:
    """Return a diagnosis hint when the map frame is tilted, else None."""
    if result['level']:
        return None
    return (
        f'Map frame is tilted {result["tilt_deg"]:.1f} deg from gravity (measured from '
        f'{result["imu_topic"]} over {result["span_sec"]:.0f} s). Autoware expects z up; '
        'map with RKO-LIO initialization_phase enabled from a still start.'
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run_dir', type=Path, help='Map output directory (traj_corrected.tum).')
    parser.add_argument(
        '--bag', type=Path, help='Input rosbag2 (default: from run_manifest.json).'
    )
    parser.add_argument('--imu-topic', help='IMU topic (default: the only Imu topic in the bag).')
    parser.add_argument(
        '--max-tilt-deg', type=float, default=DEFAULT_MAX_TILT_DEG,
        help=f'Tilt reported as level up to this angle (default {DEFAULT_MAX_TILT_DEG}).',
    )
    parser.add_argument('--json', action='store_true', help='Print JSON.')
    args = parser.parse_args(argv)
    try:
        result = check_run(
            args.run_dir.expanduser().resolve(), args.bag, args.imu_topic, args.max_tilt_deg
        )
    except ValueError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(f'map tilt from gravity: {result["tilt_deg"]:.2f} deg '
              f'({result["imu_samples"]} IMU samples over {result["span_sec"]:.0f} s)')
        print(tilt_hint(result) or 'map frame is level')
    return 0 if result['level'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
