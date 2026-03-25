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

"""Extract a static transform chain from /tf_static in a rosbag2."""

from __future__ import annotations

import argparse
import json
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class StaticTransformRecord:
    parent_frame: str
    child_frame: str
    matrix: np.ndarray


def import_rosbags_modules():
    """Import rosbags lazily so helper tests do not require the package."""
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            'rosbags is required to extract static transforms from bags',
        ) from exc
    return AnyReader, Stores, get_typestore


def quaternion_matrix_xyzw(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> np.ndarray:
    """Convert an XYZW quaternion into a 3x3 rotation matrix."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm
    return np.array(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qz * qw),
                2.0 * (qx * qz + qy * qw),
            ],
            [
                2.0 * (qx * qy + qz * qw),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qx * qw),
            ],
            [
                2.0 * (qx * qz - qy * qw),
                2.0 * (qy * qz + qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=np.float64,
    )


def quaternion_xyzw_from_rotation_matrix(
    rotation: np.ndarray,
) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a normalized XYZW quaternion."""
    trace = float(rotation[0, 0] + rotation[1, 1] + rotation[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(
            1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2],
        ) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(
            1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2],
        ) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(
            1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1],
        ) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def transform_matrix_from_xyz_xyzw(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> np.ndarray:
    """Build a homogeneous transform matrix from XYZ + XYZW."""
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_matrix_xyzw(qx, qy, qz, qw)
    matrix[:3, 3] = np.array([x, y, z], dtype=np.float64)
    return matrix


def invert_transform_matrix(matrix: np.ndarray) -> np.ndarray:
    """Invert a rigid 4x4 transform matrix."""
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -(rotation.T @ translation)
    return inverse


def xyz_xyzw_from_transform_matrix(
    matrix: np.ndarray,
) -> tuple[float, float, float, float, float, float, float]:
    """Convert a 4x4 homogeneous matrix into XYZ + XYZW."""
    x, y, z = matrix[:3, 3].tolist()
    qx, qy, qz, qw = quaternion_xyzw_from_rotation_matrix(matrix[:3, :3])
    return x, y, z, qx, qy, qz, qw


def resolve_transform_chain(
    records: list[StaticTransformRecord],
    source_frame: str,
    target_frame: str,
) -> np.ndarray:
    """Resolve a composed transform from source_frame to target_frame."""
    if source_frame == target_frame:
        return np.eye(4, dtype=np.float64)

    adjacency: dict[str, list[tuple[str, np.ndarray]]] = {}
    for record in records:
        adjacency.setdefault(record.parent_frame, []).append(
            (record.child_frame, record.matrix),
        )
        adjacency.setdefault(record.child_frame, []).append(
            (record.parent_frame, invert_transform_matrix(record.matrix)),
        )

    queue = deque([(source_frame, np.eye(4, dtype=np.float64))])
    visited = {source_frame}
    while queue:
        current_frame, current_transform = queue.popleft()
        for neighbor_frame, edge_transform in adjacency.get(current_frame, []):
            if neighbor_frame in visited:
                continue
            next_transform = current_transform @ edge_transform
            if neighbor_frame == target_frame:
                return next_transform
            visited.add(neighbor_frame)
            queue.append((neighbor_frame, next_transform))

    raise RuntimeError(
        f'could not resolve static transform chain: {source_frame} -> {target_frame}',
    )


def extract_static_transform_records(
    bag_path: Path,
    topic: str,
) -> list[StaticTransformRecord]:
    """Read the first /tf_static message from a bag and return its transforms."""
    AnyReader, Stores, get_typestore = import_rosbags_modules()
    typestore = get_typestore(Stores.LATEST)
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [conn for conn in reader.connections if conn.topic == topic]
        if not connections:
            raise RuntimeError(f'topic not found in bag: {topic}')
        for conn, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, conn.msgtype)
            records: list[StaticTransformRecord] = []
            for transform in msg.transforms:
                records.append(
                    StaticTransformRecord(
                        parent_frame=transform.header.frame_id,
                        child_frame=transform.child_frame_id,
                        matrix=transform_matrix_from_xyz_xyzw(
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z,
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w,
                        ),
                    ),
                )
            return records
    raise RuntimeError(f'no messages found on topic: {topic}')


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Extract a static transform chain from /tf_static in a rosbag2.',
    )
    parser.add_argument('bag_path', type=Path, help='Path to a rosbag2 directory.')
    parser.add_argument(
        '--topic',
        default='/tf_static',
        help='Static TF topic to inspect (default: /tf_static).',
    )
    parser.add_argument('--source-frame', required=True, help='Source frame id.')
    parser.add_argument('--target-frame', required=True, help='Target frame id.')
    parser.add_argument(
        '--json',
        action='store_true',
        help='Emit JSON instead of space-separated values.',
    )
    return parser.parse_args()


def main() -> int:
    """Extract and print a static transform chain."""
    args = parse_args()
    bag_path = args.bag_path.expanduser().resolve()
    if not bag_path.is_dir():
        raise SystemExit(f'bag not found: {bag_path}')

    records = extract_static_transform_records(bag_path, args.topic)
    matrix = resolve_transform_chain(
        records=records,
        source_frame=args.source_frame,
        target_frame=args.target_frame,
    )
    x, y, z, qx, qy, qz, qw = xyz_xyzw_from_transform_matrix(matrix)
    if args.json:
        print(
            json.dumps(
                {
                    'bag_path': str(bag_path),
                    'topic': args.topic,
                    'source_frame': args.source_frame,
                    'target_frame': args.target_frame,
                    'translation': {'x': x, 'y': y, 'z': z},
                    'rotation': {'x': qx, 'y': qy, 'z': qz, 'w': qw},
                },
                sort_keys=True,
            ),
        )
    else:
        print(f'{x} {y} {z} {qx} {qy} {qz} {qw}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
