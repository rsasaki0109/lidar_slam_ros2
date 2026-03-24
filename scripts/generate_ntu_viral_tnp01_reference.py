#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import yaml


def parse_opencv_matrix_translation(path: Path, key: str) -> tuple[float, float, float]:
    """Extract the translation column from a 4x4 OpenCV YAML matrix."""
    text = path.read_text(encoding='utf-8', errors='replace')
    pattern = (
        rf'{re.escape(key)}:\s*!!opencv-matrix\s+'
        r'rows:\s*4\s+cols:\s*4\s+dt:\s*\w+\s+data:\s*\[(.*?)\]'
    )
    match = re.search(pattern, text, flags=re.DOTALL)
    if match is None:
        raise ValueError(f'failed to find 4x4 OpenCV matrix: {key}')

    numbers = [
        float(token.strip())
        for token in match.group(1).replace('\n', ' ').split(',')
        if token.strip()
    ]
    if len(numbers) != 16:
        raise ValueError(f'expected 16 values for {key}, got {len(numbers)}')

    return (numbers[3], numbers[7], numbers[11])


def parse_lidar_to_base_translation(path: Path) -> tuple[float, float, float]:
    """Read the LiDAR-to-base translation from the RKO-LIO YAML file."""
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    values = data.get('extrinsic_lidar2base_quat_xyzw_xyz')
    if not isinstance(values, list) or len(values) < 7:
        raise ValueError('extrinsic_lidar2base_quat_xyzw_xyz must contain 7 values')
    return (float(values[4]), float(values[5]), float(values[6]))


def derive_prism_offset(
    body_to_imu: tuple[float, float, float],
    lidar_to_base: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Replicate the existing NTU VIRAL lidar-to-prism offset convention."""
    return tuple(body + lidar for body, lidar in zip(body_to_imu, lidar_to_base))


def extract_pose_topic_to_tum(
    bag_path: Path,
    topic: str,
    out_path: Path,
) -> int:
    """Extract a PoseStamped rosbag2 topic into TUM format."""
    try:
        from rosbags.highlevel import AnyReader
    except Exception as exc:  # pragma: no cover - dependency failure is user-visible
        raise RuntimeError(
            'rosbags is required to extract the NTU VIRAL Leica reference',
        ) from exc

    rows: list[str] = []
    with AnyReader([bag_path]) as reader:
        conn = next((c for c in reader.connections if c.topic == topic), None)
        if conn is None:
            raise RuntimeError(f'topic not found in bag: {topic}')

        for connection, _, raw in reader.messages(connections=[conn]):
            msg = reader.deserialize(raw, connection.msgtype)
            hdr = getattr(msg, 'header', None)
            pose = getattr(msg, 'pose', None)
            if hdr is None or pose is None:
                continue
            stamp = getattr(hdr, 'stamp', None)
            pos = getattr(pose, 'position', None)
            ori = getattr(pose, 'orientation', None)
            if stamp is None or pos is None or ori is None:
                continue

            ts = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            rows.append(
                f'{ts:.9f} '
                f'{float(pos.x):.9f} {float(pos.y):.9f} {float(pos.z):.9f} '
                f'{float(ori.x):.9f} {float(ori.y):.9f} '
                f'{float(ori.z):.9f} {float(ori.w):.9f}',
            )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text('\n'.join(rows) + ('\n' if rows else ''), encoding='utf-8')
    return len(rows)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Extract the NTU VIRAL tnp_01 Leica reference trajectory and '
            'derive the lidar-to-prism offset used by this repo.'
        ),
    )
    parser.add_argument(
        '--source-bag',
        default='demo_data/ntu_viral/tnp_01_rosbag2',
        help='rosbag2 directory containing /leica/pose/relative',
    )
    parser.add_argument(
        '--topic',
        default='/leica/pose/relative',
        help='PoseStamped topic to export',
    )
    parser.add_argument(
        '--out',
        default='output/ntu_viral_tnp01_gt_leica.tum',
        help='Output TUM path',
    )
    parser.add_argument(
        '--prism-yaml',
        default='demo_data/ntu_viral/tnp_01/tnp_01/leica_prism.yaml',
        help='OpenCV YAML containing T_Body2Imu',
    )
    parser.add_argument(
        '--rko-param',
        default='lidarslam/param/rko_lio_ntu_viral.yaml',
        help='RKO-LIO parameter YAML used for lidar-to-base translation',
    )
    parser.add_argument(
        '--write-meta',
        default='output/ntu_viral_tnp01_reference.json',
        help='JSON sidecar containing the derived prism offset',
    )
    parser.add_argument(
        '--skip-extract',
        action='store_true',
        help='Do not regenerate the TUM file; only update the metadata sidecar',
    )
    args = parser.parse_args()

    source_bag = Path(args.source_bag).expanduser().resolve()
    out_path = Path(args.out).expanduser().resolve()
    prism_yaml = Path(args.prism_yaml).expanduser().resolve()
    rko_param = Path(args.rko_param).expanduser().resolve()
    meta_path = Path(args.write_meta).expanduser().resolve()

    body_to_imu = parse_opencv_matrix_translation(prism_yaml, 'T_Body2Imu')
    lidar_to_base = parse_lidar_to_base_translation(rko_param)
    prism_offset = derive_prism_offset(body_to_imu, lidar_to_base)

    pose_count = 0
    if not args.skip_extract:
        pose_count = extract_pose_topic_to_tum(source_bag, args.topic, out_path)
    elif out_path.is_file():
        pose_count = sum(
            1
            for line in out_path.read_text(encoding='utf-8', errors='replace').splitlines()
            if line.strip() and not line.lstrip().startswith('#')
        )

    meta = {
        'reference_tum_path': str(out_path),
        'source': 'leica_prism_gt',
        'source_bag': str(source_bag),
        'topic': args.topic,
        'pose_count': pose_count,
        'body_to_imu_translation_m': {
            'x': body_to_imu[0],
            'y': body_to_imu[1],
            'z': body_to_imu[2],
        },
        'lidar_to_base_translation_m': {
            'x': lidar_to_base[0],
            'y': lidar_to_base[1],
            'z': lidar_to_base[2],
        },
        'lidar_to_prism_translation_m': {
            'x': prism_offset[0],
            'y': prism_offset[1],
            'z': prism_offset[2],
        },
        'rko_param_path': str(rko_param),
        'prism_yaml_path': str(prism_yaml),
    }
    meta_path.parent.mkdir(parents=True, exist_ok=True)
    meta_path.write_text(json.dumps(meta, indent=2) + '\n', encoding='utf-8')

    print(f'reference_tum: {out_path}')
    print(
        'lidar_to_prism_translation_m: '
        f'[{prism_offset[0]:.6f}, {prism_offset[1]:.6f}, {prism_offset[2]:.6f}]',
    )
    print(f'reference_meta: {meta_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
