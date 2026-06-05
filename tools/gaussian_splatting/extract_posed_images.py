#!/usr/bin/env python3
"""Extract posed images from a rosbag2 for the 3DGS map deliverable.

Reads a rosbag2 (image topic + ``camera_info`` topic), resolves each image's
``world <- camera_optical`` pose from a SLAM TUM trajectory and a static
``body <- camera_optical`` extrinsic, writes the images plus a Nerfstudio
``transforms.json`` that gsplat (Apache-2.0) can train on.

Design: the pose/extrinsic/association logic lives in pure, numpy-only
functions (``resolve_world_T_camera``, ``parse_extrinsic_dict``,
``ros_stamp_to_seconds``) so it runs in the ament pytest harness with no ROS.
The rosbag2 reading and image decoding (``rosbag2_py`` / ``cv_bridge``) are
imported lazily inside ``main`` and the reader helpers, so importing this
module for testing never requires a ROS environment.

See ``docs/research/3dgs-postprocess-map-design.md``.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

import posed_images as pi


# --------------------------------------------------------------------------- #
# Pure helpers (numpy-only, ROS-free, unit-tested)
# --------------------------------------------------------------------------- #
def ros_stamp_to_seconds(sec: int, nanosec: int) -> float:
    """Convert a ROS ``builtin_interfaces/Time`` to float seconds."""
    return float(sec) + float(nanosec) * 1e-9


# Channel count per supported sensor_msgs/Image encoding.
_ENCODING_CHANNELS = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}


def decode_image(encoding: str, height: int, width: int, step: int,
                 data: bytes) -> np.ndarray:
    """Decode a raw ``sensor_msgs/Image`` payload to a canonical RGB uint8 array.

    Returns ``(H, W, 3)`` for colour encodings and ``(H, W)`` for ``mono8``.
    Avoids cv_bridge entirely (which is not numpy-2 compatible here). Handles
    row padding via ``step`` and converts BGR(A) to RGB. Raises ``ValueError``
    on an unsupported encoding or a payload shorter than ``step * height``.
    """
    enc = encoding.lower()
    if enc not in _ENCODING_CHANNELS:
        raise ValueError(f'unsupported image encoding {encoding!r}')
    channels = _ENCODING_CHANNELS[enc]
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if buf.size < step * height:
        raise ValueError(
            f'image payload {buf.size} < step*height {step * height}'
        )
    rows = buf[: step * height].reshape(height, step)
    img = rows[:, : width * channels].reshape(height, width, channels)
    if enc in ('bgr8', 'bgra8'):
        img = img[:, :, [2, 1, 0]]  # BGR(A) -> RGB, drop alpha
    elif enc == 'rgba8':
        img = img[:, :, :3]
    elif enc == 'mono8':
        return img[:, :, 0]
    return np.ascontiguousarray(img)


def parse_extrinsic_dict(data: dict) -> np.ndarray:
    """Build a 4x4 ``body <- camera_optical`` matrix from a config dict.

    Accepts either a ``matrix`` (4x4 nested list) or a
    ``translation`` + ``rotation_xyzw`` pair. Raises ``ValueError`` otherwise.
    """
    if 'matrix' in data:
        m = np.asarray(data['matrix'], dtype=float)
        if m.shape != (4, 4):
            raise ValueError(f'extrinsic matrix must be 4x4, got {m.shape}')
        return m
    if 'translation' in data and 'rotation_xyzw' in data:
        return pi.make_transform(data['translation'], data['rotation_xyzw'])
    raise ValueError(
        "extrinsic must provide 'matrix' or 'translation'+'rotation_xyzw'"
    )


def load_extrinsic(path: Optional[str | Path]) -> np.ndarray:
    """Load ``body <- camera_optical`` from a YAML file, or identity if None."""
    if path is None:
        return np.eye(4)
    import yaml  # lazy: PyYAML present in ROS env, not needed for identity

    data = yaml.safe_load(Path(path).read_text())
    return parse_extrinsic_dict(data)


def resolve_world_T_camera(
    stamp: float,
    samples: Sequence[pi.TrajectorySample],
    body_T_camera_optical: np.ndarray,
    *,
    max_extrapolation: float = 0.0,
    time_offset: float = 0.0,
) -> Optional[np.ndarray]:
    """Resolve ``world <- camera_optical`` for an image stamp.

    ``time_offset`` (seconds) is added to ``stamp`` before lookup to absorb a
    known camera/LiDAR clock skew. Returns ``None`` when the (offset) stamp
    falls outside the trajectory beyond ``max_extrapolation`` so the caller can
    drop the frame instead of inventing a pose.
    """
    try:
        world_T_body = pi.interpolate_pose(
            samples, stamp + time_offset, max_extrapolation=max_extrapolation
        )
    except ValueError:
        return None
    return pi.compose_world_T_camera(world_T_body, body_T_camera_optical)


# --------------------------------------------------------------------------- #
# ROS I/O (lazy imports; only exercised with a real bag)
# --------------------------------------------------------------------------- #
def _open_reader(bag_path: str | Path):
    import rosbag2_py

    storage = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='')
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    return reader


def read_camera_intrinsics(bag_path: str | Path, topic: str) -> pi.CameraIntrinsics:
    """Read the first ``CameraInfo`` on ``topic`` into ``CameraIntrinsics``."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo

    reader = _open_reader(bag_path)
    while reader.has_next():
        tname, raw, _ = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(raw, CameraInfo)
        return pi.CameraIntrinsics.from_camera_info(
            msg.width, msg.height, list(msg.k), list(msg.d)
        )
    raise RuntimeError(f'no CameraInfo found on topic {topic!r}')


def extract(args: argparse.Namespace) -> dict:
    """Run the full extraction and return a small summary dict."""
    import imageio.v3 as iio
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image

    samples = pi.read_tum_trajectory(args.traj)
    body_T_cam = load_extrinsic(args.extrinsic)
    intrinsics = read_camera_intrinsics(args.bag, args.camera_info_topic)

    out_dir = Path(args.out)
    images_dir = out_dir / 'images'
    images_dir.mkdir(parents=True, exist_ok=True)

    reader = _open_reader(args.bag)
    frames: list[pi.PosedImage] = []
    seen = 0
    dropped = 0
    while reader.has_next():
        tname, raw, _ = reader.read_next()
        if tname != args.camera_topic:
            continue
        if args.stride > 1 and seen % args.stride != 0:
            seen += 1
            continue
        msg = deserialize_message(raw, Image)
        stamp = ros_stamp_to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec)
        world_T_cam = resolve_world_T_camera(
            stamp, samples, body_T_cam,
            max_extrapolation=args.max_extrapolation, time_offset=args.time_offset,
        )
        if world_T_cam is None:
            dropped += 1
            seen += 1
            continue
        rel = f'images/{len(frames):05d}.png'
        rgb = decode_image(msg.encoding, msg.height, msg.width, msg.step, msg.data)
        iio.imwrite(str(out_dir / rel), rgb)
        frames.append(pi.PosedImage(rel, world_T_cam, stamp))
        seen += 1

    pi.write_transforms(out_dir / 'transforms.json', intrinsics, frames)
    return {'kept': len(frames), 'dropped': dropped, 'out': str(out_dir)}


def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--bag', required=True, help='rosbag2 directory')
    p.add_argument('--traj', required=True, help='SLAM trajectory (TUM, world<-body)')
    p.add_argument('--camera-topic', default='/image')
    p.add_argument('--camera-info-topic', default='/camera_info')
    p.add_argument('--extrinsic', default=None,
                   help='YAML with body<-camera_optical (matrix or translation+rotation_xyzw); '
                        'identity if omitted')
    p.add_argument('--out', required=True, help='output directory')
    p.add_argument('--max-extrapolation', type=float, default=0.05,
                   help='seconds an image stamp may fall outside the trajectory')
    p.add_argument('--time-offset', type=float, default=0.0,
                   help='seconds added to image stamps (camera/LiDAR clock skew)')
    p.add_argument('--stride', type=int, default=1, help='keep every Nth image')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    if args.extrinsic is None:
        print('warning: no --extrinsic given; using identity body<-camera_optical')
    summary = extract(args)
    print(f"wrote {summary['kept']} frames ({summary['dropped']} dropped) to {summary['out']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
