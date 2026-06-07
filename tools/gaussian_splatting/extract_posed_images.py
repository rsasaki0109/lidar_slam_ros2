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


def compute_clock_offset(cam_header: float, cam_bagtime: float,
                         ref_header: float, ref_bagtime: float) -> float:
    """Offset to add to camera header stamps to reach the reference clock.

    Some bags carry sensors on independent uptime clocks (e.g. a Livox LiDAR
    and a camera whose ``header.stamp`` bases differ by tens of seconds). Each
    sensor's ``header - bag_receive_time`` is a near-constant skew; the offset
    that maps camera stamps onto the trajectory/reference clock is the
    difference of those skews.
    """
    skew_cam = cam_header - cam_bagtime
    skew_ref = ref_header - ref_bagtime
    return skew_ref - skew_cam


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


def load_intrinsics_yaml(path: str | Path) -> pi.CameraIntrinsics:
    """Parse a camera intrinsics YAML (NTU VIRAL / Kalibr-style PINHOLE).

    Reads ``image_width/height``, ``projection_parameters`` (fx,fy,cx,cy) and
    ``distortion_parameters`` (k1,k2,p1,p2). Tolerant of the OpenCV ``%YAML:1.0``
    header and ``!!opencv-matrix`` tags (we only need the scalar fields, pulled
    by regex), so ``yaml.safe_load`` is not required.
    """
    import re

    text = Path(path).read_text()

    def grab(key: str, default: Optional[float] = None,
             after: Optional[str] = None) -> float:
        # Scope the search to the text after ``after`` (e.g. the
        # ``projection_parameters`` section header) so a stereo YAML's second
        # camera, or a rectification block reusing the same field names, cannot
        # win by appearing earlier in document order.
        scope = text
        if after is not None:
            anchor = re.search(rf'\b{after}\b', text)
            if anchor is not None:
                scope = text[anchor.end():]
        m = re.search(rf'\b{key}\s*:\s*([-+0-9.eE]+)', scope)
        if m is None:
            if default is None:
                raise ValueError(f'{path}: missing intrinsics field {key!r}')
            return default
        return float(m.group(1))

    return pi.CameraIntrinsics(
        width=int(grab('image_width')),
        height=int(grab('image_height')),
        fx=grab('fx', after='projection_parameters'),
        fy=grab('fy', after='projection_parameters'),
        cx=grab('cx', after='projection_parameters'),
        cy=grab('cy', after='projection_parameters'),
        distortion=(grab('k1', 0.0, after='distortion_parameters'),
                    grab('k2', 0.0, after='distortion_parameters'),
                    grab('p1', 0.0, after='distortion_parameters'),
                    grab('p2', 0.0, after='distortion_parameters'), 0.0),
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
def _bag_is_file_compressed(bag_path: str | Path) -> bool:
    """Return True if the bag's metadata declares FILE-level compression (zstd)."""
    meta = Path(bag_path) / 'metadata.yaml'
    if not meta.is_file():
        return False
    text = meta.read_text(errors='replace')
    for line in text.splitlines():
        s = line.strip()
        if s.startswith('compression_mode:'):
            return s.split(':', 1)[1].strip().strip('"').upper() == 'FILE'
    return False


def _open_reader(bag_path: str | Path):
    import rosbag2_py

    storage = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='')
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr'
    )
    # FILE-compressed bags (Autoware Leo Drive, etc.) need the compression
    # reader; the plain SequentialReader would try to open the .zstd as sqlite.
    if _bag_is_file_compressed(bag_path):
        reader = rosbag2_py.SequentialCompressionReader()
    else:
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


def _first_header_and_bagtime(bag_path: str | Path, topic: str,
                              msg_type) -> tuple[float, float]:
    """Return the (header_stamp_s, bag_receive_s) of the first ``topic`` msg."""
    from rclpy.serialization import deserialize_message

    reader = _open_reader(bag_path)
    while reader.has_next():
        tname, raw, bagt = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(raw, msg_type)
        header = ros_stamp_to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec)
        return header, bagt * 1e-9
    raise RuntimeError(f'no message found on topic {topic!r}')


def resolve_time_offset(args: argparse.Namespace) -> float:
    """Resolve ``--time-offset`` (a float, or ``auto`` via clock alignment)."""
    if str(args.time_offset).lower() != 'auto':
        return float(args.time_offset)
    if not args.clock_reference_topic:
        raise ValueError('--time-offset auto requires --clock-reference-topic')
    from sensor_msgs.msg import Image, PointCloud2

    cam_h, cam_b = _first_header_and_bagtime(args.bag, args.camera_topic, Image)
    ref_h, ref_b = _first_header_and_bagtime(
        args.bag, args.clock_reference_topic, PointCloud2
    )
    off = compute_clock_offset(cam_h, cam_b, ref_h, ref_b)
    print(f'auto time-offset: {off:.4f}s (camera -> {args.clock_reference_topic} clock)')
    return off


def extract(args: argparse.Namespace) -> dict:
    """Run the full extraction and return a small summary dict."""
    import imageio.v3 as iio
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image

    samples = pi.read_tum_trajectory(args.traj)
    body_T_cam = load_extrinsic(args.extrinsic)
    if args.intrinsics_yaml:
        intrinsics = load_intrinsics_yaml(args.intrinsics_yaml)
    else:
        intrinsics = read_camera_intrinsics(args.bag, args.camera_info_topic)
    time_offset = resolve_time_offset(args)

    undistort_map = None
    out_intrinsics = intrinsics
    if args.undistort:
        import cv2
        k = np.array([[intrinsics.fx, 0, intrinsics.cx],
                      [0, intrinsics.fy, intrinsics.cy], [0, 0, 1.0]])
        d = np.array((list(intrinsics.distortion) + [0] * 5)[:5], dtype=float)
        size = (intrinsics.width, intrinsics.height)
        new_k, _ = cv2.getOptimalNewCameraMatrix(k, d, size, 0, size)
        undistort_map = cv2.initUndistortRectifyMap(k, d, None, new_k, size, cv2.CV_16SC2)
        out_intrinsics = pi.CameraIntrinsics(
            intrinsics.width, intrinsics.height,
            float(new_k[0, 0]), float(new_k[1, 1]),
            float(new_k[0, 2]), float(new_k[1, 2]))

    out_dir = Path(args.out)
    images_dir = out_dir / 'images'
    images_dir.mkdir(parents=True, exist_ok=True)

    import rosbag2_py
    reader = _open_reader(args.bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.camera_topic]))
    frames: list[pi.PosedImage] = []
    seen = 0
    dropped = 0
    t0 = None
    while reader.has_next():
        tname, raw, bagt = reader.read_next()
        if tname != args.camera_topic:
            continue
        rel_t = 0.0 if t0 is None else (bagt * 1e-9 - t0)
        if t0 is None:
            t0 = bagt * 1e-9
        if args.end_time >= 0 and rel_t > args.end_time:
            break  # camera stamps are monotonic, no later frame qualifies
        if rel_t < args.start_time:
            continue
        if args.stride > 1 and seen % args.stride != 0:
            seen += 1
            continue
        msg = deserialize_message(raw, Image)
        stamp = ros_stamp_to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec)
        world_T_cam = resolve_world_T_camera(
            stamp, samples, body_T_cam,
            max_extrapolation=args.max_extrapolation, time_offset=time_offset,
        )
        if world_T_cam is None:
            dropped += 1
            seen += 1
            continue
        rel = f'images/{len(frames):05d}.png'
        rgb = decode_image(msg.encoding, msg.height, msg.width, msg.step, msg.data)
        if undistort_map is not None:
            import cv2
            rgb = cv2.remap(rgb, undistort_map[0], undistort_map[1], cv2.INTER_LINEAR)
        iio.imwrite(str(out_dir / rel), rgb)
        frames.append(pi.PosedImage(rel, world_T_cam, stamp))
        seen += 1

    if not frames:
        # Fail loudly here rather than writing an empty transforms.json that
        # only blows up later as an opaque torch.stack([]) in train_gsplat.
        raise RuntimeError(
            f'no image resolved a pose ({dropped} dropped): the camera stamps '
            'do not overlap the trajectory. Check --time-offset / '
            '--clock-reference-topic, --extrinsic, and that the bag and TUM '
            'trajectory cover the same interval.')
    pi.write_transforms(out_dir / 'transforms.json', out_intrinsics, frames)
    return {'kept': len(frames), 'dropped': dropped, 'out': str(out_dir)}


def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--bag', required=True, help='rosbag2 directory')
    p.add_argument('--traj', required=True, help='SLAM trajectory (TUM, world<-body)')
    p.add_argument('--camera-topic', default='/image')
    p.add_argument('--camera-info-topic', default='/camera_info')
    p.add_argument('--intrinsics-yaml', default=None,
                   help='camera intrinsics YAML (NTU/Kalibr); overrides bag camera_info')
    p.add_argument('--undistort', action='store_true',
                   help='undistort images to a pinhole model (gsplat is pinhole)')
    p.add_argument('--start-time', type=float, default=0.0,
                   help='keep images at/after this many seconds from bag start')
    p.add_argument('--end-time', type=float, default=-1.0,
                   help='keep images up to this many seconds from bag start (-1 = all)')
    p.add_argument('--extrinsic', default=None,
                   help='YAML with body<-camera_optical (matrix or translation+rotation_xyzw); '
                        'identity if omitted')
    p.add_argument('--out', required=True, help='output directory')
    p.add_argument('--max-extrapolation', type=float, default=0.05,
                   help='seconds an image stamp may fall outside the trajectory')
    p.add_argument('--time-offset', default='0.0',
                   help='seconds added to image stamps, or "auto" to align the '
                        'camera clock to --clock-reference-topic via bag receive time')
    p.add_argument('--clock-reference-topic', default=None,
                   help='PointCloud2 topic whose clock the trajectory uses '
                        '(required for --time-offset auto)')
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
