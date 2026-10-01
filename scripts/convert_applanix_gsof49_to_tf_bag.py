#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""Convert Applanix GSOF49 messages into a /tf rosbag2 sidecar."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from extract_applanix_gsof49_reference import (
    heading_deg_to_enu_yaw_deg,
    is_usable_fix,
    lla_to_enu,
    resolve_applanix_msg_dir,
    rpy_deg_to_quaternion,
)


def sec_nsec_from_ns(stamp_ns: int) -> tuple[int, int]:
    """Split a nanosecond stamp into ROS Time fields."""
    return stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000


def import_rosbags_modules():
    """Import rosbags lazily so helper tests stay lightweight."""
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            'rosbags is required to convert Applanix bags into TF bags',
        ) from exc
    return AnyReader, Writer, Stores, get_typestore, get_types_from_msg


def load_typestore_with_applanix(msg_dir: Path):
    """Build a typestore that knows both standard ROS 2 and applanix_msgs."""
    _, _, Stores, get_typestore, get_types_from_msg = import_rosbags_modules()
    typestore = get_typestore(Stores.LATEST)
    for path in sorted(msg_dir.glob('*.msg')):
        text = path.read_text(encoding='utf-8')
        msg_name = f'applanix_msgs/msg/{path.stem}'
        typestore.register(get_types_from_msg(text, msg_name))
    return typestore


def integrate_planar_velocity(
    east_m: float,
    north_m: float,
    velocity_east_mps: float,
    velocity_north_mps: float,
    previous_stamp_ns: int | None,
    current_stamp_ns: int,
) -> tuple[float, float]:
    """Integrate planar velocity between usable GSOF49 samples."""
    if previous_stamp_ns is None:
        return east_m, north_m
    dt = max(0.0, (current_stamp_ns - previous_stamp_ns) * 1e-9)
    return (
        east_m + velocity_east_mps * dt,
        north_m + velocity_north_mps * dt,
    )


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Convert Applanix GSOF49 messages into a /tf rosbag2 sidecar.',
    )
    parser.add_argument('--input', required=True, help='Input rosbag2 directory.')
    parser.add_argument('--output', required=True, help='Output rosbag2 directory.')
    parser.add_argument(
        '--topic',
        default='/lvx_client/gsof/ins_solution_49',
        help='Applanix NavigationSolutionGsof49 topic.',
    )
    parser.add_argument(
        '--output-topic',
        default='/tf',
        help='Output TF topic (default: /tf).',
    )
    parser.add_argument(
        '--odom-frame-id',
        default='odom',
        help='Parent frame_id for the generated TF (default: odom).',
    )
    parser.add_argument(
        '--child-frame-id',
        default='base_link',
        help='Child frame_id for the generated TF (default: base_link).',
    )
    parser.add_argument(
        '--planar',
        action='store_true',
        help='Publish planar odom only (zero z, roll, and pitch).',
    )
    parser.add_argument(
        '--integrate-velocity-planar',
        action='store_true',
        help='Integrate planar NED velocity into a smoother odom trajectory instead of using absolute LLA.',
    )
    parser.add_argument(
        '--applanix-msg-dir',
        type=Path,
        default=None,
        help='Path to applanix_msgs/msg containing *.msg definitions.',
    )
    parser.add_argument(
        '--force',
        action='store_true',
        help='Remove the output directory if it already exists.',
    )
    return parser.parse_args()


def main() -> int:
    """Convert GSOF49 into a TF sidecar rosbag2."""
    args = parse_args()
    input_path = Path(args.input).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    if not input_path.is_dir():
        raise SystemExit(f'input bag not found: {input_path}')
    if output_path.exists():
        if not args.force:
            raise SystemExit(f'output already exists: {output_path}')
        shutil.rmtree(output_path)

    repo_root = Path(__file__).resolve().parents[1]
    msg_dir = resolve_applanix_msg_dir(args.applanix_msg_dir, repo_root)
    AnyReader, Writer, _, _, _ = import_rosbags_modules()
    typestore = load_typestore_with_applanix(msg_dir)
    tf_message_cls = typestore.types['tf2_msgs/msg/TFMessage']
    transform_stamped_cls = typestore.types['geometry_msgs/msg/TransformStamped']
    transform_cls = typestore.types['geometry_msgs/msg/Transform']
    vector3_cls = typestore.types['geometry_msgs/msg/Vector3']
    quaternion_cls = typestore.types['geometry_msgs/msg/Quaternion']
    header_cls = typestore.types['std_msgs/msg/Header']
    time_cls = typestore.types['builtin_interfaces/msg/Time']

    origin = None
    integrated_east = 0.0
    integrated_north = 0.0
    previous_stamp_ns = None
    written = 0
    skipped = 0

    with AnyReader([input_path], default_typestore=typestore) as reader, Writer(
        output_path,
        version=8,
    ) as writer:
        connections = {conn.topic: conn for conn in reader.connections}
        if args.topic not in connections:
            raise SystemExit(f'missing GSOF49 topic: {args.topic}')
        gsof49_conn = connections[args.topic]
        output_conn = writer.add_connection(
            args.output_topic,
            'tf2_msgs/msg/TFMessage',
            typestore=typestore,
        )

        for conn, stamp_ns, raw in reader.messages(connections=[gsof49_conn]):
            msg = reader.deserialize(raw, conn.msgtype)
            status = msg.status
            lla = msg.lla
            usable, _ = is_usable_fix(
                latitude_deg=float(lla.latitude),
                longitude_deg=float(lla.longitude),
                altitude_m=float(lla.altitude),
                imu_alignment=int(status.imu_alignment),
                gnss_status=int(status.gnss),
                aligned_value=int(status.ALIGNED),
                fix_not_available_value=int(status.FIX_NOT_AVAILABLE),
                gnss_unknown_value=int(status.GNSS_UNKNOWN),
            )
            if not usable:
                skipped += 1
                continue

            if args.integrate_velocity_planar:
                velocity = msg.velocity
                integrated_east, integrated_north = integrate_planar_velocity(
                    east_m=integrated_east,
                    north_m=integrated_north,
                    velocity_east_mps=float(velocity.east),
                    velocity_north_mps=float(velocity.north),
                    previous_stamp_ns=previous_stamp_ns,
                    current_stamp_ns=stamp_ns,
                )
                east = integrated_east
                north = integrated_north
                up = 0.0
            else:
                if origin is None:
                    origin = (
                        float(lla.latitude),
                        float(lla.longitude),
                        float(lla.altitude),
                    )

                east, north, up = lla_to_enu(
                    latitude_deg=float(lla.latitude),
                    longitude_deg=float(lla.longitude),
                    altitude_m=float(lla.altitude),
                    origin_latitude_deg=origin[0],
                    origin_longitude_deg=origin[1],
                    origin_altitude_m=origin[2],
                )
            roll_deg = float(msg.roll)
            pitch_deg = float(msg.pitch)
            yaw_deg = heading_deg_to_enu_yaw_deg(float(msg.heading))
            if args.planar:
                up = 0.0
                roll_deg = 0.0
                pitch_deg = 0.0
            qx, qy, qz, qw = rpy_deg_to_quaternion(
                roll_deg=roll_deg,
                pitch_deg=pitch_deg,
                yaw_deg=yaw_deg,
            )
            sec, nanosec = sec_nsec_from_ns(stamp_ns)
            tf_msg = tf_message_cls(
                transforms=[
                    transform_stamped_cls(
                        header=header_cls(
                            stamp=time_cls(sec=sec, nanosec=nanosec),
                            frame_id=args.odom_frame_id,
                        ),
                        child_frame_id=args.child_frame_id,
                        transform=transform_cls(
                            translation=vector3_cls(x=east, y=north, z=up),
                            rotation=quaternion_cls(x=qx, y=qy, z=qz, w=qw),
                        ),
                    ),
                ],
            )
            writer.write(
                output_conn,
                stamp_ns,
                typestore.serialize_cdr(tf_msg, 'tf2_msgs/msg/TFMessage'),
            )
            previous_stamp_ns = stamp_ns
            written += 1

    print(f'wrote {output_path}')
    print(f'output_topic: {args.output_topic}')
    print(f'written_messages: {written}')
    print(f'skipped_invalid: {skipped}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
