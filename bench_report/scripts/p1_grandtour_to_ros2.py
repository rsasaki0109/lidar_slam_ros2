#!/usr/bin/env python3
"""
GrandTour (Hugging Face zarr export) Livox Mid-360 + its IMU -> ROS 2 bag (zstd-chunked MCAP), chapter 31.

Reads the extracted `livox_points` and `livox_imu` zarr groups of one mission and writes
  /livox/points  sensor_msgs/PointCloud2, frame livox_lidar: x y z intensity t (float32); the header stamp is the
                 earliest point time of the scan and t is seconds since it; the message is stored at the last point
                 time, when a driver would deliver it;
  /livox/imu     sensor_msgs/Imu, frame livox_imu, as stored (m/s^2, rad/s); with --imu <group> another IMU of the
                 mission (stim320_imu, adis_imu, ...) is written instead, on topic /<group>, in its own frame;
                 --imu a,b writes several IMUs (the extrinsics use the first).
--lidar <group> takes another LiDAR (hesai_points: the Hesai XT-32, on /hesai_points, point times in absolute
seconds instead of nanoseconds); --lidar hesai_points+livox_points merges both LiDARs into one cloud per Hesai scan,
in the Hesai frame, on /merged_points (the Livox points timed within that scan's span); --range-scale s multiplies
every point's coordinates by s (a range-scale test); --time-field time names the per-point time field `time` instead
of `t` (float32 seconds since the scan's first point either way; BIEVR-LIO reads `t` as Ouster's uint32 ns);
--consume deletes the LiDAR group's chunk files as they are read (both groups of a merge, each read forward in time;
the disk then holds the bag and the rest of the points, not both in full).
Only the first `valid` points of every scan are used, and points at the origin or non-finite are dropped. No other
stream of the mission is read (in particular no reference or GNSS/INS topic).
It also prints the RKO-LIO extrinsics and the IMU-to-prism transform of the mission (see --extrinsics), with the
chosen IMU as the base frame.
Usage: p1_grandtour_to_ros2.py [--imu <groups>] [--lidar <group>] [--range-scale s] [--consume] <mission_zarr_dir>
           <out_bag>
       p1_grandtour_to_ros2.py [--imu <group>] [--lidar <group>] --extrinsics <mission_zarr_dir>
           <box_base_to_prism: x y z qx qy qz qw>
"""
import glob
import heapq
import os
import sys

import numpy as np

import zarr

PRISM_HELP = 'box_base -> prism from grand_tour_box tf_static_metadata_new_*.bag'


def quat_to_R(q):
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    trace = np.trace(R)
    if trace > 0:
        s = 2 * np.sqrt(trace + 1)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2 * np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [s / 4, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s]
    elif R[1, 1] > R[2, 2]:
        s = 2 * np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 1] + R[1, 0]) / s, s / 4, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s]
    else:
        s = 2 * np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, s / 4, (R[1, 0] - R[0, 1]) / s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def matrix(t, q):
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q)
    T[:3, 3] = t
    return T


def zarr_transform(attrs):
    """
    4x4 matrix of a group's `transform` attribute, as T_box_base_child (child coordinates into box_base).

    The export stores the inverse of the calibration's box_base -> child transform (its livox_lidar entry is the
    inverse of tf_static_metadata_new_*.bag's box_base -> livox_lidar), so it is inverted here.
    """
    tr = attrs['transform']
    t = [tr['translation'][k] for k in 'xyz']
    q = [tr['rotation'][k] for k in 'xyzw']
    return np.linalg.inv(matrix(t, q))


def as_list(T):
    q = R_to_quat(T[:3, :3])
    return [float(v) for v in (*q, *T[:3, 3])]


def imu_topic(group):
    return '/livox/imu' if group == 'livox_imu' else f'/{group}'


def lidar_topic(group):
    if group == 'livox_points':
        return '/livox/points'
    return '/merged_points' if '+' in group else f'/{group}'


def iter_scans(z, consume_dir=None):
    """(xyz float64, absolute time in s, intensity float32) of every scan with at least 10 usable points.

    With consume_dir (the group's directory) every chunk file of the group is deleted once all of its scans were read,
    so that a long mission and its bag need not be on the disk together; the group's .complete marker goes first.
    """
    if consume_dir:
        try:
            os.remove(f'{consume_dir}/.complete')
        except FileNotFoundError:
            pass
        chunked = {name: (z[name].chunks[0], z[name].ndim) for name in z.array_keys()}
        consumed = dict.fromkeys(chunked, 0)
    valid = z['valid'][:, 0]
    first = z['time'][0, :max(1, int(valid[0]))]
    to_seconds = 1e-9 if np.nanmax(first) > 1e12 else 1.0  # Livox: ns, Hesai: s
    block = 256
    for start in range(0, len(valid), block):
        stop = min(start + block, len(valid))
        points = z['points'][start:stop]
        intensity = z['intensity'][start:stop]
        times = z['time'][start:stop]
        for j in range(stop - start):
            n = int(valid[start + j])
            xyz = points[j, :n].astype(np.float64)
            t = times[j, :n] * to_seconds
            keep = np.isfinite(xyz).all(1) & np.isfinite(t) & (np.abs(xyz).sum(1) > 0)
            if keep.sum() < 10:
                continue
            yield xyz[keep], t[keep], intensity[j, :n][keep].astype(np.float32)
        if consume_dir:
            for name, (rows, ndim) in chunked.items():
                while (consumed[name] + 1) * rows <= stop:
                    k = consumed[name]
                    for f in glob.glob(f'{consume_dir}/{name}/{k}' + ('.*' if ndim > 1 else '')):
                        os.remove(f)
                    consumed[name] += 1
    if consume_dir:  # the last, partial chunks
        for name in chunked:
            for f in glob.glob(f'{consume_dir}/{name}/[0-9]*'):
                os.remove(f)


def merged_scans(primary, secondary, T_primary_secondary, consume_dirs=(None, None)):
    """Primary scans with the secondary LiDAR's points of the same time span added, in the primary frame.
    consume_dirs: the two groups' directories for iter_scans' consume_dir (each group is read forward only)."""
    R, p = T_primary_secondary[:3, :3], T_primary_secondary[:3, 3]
    pending = []  # secondary (xyz, t, intensity) not yet assigned
    source = iter_scans(secondary, consume_dirs[1])
    exhausted = False
    for xyz, t, inten in iter_scans(primary, consume_dirs[0]):
        lo, hi = t.min(), t.max()
        while not exhausted and (not pending or pending[-1][1].max() < hi):
            try:
                pending.append(next(source))
            except StopIteration:
                exhausted = True
        parts_xyz, parts_t, parts_i = [xyz], [t], [inten]
        kept = []
        for sx, st, si in pending:
            inside = (st >= lo) & (st <= hi)
            if inside.any():
                parts_xyz.append(sx[inside] @ R.T + p)
                parts_t.append(st[inside])
                parts_i.append(si[inside])
            later = st > hi
            if later.any():
                kept.append((sx[later], st[later], si[later]))
        pending = kept
        yield np.concatenate(parts_xyz), np.concatenate(parts_t), np.concatenate(parts_i)


def extrinsics(mission, prism, group='livox_imu', lidar_group='livox_points'):
    lidar = zarr.open(f'{mission}/{lidar_group.split("+")[0]}', mode='r')
    imu = zarr.open(f'{mission}/{group}', mode='r')
    T_box_lidar = zarr_transform(dict(lidar.attrs))
    T_box_imu = zarr_transform(dict(imu.attrs))
    T_box_prism = matrix(prism[:3], prism[3:])
    T_imu_lidar = np.linalg.inv(T_box_imu) @ T_box_lidar
    T_imu_prism = np.linalg.inv(T_box_imu) @ T_box_prism
    print('extrinsic_imu2base_quat_xyzw_xyz: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]')
    print('extrinsic_lidar2base_quat_xyzw_xyz: ' + str([round(v, 9) for v in as_list(T_imu_lidar)]))
    print('T_imu_prism_quat_xyzw_xyz: ' + str([round(v, 9) for v in as_list(T_imu_prism)]))


def convert(mission, out, groups=('livox_imu',), lidar_group='livox_points', range_scale=1.0, time_field='t',
            consume=False):
    import rclpy.serialization
    import rosbag2_py
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import Imu, PointCloud2, PointField

    def stamp(seconds):
        sec = int(np.floor(seconds))
        return Time(sec=sec, nanosec=int(round((seconds - sec) * 1e9)) % 1000000000)

    lidar_groups = lidar_group.split('+')
    lidar = zarr.open(f'{mission}/{lidar_groups[0]}', mode='r')
    lidar_frame = dict(lidar.attrs).get('frame_id', lidar_groups[0])
    if len(lidar_groups) == 2:
        secondary = zarr.open(f'{mission}/{lidar_groups[1]}', mode='r')
        T = np.linalg.inv(zarr_transform(dict(lidar.attrs))) @ zarr_transform(dict(secondary.attrs))
        scans_iter = merged_scans(lidar, secondary, T, tuple(f'{mission}/{g}' if consume else None for g in lidar_groups))
    else:
        scans_iter = iter_scans(lidar, f'{mission}/{lidar_groups[0]}' if consume else None)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=out, storage_id='mcap', storage_preset_profile='zstd_fast'),
                rosbag2_py.ConverterOptions('', ''))
    topics = [(lidar_topic(lidar_group), 'sensor_msgs/msg/PointCloud2')]
    topics += [(imu_topic(g), 'sensor_msgs/msg/Imu') for g in groups]
    for topic, kind in topics:
        writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=kind, serialization_format='cdr'))

    # IMU messages are small and kept in memory; the point clouds are generated scan by scan and merged into the IMU
    # stream by time (stable: an IMU message and a scan with the same time keep this order), so that a long Hesai
    # mission does not hold every cloud in memory at once.
    imu_messages = []
    imu_samples = 0
    for group in groups:
        imu = zarr.open(f'{mission}/{group}', mode='r')
        imu_frame = dict(imu.attrs).get('frame_id', group)
        imu_t = imu['timestamp'][:]
        acc = imu['lin_acc'][:]
        gyro = imu['ang_vel'][:]
        imu_samples += len(imu_t)
        for i in range(len(imu_t)):
            msg = Imu()
            msg.header.stamp = stamp(imu_t[i])
            msg.header.frame_id = imu_frame
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, gyro[i])
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, acc[i])
            msg.orientation_covariance[0] = -1.0
            imu_messages.append((float(imu_t[i]), imu_topic(group), msg))
    imu_messages.sort(key=lambda m: m[0])

    fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
              for k, n in enumerate(('x', 'y', 'z', 'intensity', time_field))]
    scans = len(lidar['valid'])
    kept = [0]

    def cloud_messages():
        window = []  # a few scans, so that a scan ending before the previous one still comes out in time order
        for xyz, t, inten in scans_iter:
            xyz = xyz * range_scale
            t0 = float(t.min())
            cloud = np.empty((len(t), 5), dtype=np.float32)
            cloud[:, :3] = xyz
            cloud[:, 3] = inten
            cloud[:, 4] = (t - t0).astype(np.float32)
            msg = PointCloud2()
            msg.header.stamp = stamp(t0)
            msg.header.frame_id = lidar_frame
            msg.height = 1
            msg.width = len(t)
            msg.fields = fields
            msg.is_bigendian = False
            msg.point_step = 20
            msg.row_step = 20 * len(t)
            msg.data = cloud.tobytes()
            msg.is_dense = True
            window.append((float(t.max()), kept[0], lidar_topic(lidar_group), msg))
            kept[0] += 1
            if len(window) > 8:
                window.sort(key=lambda m: (m[0], m[1]))
                first = window.pop(0)
                yield first[0], first[2], first[3]
        window.sort(key=lambda m: (m[0], m[1]))
        for t, _, topic, msg in window:
            yield t, topic, msg

    for t, topic, msg in heapq.merge(imu_messages, cloud_messages(), key=lambda m: m[0]):
        writer.write(topic, rclpy.serialization.serialize_message(msg), int(round(t * 1e9)))
    kept_scans = kept[0]
    print(f'done: {kept_scans} of {scans} scans, {imu_samples} imu samples')


def main():
    args = sys.argv[1:]
    consume = '--consume' in args
    args = [a for a in args if a != '--consume']
    group, lidar_group, range_scale, time_field = 'livox_imu', 'livox_points', 1.0, 't'
    while len(args) >= 2 and args[0] in ('--imu', '--lidar', '--range-scale', '--time-field'):
        if args[0] == '--imu':
            group = args[1]
        elif args[0] == '--lidar':
            lidar_group = args[1]
        elif args[0] == '--time-field':
            time_field = args[1]
        else:
            range_scale = float(args[1])
        args = args[2:]
    if len(args) >= 1 and args[0] == '--extrinsics':
        extrinsics(args[1], [float(v) for v in args[2:9]], group.split(',')[0], lidar_group)
    elif len(args) == 2:
        convert(args[0], args[1], tuple(group.split(',')), lidar_group, range_scale, time_field, consume)
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
