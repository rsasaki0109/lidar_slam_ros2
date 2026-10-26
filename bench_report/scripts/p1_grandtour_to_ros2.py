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
seconds instead of nanoseconds); --range-scale s multiplies every point's coordinates by s (a range-scale test).
Only the first `valid` points of every scan are used, and points at the origin or non-finite are dropped. No other
stream of the mission is read (in particular no reference or GNSS/INS topic).
It also prints the RKO-LIO extrinsics and the IMU-to-prism transform of the mission (see --extrinsics), with the
chosen IMU as the base frame.
Usage: p1_grandtour_to_ros2.py [--imu <groups>] [--lidar <group>] [--range-scale s] <mission_zarr_dir> <out_bag>
       p1_grandtour_to_ros2.py [--imu <group>] [--lidar <group>] --extrinsics <mission_zarr_dir>
           <box_base_to_prism: x y z qx qy qz qw>
"""
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
    return '/livox/points' if group == 'livox_points' else f'/{group}'


def extrinsics(mission, prism, group='livox_imu', lidar_group='livox_points'):
    lidar = zarr.open(f'{mission}/{lidar_group}', mode='r')
    imu = zarr.open(f'{mission}/{group}', mode='r')
    T_box_lidar = zarr_transform(dict(lidar.attrs))
    T_box_imu = zarr_transform(dict(imu.attrs))
    T_box_prism = matrix(prism[:3], prism[3:])
    T_imu_lidar = np.linalg.inv(T_box_imu) @ T_box_lidar
    T_imu_prism = np.linalg.inv(T_box_imu) @ T_box_prism
    print('extrinsic_imu2base_quat_xyzw_xyz: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]')
    print('extrinsic_lidar2base_quat_xyzw_xyz: ' + str([round(v, 9) for v in as_list(T_imu_lidar)]))
    print('T_imu_prism_quat_xyzw_xyz: ' + str([round(v, 9) for v in as_list(T_imu_prism)]))


def convert(mission, out, groups=('livox_imu',), lidar_group='livox_points', range_scale=1.0):
    import rclpy.serialization
    import rosbag2_py
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import Imu, PointCloud2, PointField

    def stamp(seconds):
        sec = int(np.floor(seconds))
        return Time(sec=sec, nanosec=int(round((seconds - sec) * 1e9)) % 1000000000)

    lidar = zarr.open(f'{mission}/{lidar_group}', mode='r')
    lidar_frame = dict(lidar.attrs).get('frame_id', lidar_group)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=out, storage_id='mcap', storage_preset_profile='zstd_fast'),
                rosbag2_py.ConverterOptions('', ''))
    topics = [(lidar_topic(lidar_group), 'sensor_msgs/msg/PointCloud2')]
    topics += [(imu_topic(g), 'sensor_msgs/msg/Imu') for g in groups]
    for topic, kind in topics:
        writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=kind, serialization_format='cdr'))

    messages = []
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
            messages.append((imu_t[i], imu_topic(group), msg))

    fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
              for k, n in enumerate(('x', 'y', 'z', 'intensity', 't'))]
    valid = lidar['valid'][:, 0]
    scans = len(valid)
    first = lidar['time'][0, :max(1, int(valid[0]))]
    time_to_seconds = 1e-9 if np.nanmax(first) > 1e12 else 1.0  # Livox: ns, Hesai: s
    kept_scans = 0
    block = 256
    for start in range(0, scans, block):
        stop = min(start + block, scans)
        points = lidar['points'][start:stop]
        intensity = lidar['intensity'][start:stop]
        times = lidar['time'][start:stop]
        for j in range(stop - start):
            n = int(valid[start + j])
            xyz = points[j, :n].astype(np.float64)
            t = times[j, :n] * time_to_seconds
            keep = np.isfinite(xyz).all(1) & np.isfinite(t) & (np.abs(xyz).sum(1) > 0)
            if keep.sum() < 10:
                continue
            xyz, t, inten = xyz[keep] * range_scale, t[keep], intensity[j, :n][keep].astype(np.float32)
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
            messages.append((float(t.max()), lidar_topic(lidar_group), msg))
            kept_scans += 1
    messages.sort(key=lambda m: m[0])
    for t, topic, msg in messages:
        writer.write(topic, rclpy.serialization.serialize_message(msg), int(round(t * 1e9)))
    print(f'done: {kept_scans} of {scans} scans, {imu_samples} imu samples')


def main():
    args = sys.argv[1:]
    group, lidar_group, range_scale = 'livox_imu', 'livox_points', 1.0
    while len(args) >= 2 and args[0] in ('--imu', '--lidar', '--range-scale'):
        if args[0] == '--imu':
            group = args[1]
        elif args[0] == '--lidar':
            lidar_group = args[1]
        else:
            range_scale = float(args[1])
        args = args[2:]
    if len(args) >= 1 and args[0] == '--extrinsics':
        extrinsics(args[1], [float(v) for v in args[2:9]], group.split(',')[0], lidar_group)
    elif len(args) == 2:
        convert(args[0], args[1], tuple(group.split(',')), lidar_group, range_scale)
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
