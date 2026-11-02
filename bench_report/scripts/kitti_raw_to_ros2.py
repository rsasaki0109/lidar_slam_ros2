#!/usr/bin/env python3
"""KITTI raw drive -> ROS 2 bag with the Velodyne scans and the 100 Hz OXTS IMU, without storing the archives.

KITTI Odometry ships no IMU. Its training sequences are cut from KITTI raw drives (05 = 2011_09_30_drive_0018,
frames 0-2760), whose OXTS unit logs 100 Hz inertial data in the unsynced ("extract") archive. This reads, over HTTP
range requests on the official archives:
  <drive>_sync.zip     velodyne_points/data/*.bin (x y z reflectance, float32) and velodyne_points/timestamps*.txt,
  <drive>_extract.zip  oxts/data/*.txt and oxts/timestamps.txt (100 Hz),
and writes
  /kitti/velodyne/points  (sensor_msgs/PointCloud2, frame velo_link): x y z intensity, float32, no point time.
                          Header stamp: the time the scanner faced forward (velodyne_points/timestamps.txt); the
                          message is stored at the scan's end time (timestamps_end.txt), when a driver would send it.
  /kitti/oxts/imu         (sensor_msgs/Imu, frame imu_link): OXTS ax ay az (m/s^2) and wx wy wz (rad/s), the body
                          frame (x forward, y left, z up), stored at their stamps;
  <out_bag_dir>.frames.txt  one line per scan: frame number, header stamp and end stamp (ns), for scoring.
Only the inertial fields of the OXTS packets are written; its GNSS/INS position and attitude are not.
FRAMES=a:b keeps scans a..b-1 (the IMU from 1 s before the first to the last). The archives are only read.
ROTATION_DESKEW=1 (data/kitti/front_diag_2026-11-01/plan_rotation_deskew.txt): KITTI's scans are compensated for the
car's translation but hardly for its rotation, which biases yaw in sharp turns. Each point is then rotated by
exp([w * (t - 0.5) * T]): t = 0.5 * (-atan2(y, x) / pi + 1) is its time in the scan from the azimuth (0.5 facing
forward, as in KISS-ICP's KITTI loader), T the scan's end minus start time, and w the mean OXTS angular rate over the
scan in the Velodyne frame (ROTATION_DESKEW_EXTRINSIC: the rig's LiDAR-to-IMU qx,qy,qz,qw; the KITTI rig's by
default). Only the gyro is used, no ground truth.
ELEVATION_CORRECTION_DEG=d (data/kitti/front_diag_2026-11-01/plan_elevation.txt): before the rotation deskew, each
point is rotated by d about the axis (p x z) / |p x z|, raising its elevation by d (KISS-ICP's correct_kitti_scan with
its own value). d comes from the scans alone (p2_kitti_elevation_offset.py).
Usage: kitti_raw_to_ros2.py <date> <drive number> <out_bag_dir> [workers]   (e.g. 2011_09_30 0018 out 8)
"""
import bisect
import concurrent.futures
import datetime
import os
import struct
import subprocess
import sys
import zlib

import numpy as np
import rclpy.serialization
import rosbag2_py
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField

DATE, DRIVE, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
WORKERS = int(sys.argv[4]) if len(sys.argv) > 4 else 8
BASE = f'https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/{DATE}_drive_{DRIVE}'
SYNC, EXTRACT = f'{BASE}/{DATE}_drive_{DRIVE}_sync.zip', f'{BASE}/{DATE}_drive_{DRIVE}_extract.zip'


def fetch(url, start, end):
    for _ in range(5):
        r = subprocess.run(['curl', '-sS', '-L', '--fail', '-m', '600', '-r', f'{start}-{end}', url],
                           capture_output=True)
        if r.returncode == 0 and len(r.stdout) == end - start + 1:
            return r.stdout
    raise RuntimeError(f'{url} range {start}-{end} failed: {r.stderr[-300:]}')


def archive_size(url):
    r = subprocess.run(['curl', '-sS', '-L', '-I', '-m', '120', url], capture_output=True, text=True, check=True)
    return [int(line.split(':', 1)[1]) for line in r.stdout.splitlines()
            if line.lower().startswith('content-length')][-1]


def central_directory(url):
    """Every member of the zip at url: (name, method, crc, csize, usize, offset); ZIP64 aware."""
    size = archive_size(url)
    tail = fetch(url, size - 65536, size - 1)
    i = tail.rfind(b'PK\x05\x06')
    _, _, _, _, entries, cd_size, cd_off, _ = struct.unpack('<IHHHHIIH', tail[i:i + 22])
    if 0xFFFFFFFF in (cd_size, cd_off) or entries == 0xFFFF:
        j = tail.rfind(b'PK\x06\x07', 0, i)
        z64_off = struct.unpack('<Q', tail[j + 8:j + 16])[0]
        z64 = fetch(url, z64_off, z64_off + 55)
        assert z64[:4] == b'PK\x06\x06', z64[:4]
        entries, cd_size, cd_off = struct.unpack('<QQQ', z64[32:56])
    cd = fetch(url, cd_off, cd_off + cd_size - 1)
    members, pos = [], 0
    while pos + 46 <= len(cd) and cd[pos:pos + 4] == b'PK\x01\x02':
        method = struct.unpack('<H', cd[pos + 10:pos + 12])[0]
        crc, csize, usize = struct.unpack('<III', cd[pos + 16:pos + 28])
        n, m, k = struct.unpack('<HHH', cd[pos + 28:pos + 34])
        offset = struct.unpack('<I', cd[pos + 42:pos + 46])[0]
        name = cd[pos + 46:pos + 46 + n].decode()
        extra, e = cd[pos + 46 + n:pos + 46 + n + m], 0
        while e + 4 <= len(extra):
            eid, elen = struct.unpack('<HH', extra[e:e + 4])
            if eid == 1:
                vals, v = extra[e + 4:e + 4 + elen], 0
                if usize == 0xFFFFFFFF:
                    usize, v = struct.unpack('<Q', vals[v:v + 8])[0], v + 8
                if csize == 0xFFFFFFFF:
                    csize, v = struct.unpack('<Q', vals[v:v + 8])[0], v + 8
                if offset == 0xFFFFFFFF:
                    offset = struct.unpack('<Q', vals[v:v + 8])[0]
            e += 4 + elen
        members.append((name, method, crc, csize, usize, offset))
        pos += 46 + n + m + k
    assert len(members) == entries, (len(members), entries)
    return members


def load_group(url, group):
    """Members stored next to each other, fetched in one range request: [(name, bytes)]."""
    start = group[0][5]
    end = group[-1][5] + 30 + 1024 + group[-1][3]
    blob = fetch(url, start, end - 1)
    out = []
    for name, method, crc, csize, usize, offset in group:
        p = offset - start
        n, m = struct.unpack('<HH', blob[p + 26:p + 30])
        data = blob[p + 30 + n + m:p + 30 + n + m + csize]
        raw = zlib.decompress(data, -15) if method == 8 else data
        if len(raw) != usize or (zlib.crc32(raw) & 0xFFFFFFFF) != crc:
            raise RuntimeError(f'{name}: size or CRC-32 mismatch')
        out.append((name, raw))
    return out


def groups(members, max_bytes):
    """Consecutive runs of members sorted by offset, each at most about max_bytes."""
    members = sorted(members, key=lambda e: e[5])
    run = []
    for e in members:
        if run and (e[5] + e[3] - run[0][5] > max_bytes):
            yield run
            run = []
        run.append(e)
    if run:
        yield run


def stamp_ns(text):
    day, clock = text.strip().split(' ')
    hms, frac = clock.split('.')
    base = datetime.datetime.strptime(f'{day} {hms}', '%Y-%m-%d %H:%M:%S').replace(tzinfo=datetime.timezone.utc)
    return int(base.timestamp()) * 1_000_000_000 + int(frac.ljust(9, '0')[:9])


def stamp(ns):
    return Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))


def text_member(url, members, suffix):
    entry = [e for e in members if e[0].endswith(suffix)]
    assert len(entry) == 1, (suffix, len(entry))
    return load_group(url, entry)[0][1].decode()


def deskew_rotation(pts, w_velo, period_s):
    # Rotate each point by the rotation vector w_velo * (t - 0.5) * period_s, t from its azimuth.
    from scipy.spatial.transform import Rotation
    t = 0.5 * (-np.arctan2(pts[:, 1], pts[:, 0]) / np.pi + 1.0)
    rotvec = np.outer((t - 0.5) * period_s, w_velo)
    out = pts.copy()
    out[:, :3] = Rotation.from_rotvec(rotvec).apply(pts[:, :3].astype(np.float64)).astype(np.float32)
    return out


def correct_elevation(pts, delta_rad):
    # Raise each point's elevation by delta_rad: rotate it about (p x z) / |p x z| (points on the z axis stay).
    from scipy.spatial.transform import Rotation
    xyz = pts[:, :3].astype(np.float64)
    axis = np.cross(xyz, np.array([0.0, 0.0, 1.0]))
    norm = np.linalg.norm(axis, axis=1, keepdims=True)
    ok = norm[:, 0] > 1e-9
    out = pts.copy()
    out[ok, :3] = Rotation.from_rotvec(delta_rad * axis[ok] / norm[ok]).apply(xyz[ok]).astype(np.float32)
    return out


def cloud_message(raw, header_ns, w_velo=None, period_s=0.0):
    pts = np.frombuffer(raw, dtype=np.float32).reshape(-1, 4)
    if os.environ.get('ELEVATION_CORRECTION_DEG'):
        pts = correct_elevation(pts, np.radians(float(os.environ['ELEVATION_CORRECTION_DEG'])))
    if w_velo is not None:
        pts = deskew_rotation(pts, w_velo, period_s)
    msg = PointCloud2()
    msg.header.stamp = stamp(header_ns)
    msg.header.frame_id = 'velo_link'
    msg.height, msg.width = 1, len(pts)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
    msg.is_bigendian, msg.point_step = False, 16
    msg.row_step, msg.is_dense = 16 * len(pts), True
    msg.data = pts.tobytes()
    return msg


def main():
    sync = central_directory(SYNC)
    extract = central_directory(EXTRACT)
    t_mid = [stamp_ns(s) for s in text_member(SYNC, sync, 'velodyne_points/timestamps.txt').splitlines() if s]
    t_end = [stamp_ns(s) for s in text_member(SYNC, sync, 'velodyne_points/timestamps_end.txt').splitlines() if s]
    deskew = os.environ.get('ROTATION_DESKEW') == '1'
    if deskew:
        t_start = [stamp_ns(s) for s in text_member(SYNC, sync, 'velodyne_points/timestamps_start.txt').splitlines()
                   if s]
        from scipy.spatial.transform import Rotation
        q = [float(v) for v in os.environ.get(
            'ROTATION_DESKEW_EXTRINSIC', '-0.007412088,0.001015086,0.000385188,0.999971941').split(',')]
        R_imu_velo = Rotation.from_quat(q).as_matrix()  # Velodyne vectors -> IMU (the rig's LiDAR-to-base rotation)
    scans = sorted((e for e in sync if '/velodyne_points/data/' in e[0] and e[0].endswith('.bin')),
                   key=lambda e: e[0])
    assert len(scans) == len(t_mid) == len(t_end), (len(scans), len(t_mid), len(t_end))
    a, b = 0, len(scans)
    if os.environ.get('FRAMES'):
        a, b = (int(v) for v in os.environ['FRAMES'].split(':'))
    # the IMU: every OXTS packet with an increasing stamp, from 1 s before the first kept scan
    oxts = sorted((e for e in extract if '/oxts/data/' in e[0] and e[0].endswith('.txt')), key=lambda e: e[0])
    t_imu = [stamp_ns(s) for s in text_member(EXTRACT, extract, 'oxts/timestamps.txt').splitlines() if s]
    assert len(oxts) == len(t_imu), (len(oxts), len(t_imu))
    lo, hi = t_mid[a] - 1_000_000_000, t_end[b - 1]
    keep = [i for i in range(len(oxts)) if lo <= t_imu[i] <= hi]
    rows = {}
    for group in groups([oxts[i] for i in keep], 4 << 20):
        for name, raw in load_group(EXTRACT, group):
            rows[name] = [float(v) for v in raw.decode().split()]
    imu, last, dropped = [], -1, 0
    for i in keep:
        r = rows[oxts[i][0]]
        if t_imu[i] <= last:
            dropped += 1
            continue
        last = t_imu[i]
        msg = Imu()
        msg.header.stamp = stamp(t_imu[i])
        msg.header.frame_id = 'imu_link'
        msg.orientation_covariance[0] = -1.0
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = r[11], r[12], r[13]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = r[17], r[18], r[19]
        imu.append((t_imu[i], msg))
    imu_ns = [t for t, _ in imu]
    gaps = np.diff([t for t, _ in imu]) * 1e-9
    print(f'imu: {len(imu)} samples ({dropped} with a stamp not after the one before dropped), '
          f'interval median {np.median(gaps) * 1e3:.2f} ms, max {gaps.max() * 1e3:.1f} ms', flush=True)

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=OUT, storage_id='mcap', storage_preset_profile='zstd_fast'),
                rosbag2_py.ConverterOptions('', ''))
    for topic, kind in (('/kitti/velodyne/points', 'sensor_msgs/msg/PointCloud2'),
                        ('/kitti/oxts/imu', 'sensor_msgs/msg/Imu')):
        writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=kind, serialization_format='cdr'))
    # the archive stores the scans in no particular order: one range request per scan, written in time order
    k, written, points = 0, 0, 0
    with concurrent.futures.ThreadPoolExecutor(WORKERS) as pool:
        for begin in range(a, b, 4 * WORKERS):
            idx = list(range(begin, min(b, begin + 4 * WORKERS)))
            for i, (_, raw) in zip(idx, pool.map(lambda j: load_group(SYNC, [scans[j]])[0], idx)):
                assert t_end[i] > t_mid[i] and (i == a or t_end[i] > t_end[i - 1]), i
                if deskew:
                    lo_k = bisect.bisect_left(imu_ns, t_start[i])
                    hi_k = bisect.bisect_right(imu_ns, t_end[i])
                    rates = [imu[j][1].angular_velocity for j in range(lo_k, hi_k)]
                    w_imu = np.mean([[r.x, r.y, r.z] for r in rates], axis=0) if rates else np.zeros(3)
                    msg = cloud_message(raw, t_mid[i], R_imu_velo.T @ w_imu, (t_end[i] - t_start[i]) * 1e-9)
                else:
                    msg = cloud_message(raw, t_mid[i])
                while k < len(imu) and imu[k][0] <= t_end[i]:
                    writer.write('/kitti/oxts/imu', rclpy.serialization.serialize_message(imu[k][1]), imu[k][0])
                    k += 1
                writer.write('/kitti/velodyne/points', rclpy.serialization.serialize_message(msg), t_end[i])
                written += 1
                points += msg.width
            print(f'{written}/{b - a} scans, {k} imu', flush=True)
    del writer
    with open(f'{OUT.rstrip("/")}.frames.txt', 'w') as f:
        for i in range(a, b):
            f.write(f'{i} {t_mid[i]} {t_end[i]}\n')
    print(f'done: {written} scans (frames {a}-{b - 1}), {points} points, {k} imu samples', flush=True)


if __name__ == '__main__':
    main()
