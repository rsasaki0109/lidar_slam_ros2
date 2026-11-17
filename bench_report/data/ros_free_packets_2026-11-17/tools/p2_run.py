"""P2: RKO-LIO in the package on the packets vs on the node's clouds (same IMU, same log times)."""
import hashlib, json, sys, time
from pathlib import Path
from lidarslam import odometry

bag, ref_input, out = sys.argv[1], sys.argv[2], Path(sys.argv[3])
base = odometry.load_parameters('/home/user/lidar_slam_ros2/lidarslam/param/rko_lio_kitti_odometry.yaml')
base.update({
    'imu_topic': '/sensing/imu/imu_data', 'base_frame': 'base_link',
    # /tf_static: base_link -> sensor_kit_base_link -> velodyne_front_base_link -> velodyne_front
    'extrinsic_lidar2base_quat_xyzw_xyz': [-0.018406, 0.100165, 0.001853, 0.994799,
                                           6.20751059, -0.12360541, 0.20691797],
    'extrinsic_imu2base_quat_xyzw_xyz': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    'deskew': True,
    # velodyne_pointcloud's time field: seconds from the scan stamp (19-77 ms here)
    'lidar_timestamps.force_relative': True,
})
report = {}
for name, log, topic in (('packets', bag, '/sensing/lidar/front/velodyne_packets'),
                         ('node_clouds', ref_input, '/ref/front/points')):
    t0 = time.perf_counter()
    summary = odometry.run(log, dict(base, lidar_topic=topic), out / name)
    summary['seconds'] = round(time.perf_counter() - t0, 1)
    for f in ('odometry_tum.txt', 'odometry.csv'):
        summary['sha256_' + f] = hashlib.sha256((out / name / f).read_bytes()).hexdigest()
    report[name] = summary
report['identical'] = all(report['packets'][k] == report['node_clouds'][k]
                          for k in ('sha256_odometry_tum.txt', 'sha256_odometry.csv', 'poses'))
print(json.dumps(report, indent=1))
