"""P3: `lidarslam map` on a PointCloud2 + IMU log; sys.argv[1] picks the package directory."""
import sys
sys.path.insert(0, sys.argv[1])
import lidarslam, json
from lidarslam import backend, mapping
from lidarslam.odometry import load_parameters
assert lidarslam.__file__.startswith(sys.argv[1]), lidarslam.__file__
odo = load_parameters('/home/user/lidar_slam_ros2/lidarslam/param/rko_lio_kitti_odometry.yaml')
odo.update({'lidar_topic': '/ref/front/points', 'imu_topic': '/sensing/imu/imu_data', 'base_frame': 'base_link',
            'extrinsic_lidar2base_quat_xyzw_xyz': [-0.018406, 0.100165, 0.001853, 0.994799, 6.20751059, -0.12360541, 0.20691797],
            'extrinsic_imu2base_quat_xyzw_xyz': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            'deskew': True, 'lidar_timestamps.force_relative': True})
be = backend.load_parameters('/home/user/lidar_slam_ros2/lidarslam/param/lidarslam.yaml')
print(json.dumps(mapping.run(sys.argv[2], odo, be, sys.argv[3], backend_input=sys.argv[3] + '_input'), default=str))
