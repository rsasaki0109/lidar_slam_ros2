#!/usr/bin/env python3
"""Rig files of the second blind run from the datasets' calibration files (bench_report/data/blind_v7_run2_2026-11-10).

usage: p8_make_rigs.py <data dir>
Writes <data dir>/rig_<name>.yaml (topics, the LiDAR pose in the IMU frame, FAST-LIO2's sensor settings, the converter's
options) and <data dir>/g_params_<name>.yaml (G's complete parameter file: rig keys, the frozen G22 keys and, for a
tactical-grade gyro, the keys of rig_tactical_gyro_2026-11-09.yaml).

FusionPortable V2: ouster00.yaml gives the body IMU (STIM300) pose in the LiDAR frame (quaternion_sensor_body_imu as
w, x, y, z and translation_sensor_body_imu); the uncommented entries are taken. MCD: the yaml gives each sensor's pose
in the body frame, which is the VN-100 (ATV) or VN-200 (handheld) frame; os_sensor is the LiDAR. PASTEL: the OS0-128
with the OS1-16's IMU, the pair of the authors' FAST-LIO file (selection_h8.txt).
"""
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

REPO = '/home/user/lidar_slam_ros2'
OUSTER_FIELDS = 'x,y,z,intensity,t,reflectivity,ring,ambient,range'


def fusionportable(path):
    c = yaml.safe_load(open(path))
    q = c['quaternion_sensor_body_imu']['data']  # w, x, y, z
    R = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    t = np.asarray(c['translation_sensor_body_imu']['data'], dtype=float)
    return R.T, -R.T @ t  # the LiDAR pose in the IMU frame


def mcd(path):
    T = np.asarray(yaml.safe_load(open(path))['body']['os_sensor']['T'], dtype=float)
    return T[:3, :3], T[:3, 3]


def pastel(path):
    # os0_128_lidar.yaml is an OpenCV yaml; T_OS1_IMU_OS0 is the OS0-128 pose in the OS1-16 IMU frame (the matrix the
    # authors' FAST-LIO file passes as extrinsic_R / extrinsic_T)
    text = open(path).read()
    block = text[text.index('T_OS1_IMU_OS0'):]
    data = block[block.index('data:'):]
    T = np.array([float(v) for v in data[data.index('[') + 1:data.index(']')].split(',')]).reshape(4, 4)
    R = T[:3, :3].copy()
    # As printed the matrix is a reflection (determinant -1): its [2, 2] entry is -0.9992. With +0.9992 it is the
    # rotation the dataset's own URDF gives between the two sensors, and the three IMUs of the pipeline bag agree
    # that the sensors are mounted the same way up (selection_h8.txt). The sign is corrected for every method.
    if np.linalg.det(R) < 0:
        R[2, 2] = -R[2, 2]
    U, _, Vt = np.linalg.svd(R)  # the file gives four decimals; take the nearest rotation
    R = U @ Vt
    assert np.linalg.det(R) > 0.99
    return R, T[:3, 3]


def main():
    d = sys.argv[1]
    g22 = yaml.safe_load(open(f'{REPO}/configs/sota_v7/rko_lio_global_G22_2026-11-10.yaml'))['/**']['ros__parameters']
    tac = yaml.safe_load(open(f'{REPO}/configs/sota_v7/rig_tactical_gyro_2026-11-09.yaml'))['/**']['ros__parameters']
    fp = dict(lidar='/os_cloud_node/points', imu='/stim300/imu/data_raw', lines=128, tactical=True, offset=0.0)
    rigs = {
        'fp_handheld': dict(fp, ext=fusionportable(f'{d}/calibration/20230403_calib__ouster00.yaml')),
        'fp_ugv': dict(fp, ext=fusionportable(f'{d}/calibration/20230426_calib__ouster00.yaml')),
        'fp_legged': dict(fp, ext=fusionportable(f'{d}/calibration/20230912_calib__ouster00.yaml')),
        'mcd_atv': dict(lidar='/os_cloud_node/points', imu='/vn100/imu', lines=128, tactical=False, offset=-0.1,
                        ext=mcd(f'{d}/calibration/mcd_atv.yaml')),
        'mcd_hhs': dict(lidar='/os_cloud_node/points', imu='/vn200/imu', lines=64, tactical=False, offset=-0.1,
                        ext=mcd(f'{d}/calibration/mcd_hhs.yaml')),
        'pastel': dict(lidar='/os0_128/points_filtered', imu='/os1_16/imu', lines=128, tactical=False, offset=0.0,
                       ext=pastel(f'{d}/calibration/pastel/os0_128_lidar.yaml')),
    }
    for name, r in rigs.items():
        R, t = r['ext']
        q = Rotation.from_matrix(R).as_quat()  # x, y, z, w
        ext = [round(float(v), 9) for v in list(q) + list(t)]
        rig = {'lidar_topic': r['lidar'], 'imu_topic': r['imu'], 'lidar_to_imu_quat_xyzw_xyz': ext,
               'g_params': f'{REPO}/{d.split(REPO + "/")[-1]}/g_params_{name}.yaml' if d.startswith(REPO)
               else f'{d}/g_params_{name}.yaml',
               'tactical_gyro': r['tactical'],
               'fastlio_config': 'ouster64.yaml',
               'fastlio_preprocess': {'lidar_type': 3, 'scan_line': r['lines'], 'timestamp_unit': 3, 'blind': 0.5},
               'convert': {'keep': OUSTER_FIELDS, 'drop_zero': True, 'lidar_stamp_offset': r['offset']}}
        yaml.safe_dump(rig, open(f'{d}/rig_{name}.yaml', 'w'), sort_keys=False)
        keys = {'base_frame': 'base_link', 'odom_frame': 'odom', 'lidar_frame': '', 'imu_frame': '',
                'odom_topic': '/rko_lio/odometry', 'publish_deskewed_scan': False, 'dump_results': True,
                'run_name': 'front', 'imu_acceleration_unit': 'auto',
                'extrinsic_imu2base_quat_xyzw_xyz': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                'extrinsic_lidar2base_quat_xyzw_xyz': ext}
        keys.update(g22)
        if r['tactical']:
            keys.update(tac)
        yaml.safe_dump({'/**': {'ros__parameters': keys}}, open(f'{d}/g_params_{name}.yaml', 'w'), sort_keys=False)
        print(name, ext)


if __name__ == '__main__':
    main()
