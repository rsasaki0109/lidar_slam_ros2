#!/usr/bin/env python3
"""Write a GNSS-only bag (/gnss/fix, sensor_msgs/NavSatFix, mcap) from a CSV of p3_ms_gnss_gate.py export.

For graph_slam_multisession_join's multisession_gnss_bag_paths when a session's backend input was recorded without
GNSS (plan multisession_gnss_2026-11-07/plan_build.txt). Run with the ROS environment sourced and `python3 -P`.

usage: p3_ms_gnss_bag.py GNSS_CSV OUT_BAG_DIR
"""
import math
import sys

import numpy as np


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import NavSatFix
    rows = np.loadtxt(sys.argv[1], delimiter=',', skiprows=1, ndmin=2)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=sys.argv[2], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name='/gnss/fix', type='sensor_msgs/msg/NavSatFix', serialization_format='cdr'))
    for stamp, lat, lon, alt, status, cov_xx, cov_yy in rows:
        m = NavSatFix()
        sec = math.floor(stamp)
        m.header.stamp.sec = int(sec)
        m.header.stamp.nanosec = min(int(round((stamp - sec) * 1e9)), 999999999)
        m.header.frame_id = 'gnss'
        m.latitude, m.longitude, m.altitude = float(lat), float(lon), float(alt)
        m.status.status = int(status)
        cov = [0.0] * 9
        cov[0], cov[4] = float(cov_xx), float(cov_yy)
        m.position_covariance = cov
        writer.write('/gnss/fix', serialize_message(m), int(round(stamp * 1e9)))
    print(f'{len(rows)} fixes -> {sys.argv[2]}')


if __name__ == '__main__':
    main()
