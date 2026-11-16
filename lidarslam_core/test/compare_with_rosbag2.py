"""Check lidarslam.io against ROS 2's own reader on a real log (needs a ROS 2 environment).

Reads every PointCloud2, Imu and NavSatFix message of a rosbag2 with rosbag2_py and with
lidarslam.io.SensorLog and requires identical headers, layouts, data bytes and values.

    source /opt/ros/<distro>/setup.bash
    PYTHONPATH=<site-packages with lidarslam>:$PYTHONPATH python3 compare_with_rosbag2.py BAG
"""

from __future__ import annotations

import sys

from lidarslam.io import SUPPORTED_TYPES, SensorLog
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def ros_records(bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {name: get_message(t) for name, t in types.items() if t in SUPPORTED_TYPES}
    while reader.has_next():
        topic, raw, log_time = reader.read_next()
        if topic in classes:
            yield topic, log_time, types[topic], deserialize_message(raw, classes[topic])


def same_header(a, b) -> bool:
    return (a.stamp.sec, a.stamp.nanosec, a.frame_id) == (
        b.header.stamp.sec, b.header.stamp.nanosec, b.header.frame_id)


def compare(ros_msg, msgtype: str, ours) -> bool:
    if not same_header(ros_msg.header, ours):
        return False
    if msgtype == 'sensor_msgs/msg/PointCloud2':
        return (
            (ros_msg.height, ros_msg.width, ros_msg.is_bigendian, ros_msg.point_step,
             ros_msg.row_step, ros_msg.is_dense)
            == (ours.height, ours.width, ours.is_bigendian, ours.point_step, ours.row_step,
                ours.is_dense)
            and [(f.name, f.offset, f.datatype, f.count) for f in ros_msg.fields]
            == [(f.name, f.offset, f.datatype, f.count) for f in ours.fields]
            and bytes(ros_msg.data) == ours.data)
    if msgtype == 'sensor_msgs/msg/Imu':
        q, w, a = ros_msg.orientation, ros_msg.angular_velocity, ros_msg.linear_acceleration
        return (
            [q.x, q.y, q.z, q.w] == ours.orientation
            and [w.x, w.y, w.z] == ours.angular_velocity
            and [a.x, a.y, a.z] == ours.linear_acceleration
            and list(ros_msg.orientation_covariance) == ours.orientation_covariance
            and list(ros_msg.angular_velocity_covariance) == ours.angular_velocity_covariance
            and list(ros_msg.linear_acceleration_covariance)
            == ours.linear_acceleration_covariance)
    return (
        (ros_msg.status.status, ros_msg.status.service, ros_msg.latitude, ros_msg.longitude,
         ros_msg.altitude, ros_msg.position_covariance_type)
        == (ours.status, ours.service, ours.latitude, ours.longitude, ours.altitude,
            ours.position_covariance_type)
        and list(ros_msg.position_covariance) == ours.position_covariance)


def main(bag: str) -> int:
    counts: dict[str, int] = {}
    mismatches = 0
    with SensorLog(bag) as log:
        ours = log.records()
        for topic, log_time, msgtype, ros_msg in ros_records(bag):
            record = next(ours)
            ok = (record.topic == topic and record.log_time_ns == log_time
                  and compare(ros_msg, msgtype, record.message))
            mismatches += not ok
            counts[topic] = counts.get(topic, 0) + 1
        leftover = sum(1 for _ in ours)
    for topic, n in sorted(counts.items()):
        print(f'{topic}: {n} messages compared')
    print(f'mismatches: {mismatches}, messages only in lidarslam.io: {leftover}')
    return 0 if mismatches == 0 and leftover == 0 else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
