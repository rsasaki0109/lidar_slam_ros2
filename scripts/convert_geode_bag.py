#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Convert a GEODE ROS 1 bag to a ROS 2 bag with only the LiDAR and the Xsens IMU.

Ouster (beta) PointCloud2 clouds are copied. Livox (gamma) livox_ros_driver/CustomMsg
clouds become PointCloud2 with x, y, z, intensity (the reflectivity) and t, the point's
offset from the scan start in ns.

Velodyne (alpha) clouds carry `time` in seconds relative to a stamp at the scan end, so
most offsets are negative. FAST-LIO2 and Point-LIO expect offsets from the scan start
and skip deskewing the rest. Their stamp is moved to the first point and `time` made
non-negative, which keeps every point's absolute time. With --ros1-out the same
normalized LiDAR and IMU are also written as a ROS 1 bag for the ROS 1 rivals.

usage: convert_geode_bag.py INPUT.bag OUTPUT_DIR [--ros1-out OUTPUT.bag]
"""

import argparse
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.rosbag1 import Writer as Writer1
from rosbags.rosbag2 import Writer
from rosbags.typesys import get_types_from_msg, get_typestore, Stores

LIDAR_TOPICS = ('/velodyne_points', '/ouster/points', '/livox/lidar')
IMU_TOPIC = '/imu/data'
LIVOX_POINT = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                        ('intensity', '<f4'), ('t', '<u4')])


def normalize_velodyne(message, ros1):
    """Stamp the cloud at its first point and make `time` non-negative (seconds)."""
    names = [field.name for field in message.fields]
    offset = message.fields[names.index('time')].offset
    data = np.frombuffer(bytes(message.data), dtype=np.uint8).copy()
    count = message.width * message.height
    times = np.ndarray((count,), dtype='<f4', buffer=data, offset=offset,
                       strides=(message.point_step,))
    if count == 0:
        return message
    first = float(times.min())
    times -= first
    stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
    stamp_ns += int(round(first * 1e9))
    Time = ros1.types['builtin_interfaces/msg/Time']
    message.header.stamp = Time(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)
    message.data = data
    return message


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('source', type=Path, help='GEODE ROS 1 bag')
    parser.add_argument('target', type=Path, help='new ROS 2 bag directory')
    parser.add_argument('--ros1-out', type=Path, help='also write the LiDAR and IMU as ROS 1')
    args = parser.parse_args()
    source, target, ros1_out = args.source, args.target, args.ros1_out
    ros1 = get_typestore(Stores.ROS1_NOETIC)
    ros2 = get_typestore(Stores.ROS2_HUMBLE)
    Header = ros2.types['std_msgs/msg/Header']
    Time = ros2.types['builtin_interfaces/msg/Time']
    PointCloud2 = ros2.types['sensor_msgs/msg/PointCloud2']
    PointField = ros2.types['sensor_msgs/msg/PointField']
    Imu = ros2.types['sensor_msgs/msg/Imu']

    def header(ros1_header):
        return Header(stamp=Time(sec=ros1_header.stamp.sec, nanosec=ros1_header.stamp.nanosec),
                      frame_id=ros1_header.frame_id)

    with Reader(source) as reader:
        for connection in reader.connections:
            if connection.msgtype.startswith('livox_ros_driver/'):
                ros1.register(get_types_from_msg(connection.msgdef.data, connection.msgtype))
        wanted = [c for c in reader.connections if c.topic in LIDAR_TOPICS + (IMU_TOPIC,)]
        writer1 = Writer1(ros1_out) if ros1_out else None
        if writer1:
            writer1.open()
            outputs1 = {c.topic: writer1.add_connection(c.topic, c.msgtype, typestore=ros1)
                        for c in wanted}
        with Writer(target, version=8) as writer:
            outputs = {}
            for connection in wanted:
                msgtype = ('sensor_msgs/msg/Imu' if connection.topic == IMU_TOPIC
                           else 'sensor_msgs/msg/PointCloud2')
                outputs[connection.topic] = (writer.add_connection(
                    connection.topic, msgtype, typestore=ros2), msgtype)
            for connection, stamp, raw in reader.messages(connections=wanted):
                message = ros1.deserialize_ros1(raw, connection.msgtype)
                if connection.topic == '/velodyne_points':
                    message = normalize_velodyne(message, ros1)
                    raw = ros1.serialize_ros1(message, connection.msgtype)
                if writer1:
                    writer1.write(outputs1[connection.topic], stamp, raw)
                output, msgtype = outputs[connection.topic]
                if connection.topic == IMU_TOPIC:
                    converted = Imu(
                        header=header(message.header),
                        orientation=ros2.types['geometry_msgs/msg/Quaternion'](
                            x=message.orientation.x, y=message.orientation.y,
                            z=message.orientation.z, w=message.orientation.w),
                        orientation_covariance=message.orientation_covariance,
                        angular_velocity=ros2.types['geometry_msgs/msg/Vector3'](
                            x=message.angular_velocity.x, y=message.angular_velocity.y,
                            z=message.angular_velocity.z),
                        angular_velocity_covariance=message.angular_velocity_covariance,
                        linear_acceleration=ros2.types['geometry_msgs/msg/Vector3'](
                            x=message.linear_acceleration.x, y=message.linear_acceleration.y,
                            z=message.linear_acceleration.z),
                        linear_acceleration_covariance=message.linear_acceleration_covariance)
                elif connection.msgtype.startswith('livox_ros_driver/'):
                    points = np.zeros(len(message.points), dtype=LIVOX_POINT)
                    for index, point in enumerate(message.points):
                        points[index] = (point.x, point.y, point.z, point.reflectivity,
                                         point.offset_time)
                    fields = [PointField(name=name, offset=LIVOX_POINT.fields[name][1],
                                         datatype=6 if name == 't' else 7, count=1)
                              for name in LIVOX_POINT.names]
                    converted = PointCloud2(
                        header=header(message.header), height=1, width=len(points),
                        fields=fields, is_bigendian=False, point_step=LIVOX_POINT.itemsize,
                        row_step=LIVOX_POINT.itemsize * len(points),
                        data=points.view(np.uint8), is_dense=True)
                else:
                    converted = PointCloud2(
                        header=header(message.header), height=message.height,
                        width=message.width,
                        fields=[PointField(name=f.name, offset=f.offset, datatype=f.datatype,
                                           count=f.count) for f in message.fields],
                        is_bigendian=message.is_bigendian, point_step=message.point_step,
                        row_step=message.row_step, data=message.data, is_dense=message.is_dense)
                writer.write(output, stamp, ros2.serialize_cdr(converted, msgtype))
        if writer1:
            writer1.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
