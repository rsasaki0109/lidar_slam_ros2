#!/usr/bin/env python3
"""Write a copy of a backend-input bag whose /rko_lio/frame points are moved into the base frame
(p_base = T_base_lidar * p_lidar, the exp04 RKO-LIO extrinsic of frame_check.py). Every other field and
/rko_lio/odometry are copied unchanged. Usage: pretransform_bag.py <in_bag> <out_bag>
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import frame_check as fc  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message, serialize_message  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402


def main():
    src, dst = sys.argv[1], sys.argv[2]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=src, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=dst, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for meta in reader.get_all_topics_and_types():
        writer.create_topic(meta)
    T = fc.se3(fc.rot(fc.QX, fc.QY, fc.QZ, fc.QW), [fc.TX, fc.TY, fc.TZ])
    moved = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/rko_lio/frame':
            m = deserialize_message(data, PointCloud2)
            offsets = {f.name: f.offset for f in m.fields}
            assert all(f.datatype == 7 for f in m.fields if f.name in 'xyz'), 'x/y/z must be FLOAT32'
            raw = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height * m.width, m.point_step).copy()
            xyz = np.stack([raw[:, offsets[a]:offsets[a] + 4].copy().view(np.float32).reshape(-1)
                            for a in 'xyz'], axis=1).astype(np.float64)
            out = (xyz @ T[:3, :3].T + T[:3, 3]).astype(np.float32)
            for k, a in enumerate('xyz'):
                raw[:, offsets[a]:offsets[a] + 4] = out[:, k].copy().view(np.uint8).reshape(-1, 4)
            m.data = raw.reshape(-1).tobytes()
            m.header.frame_id = 'base_link'
            data = serialize_message(m)
            moved += 1
        writer.write(topic, data, t)
    del writer
    print('transformed', moved, 'clouds')


if __name__ == '__main__':
    main()
