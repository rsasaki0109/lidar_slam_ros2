"""P2 input: bag1's IMU and front packet log times, each packet scan replaced by the node's cloud."""
import sys
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore
from lidarslam.velodyne import register_types

bag, ref, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
ts = get_typestore(Stores.LATEST); register_types(ts)
with AnyReader([ref], default_typestore=ts) as r:
    conn = [c for c in r.connections if c.topic == '/ref/front/points']
    clouds = [raw for _, _, raw in r.messages(connections=conn)]
with AnyReader([bag], default_typestore=ts) as r, Writer(out, version=8, storage_plugin=StoragePlugin.MCAP) as w:
    conns = [c for c in r.connections if c.topic in ('/sensing/imu/imu_data', '/sensing/lidar/front/velodyne_packets')]
    imu = w.add_connection('/sensing/imu/imu_data', 'sensor_msgs/msg/Imu', typestore=ts)
    pts = w.add_connection('/ref/front/points', 'sensor_msgs/msg/PointCloud2', typestore=ts)
    i = 0
    for c, t, raw in r.messages(connections=conns):
        if c.topic == '/sensing/imu/imu_data':
            w.write(imu, t, raw)
        else:
            w.write(pts, t, clouds[i]); i += 1
    assert i == len(clouds), (i, len(clouds))
print('clouds', len(clouds))
