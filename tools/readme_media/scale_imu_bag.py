"""Copy a rosbag2 keeping /livox/points and rescaling /livox/imu acceleration from g to m/s^2."""
from pathlib import Path
import sys

from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import get_typestore, Stores

G = 9.80665
src, dst = Path(sys.argv[1]), Path(sys.argv[2])
ts = get_typestore(Stores.ROS2_HUMBLE)
with Reader(src) as reader, Writer(dst, version=8) as writer:
    conns = {}
    for c in reader.connections:
        if c.topic in ('/livox/points', '/livox/imu'):
            conns[c.id] = writer.add_connection(c.topic, c.msgtype, typestore=ts)
    keep = [c for c in reader.connections if c.id in conns]
    n = 0
    for c, t, raw in reader.messages(connections=keep):
        if c.topic == '/livox/imu':
            msg = ts.deserialize_cdr(raw, c.msgtype)
            a = msg.linear_acceleration
            a.x, a.y, a.z = a.x * G, a.y * G, a.z * G
            raw = ts.serialize_cdr(msg, c.msgtype)
        writer.write(conns[c.id], t, raw)
        n += 1
print('wrote', n)
