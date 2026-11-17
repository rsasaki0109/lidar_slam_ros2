"""P1: every scan the package decodes equals velodyne_transform_node's cloud (all PointCloud2 fields)."""
import hashlib, json, sys, time
from lidarslam.io import SensorLog
from lidarslam.velodyne import Decoder

bag, ref = sys.argv[1], sys.argv[2]
streams = ['front', 'left', 'right']
decoder = Decoder()  # VLP16, VLP16db.yaml, min 0.9, max 200, view 0 / 2 pi: the benchmark's parameters

def key(c):
    return (c.header.stamp.sec, c.header.stamp.nanosec, c.header.frame_id, c.height, c.width,
            [(f.name, f.offset, f.datatype, f.count) for f in c.fields], c.is_bigendian,
            c.point_step, c.row_step, c.is_dense)

report = {}
with SensorLog(ref) as r:
    refs = {s: [] for s in streams}
    for rec in r.records([f'/ref/{s}/points' for s in streams]):
        refs[rec.topic.split('/')[2]].append((key(rec.message), bytes(rec.message.data)))
with SensorLog(bag, velodyne=decoder) as b:
    got = {s: [] for s in streams}
    t0 = time.perf_counter()
    for rec in b.records([f'/sensing/lidar/{s}/velodyne_packets' for s in streams]):
        got[rec.topic.split('/')[3]].append((key(rec.message), bytes(rec.message.data)))
    elapsed = time.perf_counter() - t0
for s in streams:
    a, e = got[s], refs[s]
    meta_diff = sum(1 for x, y in zip(a, e) if x[0] != y[0])
    data_diff = sum(1 for x, y in zip(a, e) if x[1] != y[1])
    report[s] = {'package_scans': len(a), 'node_clouds': len(e), 'metadata_differ': meta_diff,
                 'data_differ': data_diff, 'points': sum(x[0][4] for x in a),
                 'sha256_package': hashlib.sha256(b''.join(x[1] for x in a)).hexdigest(),
                 'sha256_node': hashlib.sha256(b''.join(x[1] for x in e)).hexdigest()}
    if meta_diff:
        i = next(i for i, (x, y) in enumerate(zip(a, e)) if x[0] != y[0])
        report[s]['first_metadata_diff'] = {'index': i, 'package': repr(a[i][0]), 'node': repr(e[i][0])}
report['read_and_decode_s'] = round(elapsed, 2)
print(json.dumps(report, indent=1))
