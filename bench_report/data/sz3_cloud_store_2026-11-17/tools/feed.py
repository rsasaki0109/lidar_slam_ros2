"""Stream every 5th /rko_lio/frame cloud of a backend input as [u64 n][x][y][z][i] float32 planes."""
import struct
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

DTYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}

out = sys.stdout.buffer
with AnyReader([Path(sys.argv[1])]) as reader:
    conns = [c for c in reader.connections if c.topic == '/rko_lio/frame']
    for index, (conn, _t, raw) in enumerate(reader.messages(connections=conns)):
        if index % 5:
            continue
        msg = reader.deserialize(raw, conn.msgtype)
        n = msg.width * msg.height
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
        fields = {f.name: f for f in msg.fields}
        planes = []
        for name in ('x', 'y', 'z', 'intensity'):
            f = fields.get(name)
            if f is None:
                planes.append(np.zeros(n, np.float32))
                continue
            dt = np.dtype(DTYPES[f.datatype]).newbyteorder('>' if msg.is_bigendian else '<')
            col = buf[:, f.offset:f.offset + dt.itemsize].copy().view(dt).reshape(n)
            planes.append(col.astype(np.float32))
        out.write(struct.pack('<Q', n))
        for p in planes:
            out.write(np.ascontiguousarray(p, dtype='<f4').tobytes())
