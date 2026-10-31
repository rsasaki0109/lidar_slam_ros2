#!/usr/bin/env python3
"""How many LiDAR frames the offline node reads while its oldest frame waits for IMU data.

The offline node (rko_lio) registers its oldest queued frame once an IMU sample later than the frame's last point has
been read. While it waits, the bag reader reads on and a full queue drops its oldest frame. For each LiDAR frame, this
counts the LiDAR frames read from it (itself included) until the first IMU sample past its end, in the bag's order. The
end is taken as the header stamp + END_S. Frames with no IMU sample past their end are counted apart.
Usage: p4_lidar_queue_check.py <bag_dir> [<bag_dir>...]  (topics found from the bag: one Imu and one PointCloud2)
"""
import struct
import sys

import rosbag2_py

END_S = 0.2


def stamp(data):
    # CDR: a 4-byte encapsulation header, then std_msgs/Header's stamp (int32 sec, uint32 nanosec).
    little = data[1] == 1
    sec, nsec = struct.unpack('<iI' if little else '>iI', bytes(data[4:12]))
    return sec + nsec * 1e-9


def check(bag):
    info = rosbag2_py.Info().read_metadata(bag, '')
    types = {t.topic_metadata.name: t.topic_metadata.type for t in info.topics_with_message_count}
    imu = [n for n, t in types.items() if t == 'sensor_msgs/msg/Imu']
    lidar = [n for n, t in types.items() if t == 'sensor_msgs/msg/PointCloud2']
    if len(imu) != 1 or len(lidar) != 1:
        return f'{bag},skipped: imu {imu} lidar {lidar}'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=info.storage_identifier),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[imu[0], lidar[0]]))
    waiting = []  # [end, frames read since] of frames still waiting, oldest first
    worst = 0
    frames = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        t = stamp(data)
        if topic == lidar[0]:
            frames += 1
            waiting.append([t + END_S, 0])
            for entry in waiting:
                entry[1] += 1
        else:
            while waiting and t > waiting[0][0]:
                worst = max(worst, waiting.pop(0)[1])
            # A later frame can end before an older one (stamps out of order): it is counted when it reaches the front.
    return f'{bag},{lidar[0]},{imu[0]},{frames},{worst},{len(waiting)}'


print('bag,lidar_topic,imu_topic,frames,most_frames_read_while_waiting,frames_never_past')
for bag in sys.argv[1:]:
    print(check(bag), flush=True)
