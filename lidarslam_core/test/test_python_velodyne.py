"""Velodyne packet decoding in the package (lidarslam.velodyne) on a synthetic VLP-16 packet.

One packet with three returns of laser 0 (ring 0 in VLP16db.yaml): 10 m at azimuth 0, 0.5 m
(under min_range) and 120 m (over max_range here). The point is checked against the
velodyne_pointcloud VLP-16 formula; the cloud layout is velodyne_transform_node's unorganized
XYZIRT cloud. Identity with velodyne_transform_node on real packets (Leo Drive, 1,093 scans) is
checked in bench_report/data/ros_free_packets_2026-11-17.
"""

from __future__ import annotations

import math
from pathlib import Path
import struct

import numpy as np
import pytest

lidarslam = pytest.importorskip('lidarslam')
core = pytest.importorskip('lidarslam._core')
if not hasattr(core, 'VelodyneDecoder'):
    pytest.skip('lidarslam built without the Velodyne decoder', allow_module_level=True)

from lidarslam import velodyne  # noqa: E402
from lidarslam.io import SensorLog  # noqa: E402

UPPER_BANK = 0xEEFF
SCAN_STAMP = (1_700_000_000, 0)
PACKET_STAMP = (1_700_000_000, 50_000_000)  # 0.05 s after the scan stamp


def _packet(returns: dict[tuple[int, int, int], tuple[int, int]]) -> bytes:
    """A VLP-16 packet: 12 blocks 0.2 deg apart; returns[(block, firing, dsr)] = (units, intensity)."""
    data = bytearray()
    for block in range(12):
        data += struct.pack('<HH', UPPER_BANK, block * 20)
        for firing in range(2):
            for dsr in range(16):
                units, intensity = returns.get((block, firing, dsr), (0, 0))
                data += struct.pack('<HB', units, intensity)
    data += struct.pack('<IBB', 0, 0x37, 0x22)  # timestamp, strongest return, VLP-16
    assert len(data) == 1206
    return bytes(data)


RETURNS = {(0, 0, 0): (5000, 100),    # 10 m (2 mm units), intensity 100
           (1, 1, 3): (250, 7),       # 0.5 m: under min_range 0.9
           (2, 0, 5): (60000, 9)}     # 120 m: over max_range 100 below


def _header() -> core.Header:
    header = core.Header()
    header.stamp.sec, header.stamp.nanosec = SCAN_STAMP
    header.frame_id = 'velodyne_front'
    return header


def test_vlp16_point_layout_and_range_limits() -> None:
    decoder = velodyne.Decoder(max_range=100.0)
    assert decoder.num_lasers == 16
    cloud = decoder._decoder.decode(_header(), [(*PACKET_STAMP, _packet(RETURNS))])

    assert (cloud.header.stamp.sec, cloud.header.stamp.nanosec) == SCAN_STAMP
    assert cloud.header.frame_id == 'velodyne_front'
    assert [(f.name, f.offset, f.datatype, f.count) for f in cloud.fields] == [
        ('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1), ('intensity', 12, 7, 1),
        ('ring', 16, 4, 1), ('time', 18, 7, 1)]
    assert (cloud.height, cloud.width, cloud.point_step, cloud.row_step) == (1, 1, 22, 22)
    assert cloud.is_dense and not cloud.is_bigendian

    x, y, z, intensity, ring, time = struct.unpack('<ffffHf', bytes(cloud.data))
    calibration = velodyne.load_calibration(velodyne.calibration_for_model('VLP16'))
    laser = next(laser for laser in calibration['lasers'] if laser['laser_id'] == '0')
    vert = np.float32(float(laser['vert_correction']))
    offset = np.float32(float(laser['vert_offset_correction']))
    distance = np.float32(5000) * np.float32(0.002)
    # azimuth 0: x is the distance in the xy plane, y is 0, z the vertical part
    assert x == pytest.approx(distance * np.cos(vert) - offset * np.sin(vert), abs=1e-5)
    assert y == 0.0
    assert z == pytest.approx(distance * np.sin(vert) + offset * np.cos(vert), abs=1e-5)
    assert (intensity, ring) == (100.0, 0)  # laser 0 is the lowest beam: ring 0
    assert time == pytest.approx(0.05, abs=1e-7)  # packet - scan stamp + timing offset 0


def test_defaults_and_parameters() -> None:
    settings, rest = velodyne.split_parameters(
        {'velodyne.max_range': 50.0, 'lidar_topic': '/points'})
    assert rest == {'lidar_topic': '/points'}
    assert settings['model'] == 'VLP16' and settings['max_range'] == 50.0
    assert settings['min_range'] == 0.9 and settings['view_width'] == pytest.approx(2 * math.pi)
    with pytest.raises(KeyError, match='velodyne.rpm'):
        velodyne.split_parameters({'velodyne.rpm': 600})
    with pytest.raises(ValueError, match='no default calibration'):
        velodyne.Decoder(model='HDL64')
    # 120 m is kept with the default max_range of 200 m
    cloud = velodyne.Decoder()._decoder.decode(_header(), [(*PACKET_STAMP, _packet(RETURNS))])
    assert cloud.width == 2


def test_sensor_log_decodes_velodyne_scans(tmp_path: Path) -> None:
    from rosbags.rosbag2 import StoragePlugin, Writer
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.LATEST)
    velodyne.register_types(typestore)
    scan_type = typestore.types['velodyne_msgs/msg/VelodyneScan']
    packet_type = typestore.types['velodyne_msgs/msg/VelodynePacket']
    time_type = typestore.types['builtin_interfaces/msg/Time']
    header_type = typestore.types['std_msgs/msg/Header']
    packet = packet_type(stamp=time_type(sec=PACKET_STAMP[0], nanosec=PACKET_STAMP[1]),
                         data=np.frombuffer(_packet(RETURNS), dtype=np.uint8))
    scan = scan_type(header=header_type(stamp=time_type(sec=SCAN_STAMP[0], nanosec=SCAN_STAMP[1]),
                                        frame_id='velodyne_front'), packets=[packet, packet])
    bag = tmp_path / 'packets'
    with Writer(bag, version=8, storage_plugin=StoragePlugin.MCAP) as writer:
        connection = writer.add_connection(
            '/velodyne_packets', 'velodyne_msgs/msg/VelodyneScan', typestore=typestore)
        writer.write(connection, 1_700_000_000_100_000_000,
                     typestore.serialize_cdr(scan, 'velodyne_msgs/msg/VelodyneScan'))

    with SensorLog(bag) as log:  # no decoder: the packets are not a supported topic
        with pytest.raises(KeyError, match='Velodyne decoder'):
            list(log.records(['/velodyne_packets']))
    with SensorLog(bag, velodyne=velodyne.Decoder(max_range=100.0)) as log:
        records = list(log.records(['/velodyne_packets']))
    assert len(records) == 1
    cloud = records[0].message
    assert cloud.width == 2  # one point of each of the two packets
    assert cloud.header.frame_id == 'velodyne_front'
