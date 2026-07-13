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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
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

"""Unit tests for the PCD-sequence rosbag converter."""

import importlib.util
from pathlib import Path
import struct

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'pcd_sequence', ROOT / 'scripts/pcd_sequence_to_rosbag2.py')
converter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(converter)


def test_reads_packed_binary_xyzi_pcd(tmp_path):
    path = tmp_path / 'cloud.pcd'
    data = struct.pack('<8f', 1, 2, 3, 0.5, 4, 5, 6, 0.7)
    path.write_bytes(
        b'VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
        b'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\n'
        b'DATA binary\n' + data)

    points, packed = converter.read_binary_xyzi_pcd(path)

    assert points == 2
    assert packed == data


def test_converts_matrix_pose_to_tum():
    line = '1 0 0 2 0 1 0 3 0 0 1 4'

    fields = converter.matrix_line_to_tum(0.2, line).split()

    assert [float(value) for value in fields[:4]] == [0.2, 2.0, 3.0, 4.0]
    assert [float(value) for value in fields[4:]] == [0.0, 0.0, 0.0, 1.0]


def test_loads_kitti_calib_and_converts_camera_pose_to_velodyne(tmp_path):
    calib = tmp_path / 'calib.txt'
    # camera_T_velodyne translates a Velodyne origin to camera x=1.
    calib.write_text('Tr: 1 0 0 1 0 1 0 0 0 0 1 0\n')

    camera_to_velodyne = converter.load_kitti_camera_to_velodyne(calib)
    fields = converter.matrix_line_to_tum(
        0.0, '1 0 0 10 0 1 0 0 0 0 1 0', camera_to_velodyne).split()

    assert [float(value) for value in fields[1:4]] == [9.0, 0.0, 0.0]


def test_gt_frame_contract_prevents_double_calibration(tmp_path):
    calib = tmp_path / 'calib.txt'
    calib.write_text('Tr: 1 0 0 1 0 1 0 0 0 0 1 0\n')

    with pytest.raises(ValueError, match='requires --gt-frame camera'):
        converter.ground_truth_body_transform('lidar', calib)
    with pytest.raises(ValueError, match='requires --calib'):
        converter.ground_truth_body_transform('camera', None)
    assert converter.ground_truth_body_transform('lidar', None) is None
    np.testing.assert_allclose(
        converter.ground_truth_body_transform('camera', calib),
        converter.load_kitti_camera_to_velodyne(calib))


def test_rejects_non_xyzi_layout(tmp_path):
    path = tmp_path / 'cloud.pcd'
    path.write_bytes(
        b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
        b'POINTS 1\nDATA binary\n' + struct.pack('<3f', 1, 2, 3))

    with pytest.raises(ValueError, match='expected packed'):
        converter.read_binary_xyzi_pcd(path)


def test_stride_sampling_is_deterministic_and_keeps_order():
    values = np.arange(40, dtype='<f4').reshape(10, 4)

    points, packed = converter.stride_xyzi(10, values.tobytes(), 3)

    assert points == 4
    np.testing.assert_array_equal(
        np.frombuffer(packed, dtype='<f4').reshape(-1, 4), values[::3])


def test_writes_stamp_paired_backend_cloud_and_odometry(tmp_path):
    cloud = tmp_path / 'cloud.pcd'
    data = struct.pack('<4f', 1, 2, 3, 0.5)
    cloud.write_bytes(
        b'VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
        b'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\n'
        b'DATA binary\n' + data)
    bag = tmp_path / 'bag'

    converter.write_bag(
        [(cloud, 1.25)], bag, '/rko_lio/frame', 'velodyne', False,
        estimate_lines=['1 0 0 4 0 1 0 5 0 0 1 6'])

    from rosbags.highlevel import AnyReader
    with AnyReader([bag]) as reader:
        counts = {connection.topic: connection.msgcount for connection in reader.connections}
        odom_connection = next(
            connection for connection in reader.connections
            if connection.topic == '/rko_lio/odometry')
        _, _, raw = next(reader.messages(connections=[odom_connection]))
        odom = reader.deserialize(raw, odom_connection.msgtype)
    assert counts == {'/rko_lio/frame': 1, '/rko_lio/odometry': 1}
    assert odom.header.stamp.sec == 1
    assert odom.header.stamp.nanosec == 250_000_000
    assert (odom.pose.pose.position.x, odom.pose.pose.position.y,
            odom.pose.pose.position.z) == (4.0, 5.0, 6.0)
