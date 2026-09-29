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

"""Tests for the numpy-only helpers in colorize_from_bag (no ROS needed)."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'


def _load():
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import colorize_from_bag

    return colorize_from_bag


cfb = _load()


class _Vec3:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _Quat:
    def __init__(self, x, y, z, w):
        self.x, self.y, self.z, self.w = x, y, z, w


# --------------------------------------------------------------------------- #
# nearest_index
# --------------------------------------------------------------------------- #
def test_nearest_index_picks_closest():
    stamps = [100, 200, 300, 400]
    # 260-200=60, 300-260=40 -> index 2 wins.
    assert cfb.nearest_index(stamps, 260) == 2
    assert cfb.nearest_index(stamps, 240) == 1  # 40 vs 60 -> index 1


def test_nearest_index_exact_and_ends():
    stamps = [10, 20, 30]
    assert cfb.nearest_index(stamps, 20) == 1
    assert cfb.nearest_index(stamps, -5) == 0
    assert cfb.nearest_index(stamps, 999) == 2


def test_nearest_index_empty_raises():
    import pytest
    with pytest.raises(ValueError):
        cfb.nearest_index([], 5)


def test_select_synced_time_reselects_cloud_nearest_slower_image():
    clouds = [0, 100, 200, 300, 400]
    images = [40, 340]
    # frac=.4 initially picks cloud 200 -> image 340, then cloud 300 is closer.
    assert cfb.select_synced_time(clouds, images, 0.4) == (300, 340)


def test_select_synced_time_searches_neighbour_images_for_smallest_offset():
    clouds = [0, 100, 200, 300, 400]
    images = [40, 170, 305]
    # The closest image to target cloud 200 is 170 (30 ms), but neighbour 305
    # has a 5 ms pairing with cloud 300 and therefore gives sharper colour.
    assert cfb.select_synced_time(clouds, images, 0.4, search_radius=1) == (300, 305)


def test_select_synced_time_clamps_fraction_and_rejects_empty():
    import pytest
    assert cfb.select_synced_time([100, 200], [180], -1) == (200, 180)
    assert cfb.select_synced_time([100, 200], [180], 2) == (200, 180)
    with pytest.raises(ValueError, match='point-cloud'):
        cfb.select_synced_time([], [1], 0.5)
    with pytest.raises(ValueError, match='image'):
        cfb.select_synced_time([1], [], 0.5)


# --------------------------------------------------------------------------- #
# transform_msg_to_matrix
# --------------------------------------------------------------------------- #
def test_transform_identity():
    T = cfb.transform_msg_to_matrix(_Vec3(0, 0, 0), _Quat(0, 0, 0, 1))
    np.testing.assert_allclose(T, np.eye(4), atol=1e-9)


def test_transform_translation_only():
    T = cfb.transform_msg_to_matrix(_Vec3(1, 2, 3), _Quat(0, 0, 0, 1))
    np.testing.assert_allclose(T[:3, 3], [1, 2, 3], atol=1e-9)
    np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-9)


# --------------------------------------------------------------------------- #
# manual extrinsic input (bags without TF)
# --------------------------------------------------------------------------- #
def test_extrinsic_matrix_from_cli_values_normalizes_quaternion():
    T = cfb.extrinsic_matrix([1, 2, 3, 0, 0, 0, 2])
    np.testing.assert_allclose(T, np.array([
        [1, 0, 0, 1], [0, 1, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1],
    ]), atol=1e-9)


def test_extrinsic_matrix_from_json_object(tmp_path):
    path = tmp_path / 'extrinsic.json'
    path.write_text(
        '{"translation": [4, 5, 6], "rotation_xyzw": [0, 0, 0, 1]}')
    T = cfb.extrinsic_matrix(path=path)
    np.testing.assert_allclose(T[:3, 3], [4, 5, 6], atol=1e-9)
    np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-9)


def test_extrinsic_matrix_from_json_list(tmp_path):
    path = tmp_path / 'extrinsic.json'
    path.write_text('[1, 2, 3, 0, 0, 0, 1]')
    np.testing.assert_allclose(
        cfb.extrinsic_matrix(path=path),
        cfb.extrinsic_matrix([1, 2, 3, 0, 0, 0, 1]))


def test_extrinsic_matrix_inverts_official_vlcal_result(tmp_path):
    path = tmp_path / 'calib.json'
    path.write_text(
        '{"results": {"T_lidar_camera": [1, 2, 3, 0, 0, 0, 1]}}')
    T = cfb.extrinsic_matrix(path=path)
    np.testing.assert_allclose(T[:3, 3], [-1, -2, -3], atol=1e-9)
    np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-9)


def test_extrinsic_matrix_none_keeps_tf_mode():
    assert cfb.extrinsic_matrix() is None


def test_extrinsic_matrix_rejects_bad_inputs(tmp_path):
    import pytest
    with pytest.raises(ValueError, match='7 values'):
        cfb.extrinsic_matrix([1, 2, 3])
    with pytest.raises(ValueError, match='non-zero'):
        cfb.extrinsic_matrix([0, 0, 0, 0, 0, 0, 0])
    path = tmp_path / 'extrinsic.json'
    path.write_text('{"translation": [1, 2, 3]}')
    with pytest.raises(ValueError, match='rotation_xyzw'):
        cfb.extrinsic_matrix(path=path)


# --------------------------------------------------------------------------- #
# diagnostic projection overlay geometry
# --------------------------------------------------------------------------- #
def test_projection_diagnostics_marks_occluded_point():
    K = np.array([[100, 0, 50], [0, 100, 50], [0, 0, 1]], dtype=float)
    points = np.array([
        [0, 0, 2],       # nearest surface at the principal point
        [0, 0, 8],       # same pixel, hidden behind the near surface
        [100, 0, 1],     # outside the image
    ], dtype=float)
    result = cfb.projection_diagnostics(
        points, np.eye(4), K, 100, 100, zbuf_bin=4, depth_tol=0.15)
    assert result['indices'].tolist() == [0, 1]
    np.testing.assert_allclose(result['u'], [50, 50])
    np.testing.assert_allclose(result['v'], [50, 50])
    assert result['visible'].tolist() == [True, False]


def test_projection_diagnostics_handles_no_in_frame_points():
    K = np.eye(3)
    result = cfb.projection_diagnostics(
        np.array([[0, 0, -1]], dtype=float), np.eye(4), K, 10, 10)
    assert result['indices'].size == 0
    assert result['visible'].size == 0


def test_projection_diagnostics_rejects_bad_zbuffer_bin():
    import pytest
    with pytest.raises(ValueError, match='zbuf_bin'):
        cfb.projection_diagnostics(
            np.zeros((1, 3)), np.eye(4), np.eye(3), 10, 10, zbuf_bin=0)


# --------------------------------------------------------------------------- #
# merge_colorings
# --------------------------------------------------------------------------- #
def test_merge_single_camera_passthrough():
    rgb = np.array([[10, 20, 30], [40, 50, 60]], dtype=np.uint8)
    seen = np.array([True, False])
    counts = np.array([1, 0], dtype=np.uint16)
    out, out_seen = cfb.merge_colorings([(rgb, seen, counts)], default_rgb=(7, 7, 7))
    np.testing.assert_array_equal(out[0], [10, 20, 30])
    np.testing.assert_array_equal(out[1], [7, 7, 7])  # unseen -> default
    assert out_seen.tolist() == [True, False]


def test_merge_count_weighted_blend():
    # Point 0: cam A (count 3) says red, cam B (count 1) says blue -> mostly red.
    a = (np.array([[200, 0, 0]], dtype=np.uint8), np.array([True]),
         np.array([3], dtype=np.uint16))
    b = (np.array([[0, 0, 200]], dtype=np.uint8), np.array([True]),
         np.array([1], dtype=np.uint16))
    out, seen = cfb.merge_colorings([a, b])
    assert seen[0]
    # (3*200 + 1*0)/4 = 150 red ; (3*0 + 1*200)/4 = 50 blue
    np.testing.assert_array_equal(out[0], [150, 0, 50])


def test_merge_union_of_coverage():
    # Cam A sees only point 0, cam B only point 1 -> both coloured after merge.
    a = (np.array([[100, 0, 0], [0, 0, 0]], dtype=np.uint8),
         np.array([True, False]), np.array([1, 0], dtype=np.uint16))
    b = (np.array([[0, 0, 0], [0, 100, 0]], dtype=np.uint8),
         np.array([False, True]), np.array([0, 1], dtype=np.uint16))
    out, seen = cfb.merge_colorings([a, b], default_rgb=(9, 9, 9))
    assert seen.tolist() == [True, True]
    np.testing.assert_array_equal(out[0], [100, 0, 0])
    np.testing.assert_array_equal(out[1], [0, 100, 0])


def test_merge_empty_raises():
    import pytest
    with pytest.raises(ValueError):
        cfb.merge_colorings([])


def test_colour_statistics_distinguishes_rgb_from_repeated_luminance():
    rgb = np.array([[100, 100, 100], [200, 50, 20], [0, 100, 20]], np.uint8)
    stats = cfb.colour_statistics(rgb, np.array([True, True, False]))
    assert stats['mean_channel_range'] == 90.0
    assert stats['chromatic_fraction_10'] == 0.5
    assert stats['unique_colours'] == 2


def test_colour_statistics_handles_no_visible_points():
    stats = cfb.colour_statistics(
        np.zeros((1, 3), np.uint8), np.array([False]))
    assert stats['chromatic_fraction_10'] == 0.0
    assert stats['unique_colours'] == 0


def test_projection_quality_cli_defaults_are_frozen():
    args = cfb.build_parser().parse_args(['bag', 'output'])

    assert args.zbuf_bin == 1
    assert args.depth_tol == 0.15
    assert args.interp == 'edge-aware'
    assert args.edge_threshold == 48.0


def test_transform_optical_rotation_is_a_valid_axis_permutation():
    # The standard camera_link<->optical quaternion (0.5,-0.5,0.5,-0.5) must
    # produce a proper rotation (orthonormal, det +1) that maps each unit axis
    # onto a signed unit axis. Lock in the concrete mapping as a regression.
    T = cfb.transform_msg_to_matrix(_Vec3(0, 0, 0), _Quat(0.5, -0.5, 0.5, -0.5))
    R = T[:3, :3]
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert abs(np.linalg.det(R) - 1.0) < 1e-9
    np.testing.assert_allclose(R @ np.array([1.0, 0.0, 0.0]), [0, -1, 0], atol=1e-9)
    np.testing.assert_allclose(R @ np.array([0.0, 0.0, 1.0]), [1, 0, 0], atol=1e-9)


@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8'])
@pytest.mark.parametrize('padding', [0, 2])
def test_image_to_rgb_respects_row_stride(encoding, padding):
    """Padding bytes must never become image samples in direct bag coloring."""
    from types import SimpleNamespace

    rgb = np.array([[[10, 20, 30], [40, 50, 60]],
                    [[70, 80, 90], [100, 110, 120]]], dtype=np.uint8)
    pixels = rgb[:, :, ::-1] if encoding.startswith('bgr') else rgb
    if encoding in ('rgba8', 'bgra8'):
        pixels = np.concatenate([pixels, np.full((2, 2, 1), 255, dtype=np.uint8)], axis=2)
    elif encoding == 'mono8':
        pixels = rgb[:, :, :1]
        rgb = np.repeat(pixels, 3, axis=2)
    step = 2 * pixels.shape[2] + padding
    data = b''.join(row.tobytes() + bytes([213]) * padding for row in pixels)
    message = SimpleNamespace(encoding=encoding, height=2, width=2, step=step, data=data)
    actual = cfb._image_to_rgb(message, np.eye(3), np.zeros(5), False)
    np.testing.assert_array_equal(actual, rgb)
    assert actual.flags.c_contiguous


@pytest.mark.parametrize('model,coefficients,undistort', [
    ('plumb_bob', [0.1, -0.02, 0.003, -0.001, 0.01], True),
    ('rational_polynomial', [0.1, -0.02, 0.003, -0.001, 0.01, 0.15, 0.02, 0.01], True),
    ('equidistant', [0.1, -0.02, 0.003, -0.001], True),
    ('equidistant', [0.0, 0.0, 0.0, 0.0], True),
    ('equidistant', [0.1, -0.02, 0.003, -0.001], False),
])
def test_direct_coloring_uses_camera_info_model(
        tmp_path, monkeypatch, model, coefficients, undistort):
    """The image passed to pinhole projection must be rectified to the same K."""
    cv2 = pytest.importorskip('cv2')
    pytest.importorskip('rclpy.time')
    from types import SimpleNamespace

    width, height = 64, 48
    y, x = np.indices((height, width))
    rgb = np.stack([x * 3, y * 5, (x + y) * 2], axis=-1).astype(np.uint8)
    k = np.array([[32.0, 0.0, 31.5], [0.0, 33.0, 23.5], [0.0, 0.0, 1.0]])
    image = SimpleNamespace(encoding='rgb8', height=height, width=width,
                            step=width * 3, data=rgb.tobytes())
    info = SimpleNamespace(k=k.ravel(), d=coefficients, width=width, height=height,
                           distortion_model=model)
    args = cfb.build_parser().parse_args([
        'bag', str(tmp_path / 'colored'), '--extrinsic', '0', '0', '0', '0', '0', '0', '1'])
    args.no_undistort = not undistort
    monkeypatch.setattr(cfb, '_collect', lambda *a, **kw: (
        None, {args.camera_info_topic: info}, [1], {args.image_topic: [1]}, {}))
    monkeypatch.setattr(cfb, '_grab_messages', lambda *a: {
        (args.pc_topic, 1): None, (args.image_topic, 1): image})
    monkeypatch.setattr(cfb, '_read_xyz', lambda _msg: np.array([[0.0, 0.0, 2.0]]))
    d = np.asarray(coefficients)
    expected = rgb
    if undistort:
        if model == 'equidistant':
            expected = cv2.fisheye.undistortImage(rgb, k, d, Knew=k)
        else:
            expected = cv2.undistort(rgb, k, d)
    calls = []

    def project(points, poses, intrinsics, images, w, h, **kwargs):
        np.testing.assert_array_equal(intrinsics, k)
        np.testing.assert_array_equal(images[0], expected)
        assert (w, h) == (width, height)
        calls.append(True)
        return np.array([[10, 20, 30]], dtype=np.uint8), np.array([True]), np.array([1])

    monkeypatch.setattr(cfb.pcio, 'colorize_by_projection_robust', project)
    assert cfb.colorize_bag_frame(args)['colored'] == 1
    assert calls == [True]


@pytest.mark.parametrize('model,d', [
    ('equidistant', [0., 0., 0., 0.]),
    ('equidistant', [0.1, -0.02, 0.003, -0.001]),
    ('plumb_bob', [0.2, -0.02, 0.003, -0.001, 0.01]),
    ('rational_polynomial', [0.1, -0.02, 0.003, -0.001, 0.01, 0.15, 0.02, 0.01]),
])
@pytest.mark.parametrize('cropped', [False, True])
def test_raw_image_coloring_projects_camera_model(tmp_path, monkeypatch, model, d, cropped):
    """Actual sampling and occlusion must use distorted pixels, as must overlays."""
    cv2 = pytest.importorskip('cv2')
    pytest.importorskip('rclpy.time')
    from types import SimpleNamespace

    width, height = 640, 480
    k = np.array([[200., 0., 320.], [0., 200., 240.], [0., 0., 1.]])
    xyz = np.array([[1., 0., 1.], [2., 0., 2.], [0., 0., -1.]])
    if model == 'equidistant':
        uv = cv2.fisheye.projectPoints(xyz[:1, None], np.zeros(3), np.zeros(3),
                                       k, np.array(d))[0].reshape(2)
    else:
        uv = cv2.projectPoints(xyz[:1, None], np.zeros(3), np.zeros(3),
                               k, np.array(d))[0].reshape(2)
    if cropped:
        uv = (uv - [160, 120]) / 2
        width, height = 240, 180
    u, v = np.round(uv).astype(int)
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    rgb[v, u] = [211, 71, 33]
    info = SimpleNamespace(k=k.ravel(), d=d, width=640, height=480,
                           distortion_model=model)
    if cropped:
        info.binning_x = info.binning_y = 2
        info.roi = SimpleNamespace(x_offset=160, y_offset=120, width=480, height=360)
    image = SimpleNamespace(encoding='rgb8', width=width, height=height,
                            step=width * 3, data=rgb.tobytes())
    args = cfb.build_parser().parse_args([
        'bag', str(tmp_path / 'raw'), '--no-undistort', '--interp', 'nearest',
        '--extrinsic', '0', '0', '0', '0', '0', '0', '1'])
    args.normalize_exposure = False
    args.diagnostic_overlay = str(tmp_path / 'overlay.png')
    monkeypatch.setattr(cfb, '_collect', lambda *a, **kw: (
        None, {args.camera_info_topic: info}, [1], {args.image_topic: [1]}, {}))
    monkeypatch.setattr(cfb, '_grab_messages', lambda *a: {
        (args.pc_topic, 1): None, (args.image_topic, 1): image})
    monkeypatch.setattr(cfb, '_read_xyz', lambda _: xyz)
    written = []

    def write(path, points, colors):
        written.append((points.copy(), colors.copy()))
        return path
    monkeypatch.setattr(cfb.pcio, 'write_ply', write)
    overlays = []

    def overlay(path, image, diagnostics, **kwargs):
        overlays.append(diagnostics)
        return path
    monkeypatch.setattr(cfb, '_write_diagnostic_overlay', overlay)
    result = cfb.colorize_bag_frame(args)
    assert result['colored'] == 1
    np.testing.assert_array_equal(written[0][0], xyz)
    np.testing.assert_array_equal(written[0][1][0], [211, 71, 33])
    np.testing.assert_array_equal(written[0][1][1:], [args.default_rgb] * 2)
    np.testing.assert_allclose([overlays[0]['u'][0], overlays[0]['v'][0]], uv)
    assert overlays[0]['visible'].tolist() == [True, False]


@pytest.mark.parametrize('option', ['overlap_color_balance',
                                    'calibration_sigma_multiplier',
                                    'min_projected_scale', 'view_score_power'])
def test_raw_projection_rejects_pinhole_only_weighting(option):
    """Do not combine raw pixel coordinates with pinhole-only quality helpers."""
    with pytest.raises(ValueError, match='rectify images'):
        cfb.pcio.colorize_by_projection_robust(
            np.array([[0., 0., 1.]]), [np.eye(4)], np.eye(3),
            [np.zeros((2, 2, 3), dtype=np.uint8)], 2, 2,
            distortion=np.zeros(4), distortion_model='equidistant', **{option: 1})


@pytest.mark.parametrize('model,d,x', [
    ('plumb_bob', [-1., 0., 0., 0., 0.], 1.),
    ('rational_polynomial', [-1., 0., 0., 0., 0., 0., 0., 0.], 1.),
    ('equidistant', [-1., 0., 0., 0.], np.tan(1.)),
])
def test_folded_lens_ray_cannot_color_or_occlude(model, d, x):
    """An outer ray folding to the optical center must not hide a valid ray."""
    pytest.importorskip('cv2')
    k = np.array([[20., 0., 32.], [0., 20., 24.], [0., 0., 1.]])
    xyz = np.array([[x, 0., 1.], [0., 0., 2.], [np.nan, 0., 1.]])
    rgb = np.zeros((48, 64, 3), dtype=np.uint8)
    rgb[24, 32] = [210, 90, 40]
    colors, seen = cfb.pcio.colorize_by_projection_robust(
        xyz, [np.eye(4)], k, [rgb], 64, 48, distortion=d,
        distortion_model=model, normalize_exposure=False, interp='nearest')
    assert seen.tolist() == [False, True, False]
    np.testing.assert_array_equal(colors, [[128, 128, 128], [210, 90, 40],
                                           [128, 128, 128]])
    diag = cfb.projection_diagnostics(xyz, np.eye(4), k, 64, 48,
                                      distortion=d, distortion_model=model)
    assert diag['indices'].tolist() == [1]
    assert diag['visible'].tolist() == [True]


@pytest.mark.parametrize('phase', ['collect', 'grab'])
def test_bag_pairing_uses_sensor_stamps_despite_recording_delay(monkeypatch, phase):
    """A delayed cloud must match acquisition time, not a nearby receipt."""
    pytest.importorskip('rclpy.serialization')
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from types import SimpleNamespace
    import extract_posed_images

    cloud = PointCloud2()
    cloud.header.stamp.nanosec = 100
    image = Image()
    image.header.stamp.nanosec = 105
    late_image = Image()
    late_image.header.stamp.nanosec = 410
    # Duplicate header timestamps retain the first matching message.
    duplicate = Image()
    duplicate.header.stamp.nanosec = 105
    duplicate.header.frame_id = 'later_duplicate'
    records = [('/info', serialize_message(CameraInfo()), 1),
               ('/image', serialize_message(image), 150),
               ('/cloud', serialize_message(cloud), 400),
               ('/ignored', b'not a ROS message', 401),
               ('/image', serialize_message(late_image), 415),
               ('/image', serialize_message(duplicate), 420)]
    types = {'/info': 'sensor_msgs/msg/CameraInfo',
             '/image': 'sensor_msgs/msg/Image',
             '/cloud': 'sensor_msgs/msg/PointCloud2'}

    class Reader:
        def __init__(self):
            self.records = list(records)

        def get_all_topics_and_types(self):
            return [SimpleNamespace(name=k, type=v) for k, v in types.items()]

        def has_next(self):
            return bool(self.records)

        def read_next(self):
            return self.records.pop(0)

    monkeypatch.setattr(extract_posed_images, '_open_reader', lambda _: Reader())
    if phase == 'collect':
        _, _, clouds, images, _ = cfb._collect(
            'bag', '/cloud', [('/image', '/info', 'camera')], need_tf=False)
        assert clouds == [100]
        assert images['/image'] == [105, 105, 410]
        assert cfb.select_synced_time(clouds, images['/image'], .5) == (100, 105)
    else:
        messages = cfb._grab_messages('bag', {('/cloud', 100), ('/image', 105)}, types)
        assert messages[('/cloud', 100)].header.stamp.nanosec == 100
        assert messages[('/image', 105)].header.frame_id == ''


@pytest.mark.parametrize('raw', [False, True])
@pytest.mark.parametrize('roi,bins,size,pixel', [
    ((0, 0, 0, 0), (0, 0), (64, 48), (40, 28)),
    ((16, 8, 32, 32), (1, 1), (32, 32), (24, 20)),
    ((0, 0, 0, 0), (2, 2), (32, 24), (20, 14)),
    ((16, 8, 32, 32), (2, 2), (16, 16), (12, 10)),
    ((16, 8, 32, 32), (2, 1), (16, 32), (12, 20)),
    ((40, 8, 32, 32), (1, 1), (32, 32), None),
    ((16, 8, 0, 32), (1, 1), (32, 32), None),
    ((0, 0, 0, 0), (2, 2), (64, 48), None),
    ((0, 0, 0, 0), (128, 128), (0, 0), None),
])
def test_direct_roi_binning_samples_known_pixel(tmp_path, monkeypatch, raw, roi, bins, size, pixel):
    """Cropping/binning must preserve a known ray's color and occlusion."""
    pytest.importorskip('cv2')
    pytest.importorskip('rclpy.time')
    from types import SimpleNamespace

    w, h = size
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    if pixel is not None:
        rgb[pixel[1], pixel[0]] = [211, 71, 33]
    image = SimpleNamespace(encoding='rgb8', width=w, height=h, step=w * 3, data=rgb.tobytes())
    info = SimpleNamespace(
        k=[40., 0., 32., 0., 40., 24., 0., 0., 1.], d=[0.] * 5,
        width=64, height=48, distortion_model='plumb_bob', binning_x=bins[0], binning_y=bins[1],
        roi=SimpleNamespace(x_offset=roi[0], y_offset=roi[1], width=roi[2], height=roi[3]))
    args = cfb.build_parser().parse_args([
        'bag', str(tmp_path / 'roi'), '--interp', 'nearest',
        '--extrinsic', '0', '0', '0', '0', '0', '0', '1'])
    args.no_undistort = raw
    xyz = np.array([[.4, .2, 2.], [.8, .4, 4.]])
    monkeypatch.setattr(cfb, '_collect', lambda *a, **kw: (
        None, {args.camera_info_topic: info}, [1], {args.image_topic: [1]}, {}))
    monkeypatch.setattr(cfb, '_grab_messages', lambda *a: {
        (args.pc_topic, 1): None, (args.image_topic, 1): image})
    monkeypatch.setattr(cfb, '_read_xyz', lambda _: xyz)
    if pixel is None:
        with pytest.raises(ValueError, match='CameraInfo ROI/binning'):
            cfb.colorize_bag_frame(args)
        return
    result = cfb.colorize_bag_frame(args)
    points, colors = cfb.pcio.read_ply_xyz(result['full_ply'])
    assert result['colored'] == 1
    np.testing.assert_allclose(points, xyz)
    np.testing.assert_array_equal(colors, [[211, 71, 33], args.default_rgb])
