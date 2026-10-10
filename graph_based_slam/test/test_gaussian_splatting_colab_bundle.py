# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
"""Tests for the Colab 3DGS bundle packer."""

from __future__ import annotations

import io
import json
from pathlib import Path
import sys
import zipfile

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'


def _load():
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import pack_colab_bundle
    import pointcloud_io
    import posed_images
    import train_gsplat

    return pack_colab_bundle, pointcloud_io, posed_images, train_gsplat


pcb, pcio, pi, tg = _load()


def _posed_dir(tmp_path, count=4):
    Image = pytest.importorskip('PIL.Image')
    intr = pi.CameraIntrinsics(32, 24, 30.0, 30.0, 16.0, 12.0)
    (tmp_path / 'images').mkdir(parents=True)
    frames = []
    for i in range(count):
        Image.fromarray(np.full((24, 32, 3), 40 * i, dtype=np.uint8)).save(
            tmp_path / 'images' / f'{i:05d}.png')
        frames.append(pi.PosedImage(
            f'images/{i:05d}.png',
            pi.make_transform([float(i), 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            float(i)))
    pi.write_transforms(tmp_path / 'transforms.json', intr, frames)
    return tmp_path / 'transforms.json'


def test_thin_init_cloud_drops_grey_and_caps():
    xyz = np.array([[0.0, 0, 0], [0.001, 0, 0], [1.0, 0, 0], [2.0, 0, 0]])
    rgb = np.array([[10, 20, 30], [10, 20, 30], [128, 128, 128], [1, 2, 3]],
                   dtype=np.uint8)
    out_xyz, out_rgb = pcb.thin_init_cloud(xyz, rgb, voxel=0.01, max_points=0)
    assert len(out_xyz) == 2 and [128, 128, 128] not in out_rgb.tolist()
    capped, _ = pcb.thin_init_cloud(xyz, rgb, voxel=0.01, max_points=1)
    assert len(capped) == 1


def test_pack_bundle_round_trips_through_load_transforms(tmp_path):
    transforms = _posed_dir(tmp_path / 'src')
    ply = pcio.write_ply(tmp_path / 'map.ply',
                         np.array([[0.0, 0, 0], [1.0, 0, 0]]),
                         np.array([[200, 10, 10], [128, 128, 128]], dtype=np.uint8))
    summary = pcb.pack_bundle(transforms, tmp_path / 'bundle.zip', init_ply=ply,
                              frame_stride=2)
    assert summary['frames'] == 2 and summary['init_points'] == 1
    with zipfile.ZipFile(tmp_path / 'bundle.zip') as bundle:
        names = set(bundle.namelist())
        assert {'transforms.json', 'init.ply', 'images/00000.jpg',
                'images/00002.jpg'} <= names
        doc = json.loads(bundle.read('transforms.json'))
        assert [f['file_path'] for f in doc['frames']] == [
            'images/00000.jpg', 'images/00002.jpg']
        from PIL import Image
        assert Image.open(io.BytesIO(bundle.read('images/00002.jpg'))).size == (32, 24)
        bundle.extractall(tmp_path / 'unpacked')
    ds = tg.load_transforms(tmp_path / 'unpacked' / 'transforms.json')
    assert len(ds['image_paths']) == 2
    xyz, rgb = pcio.read_ply_xyz(tmp_path / 'unpacked' / 'init.ply')
    np.testing.assert_array_equal(rgb, [[200, 10, 10]])


def test_pack_bundle_validates_options(tmp_path):
    transforms = _posed_dir(tmp_path / 'src', count=1)
    with pytest.raises(ValueError):
        pcb.pack_bundle(transforms, tmp_path / 'b.zip', jpeg_quality=0)
    with pytest.raises(ValueError):
        pcb.pack_bundle(transforms, tmp_path / 'b.zip', frame_stride=0)


def test_pack_bundle_applies_exposure_gains(tmp_path, monkeypatch):
    transforms = _posed_dir(tmp_path / 'src', count=2)
    ply = pcio.write_ply(tmp_path / 'map.ply', np.array([[0.0, 0, 5.0]]),
                         np.array([[50, 50, 50]], dtype=np.uint8))
    monkeypatch.setattr(
        pcb.pcio, 'estimate_overlap_rgb_gains',
        lambda *args, **kwargs: np.array([[1.0, 1.0, 1.0], [0.5, 0.5, 0.5]],
                                         dtype=np.float32))
    summary = pcb.pack_bundle(transforms, tmp_path / 'bundle.zip', init_ply=ply,
                              harmonize_exposure=True)
    assert summary['exposure_gain_min'] == 0.5
    from PIL import Image
    with zipfile.ZipFile(tmp_path / 'bundle.zip') as bundle:
        doc = json.loads(bundle.read('transforms.json'))
        assert doc['frames'][1]['exposure_gain'] == [0.5, 0.5, 0.5]
        second = np.asarray(Image.open(io.BytesIO(bundle.read('images/00001.jpg'))))
    # Frame 1 is a flat 40 grey; halved it is about 20 after JPEG.
    assert abs(float(second.mean()) - 20.0) < 2.0
    with pytest.raises(ValueError):
        pcb.pack_bundle(transforms, tmp_path / 'b.zip', harmonize_exposure=True)
