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

"""Tests for public MID-360 dataset intake helpers."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
DOWNLOAD_SCRIPT = SCRIPT_DIR / 'download_mid360_robot_public_dataset.py'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_public_datasets import (  # noqa: E402
    get_public_dataset,
    public_dataset_registry,
    PublicDataset,
    PublicDatasetFile,
    PublicDatasetIntake,
    PublicDatasetIntakeOptions,
)


def test_public_dataset_registry_contains_recommended_mid360_sources():
    datasets = {dataset.id: dataset for dataset in public_dataset_registry()}

    assert 'driving_slam_mid360' in datasets
    assert 'hard_pointcloud_mid360_outdoor_kidnap_a' in datasets
    assert datasets['driving_slam_mid360'].file_by_id().md5 == (
        '0836c50859bb1af591966b69da166186'
    )
    assert datasets['hard_pointcloud_mid360_outdoor_kidnap_a'].profile[
        'expected_pointcloud_topic'
    ] == '/livox/points'
    hard_files = {
        file_record.id
        for file_record in datasets['hard_pointcloud_mid360_outdoor_kidnap_a'].files
    }
    assert 'outdoor_kidnap_b' in hard_files
    assert 'outdoor_hard_01a' in hard_files


def test_get_public_dataset_rejects_unknown_id():
    try:
        get_public_dataset('missing_dataset')
    except ValueError as exc:
        assert 'unknown public MID-360 dataset' in str(exc)
    else:
        raise AssertionError('missing dataset id should raise ValueError')


def test_public_dataset_dry_run_writes_plan_without_archive(tmp_path: Path):
    intake = PublicDatasetIntake(REPO_ROOT)
    report = intake.run(
        PublicDatasetIntakeOptions(
            dataset_id='driving_slam_mid360',
            dataset_root=tmp_path / 'datasets',
            dry_run=True,
        )
    )

    assert report['status'] == 'DRY_RUN'
    assert 'download_mid360_robot_public_dataset.py' in report['commands']['download']
    assert report['commands']['recording_check'] == ''
    assert Path(report['manifest_json']).is_file()
    assert not Path(report['archive_path']).exists()


def test_public_dataset_intake_extracts_local_zip_and_finds_bag(tmp_path: Path):
    archive_src = tmp_path / 'source.zip'
    with zipfile.ZipFile(archive_src, 'w') as archive:
        archive.writestr(
            'bag/metadata.yaml',
            '\n'.join([
                'rosbag2_bagfile_information:',
                '  duration:',
                '    nanoseconds: 1000000000',
                '  message_count: 0',
                '  topics_with_message_count: []',
            ]),
        )
    md5 = hashlib.md5(archive_src.read_bytes()).hexdigest()
    dataset = PublicDataset(
        id='local_public_mid360',
        title='Local public MID-360 fixture',
        source_url='file://fixture',
        description='local fixture',
        license='test',
        citation='test',
        files=(
            PublicDatasetFile(
                id='fixture',
                filename='fixture.zip',
                url=archive_src.as_uri(),
                md5=md5,
                size_label='small',
            ),
        ),
        default_file_id='fixture',
        profile={
            'robot_name': 'local_public_mid360',
            'base_frame': 'base_link',
            'lidar_frame': 'livox_frame',
            'imu_frame': 'livox_frame',
            'expected_pointcloud_topic': '',
            'expected_imu_topic': '/livox/imu',
        },
    )

    report = PublicDatasetIntake(
        REPO_ROOT,
        registry={'local_public_mid360': dataset},
    ).run(
        PublicDatasetIntakeOptions(
            dataset_id='local_public_mid360',
            dataset_root=tmp_path / 'datasets',
        )
    )

    assert report['status'] == 'READY'
    assert Path(report['archive_path']).is_file()
    assert Path(report['profile_path']).is_file()
    assert Path(report['selected_bag_path']).name == 'bag'
    assert 'check_mid360_robot_recording.sh' in report['commands']['recording_check']

    manifest = json.loads(Path(report['manifest_json']).read_text(encoding='utf-8'))
    assert manifest['selected_bag_path'] == report['selected_bag_path']


def test_public_dataset_cli_list_json():
    result = subprocess.run(
        [sys.executable, str(DOWNLOAD_SCRIPT), '--list', '--json'],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    payload = json.loads(result.stdout)

    ids = {dataset['id'] for dataset in payload['datasets']}
    assert 'driving_slam_mid360' in ids
    assert 'hard_pointcloud_mid360_outdoor_kidnap_a' in ids


def test_public_dataset_cli_dry_run(tmp_path: Path):
    result = subprocess.run(
        [
            sys.executable,
            str(DOWNLOAD_SCRIPT),
            '--dataset',
            'driving_slam_mid360',
            '--dataset-root',
            str(tmp_path / 'datasets'),
            '--dry-run',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert report['status'] == 'DRY_RUN'
    assert report['dataset']['id'] == 'driving_slam_mid360'
    assert Path(report['manifest_json']).is_file()
