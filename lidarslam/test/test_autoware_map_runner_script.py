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

"""Regression tests for the one-shot Autoware map runner script."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess

import jsonschema
import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_autoware_map_from_bag.py'
BEGINNER_SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_autoware_map_beginner.sh'


def _load_module():
    spec = importlib.util.spec_from_file_location('run_autoware_map_from_bag', SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_metadata(tmp_path: Path, bag_name: str, topics: list[tuple[str, str, int]]) -> Path:
    bag_dir = tmp_path / bag_name
    bag_dir.mkdir()
    storage_name = f'{bag_name}_0.db3'
    metadata = {
        'rosbag2_bagfile_information': {
            'duration': {'nanoseconds': 2_000_000_000},
            'message_count': sum(count for _, _, count in topics),
            'storage_identifier': 'sqlite3',
            'relative_file_paths': [storage_name],
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': name,
                        'type': msg_type,
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': '',
                    },
                    'message_count': count,
                }
                for name, msg_type, count in topics
            ],
        },
    }
    (bag_dir / storage_name).write_bytes(b'rosbag2 fixture\n')
    (bag_dir / 'metadata.yaml').write_text(yaml.safe_dump(metadata), encoding='utf-8')
    return bag_dir


def test_runner_script_supports_profiles_and_viewers():
    script = SCRIPT_PATH.read_text(encoding='utf-8')

    assert 'preflight_autoware_map_bag.py' in script
    assert 'diagnose_autoware_map_run.py' in script
    assert 'rko_lio_graph_public_path' in script
    assert 'rko_lio_graph_mid360_preset' in script
    assert 'pointcloud_gnss_smoke' in script
    assert 'packet_applanix_smoke' in script
    assert '--viewer' in script
    assert 'verify_autoware_map.py' in script
    assert 'Next steps:' in script
    assert 'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh' in script
    assert 'run_graph_slam_pointcloud_map_in_autoware.sh' in script
    assert '--dry-run' in script
    assert '--resume' in script
    assert 'run_manifest.json' in script
    assert 'artifact_checksums' in script


def test_runner_help_is_user_facing():
    result = subprocess.run(
        ['python3', str(SCRIPT_PATH), '--help'],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert 'Autoware-compatible map workflow' in result.stdout
    assert 'The input must be the rosbag2 directory' in result.stdout
    assert 'Workflow profiles:' in result.stdout
    assert 'Expected successful outputs:' in result.stdout
    assert '--output-dir <dir>' in result.stdout
    assert '--resume' in result.stdout


def test_runner_rejects_db3_file_without_traceback(tmp_path: Path):
    db3_path = tmp_path / 'demo_0.db3'
    db3_path.write_text('', encoding='utf-8')

    result = subprocess.run(
        ['python3', str(SCRIPT_PATH), str(db3_path), '--dry-run'],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error:' in result.stderr
    assert 'not the .db3 file' in result.stderr
    assert 'Traceback' not in result.stderr


def test_runner_rejects_output_file_without_traceback(tmp_path: Path):
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_file = tmp_path / 'map_output'
    output_file.write_text('', encoding='utf-8')

    result = subprocess.run(
        [
            'python3',
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_file),
            '--dry-run',
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'output directory path is a file' in result.stderr
    assert 'Traceback' not in result.stderr


def test_manifest_helpers_capture_identity_and_finalize_atomically(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    final_dir = tmp_path / 'map_output'
    working_dir = tmp_path / 'map_output.partial'
    working_dir.mkdir()
    plan = {
        'profile_id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO + graph_based_slam public path',
        'command': ['python3', '-c', 'pass'],
    }

    manifest = module._build_manifest(bag_dir, final_dir, working_dir, plan)
    module._write_manifest(working_dir, manifest)
    (working_dir / 'artifact.txt').write_text('map artifact\n', encoding='utf-8')
    manifest['output']['artifact_checksums'] = module._artifact_checksums(working_dir)
    module._write_manifest(working_dir, manifest)
    module._finalize_output(working_dir, final_dir)

    saved = json.loads((final_dir / 'run_manifest.json').read_text(encoding='utf-8'))
    schema = json.loads(
        (
            REPO_ROOT / 'docs' / 'schemas' / 'run-manifest-v2.schema.json'
        ).read_text(encoding='utf-8')
    )
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.validate(saved, schema)
    assert saved['schema_version'] == 2
    assert saved['lifecycle'] == {
        'last_error': None,
        'resume_count': 0,
        'runner_exit_code': None,
        'stage': 'initialized',
        'verification_enabled': True,
    }
    assert saved['input']['metadata_sha256'] == module._sha256(
        bag_dir / 'metadata.yaml'
    )
    assert saved['input']['storage_identifier'] == 'sqlite3'
    assert saved['input']['storage_files'] == [{
        'path': 'demo_bag_0.db3',
        'sha256': module._sha256(bag_dir / 'demo_bag_0.db3'),
        'size_bytes': 16,
    }]
    assert saved['software']['product_version'] == '0.6.0'
    assert saved['software']['package_versions']['lidarslam'] == '0.6.0'
    assert isinstance(saved['software']['git_dirty'], bool)
    assert saved['profile']['id'] == 'rko_lio_graph_public_path'
    assert saved['output']['artifact_checksums'][0]['path'] == 'artifact.txt'
    assert not working_dir.exists()


def test_manifest_rejects_storage_path_outside_bag(tmp_path: Path):
    module = _load_module()
    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir()
    (tmp_path / 'outside.db3').write_bytes(b'outside\n')
    metadata = {
        'rosbag2_bagfile_information': {
            'storage_identifier': 'sqlite3',
            'relative_file_paths': ['../outside.db3'],
            'topics_with_message_count': [],
        },
    }
    (bag_dir / 'metadata.yaml').write_text(
        yaml.safe_dump(metadata),
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match='outside the bag'):
        module._bag_identity(bag_dir)


def test_postprocessing_lock_rejects_concurrent_owner(tmp_path: Path):
    module = _load_module()
    output_dir = tmp_path / 'map_output'

    with module._postprocess_lock(output_dir):
        with pytest.raises(RuntimeError, match='post-processing is already active'):
            with module._postprocess_lock(output_dir):
                pytest.fail('a second post-processing owner acquired the lock')


def test_main_writes_success_manifest_and_rejects_collision(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'map_output'

    def fake_plan(bag_path, profile_id, output_dir, verify_map):
        del bag_path, profile_id, verify_map
        working_dir = output_dir
        command_script = '\n'.join([
            'from pathlib import Path',
            f'root = Path({str(working_dir)!r})',
            "(root / 'pointcloud_map').mkdir(parents=True, exist_ok=True)",
            (
                "(root / 'pointcloud_map' / 'pointcloud_map_metadata.yaml')"
                ".write_text('tiles: []\\n')"
            ),
            "(root / 'map_projector_info.yaml').write_text('projector_type: Local\\n')",
        ])
        return {
            'payload': {},
            'profile_id': 'rko_lio_graph_public_path',
            'label': 'RKO-LIO + graph_based_slam public path',
            'command': ['python3', '-c', command_script],
            'output_dir': working_dir,
        }

    def fake_verify(run_dir: Path, enabled: bool):
        assert enabled is True
        (run_dir / 'verify_autoware_map.log').write_text(
            'RESULT: PASS\nPASS: 1 | WARN: 0 | FAIL: 0\n',
            encoding='utf-8',
        )

    monkeypatch.setattr(module, 'build_execution_plan', fake_plan)
    monkeypatch.setattr(module, 'maybe_verify_map', fake_verify)
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
        ],
    )

    assert module.main() == 0
    assert output_dir.is_dir()
    assert not output_dir.with_name('map_output.partial').exists()
    manifest = json.loads(
        (output_dir / 'run_manifest.json').read_text(encoding='utf-8')
    )
    schema = json.loads(
        (
            REPO_ROOT / 'docs' / 'schemas' / 'run-manifest-v2.schema.json'
        ).read_text(encoding='utf-8')
    )
    jsonschema.validate(manifest, schema)
    assert manifest['status'] == 'succeeded'
    assert manifest['execution']['exit_code'] == 0
    assert manifest['lifecycle']['stage'] == 'complete'
    assert manifest['lifecycle']['runner_exit_code'] == 0
    assert manifest['output']['finalized'] is True
    assert manifest['output']['diagnosis_status'] == 'success'
    assert any(
        item['path'] == 'autoware_map_diagnosis.json'
        for item in manifest['output']['artifact_checksums']
    )

    assert module.main() == 2


@pytest.mark.parametrize(
    ('workflow_error', 'expected_status', 'expected_exit_code'),
    [
        (subprocess.CompletedProcess([], 17), 'failed', 17),
        (KeyboardInterrupt(), 'interrupted', 130),
    ],
)
def test_main_retains_terminal_manifest_for_failed_and_interrupted_runs(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    workflow_error,
    expected_status: str,
    expected_exit_code: int,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'failed_map'
    plan = {
        'payload': {},
        'profile_id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO + graph_based_slam public path',
        'command': ['map-workflow'],
        'output_dir': output_dir.with_name('failed_map.partial'),
    }
    monkeypatch.setattr(module, 'build_execution_plan', lambda **kwargs: plan)
    monkeypatch.setattr(module, '_git_state', lambda: {
        'commit': '0' * 40,
        'dirty': False,
    })
    monkeypatch.setattr(module, 'maybe_verify_map', lambda *args, **kwargs: None)

    if isinstance(workflow_error, BaseException):
        def fake_run(*args, **kwargs):
            raise workflow_error
    else:
        def fake_run(*args, **kwargs):
            return workflow_error
    monkeypatch.setattr(module.subprocess, 'run', fake_run)
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
        ],
    )

    assert module.main() == expected_exit_code
    manifest = json.loads(
        (output_dir / 'run_manifest.json').read_text(encoding='utf-8')
    )
    assert manifest['status'] == expected_status
    assert manifest['execution']['exit_code'] == expected_exit_code
    assert manifest['lifecycle']['stage'] == 'complete'
    assert manifest['lifecycle']['runner_exit_code'] == expected_exit_code
    assert manifest['output']['finalized'] is True
    assert not output_dir.with_name('failed_map.partial').exists()


def test_main_preserves_failed_manifest_when_post_finalize_diagnosis_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'map_output'
    plan = {
        'payload': {},
        'profile_id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO + graph_based_slam public path',
        'command': ['python3', '-c', 'pass'],
        'output_dir': output_dir.with_name('map_output.partial'),
    }
    monkeypatch.setattr(module, 'build_execution_plan', lambda **kwargs: plan)
    monkeypatch.setattr(module, 'maybe_verify_map', lambda *args, **kwargs: None)

    def fail_diagnosis(*args, **kwargs):
        del args, kwargs
        raise RuntimeError('diagnosis fixture failure')

    monkeypatch.setattr(module, 'write_diagnostics', fail_diagnosis)
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
        ],
    )

    assert module.main() == 70
    manifest = json.loads(
        (output_dir / 'run_manifest.json').read_text(encoding='utf-8')
    )
    assert manifest['status'] == 'failed'
    assert manifest['execution']['exit_code'] == 0
    assert manifest['lifecycle']['runner_exit_code'] == 70
    assert manifest['lifecycle']['last_error'] == 'diagnosis fixture failure'
    assert manifest['output']['finalized'] is True


@pytest.mark.parametrize(
    ('resume_from_final', 'stage'),
    [
        (False, 'workflow_finished'),
        (True, 'finalizing'),
    ],
)
def test_main_resumes_only_terminal_postprocessing_without_rerunning_workflow(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    resume_from_final: bool,
    stage: str,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'map_output'
    working_dir = tmp_path / 'map_output.partial'
    plan = {
        'payload': {},
        'profile_id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO + graph_based_slam public path',
        'command': ['map-workflow', '--output-dir', str(working_dir)],
        'output_dir': working_dir,
    }
    software = {
        'product_version': '0.6.0',
        'git_commit': '0' * 40,
        'git_dirty': False,
        'package_versions': {'lidarslam': '0.6.0'},
        'ros_distro': 'jazzy',
    }
    monkeypatch.setattr(module, 'build_execution_plan', lambda **kwargs: plan)
    monkeypatch.setattr(module, '_software_identity', lambda: software)

    manifest = module._build_manifest(bag_dir, output_dir, working_dir, plan)
    manifest['status'] = 'succeeded'
    manifest['execution'].update({
        'started_at': '2026-07-27T00:00:00Z',
        'finished_at': '2026-07-27T00:01:00Z',
        'exit_code': 0,
    })
    manifest['lifecycle']['stage'] = stage
    run_dir = output_dir if resume_from_final else working_dir
    run_dir.mkdir()
    (run_dir / 'pointcloud_map').mkdir()
    (run_dir / 'pointcloud_map' / 'pointcloud_map_metadata.yaml').write_text(
        'tiles: []\n',
        encoding='utf-8',
    )
    (run_dir / 'map_projector_info.yaml').write_text(
        'projector_type: Local\n',
        encoding='utf-8',
    )
    module._write_manifest(run_dir, manifest)

    def fake_verify(current_dir: Path, enabled: bool):
        assert enabled is True
        (current_dir / 'verify_autoware_map.log').write_text(
            'RESULT: PASS\n',
            encoding='utf-8',
        )

    def fake_diagnosis(current_dir: Path, current_bag: Path):
        assert current_bag == bag_dir
        (current_dir / 'autoware_map_diagnosis.md').write_text(
            '# success\n',
            encoding='utf-8',
        )
        (current_dir / 'autoware_map_diagnosis.json').write_text(
            '{"status":"success"}\n',
            encoding='utf-8',
        )
        return {'status': 'success'}

    def fail_if_workflow_runs(*args, **kwargs):
        del args, kwargs
        raise AssertionError('resume must not execute the map workflow')

    monkeypatch.setattr(module, 'maybe_verify_map', fake_verify)
    monkeypatch.setattr(module, 'write_diagnostics', fake_diagnosis)
    monkeypatch.setattr(module.subprocess, 'run', fail_if_workflow_runs)
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--resume',
        ],
    )

    assert module.main() == 0
    assert output_dir.is_dir()
    assert not working_dir.exists()
    saved = json.loads(
        (output_dir / 'run_manifest.json').read_text(encoding='utf-8')
    )
    assert saved['status'] == 'succeeded'
    assert saved['execution']['exit_code'] == 0
    assert saved['lifecycle'] == {
        'last_error': None,
        'resume_count': 1,
        'runner_exit_code': 0,
        'stage': 'complete',
        'verification_enabled': True,
    }
    assert saved['output']['finalized'] is True


@pytest.mark.parametrize(
    ('schema_version', 'stage', 'expected_error'),
    [
        (2, 'workflow_running', 'workflow may still be running'),
        (1, 'workflow_finished', 'resume requires run manifest schema v2'),
    ],
)
def test_main_refuses_unsafe_or_legacy_resume_state(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture,
    schema_version: int,
    stage: str,
    expected_error: str,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'map_output'
    working_dir = tmp_path / 'map_output.partial'
    plan = {
        'payload': {},
        'profile_id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO + graph_based_slam public path',
        'command': ['map-workflow'],
        'output_dir': working_dir,
    }
    monkeypatch.setattr(module, 'build_execution_plan', lambda **kwargs: plan)
    monkeypatch.setattr(module, '_software_identity', lambda: {
        'product_version': '0.6.0',
        'git_commit': '0' * 40,
        'git_dirty': False,
        'package_versions': {'lidarslam': '0.6.0'},
        'ros_distro': 'jazzy',
    })
    manifest = module._build_manifest(bag_dir, output_dir, working_dir, plan)
    manifest['schema_version'] = schema_version
    manifest['lifecycle']['stage'] = stage
    manifest['status'] = 'running' if stage == 'workflow_running' else 'succeeded'
    manifest['execution'].update({
        'started_at': '2026-07-27T00:00:00Z',
        'finished_at': (
            None if stage == 'workflow_running' else '2026-07-27T00:01:00Z'
        ),
        'exit_code': None if stage == 'workflow_running' else 0,
    })
    working_dir.mkdir()
    module._write_manifest(working_dir, manifest)
    original_manifest = (working_dir / 'run_manifest.json').read_text(encoding='utf-8')
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--resume',
        ],
    )

    assert module.main() == 2
    assert expected_error in capsys.readouterr().err
    assert (working_dir / 'run_manifest.json').read_text(
        encoding='utf-8'
    ) == original_manifest


def test_main_refuses_resume_when_bag_identity_changed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'map_output'
    working_dir = tmp_path / 'map_output.partial'
    plan = {
        'payload': {},
        'profile_id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO + graph_based_slam public path',
        'command': ['map-workflow'],
        'output_dir': working_dir,
    }
    monkeypatch.setattr(module, 'build_execution_plan', lambda **kwargs: plan)
    monkeypatch.setattr(module, '_software_identity', lambda: {
        'product_version': '0.6.0',
        'git_commit': '0' * 40,
        'git_dirty': False,
        'package_versions': {'lidarslam': '0.6.0'},
        'ros_distro': 'jazzy',
    })
    manifest = module._build_manifest(bag_dir, output_dir, working_dir, plan)
    manifest['status'] = 'succeeded'
    manifest['execution'].update({
        'started_at': '2026-07-27T00:00:00Z',
        'finished_at': '2026-07-27T00:01:00Z',
        'exit_code': 0,
    })
    manifest['lifecycle']['stage'] = 'workflow_finished'
    working_dir.mkdir()
    module._write_manifest(working_dir, manifest)
    (bag_dir / 'demo_bag_0.db3').write_bytes(b'changed bag fixture\n')
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--resume',
        ],
    )

    assert module.main() == 2
    assert 'resume input identity mismatch' in capsys.readouterr().err
    assert working_dir.is_dir()
    assert not output_dir.exists()


def test_runner_rejects_incompatible_profile_with_available_hint(tmp_path: Path):
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )

    result = subprocess.run(
        [
            'python3',
            str(SCRIPT_PATH),
            str(bag_dir),
            '--profile',
            'pointcloud_gnss_smoke',
            '--dry-run',
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert (
        'profile is not compatible with this bag: pointcloud_gnss_smoke'
        in result.stderr
    )
    assert 'Available profiles: rko_lio_graph_public_path' in result.stderr
    assert 'Traceback' not in result.stderr


def test_dogfood_script_can_skip_viewer():
    script = (REPO_ROOT / 'scripts' / 'run_rko_lio_graph_autoware_dogfood.sh').read_text(
        encoding='utf-8'
    )

    assert '--skip-viewer' in script
    assert 'if [[ "$SKIP_VIEWER" == "false" ]]; then' in script


def test_beginner_wrapper_exposes_simple_viewer_flags():
    script = BEGINNER_SCRIPT_PATH.read_text(encoding='utf-8')

    assert 'run_autoware_map_from_bag.py' in script
    assert '--foxglove' in script
    assert '--autoware' in script
    assert '--no-viewer' in script
    assert '--dry-run' in script
    assert '--resume' in script
    assert '--preflight-only' in script
    assert 'metadata.yaml not found' in script
    assert 'not a .db3 file' in script


def test_beginner_wrapper_help_is_user_facing():
    result = subprocess.run(
        ['bash', str(BEGINNER_SCRIPT_PATH), '--help'],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert 'Autoware-compatible map workflow' in result.stderr
    assert 'The input must be the rosbag2 directory' in result.stderr
    assert 'Expected successful outputs:' in result.stderr
    assert '--output-dir <dir>' in result.stderr
    assert '--viewer-rebuild' in result.stderr
    assert '--resume' in result.stderr


def test_beginner_wrapper_rejects_missing_option_value_before_bag_validation(
    tmp_path: Path,
):
    result = subprocess.run(
        [
            'bash',
            str(BEGINNER_SCRIPT_PATH),
            str(tmp_path / 'missing_bag'),
            '--output-dir',
            '--dry-run',
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error: option requires a value: --output-dir' in result.stderr
    assert 'metadata.yaml not found' not in result.stderr


def test_beginner_wrapper_rejects_conflicting_viewer_flags(tmp_path: Path):
    result = subprocess.run(
        [
            'bash',
            str(BEGINNER_SCRIPT_PATH),
            str(tmp_path / 'missing_bag'),
            '--foxglove',
            '--autoware',
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error: viewer already set to foxglove; cannot also use --autoware' in result.stderr
    assert 'metadata.yaml not found' not in result.stderr


def test_runner_prefers_mid360_preset_for_livox_bag(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'mid360_demo_bag',
        [
            ('/livox/lidar', 'sensor_msgs/msg/PointCloud2', 20),
            ('/livox/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )

    plan = module.build_execution_plan(
        bag_path=bag_dir,
        profile_id=None,
        output_dir=tmp_path / 'out',
        verify_map=True,
    )

    assert plan['profile_id'] == 'rko_lio_graph_mid360_preset'
    assert 'lidarslam_mid360_rko_graph.yaml' in ' '.join(plan['command'])
