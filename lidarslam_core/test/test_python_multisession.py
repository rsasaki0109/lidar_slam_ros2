"""lidarslam multi-session join without ROS 2 on a synthetic backend input (CI smoke test).

The backend input of test_python_backend (two laps around a box room) is replayed by the backend,
then joined: two sessions (the same run twice) and one session split in two. The join must run
end to end, write graph_slam_multisession_join's files, store the self-similarity results and give
the same bytes twice. Whether a synthetic room joins is not the point; byte identity with the
ROS 2 executable is checked on real logs (docs/roadmap/ros-free-core-2026-11.md).
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

lidarslam = pytest.importorskip('lidarslam')
core = pytest.importorskip('lidarslam._core')
if not hasattr(core, 'run_join_tool'):
    pytest.skip('lidarslam built without the multi-session join', allow_module_level=True)

from lidarslam import backend, multisession  # noqa: E402
from lidarslam.__main__ import main as cli  # noqa: E402
from test_python_backend import _write_input  # noqa: E402
import yaml  # noqa: E402

PARAMS = {'registration_method': 'SMALL_GICP', 'gicp_max_correspondence_distance': 0.5,
          'ndt_num_threads': 1, 'voxel_leaf_size': 0.2, 'submap_distance_threshold': 1.5}


@pytest.fixture(scope='module')
def session(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, Path]:
    root = tmp_path_factory.mktemp('session')
    log = root / 'backend_input'
    _write_input(log)
    backend.run(log, dict(PARAMS), root / 'run', log=lambda level, msg: None)
    return log, root / 'run'


def _digests(directory: Path) -> dict[str, str]:
    return {p.name: hashlib.md5(p.read_bytes()).hexdigest()
            for p in sorted(directory.iterdir()) if p.is_file()}


def _join(session: tuple[Path, Path], out: Path, sessions: int = 2, **extra) -> tuple[int, list]:
    log, run = session
    params = dict(PARAMS, session_bag_paths=[str(log)] * sessions,
                  session_backend_dirs=[str(run)] * sessions, output_dir=str(out),
                  multisession_num_threads=2, **extra)
    messages: list[tuple[str, str]] = []
    code = multisession.run(params, log=lambda level, msg: messages.append((level, msg)))
    return code, messages


def test_two_sessions_are_joined_into_one_graph(session: tuple[Path, Path],
                                                tmp_path: Path) -> None:
    cache = tmp_path / 'cache'
    code, messages = _join(session, tmp_path / 'a', multisession_cache_dir=str(cache))
    assert code == 0, messages
    assert any('Multi-session join complete' in msg for _, msg in messages)
    report = yaml.safe_load((tmp_path / 'a' / 'join_report.yaml').read_text())
    assert [s['session'] for s in report['sessions']] == [0, 1]
    assert (tmp_path / 'a' / 'trajectory_session0.tum').is_file()
    header = (tmp_path / 'a' / 'inter_session_edges.csv').read_text().splitlines()[0]
    assert header.startswith('session,map_session,submap_a,submap_b')
    fingerprints = {s['fingerprint'] for s in report['self_similarity']}
    assert len(fingerprints) == 1  # the same run twice
    # every session whose self-similarity was checked is stored under its fingerprint
    checked = [s for s in report['self_similarity'] if s['check'] == 'checked']
    for entry in checked:
        assert (cache / f"self_similarity_{entry['fingerprint']}.yaml").is_file()

    code, _ = _join(session, tmp_path / 'b', multisession_cache_dir=str(tmp_path / 'cache2'))
    assert code == 0
    assert _digests(tmp_path / 'a') == _digests(tmp_path / 'b')


def test_one_session_split_in_two(session: tuple[Path, Path], tmp_path: Path) -> None:
    code, messages = _join(session, tmp_path / 'split', sessions=1, debug_split_gap_s=1.0)
    assert code == 0, messages
    report = yaml.safe_load((tmp_path / 'split' / 'join_report.yaml').read_text())
    assert report['split']['keyframes_a'] > 0 and report['split']['keyframes_b'] > 0


def test_refusals_of_the_executable(session: tuple[Path, Path], tmp_path: Path) -> None:
    code, messages = _join(session, tmp_path / 'one', sessions=1)
    assert code == 1
    assert messages[0][0] == 'ERROR' and 'need at least two sessions' in messages[0][1]
    code, messages = _join(session, tmp_path / 'e', extrinsic_lidar2base_quat_xyzw_xyz=[0.0])
    assert code == 1 and 'extrinsic_lidar2base_quat_xyzw_xyz' in messages[0][1]
    # a parameter of the wrong type, as rclcpp refuses it
    code, messages = _join(session, tmp_path / 't', multisession_keyframe_window_s=1)
    assert code == 1 and 'multisession_keyframe_window_s' in messages[-1][1]


def test_python_m_lidarslam_join(session: tuple[Path, Path], tmp_path: Path) -> None:
    log, run = session
    params = tmp_path / 'params.yaml'
    params.write_text(yaml.safe_dump({'graph_based_slam': {'ros__parameters': PARAMS}}))
    assert cli(['join', '--params', str(params),
                '-p', f"session_bag_paths:=['{log}']", '-p', f"session_backend_dirs:=['{run}']",
                '-p', 'debug_split_gap_s:=1.0', '-p', f'output_dir:={tmp_path / "cli"}']) == 0
    assert (tmp_path / 'cli' / 'join_report.yaml').is_file()
