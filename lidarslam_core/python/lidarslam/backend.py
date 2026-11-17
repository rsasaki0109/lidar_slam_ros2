"""The graph_based_slam offline backend on a recorded backend input, without ROS 2.

Takes the parameter file of the ROS 2 runner (``graph_slam_offline_runner``, node name
``graph_based_slam``), reads the odometry and deskewed clouds (and, with ``use_gnss``, the
NavSatFix messages of ``gnss_topic``) of a backend-input log with
:mod:`lidarslam.io` in log-time order, and runs the shared pipeline
(``graphslam::offline::OfflinePipeline``): submaps, loop search, pose graph, refinement and the
same output files, byte for byte, as the ROS 2 runner.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

from lidarslam import _core
from lidarslam.io import NAV_SAT_FIX, ODOMETRY, POINT_CLOUD2, SensorLog
from lidarslam.params import load_parameters as _load_parameters

NODE_NAME = 'graph_based_slam'


def load_parameters(*paths: str | Path) -> dict[str, Any]:
    """The ``graph_based_slam`` parameters of ROS 2 parameter files (later files override)."""
    return _load_parameters(paths, node=NODE_NAME)


def run(log_path: str | Path, params: dict[str, Any], out_dir: str | Path | None = None,
        log: Callable[[str, str], None] | None = None) -> dict[str, Any]:
    """Runs the backend on a backend-input log; returns a summary of the run.

    ``params`` are the runner's parameters (:func:`load_parameters`); ``bag_path`` is set to
    ``log_path`` and, when given, ``output_dir`` to ``out_dir``. ``log(level, message)``
    receives the runner's messages (default: stderr).
    """
    params = dict(params)
    params['bag_path'] = str(log_path)
    if out_dir is not None:
        params['output_dir'] = str(out_dir)
    Path(params.get('output_dir', '.')).mkdir(parents=True, exist_ok=True)

    pipeline = _core.BackendPipeline(params, log)
    if not pipeline.configure():
        raise RuntimeError('the backend could not be configured (see its log)')
    counts = {'odometry': 0, 'clouds': 0, 'gnss_fixes': 0}
    with SensorLog(log_path) as sensor_log:
        types = {t.name: t.msgtype for t in sensor_log.topics()}
        for topic, expected in ((pipeline.odometry_topic, ODOMETRY),
                                (pipeline.cloud_topic, POINT_CLOUD2)):
            if types.get(topic) != expected:
                raise KeyError(f'{topic}: not a {expected} topic of {log_path}')
        topics = [pipeline.odometry_topic, pipeline.cloud_topic]
        # As the ROS 2 runner: the fixes when the log has the topic (the backend warns when no
        # GNSS origin was set).
        if pipeline.uses_gnss and types.get(pipeline.gnss_topic) == NAV_SAT_FIX:
            topics.append(pipeline.gnss_topic)
        pipeline.begin()
        for record in sensor_log.records(topics):
            if record.topic == pipeline.odometry_topic:
                pipeline.add_odometry(record.message)
                counts['odometry'] += 1
            elif record.topic == pipeline.cloud_topic:
                pipeline.add_cloud(record.message)
                counts['clouds'] += 1
            else:
                pipeline.add_gnss_fix(record.message)
                counts['gnss_fixes'] += 1
    if not pipeline.finish():
        raise RuntimeError('the backend failed after reading the log (see its log)')
    return {
        'output_dir': pipeline.output_dir,
        'odometry_messages': counts['odometry'],
        'cloud_messages': counts['clouds'],
        'gnss_fixes': counts['gnss_fixes'],
    }
