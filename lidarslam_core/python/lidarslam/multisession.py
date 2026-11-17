"""The multi-session join (graph_slam_multisession_join) without ROS 2.

Takes the join's parameters (the ROS 2 executable's, node name ``graph_based_slam``), reads each
session's backend-input log with :mod:`lidarslam.io` in log-time order, and runs the shared tool
(``graphslam::multisession::runJoinTool``): place recognition between the sessions, the joined
pose graph, and the same output files, byte for byte, as the ROS 2 executable (join_report.yaml,
inter_session_edges.csv, trajectory_session<i>.tum and the stored self-similarity results).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, Iterator

from lidarslam import _core
from lidarslam.io import SensorLog
from lidarslam.params import load_parameters as _load_parameters

NODE_NAME = 'graph_based_slam'


def load_parameters(*paths: str | Path) -> dict[str, Any]:
    """Return the ``graph_based_slam`` parameters of ROS 2 parameter files (later override)."""
    return _load_parameters(paths, node=NODE_NAME)


def _read(path: str, topics: list[str]) -> Iterator[tuple[str, Any]]:
    """Yield (topic, message) of the given topics of a log in log-time order.

    Topics the log lacks give none, as rosbag2 gives none for them.
    """
    with SensorLog(path) as log:
        present = {topic.name for topic in log.topics()}
        wanted = [topic for topic in topics if topic in present]
        if not wanted:
            return
        for record in log.records(wanted):
            yield record.topic, record.message


def run(params: dict[str, Any], log: Callable[[str, str], None] | None = None) -> int:
    """Run the join; return the executable's exit code (0: outputs written).

    ``params`` are the executable's parameters (:func:`load_parameters` plus the session lists:
    ``session_bag_paths``, ``session_backend_dirs``, ``output_dir``, ...). ``log(level, message)``
    receives its messages (default: stderr).
    """
    return int(_core.run_join_tool(params, _read, log))
