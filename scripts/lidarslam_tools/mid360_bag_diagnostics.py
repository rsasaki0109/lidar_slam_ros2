"""MID-360 metadata, frame, rate, and TF diagnostics."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .mid360_models import MessageSample, RobotFrames, TopicSelection
from .rosbag_sampling import RosbagMessageSampler

DEFAULT_SAMPLE_MESSAGES = 20
POINTCLOUD_MIN_METADATA_HZ = 5.0
IMU_MIN_METADATA_HZ = 50.0

class Mid360BagDiagnosticsBuilder:
    """Build metadata-rate and message-sample diagnostics for a robot bag."""

    def __init__(
        self,
        sample_reader: Any | None = None,
        sample_limit: int = DEFAULT_SAMPLE_MESSAGES,
    ) -> None:
        self._sample_reader = sample_reader or RosbagMessageSampler()
        self._sample_limit = sample_limit

    def build(
        self,
        bag_path: Path,
        summary: dict[str, Any],
        topics: TopicSelection,
        frames: RobotFrames,
    ) -> dict[str, Any]:
        sample_topics = self._sample_topics(summary, topics)
        samples_by_topic, sample_reader_payload = self._read_samples(bag_path, sample_topics)
        pointcloud = self._topic_diagnostics(
            summary=summary,
            key='pointcloud2',
            topic=topics.pointcloud,
            expected_frame=frames.lidar_frame,
            min_rate_hz=POINTCLOUD_MIN_METADATA_HZ,
            samples=samples_by_topic.get(topics.pointcloud or '', []),
        )
        imu = self._topic_diagnostics(
            summary=summary,
            key='imu',
            topic=topics.imu,
            expected_frame=frames.imu_frame,
            min_rate_hz=IMU_MIN_METADATA_HZ,
            samples=samples_by_topic.get(topics.imu or '', []),
        )
        return {
            'sample_reader': sample_reader_payload,
            'topics': {
                'pointcloud': pointcloud,
                'imu': imu,
            },
            'tf': self._tf_diagnostics(summary, samples_by_topic, frames),
        }

    @staticmethod
    def _sample_topics(summary: dict[str, Any], topics: TopicSelection) -> list[str]:
        result = []
        if topics.pointcloud:
            result.append(topics.pointcloud)
        if topics.imu:
            result.append(topics.imu)
        result.extend(item['name'] for item in summary['topics'].get('tf', []))
        return list(dict.fromkeys(result))

    def _read_samples(
        self,
        bag_path: Path,
        topics: list[str],
    ) -> tuple[dict[str, list[MessageSample]], dict[str, Any]]:
        payload = {
            'attempted': False,
            'available': False,
            'limit_per_topic': self._sample_limit,
            'reason': '',
        }
        if not topics or self._sample_limit <= 0:
            payload['reason'] = 'no topics selected for sampling'
            return {}, payload

        payload['attempted'] = True
        try:
            samples = self._sample_reader.read_samples(bag_path, topics, self._sample_limit)
        except Exception as exc:  # Sampling is advisory; metadata checks still run.
            payload['reason'] = str(exc)
            return {}, payload

        sampled_count = sum(len(items) for items in samples.values())
        payload['available'] = sampled_count > 0
        if sampled_count == 0:
            payload['reason'] = 'no readable messages sampled'
        return samples, payload

    @staticmethod
    def _topic_diagnostics(
        summary: dict[str, Any],
        key: str,
        topic: str | None,
        expected_frame: str,
        min_rate_hz: float,
        samples: list[MessageSample],
    ) -> dict[str, Any]:
        record = Mid360BagDiagnosticsBuilder._find_topic(summary, key, topic)
        metadata_message_count = int(record.get('message_count', 0)) if record else 0
        metadata_rate_hz = Mid360BagDiagnosticsBuilder._metadata_rate_hz(
            metadata_message_count,
            summary.get('duration_sec'),
        )
        frame_ids = [sample.frame_id for sample in samples if sample.frame_id]
        unique_frame_ids = sorted(set(frame_ids))
        sample_span_sec = Mid360BagDiagnosticsBuilder._sample_span_sec(samples)
        return {
            'topic': topic,
            'metadata_message_count': metadata_message_count,
            'metadata_rate_hz': metadata_rate_hz,
            'min_metadata_rate_hz': min_rate_hz,
            'sampled_message_count': len(samples),
            'sample_time_span_sec': sample_span_sec,
            'sample_observed_rate_hz': Mid360BagDiagnosticsBuilder._sample_rate_hz(
                len(samples),
                sample_span_sec,
            ),
            'sampled_frame_ids': unique_frame_ids,
            'frame_id_changes': Mid360BagDiagnosticsBuilder._frame_id_changes(frame_ids),
            'stable_frame_id': None if not frame_ids else len(unique_frame_ids) == 1,
            'expected_frame_id': expected_frame,
            'matches_expected_frame': (
                None if not frame_ids else expected_frame in unique_frame_ids
            ),
        }

    @staticmethod
    def _find_topic(
        summary: dict[str, Any],
        key: str,
        topic: str | None,
    ) -> dict[str, Any] | None:
        if not topic:
            return None
        for item in summary['topics'].get(key, []):
            if item['name'] == topic:
                return item
        return None

    @staticmethod
    def _metadata_rate_hz(message_count: int, duration_sec: Any) -> float | None:
        if duration_sec is None:
            return None
        duration = float(duration_sec)
        if duration <= 0.0:
            return None
        return message_count / duration

    @staticmethod
    def _sample_span_sec(samples: list[MessageSample]) -> float | None:
        timestamps = [sample.timestamp_ns for sample in samples if sample.timestamp_ns is not None]
        if len(timestamps) < 2:
            return None
        span_ns = int(timestamps[-1]) - int(timestamps[0])
        if span_ns <= 0:
            return None
        return span_ns / 1e9

    @staticmethod
    def _sample_rate_hz(message_count: int, span_sec: float | None) -> float | None:
        if message_count < 2 or span_sec is None or span_sec <= 0.0:
            return None
        return (message_count - 1) / span_sec

    @staticmethod
    def _frame_id_changes(frame_ids: list[str]) -> int:
        if len(frame_ids) < 2:
            return 0
        return sum(1 for before, after in zip(frame_ids, frame_ids[1:]) if before != after)

    @staticmethod
    def _tf_diagnostics(
        summary: dict[str, Any],
        samples_by_topic: dict[str, list[MessageSample]],
        frames: RobotFrames,
    ) -> dict[str, Any]:
        tf_topics = [item['name'] for item in summary['topics'].get('tf', [])]
        samples = []
        for topic in tf_topics:
            samples.extend(samples_by_topic.get(topic, []))

        pairs = []
        seen = set()
        for sample in samples:
            for parent, child in sample.tf_pairs:
                if (parent, child) not in seen:
                    seen.add((parent, child))
                    pairs.append({'parent': parent, 'child': child})

        graph_pairs = [(item['parent'], item['child']) for item in pairs]
        return {
            'topics': tf_topics,
            'sampled_message_count': len(samples),
            'frame_pairs': pairs,
            'base_to_lidar_connected': Mid360BagDiagnosticsBuilder._frames_connected(
                graph_pairs,
                frames.base_frame,
                frames.lidar_frame,
            ),
            'base_to_imu_connected': Mid360BagDiagnosticsBuilder._frames_connected(
                graph_pairs,
                frames.base_frame,
                frames.imu_frame,
            ),
        }

    @staticmethod
    def _frames_connected(
        pairs: list[tuple[str, str]],
        source_frame: str,
        target_frame: str,
    ) -> bool | None:
        if source_frame == target_frame:
            return True
        if not pairs:
            return None
        adjacency: dict[str, list[str]] = {}
        for parent, child in pairs:
            adjacency.setdefault(parent, []).append(child)
            adjacency.setdefault(child, []).append(parent)

        queue = [source_frame]
        visited = {source_frame}
        while queue:
            current = queue.pop(0)
            for next_frame in adjacency.get(current, []):
                if next_frame == target_frame:
                    return True
                if next_frame in visited:
                    continue
                visited.add(next_frame)
                queue.append(next_frame)
        return False
