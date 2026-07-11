"""Optional ROS message sampling isolated from MID-360 domain logic."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .mid360_models import MessageSample


class RosbagMessageSampler:
    """Sample standard ROS messages from a rosbag2 directory."""

    def read_samples(self, bag_path: Path, topics: list[str],
                     limit_per_topic: int) -> dict[str, list[MessageSample]]:
        if limit_per_topic <= 0:
            return {topic: [] for topic in topics}
        try:
            from rosbags.highlevel import AnyReader
            from rosbags.typesys import Stores, get_typestore
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError('rosbags is required for message sampling') from exc

        remaining = {topic: limit_per_topic for topic in topics if topic}
        samples: dict[str, list[MessageSample]] = {topic: [] for topic in remaining}
        if not remaining:
            return samples
        typestore = get_typestore(Stores.LATEST)
        with AnyReader([bag_path], default_typestore=typestore) as reader:
            connections = [conn for conn in reader.connections if conn.topic in remaining]
            for connection, timestamp_ns, raw in reader.messages(connections=connections):
                topic = connection.topic
                if remaining[topic] <= 0:
                    continue
                msg = reader.deserialize(raw, connection.msgtype)
                samples[topic].append(
                    self._sample_message(topic, connection.msgtype, timestamp_ns, msg))
                remaining[topic] -= 1
                if all(count <= 0 for count in remaining.values()):
                    break
        return samples

    @staticmethod
    def _sample_message(topic: str, msg_type: str, timestamp_ns: int | None,
                        msg: Any) -> MessageSample:
        header = getattr(msg, 'header', None)
        frame_id = str(getattr(header, 'frame_id', '') or '') if header else ''
        return MessageSample(
            topic=topic,
            msg_type=msg_type,
            timestamp_ns=int(timestamp_ns) if timestamp_ns is not None else None,
            header_stamp_ns=RosbagMessageSampler._header_stamp_ns(header),
            frame_id=frame_id,
            tf_pairs=tuple(RosbagMessageSampler._tf_pairs(msg)),
        )

    @staticmethod
    def _header_stamp_ns(header: Any) -> int | None:
        if header is None or getattr(header, 'stamp', None) is None:
            return None
        sec = getattr(header.stamp, 'sec', None)
        nanosec = getattr(header.stamp, 'nanosec', None)
        if sec is None or nanosec is None:
            return None
        return int(sec) * 1_000_000_000 + int(nanosec)

    @staticmethod
    def _tf_pairs(msg: Any) -> list[tuple[str, str]]:
        pairs = []
        for transform in getattr(msg, 'transforms', []) or []:
            header = getattr(transform, 'header', None)
            parent = str(getattr(header, 'frame_id', '') or '') if header else ''
            child = str(getattr(transform, 'child_frame_id', '') or '')
            if parent and child:
                pairs.append((parent, child))
        return pairs
