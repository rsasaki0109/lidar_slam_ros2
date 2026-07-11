"""Storage discovery and process supervision for LiDAR SLAM commands."""

from __future__ import annotations

import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
from typing import Iterable

import yaml


DEFAULT_SCAN_ROOTS = (Path('/media'), Path('/mnt'))
MAP_POINTS_RE = re.compile(r'number of points in the map\s*:\s*(\d+)')


def read_bag_summary(bag: Path) -> dict:
    """Read the small, stable subset of rosbag2 metadata used by the UI."""
    try:
        payload = yaml.safe_load((bag / 'metadata.yaml').read_text(encoding='utf-8')) or {}
        info = payload.get('rosbag2_bagfile_information') or {}
        duration = info.get('duration') or {}
        topics = info.get('topics_with_message_count') or []
        topic_names = {
            str((row.get('topic_metadata') or {}).get('name') or '') for row in topics
        }
        return {
            'duration_sec': float(duration.get('nanoseconds') or 0) / 1e9,
            'message_count': int(info.get('message_count') or 0),
            'topics': topic_names,
            'supported': '/hesai/pandar' in topic_names,
        }
    except (OSError, TypeError, ValueError, yaml.YAMLError):
        return {'duration_sec': 0.0, 'message_count': 0, 'topics': set(), 'supported': False}


def discover_bags(roots: Iterable[Path]) -> list[tuple[Path, dict]]:
    """Find rosbag2 directories under mounted storage, without following links."""
    found: list[tuple[Path, dict]] = []
    seen: set[Path] = set()
    for root in roots:
        root = root.expanduser()
        if not root.is_dir():
            continue
        for metadata in root.glob('**/metadata.yaml'):
            bag = metadata.parent.resolve()
            if bag not in seen:
                seen.add(bag)
                found.append((bag, read_bag_summary(bag)))
    return sorted(found, key=lambda item: str(item[0]))


def resolve_bag(value: str, candidates: list[tuple[Path, dict]]) -> Path:
    path = Path(value).expanduser()
    if (path / 'metadata.yaml').is_file():
        return path.resolve()
    matches = [bag for bag, _ in candidates if bag.name in {value, f'{value}_ros2'}]
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        raise ValueError(f'「{value}」が複数あります。フルパスで指定してください。')
    raise ValueError(f'ROS bag「{value}」が見つかりません。--list で候補を確認してください。')


def default_output_dir(bag: Path) -> Path:
    """Keep large generated maps on the same mounted drive when possible."""
    parts = bag.parts
    if len(parts) >= 4 and parts[1] in {'media', 'mnt'}:
        mount_root = Path(*parts[:4]) if parts[1] == 'media' else Path(*parts[:3])
        return mount_root / 'lidarslam_work' / 'output' / 'maps'
    return Path.home() / 'lidarslam_work' / 'output' / 'maps'


def free_gib(path: Path) -> float:
    probe = path
    while not probe.exists() and probe != probe.parent:
        probe = probe.parent
    return shutil.disk_usage(probe).free / 1024**3


def latest_map_points(log_path: Path) -> int:
    """Return the latest published map size from the tail of a SLAM log."""
    try:
        with log_path.open('rb') as stream:
            stream.seek(0, os.SEEK_END)
            stream.seek(max(0, stream.tell() - 64 * 1024))
            text = stream.read().decode('utf-8', errors='replace')
    except OSError:
        return 0
    matches = MAP_POINTS_RE.findall(text)
    return int(matches[-1]) if matches else 0


def clock(seconds: float) -> str:
    seconds = max(0, round(seconds))
    minutes, sec = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    return f'{hours:d}:{minutes:02d}:{sec:02d}' if hours else f'{minutes:02d}:{sec:02d}'


def progress_line(elapsed: float, expected: float, points: int) -> str:
    ratio = min(0.99, elapsed / expected) if expected > 0 else 0.0
    width = 20
    filled = min(width, int(ratio * width))
    bar = '#' * filled + '-' * (width - filled)
    point_text = f'{points:,}点' if points else '準備中'
    return (f'[{bar}] {ratio * 100:5.1f}%  残り約{clock(max(0.0, expected - elapsed))}'
            f'  地図 {point_text}')


def run_with_progress(command: list[str], env: dict[str, str], expected_sec: float,
                      slam_log: Path) -> tuple[int, bool]:
    """Run the SLAM process group, displaying progress and stopping it safely."""
    process = subprocess.Popen(command, env=env, start_new_session=True)
    started = time.monotonic()
    last_plain_update = -5
    interrupted = False
    try:
        while process.poll() is None:
            elapsed = time.monotonic() - started
            line = progress_line(elapsed, expected_sec, latest_map_points(slam_log))
            if sys.stdout.isatty():
                print('\r' + line, end='', flush=True)
            elif elapsed - last_plain_update >= 5:
                print(line, flush=True)
                last_plain_update = elapsed
            time.sleep(1)
    except KeyboardInterrupt:
        interrupted = True
        print('\n停止要求を受け取りました。途中地図を安全に保存しています…', flush=True)
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
    if sys.stdout.isatty():
        print()
    return process.returncode or 0, interrupted
