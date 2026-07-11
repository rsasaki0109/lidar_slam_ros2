#!/usr/bin/env python3
"""Friendly launcher for discovering and mapping supported ROS 2 bags."""

from __future__ import annotations

import argparse
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


REPO_ROOT = Path(__file__).resolve().parents[1]
HILTI_RUNNER = REPO_ROOT / 'tools/gaussian_splatting/bim_reference_scripts/run_hilti_slam.sh'
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
            if bag in seen:
                continue
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


def _free_gib(path: Path) -> float:
    probe = path
    while not probe.exists() and probe != probe.parent:
        probe = probe.parent
    return shutil.disk_usage(probe).free / 1024**3


def _latest_map_points(log_path: Path) -> int:
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


def _clock(seconds: float) -> str:
    seconds = max(0, round(seconds))
    minutes, sec = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    return f'{hours:d}:{minutes:02d}:{sec:02d}' if hours else f'{minutes:02d}:{sec:02d}'


def _progress_line(elapsed: float, expected: float, points: int) -> str:
    ratio = min(0.99, elapsed / expected) if expected > 0 else 0.0
    percent = ratio * 100
    remaining = max(0.0, expected - elapsed)
    width = 20
    filled = min(width, int(ratio * width))
    bar = '#' * filled + '-' * (width - filled)
    point_text = f'{points:,}点' if points else '準備中'
    return f'[{bar}] {percent:5.1f}%  残り約{_clock(remaining)}  地図 {point_text}'


def _run_with_progress(command: list[str], env: dict[str, str], expected_sec: float,
                       slam_log: Path) -> tuple[int, bool]:
    """Run the SLAM process group, displaying progress and stopping it safely."""
    process = subprocess.Popen(command, env=env, start_new_session=True)
    started = time.monotonic()
    last_plain_update = -5
    interrupted = False
    try:
        while process.poll() is None:
            elapsed = time.monotonic() - started
            line = _progress_line(elapsed, expected_sec, _latest_map_points(slam_log))
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


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='SSDのROS bagを見つけて、LiDAR SLAM地図を簡単に生成します。',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('bag', nargs='?', help='exp01、exp01_ros2、またはbagのフルパス')
    parser.add_argument('--list', action='store_true', help='見つかったbagを一覧表示して終了')
    parser.add_argument('--all', action='store_true', help='未対応センサーのbagも一覧に表示')
    parser.add_argument('--scan-root', type=Path, action='append', help='探索場所（複数指定可）')
    parser.add_argument('-o', '--output-dir', type=Path, help='地図の保存先')
    parser.add_argument('--duration', default='full', help='処理時間（秒、またはfull）')
    parser.add_argument('--rate', type=float, default=0.5, help='bag再生速度')
    parser.add_argument('--force', action='store_true', help='既存の地図を上書き')
    parser.add_argument('--yes', action='store_true', help='確認なしで実行')
    parser.add_argument('--dry-run', action='store_true', help='実行内容だけ表示')
    return parser


def _show_bags(candidates: list[tuple[Path, dict]], show_all: bool = False) -> None:
    visible = candidates if show_all else [item for item in candidates if item[1]['supported']]
    print('見つかったROS bag:')
    if not visible:
        print('  ありません。SSDがマウントされているか確認してください。')
        return
    for index, (bag, summary) in enumerate(visible, 1):
        support = '実行可能' if summary['supported'] else '未対応センサー'
        print(f"  {index}. {bag.name:<22} {summary['duration_sec']:7.1f}秒  {support}")
        print(f'     {bag}')
    hidden = len(candidates) - len(visible)
    if hidden:
        print(f'\n  未対応センサーのbag {hidden}件は非表示です（--all で表示）。')


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    roots = args.scan_root or list(DEFAULT_SCAN_ROOTS)
    candidates = discover_bags(roots)
    if args.list or not args.bag:
        _show_bags(candidates, show_all=args.all)
        if not args.bag and not args.list:
            print('\n実行例: ./scripts/run_lidar_slam.py exp01')
        return 0
    try:
        bag = resolve_bag(args.bag, candidates)
    except ValueError as exc:
        print(f'エラー: {exc}', file=sys.stderr)
        return 2
    summary = read_bag_summary(bag)
    if not summary['supported']:
        print('エラー: このbagには対応済みの /hesai/pandar がありません。', file=sys.stderr)
        print('現在のランチャーはHILTI Pandarデータに対応しています。', file=sys.stderr)
        return 2
    if args.rate <= 0:
        print('エラー: --rate は0より大きくしてください。', file=sys.stderr)
        return 2
    try:
        requested_duration = (summary['duration_sec'] if args.duration == 'full'
                              else min(summary['duration_sec'], float(args.duration)))
    except ValueError:
        print('エラー: --duration は秒数または full を指定してください。', file=sys.stderr)
        return 2
    if requested_duration <= 0:
        print('エラー: --duration は0より大きくしてください。', file=sys.stderr)
        return 2
    expected_sec = requested_duration / args.rate + 11.0
    out = (args.output_dir or default_output_dir(bag)).expanduser().resolve()
    name = bag.name.removesuffix('_ros2')
    map_path = out / f'{name}_map.ply'
    print('=== LiDAR SLAM かんたん実行 ===')
    print(f'入力      : {bag}')
    print(f'収録時間  : {summary["duration_sec"]:.1f} 秒')
    print(f'出力      : {map_path}')
    print(f'空き容量  : {_free_gib(out):.1f} GiB')
    print(f'再生速度  : {args.rate}x')
    print(f'完了予想  : 約{_clock(expected_sec)}（起動・保存時間を含む）')
    if map_path.exists() and not args.force:
        print(f'\n既存の地図を再利用します: {map_path} ({map_path.stat().st_size / 1e6:.1f} MB)')
        print('作り直す場合は --force を付けてください。')
        return 0
    command = ['bash', str(HILTI_RUNNER), str(bag), name, str(args.duration), str(args.rate)]
    print('\n実行内容: rosbagを再生し、SLAM点群をPLYへ保存します。')
    print('コマンド: ' + ' '.join(command))
    if args.dry_run:
        print('\ndry-runのため実行していません。')
        return 0
    if not args.yes and sys.stdin.isatty():
        if input('\n開始しますか？ [Y/n] ').strip().lower() not in {'', 'y', 'yes'}:
            print('キャンセルしました。')
            return 0
    out.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env['HILTI_SLAM_OUT'] = str(out)
    returncode, interrupted = _run_with_progress(
        command, env, expected_sec, out / f'{name}_slam.log')
    if returncode and not interrupted:
        print(f'\nSLAMに失敗しました（終了コード {returncode}）。', file=sys.stderr)
        print(f'ログ: {out / (name + "_slam.log")}', file=sys.stderr)
        return returncode
    if interrupted:
        if map_path.is_file():
            print(f'途中地図を保存しました: {map_path} ({map_path.stat().st_size / 1e6:.1f} MB)')
        else:
            print('地図が作られる前に停止しました。', file=sys.stderr)
        return 130
    if not map_path.is_file():
        print(f'エラー: 処理は終了しましたが地図がありません: {map_path}', file=sys.stderr)
        return 1
    print(f'\n完了しました: {map_path} ({map_path.stat().st_size / 1e6:.1f} MB)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
