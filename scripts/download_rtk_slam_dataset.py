#!/usr/bin/env python3

"""
Download the public RTK-SLAM dataset (Livox MID-360 + total-station GT).

RTK-SLAM Dataset, Univ. Stuttgart ifp (arXiv:2604.07151, CC-BY 4.0):
huggingface.co/datasets/Willyzw/rtk-slam-dataset. The bags are large
(ROS2 .db3 from ~10 to ~30 GB per sequence), so fetch one sequence at a time.

The ground-truth checkpoints are NOT in the dataset repo: they live in the
small eval repo (github.com/Willyzw/rtk-slam-eval) as
``ground_truth/<sequence>.csv``, alongside example SLAM trajectories. Use
``--eval-assets`` to clone that repo (a few MB) — it is all that is needed to
build a reference with ``generate_rtk_slam_reference.py``; the big bag is only
needed to run our own SLAM and produce the estimate trajectory.

Attribution (CC-BY 4.0): cite Zhang, Ress, Skuddis, Soergel, Haala, "An
RTK-SLAM Dataset for Absolute Accuracy Evaluation in GNSS-Degraded
Environments", arXiv:2604.07151.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

HF_REPO = 'Willyzw/rtk-slam-dataset'
HF_RESOLVE = f'https://huggingface.co/datasets/{HF_REPO}/resolve/main'
EVAL_REPO_URL = 'https://github.com/Willyzw/rtk-slam-eval.git'

# Per-sequence ROS2 payloads (path under the repo -> expected size in bytes).
# Sizes are from the HuggingFace tree API and double as a post-download check.
SEQUENCES = {
    'construction_seq2': 10656124000,
    'construction_seq1': 13180900000,
    'stadtgarten_seq2': 16793700000,
    'stadtgarten_seq1': 30263600000,
}


def _human_gb(num_bytes: int) -> str:
    return f'{num_bytes / 1e9:.1f} GB'


def _download(url: str, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    # wget -c is resumable, which matters for multi-GB LFS payloads.
    cmd = ['wget', '-c', '-O', str(dest), url]
    print(f'  {url}\n    -> {dest}')
    subprocess.run(cmd, check=True)


def fetch_sequence(sequence: str, dest_root: Path, size_check: bool = True) -> None:
    """Download one ROS2 sequence (.db3 + metadata.yaml) into dest_root/ros2/<seq>."""
    if sequence not in SEQUENCES:
        raise SystemExit(f'unknown sequence: {sequence} (choices: {list(SEQUENCES)})')
    expected = SEQUENCES[sequence]
    print(f'[{sequence}] ~{_human_gb(expected)} (ROS2 .db3)')
    seq_dir = dest_root / 'ros2' / sequence
    db3 = seq_dir / f'{sequence}.db3'
    _download(f'{HF_RESOLVE}/ros2/{sequence}/{sequence}.db3', db3)
    _download(f'{HF_RESOLVE}/ros2/{sequence}/metadata.yaml', seq_dir / 'metadata.yaml')
    if size_check and db3.is_file():
        actual = db3.stat().st_size
        # Registry sizes are rounded; allow a 1% band rather than an exact match.
        if abs(actual - expected) > max(expected * 0.01, 1_000_000):
            print(
                f'  WARNING: {db3.name} is {_human_gb(actual)}, '
                f'expected ~{_human_gb(expected)}',
                file=sys.stderr,
            )


def fetch_eval_assets(dest_root: Path) -> None:
    """Clone the eval repo (ground-truth checkpoints + example trajectories)."""
    target = dest_root / 'rtk_slam_eval'
    if (target / '.git').is_dir():
        print(f'[eval-assets] already present: {target}')
        return
    print(f'[eval-assets] cloning {EVAL_REPO_URL} -> {target}')
    subprocess.run(
        ['git', 'clone', '--depth', '1', EVAL_REPO_URL, str(target)],
        check=True,
    )


def main() -> int:
    """CLI entry point for fetching RTK-SLAM bags and/or eval assets."""
    parser = argparse.ArgumentParser(
        description='Download the public RTK-SLAM dataset (bags and/or eval assets).',
    )
    parser.add_argument(
        '--sequence',
        default='construction_seq2',
        help="ROS2 sequence to fetch, or 'all'. Smallest is construction_seq2 "
             '(~10.7 GB). Default: construction_seq2.',
    )
    parser.add_argument(
        '--dest',
        default='datasets/rtk_slam',
        help='Destination root (default: datasets/rtk_slam, which is gitignored)',
    )
    parser.add_argument(
        '--eval-assets',
        action='store_true',
        help='Also clone the eval repo (ground-truth CSVs + example trajectories)',
    )
    parser.add_argument(
        '--eval-assets-only',
        action='store_true',
        help='Clone only the eval assets; skip the large bag download',
    )
    parser.add_argument(
        '--list',
        action='store_true',
        help='List the available sequences and their sizes, then exit',
    )
    args = parser.parse_args()

    if args.list:
        for name, size in SEQUENCES.items():
            print(f'{name:20s} ~{_human_gb(size)}')
        return 0

    dest_root = Path(args.dest).expanduser().resolve()

    if args.eval_assets_only:
        fetch_eval_assets(dest_root)
        return 0

    sequences = list(SEQUENCES) if args.sequence == 'all' else [args.sequence]
    for sequence in sequences:
        fetch_sequence(sequence, dest_root)

    if args.eval_assets:
        fetch_eval_assets(dest_root)

    print('done. Ground truth + example trajectories come from --eval-assets; '
          'build a reference with scripts/generate_rtk_slam_reference.py.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
