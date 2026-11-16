"""python -m lidarslam inspect <log>: list a sensor log's topics without ROS 2."""

from __future__ import annotations

import argparse
import sys

from lidarslam.io import SUPPORTED_TYPES, SensorLog


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog='lidarslam')
    sub = parser.add_subparsers(dest='command', required=True)
    inspect = sub.add_parser('inspect', help='list topics, message types and counts of a log')
    inspect.add_argument('log', help='rosbag2 directory or ROS 1 .bag file')
    args = parser.parse_args(argv)

    if args.command == 'inspect':
        with SensorLog(args.log) as log:
            for topic in log.topics():
                mark = '*' if topic.msgtype in SUPPORTED_TYPES else ' '
                print(f'{mark} {topic.name}  {topic.msgtype}  {topic.count}')
        print('* = read by the mapping pipeline')
    return 0


if __name__ == '__main__':
    sys.exit(main())
