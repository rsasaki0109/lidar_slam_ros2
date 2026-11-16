"""The lidarslam-map command of the pip package.

The package carries the product scripts in the tree the ROS 2 package installs under
share/lidarslam (product/scripts, product/schemas, param), so this command runs the same
lidarslam_cli.py with this interpreter. Without ROS 2 the map commands use the core engine
(run|start --engine core, the default when the ros2 command is not found).
"""

from __future__ import annotations

import os
from pathlib import Path
import sys

SHARE_DIR = Path(__file__).resolve().parent / 'share' / 'lidarslam'
CLI_PATH = SHARE_DIR / 'product' / 'scripts' / 'lidarslam_cli.py'


def main(argv: list[str] | None = None) -> int:
    if not CLI_PATH.is_file():
        print(
            f'error: the lidarslam-map product files are missing from this install ({CLI_PATH})',
            file=sys.stderr)
        return 70
    args = sys.argv[1:] if argv is None else list(argv)
    os.execv(sys.executable, [sys.executable, str(CLI_PATH), *args])
    return 70  # not reached


if __name__ == '__main__':
    raise SystemExit(main())
