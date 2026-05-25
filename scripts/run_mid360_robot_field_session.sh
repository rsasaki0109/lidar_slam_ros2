#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

exec python3 "${SCRIPT_DIR}/run_mid360_robot_field_session.py" "$@"
