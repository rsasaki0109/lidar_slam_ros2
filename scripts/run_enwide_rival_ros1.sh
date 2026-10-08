#!/usr/bin/env bash
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Run FAST-LIO2 or Point-LIO on one ENWIDE sequence and score it like the profile.
#
# usage: run_enwide_rival_ros1.sh fast_lio|point_lio SEQUENCE_DIR OUTPUT_DIR
#
# SEQUENCE_DIR is a scripts/download_enwide.sh directory with the ROS 1 bag.
# Build the image first:
#   docker build -f docker/enwide_rivals_ros1.Dockerfile -t enwide-rivals-ros1 docker
# The bag is played once in real time, so a run is not repeatable bit for bit.
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
[[ $# -eq 3 ]] || { sed -n '/^# Run FAST-LIO2/,/^# The bag is played/p' "$0" >&2; exit 2; }
METHOD=$1
SEQUENCE_DIR=$(realpath "$2")
OUTPUT_DIR=$(realpath -m "$3")
SEQUENCE=$(basename "${SEQUENCE_DIR}")
case "${METHOD}" in
  fast_lio) PACKAGE_DIR=FAST_LIO; TOPIC=/Odometry ;;
  point_lio) PACKAGE_DIR=Point-LIO; TOPIC=/aft_mapped_to_init ;;
  *) echo "unknown method: ${METHOD}" >&2; exit 2 ;;
esac
BAG=$(find "${SEQUENCE_DIR}" -maxdepth 1 -name '*.bag' | head -n 1)
GT="${SEQUENCE_DIR}/gt-${SEQUENCE}.csv"
CONFIG="${REPO_ROOT}/configs/enwide/rivals/${METHOD}_os0.yaml"
[[ -f "${BAG}" && -f "${GT}" ]] || { echo "bag or ground truth missing in ${SEQUENCE_DIR}" >&2; exit 2; }
[[ ! -e "${OUTPUT_DIR}" ]] || { echo "output already exists: ${OUTPUT_DIR}" >&2; exit 2; }
mkdir -p "${OUTPUT_DIR}"

# Inside the container: start the method, record its odometry, play the bag once.
/usr/bin/time -v -o "${OUTPUT_DIR}/time.txt" docker run --rm \
  -v "${BAG}:/data/in.bag:ro" \
  -v "${CONFIG}:/root/rivals_ws/src/${PACKAGE_DIR}/config/ouster64.yaml:ro" \
  -v "${OUTPUT_DIR}:/out" \
  enwide-rivals-ros1 bash -c "
    source /root/rivals_ws/devel/setup.bash
    roscore > /out/roscore.log 2>&1 &
    sleep 5
    rosbag record -O /out/odom.bag ${TOPIC} > /out/record.log 2>&1 &
    record=\$!
    roslaunch ${METHOD} mapping_ouster64.launch rviz:=false > /out/method.log 2>&1 &
    launch=\$!
    sleep 8
    rosbag play -q /data/in.bag > /out/play.log 2>&1
    sleep 10
    kill -INT \$record; sleep 5
    kill -INT \$launch; sleep 5
    rostopic echo -b /out/odom.bag -p ${TOPIC} > /out/odom.csv" > "${OUTPUT_DIR}/docker.log" 2>&1

# Odometry (IMU frame) to TUM, then to the prism and the profile's position score.
python3 - "${OUTPUT_DIR}" <<'PY'
import csv
from pathlib import Path
import sys

out = Path(sys.argv[1])
rows = list(csv.reader((out / 'odom.csv').open()))
header = rows[0]
columns = ['field.header.stamp'] + [f'field.pose.pose.position.{a}' for a in 'xyz'] + [
    f'field.pose.pose.orientation.{a}' for a in 'xyzw']
with (out / 'traj_raw.tum').open('w') as stream:
    for row in rows[1:]:
        values = [float(row[header.index(column)]) for column in columns]
        values[0] *= 1e-9
        stream.write(' '.join(f'{value:.9f}' for value in values) + '\n')
PY
OFFSET=$(python3 -c "import json,sys; o=json.load(open(sys.argv[1]))['base_to_prism_translation_m']; print(o['x'], o['y'], o['z'])" \
  "${REPO_ROOT}/configs/enwide/os_imu_to_prism.json")
read -r TX TY TZ <<<"${OFFSET}"
python3 "${SCRIPT_DIR}/apply_tum_frame_offset.py" --in "${OUTPUT_DIR}/traj_raw.tum" \
  --out "${OUTPUT_DIR}/traj_raw_prism.tum" --tx "${TX}" --ty "${TY}" --tz "${TZ}"
python3 "${SCRIPT_DIR}/score_position_only_trajectory.py" --reference "${GT}" \
  --estimate "${OUTPUT_DIR}/traj_raw_prism.tum" --output "${OUTPUT_DIR}/position_score.json" \
  --max-time-gap 0.11
