#!/usr/bin/env bash
set -uo pipefail

# ============================================================
# Prepare Autoware Map from graph_based_slam Output
# ============================================================
# Converts SLAM output (grid PCDs + pose_graph.g2o) into an
# Autoware-compatible map directory with:
#   - pointcloud_map.pcd (single merged, coordinate-transformed)
#   - lanelet2_map.osm (auto-generated from trajectory)
#   - map_projector_info.yaml
#
# Usage:
#   bash scripts/prepare_autoware_map_from_graph_slam.sh \
#     --pcd-dir ./pointcloud_map \
#     --g2o ./pose_graph.g2o \
#     --output ./autoware_map
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    cat <<'EOF'
Usage: bash scripts/prepare_autoware_map_from_graph_slam.sh [options]

Required:
  --pcd-dir DIR       Directory containing grid-divided PCD files
  --g2o FILE          Pose graph file (VERTEX_SE3:QUAT format)
  --output DIR        Output Autoware map directory

Optional:
  --lane-width FLOAT  Lane width in metres (default: 7.0)
  --resolution FLOAT  Lanelet2 resampling resolution (default: 2.0)
  -h, --help          Show this help
EOF
}

PCD_DIR=""
G2O_FILE=""
OUTPUT_DIR=""
LANE_WIDTH=7.0
RESOLUTION=2.0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --pcd-dir)      PCD_DIR="$2"; shift 2 ;;
        --g2o)          G2O_FILE="$2"; shift 2 ;;
        --output)       OUTPUT_DIR="$2"; shift 2 ;;
        --lane-width)   LANE_WIDTH="$2"; shift 2 ;;
        --resolution)   RESOLUTION="$2"; shift 2 ;;
        -h|--help)      usage; exit 0 ;;
        *)              echo "Unknown option: $1" >&2; usage; exit 1 ;;
    esac
done

[ -z "${PCD_DIR}" ] && { echo "ERROR: --pcd-dir required" >&2; exit 1; }
[ -z "${G2O_FILE}" ] && { echo "ERROR: --g2o required" >&2; exit 1; }
[ -z "${OUTPUT_DIR}" ] && { echo "ERROR: --output required" >&2; exit 1; }
[ -d "${PCD_DIR}" ] || { echo "ERROR: PCD dir not found: ${PCD_DIR}" >&2; exit 1; }
[ -f "${G2O_FILE}" ] || { echo "ERROR: G2O file not found: ${G2O_FILE}" >&2; exit 1; }

mkdir -p "${OUTPUT_DIR}"

echo "=== Prepare Autoware Map ==="
echo "  PCD dir:      ${PCD_DIR}"
echo "  G2O file:     ${G2O_FILE}"
echo "  Output:       ${OUTPUT_DIR}"
echo "  Lane width:   ${LANE_WIDTH}m"
echo ""

# --- Step 1: Extract trajectory from g2o ---
echo "[1/4] Extracting trajectory from pose graph..."
TRAJ_FILE="${OUTPUT_DIR}/traj_slam.tum"

python3 - "${G2O_FILE}" "${TRAJ_FILE}" << 'PYEOF'
import sys
import numpy as np
from pathlib import Path

g2o_path, tum_path = Path(sys.argv[1]), Path(sys.argv[2])

vertices = []
for line in g2o_path.read_text().splitlines():
    if line.startswith('VERTEX_SE3:QUAT'):
        p = line.split()
        vertices.append([float(v) for v in p[1:9]])

arr = np.array(vertices)
dist = np.sum(np.linalg.norm(np.diff(arr[:, 1:4], axis=0), axis=1))
print(f"  {len(arr)} vertices, distance: {dist:.1f}m")

np.savetxt(str(tum_path), arr, fmt='%.10f')
print(f"  Saved {tum_path}")
PYEOF

# --- Step 2: Merge and transform PCD ---
echo "[2/4] Merging and transforming PCD files..."

python3 - "${PCD_DIR}" "${OUTPUT_DIR}/pointcloud_map.pcd" << 'PYEOF'
import struct, sys
import numpy as np
from pathlib import Path

try:
    import lzf
except ImportError:
    import subprocess
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', '--user', '--break-system-packages', '-q', 'python-lzf'])
    import lzf

pcd_dir, out_path = Path(sys.argv[1]), Path(sys.argv[2])

pcd_files = sorted([f for f in pcd_dir.glob('*.pcd') if f.name != 'pointcloud_map.pcd'])
print(f"  {len(pcd_files)} PCD files")

all_points = []
for pf in pcd_files:
    with open(pf, 'rb') as f:
        n_points, data_type = 0, ''
        while True:
            line = f.readline().decode('ascii', errors='replace').strip()
            if line.startswith('POINTS'): n_points = int(line.split()[1])
            if line.startswith('DATA'): data_type = line; break
        if n_points == 0: continue
        if 'binary_compressed' in data_type:
            cs = struct.unpack('I', f.read(4))[0]
            us = struct.unpack('I', f.read(4))[0]
            raw = lzf.decompress(f.read(cs), us)
            fs = n_points * 4
            all_points.append(np.column_stack([
                np.frombuffer(raw[0:fs], dtype=np.float32),
                np.frombuffer(raw[fs:2*fs], dtype=np.float32),
                np.frombuffer(raw[2*fs:3*fs], dtype=np.float32),
            ]))

pts = np.vstack(all_points)
print(f"  {len(pts)} total points")

n = len(pts)
with open(out_path, 'wb') as f:
    hdr = (f"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
           f"FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
           f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
           f"POINTS {n}\nDATA binary\n")
    f.write(hdr.encode())
    f.write(pts.astype(np.float32).tobytes())

import os
print(f"  Wrote {out_path} ({os.path.getsize(out_path)/1024/1024:.1f} MB)")
PYEOF

# --- Step 3: Generate lanelet2 ---
echo "[3/4] Generating lanelet2 map..."

ORIGIN_LAT=0.0
ORIGIN_LON=0.0
echo "  Origin: lat=${ORIGIN_LAT}, lon=${ORIGIN_LON}"

python3 "${SCRIPT_DIR}/simple_lanelet2_generator.py" \
    --input "${OUTPUT_DIR}/traj_slam.tum" \
    --output "${OUTPUT_DIR}/lanelet2_map.osm" \
    --lane-width "${LANE_WIDTH}" \
    --origin-lat "${ORIGIN_LAT}" \
    --origin-lon "${ORIGIN_LON}" \
    --resolution "${RESOLUTION}"

# --- Step 4: Generate map_projector_info.yaml ---
echo "[4/4] Writing map_projector_info.yaml..."
cat > "${OUTPUT_DIR}/map_projector_info.yaml" << 'YAML'
projector_type: local
YAML

echo ""
echo "=== Done ==="
ls -lh "${OUTPUT_DIR}/"
echo ""
echo "To use: load ${OUTPUT_DIR}/ in Autoware map loaders"
