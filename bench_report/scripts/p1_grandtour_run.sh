#!/usr/bin/env bash
# p1_grandtour_run.sh <hf_mission_folder> <short_name> <label> [ws] (chapter 31): one GrandTour mission end to end with
# the global configuration G (G_CONFIG, default configs/sota_v7/rko_lio_global_G12_2026-10-26.yaml; the COMFORT test
# batch of 2026-10-26 ran the G11 file rko_lio_global_G_2026-10-26.yaml with the rko_lio 7a1f45e build).
#   1. download only livox_points.tar and livox_imu.tar from the Hugging Face export (no reference or GNSS stream),
#   2. convert them to a ROS 2 bag (p1_grandtour_to_ros2.py),
#   3. write the mission's RKO-LIO parameters: G plus the rig keys (IMU frame as base, LiDAR extrinsic from the export),
#   4. run the RKO-LIO offline node once,
#   5. write <out>/<short_name>.tum as T_world_prism(t) = T_world_imu(t) T_imu_prism, the benchmark's required frame
#      (box_base -> prism from grand_tour_box's tf_static_metadata_new_*.bag, the same in all nine of them).
# Outputs go to /home/user/bench/grandtour/<label>/<short_name>/.
set -euo pipefail
folder=$1; short=$2; label=$3; ws=${4:-/home/user/ws/rko_g12}
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
D=/home/user/data/grandtour/${short}; O=/home/user/bench/grandtour/${label}/${short}
PRISM="0.3088378015503127 -0.038312997211241984 -0.2349441159711169 -0.005724388155828644 -0.0013879431347362404 -0.004742721620119043 0.9999714053840392"
U=https://huggingface.co/datasets/leggedrobotics/grand_tour_dataset/resolve/main/${folder}/data
mkdir -p "${D}/z" "${O}"
set +u; source /opt/ros/jazzy/setup.bash; set -u
if [[ ! -d ${D}/bag ]]; then
  for f in livox_points livox_imu; do
    [[ -d ${D}/z/${f} ]] || { curl -sS -L --fail -o "${D}/${f}.tar" "${U}/${f}.tar"; tar xf "${D}/${f}.tar" -C "${D}/z"; rm -f "${D}/${f}.tar"; }
  done
  "${PY}" "${S}/p1_grandtour_to_ros2.py" "${D}/z" "${D}/bag" | tee "${D}/convert.log"
fi
# shellcheck disable=SC2086
"${PY}" "${S}/p1_grandtour_to_ros2.py" --extrinsics "${D}/z" ${PRISM} > "${O}/extrinsics.txt"
"${PY}" - "${G_CONFIG:-${REPO}/configs/sota_v7/rko_lio_global_G12_2026-10-26.yaml}" "${O}/extrinsics.txt" "${O}/params.yaml" <<'PY'
import sys
import yaml
g = yaml.safe_load(open(sys.argv[1]))['/**']['ros__parameters']
ext = {k: v for k, v in (yaml.safe_load(open(sys.argv[2])) or {}).items() if k.startswith('extrinsic_')}
rig = {'base_frame': 'base_link', 'odom_frame': 'odom', 'lidar_frame': '', 'imu_frame': '', 'odom_topic': '/rko_lio/odometry',
       'publish_deskewed_scan': False, 'dump_results': True, 'run_name': 'front', 'imu_acceleration_unit': 'auto'}
params = {**rig, **ext, **g}
yaml.safe_dump({'/**': {'ros__parameters': params}}, open(sys.argv[3], 'w'), sort_keys=False)
PY
RUN_TIMEOUT=7200 bash /home/user/scripts/run_front.sh "${label}_${short}" "${ws}" "${O}/params.yaml" "${D}/bag" /livox/points \
  /livox/imu "${O}/run" /home/user/bench/grandtour/runs.csv > /dev/null
"${PY}" - "${O}/run/front_0/front_tum_0.txt" "${O}/extrinsics.txt" "${O}/${short}.tum" <<'PY'
import sys
import numpy as np
import yaml
sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_to_ros2 import R_to_quat, matrix  # noqa: E402
est = np.loadtxt(sys.argv[1])
q_t = yaml.safe_load(open(sys.argv[2]))['T_imu_prism_quat_xyzw_xyz']
T_imu_prism = matrix(q_t[4:], q_t[:4])
with open(sys.argv[3], 'w') as f:
    f.write('# timestamp x y z qx qy qz qw  (T_world_prism)\n')
    for row in est:
        T = matrix(row[1:4], row[4:8]) @ T_imu_prism
        q = R_to_quat(T[:3, :3])
        f.write(f'{row[0]:.9f} {T[0, 3]:.6f} {T[1, 3]:.6f} {T[2, 3]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n')
print(f'{len(est)} poses -> {sys.argv[3]}')
PY
grep -o "Close-scene settings used on.*" "${O}/run/stdout.log" || true
