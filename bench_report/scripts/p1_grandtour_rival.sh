#!/usr/bin/env bash
# p1_grandtour_rival.sh <rival: bievr> <hf_mission_folder> <short_name> <label> (chapter 33): one GrandTour development
# mission with a published rival, Hesai XT-32 + STIM320, the rival's own parameters, only the sensor settings changed.
#   1. Hesai points and the STIM320 into a ROS 2 bag (per-point time field `time`, float32 s since the scan start),
#   2. the rival's sensor config from the export's extrinsics (LiDAR -> IMU),
#   3. one offline run, then <out>/<short>.tum as T_world_prism(t) = T_world_imu(t) T_imu_prism.
# Outputs go to /home/user/bench/grandtour/<label>/<short_name>/.
set -euo pipefail
rival=$1; folder=$2; short=$3; label=$4
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
D=/home/user/data/grandtour/${short}; O=/home/user/bench/grandtour/${label}/${short}; BAG=${D}/bag_rival_hesai_stim
PRISM="0.3088378015503127 -0.038312997211241984 -0.2349441159711169 -0.005724388155828644 -0.0013879431347362404 -0.004742721620119043 0.9999714053840392"
U=https://huggingface.co/datasets/leggedrobotics/grand_tour_dataset/resolve/main/${folder}/data
mkdir -p "${D}/z" "${O}"
set +u; source /opt/ros/jazzy/setup.bash; set -u
if [[ ! -d ${BAG} ]]; then
  for f in hesai_points stim320_imu; do
    [[ -d ${D}/z/${f} ]] || curl -sS -L --fail "${U}/${f}.tar" | tar xf - -C "${D}/z"
  done
  "${PY}" "${S}/p1_grandtour_to_ros2.py" --imu stim320_imu --lidar hesai_points --time-field time "${D}/z" "${BAG}" \
    | tee "${D}/convert_rival.log"
fi
# shellcheck disable=SC2086
"${PY}" "${S}/p1_grandtour_to_ros2.py" --imu stim320_imu --lidar hesai_points --extrinsics "${D}/z" ${PRISM} \
  > "${O}/extrinsics.txt"
case ${rival} in
  bievr)
    BW=/home/user/ws/bievr; SRC=/home/user/rivals/BIEVR-LIO
    "${PY}" - "${O}/extrinsics.txt" "${SRC}/config/params.yaml" "${O}" <<'PY'
import sys
import numpy as np
import yaml
sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_to_ros2 import quat_to_R  # noqa: E402
ext = yaml.safe_load(open(sys.argv[1]))
q = ext['extrinsic_lidar2base_quat_xyzw_xyz']
R = quat_to_R(q[:4])
sensor = {'topics': {'pointcloud': '/hesai_points', 'imu': '/stim320_imu'},
          'calibration': {'translation': [float(v) for v in q[4:]], 'rotation': [float(v) for v in R.reshape(-1)]},
          'lidar': {'min_range_m': 0.5, 'max_range_m': 100}}
yaml.safe_dump(sensor, open(f'{sys.argv[3]}/sensor.yaml', 'w'), sort_keys=False)
params = yaml.safe_load(open(sys.argv[2]))
params['debug']['trajectory_path'] = f'{sys.argv[3]}/run_tum.txt'
params['debug']['dashboard'] = False
yaml.safe_dump(params, open(f'{sys.argv[3]}/params.yaml', 'w'), sort_keys=False)
PY
    set +u; source "${BW}/install/setup.bash"; set -u
    t0=$(date +%s.%N)
    timeout -s INT 7200 "${BW}/install/bievr_lio_ros2/lib/bievr_lio_ros2/process_bag" --sensor_config_file "${O}/sensor.yaml" \
      --params_file "${O}/params.yaml" --bag "${BAG}" > "${O}/stdout.log" 2>&1 || echo "process_bag exit $?" >> "${O}/stdout.log"
    t1=$(date +%s.%N)
    echo "wall_seconds $(python3 -c "print(f'{${t1}-${t0}:.1f}')")" > "${O}/timing.txt"
    ;;
  *) echo "unknown rival ${rival}"; exit 1 ;;
esac
"${PY}" - "${O}/run_tum.txt" "${O}/extrinsics.txt" "${O}/${short}.tum" <<'PY'
import sys
import numpy as np
import yaml
sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_to_ros2 import R_to_quat, matrix  # noqa: E402
est = np.loadtxt(sys.argv[1], comments='#')
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
