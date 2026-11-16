#!/usr/bin/env bash
# p8_blind_method.sh <method> <rig.yaml> <bag> <out_dir> (bench_report/data/blind_v7_run2_2026-11-10): one run of one
# method on one converted bag; p5_blind_method.sh of the first blind run with the rig's own FAST-LIO2 sensor settings.
# <method> is G (RKO-LIO with the frozen G22, this repository's build), rko_upstream (PyPI rko_lio 0.4.0), bievr
# (BIEVR-LIO 2112169), fastlio (FAST-LIO2 cc3c9e8, bag played at rate 1.0) or glim (GLIM v1.2.2, CPU build).
# <rig.yaml> (p8_make_rigs.py) holds lidar_topic, imu_topic, lidar_to_imu_quat_xyzw_xyz (the LiDAR pose in the IMU
# frame), g_params (G's complete parameter file), fastlio_config (the published FAST-LIO2 file to start from) and
# fastlio_preprocess (its sensor settings: lidar_type, scan_line, timestamp_unit, blind).
# Writes <out>/traj_imu.tum and <out>/traj.tum (both T_world_imu), <out>/time.txt (/usr/bin/time -v of the method's
# process), <out>/exit.txt and <out>/stdout.log.
# G and FAST-LIO2 runs hold the exclusive VM lock /home/user/bench/blind2/quiet.lock; the others hold it shared.
set -uo pipefail
method=$1; rig=$(realpath "$2"); bag=$(realpath "$3"); out=$(realpath -m "$4")
REPO=/home/user/lidar_slam_ros2; PY=/home/user/mapshd/venv/bin/python
rm -rf "${out}"; mkdir -p "${out}"
set +u; source /opt/ros/jazzy/setup.bash; set -u
export ROS_LOG_DIR="${out}/roslog" ROS_HOME="${out}/ros_home"
export ROS_DOMAIN_ID=$((40 + RANDOM % 50)) ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
rv() { "${PY}" -c "import sys, yaml; v = yaml.safe_load(open(sys.argv[1])).get(sys.argv[2]); print(v if v is not None else '')" "${rig}" "$1"; }
LIDAR=$(rv lidar_topic); IMU=$(rv imu_topic)
TIME=(/usr/bin/time -v -o "${out}/time.txt")
case ${method} in G | fastlio) lockmode=-x ;; *) lockmode=-s ;; esac
mkdir -p /home/user/bench/blind2; exec 8>/home/user/bench/blind2/quiet.lock; flock ${lockmode} 8
echo "start_utc $(date -u +%Y-%m-%dT%H:%M:%SZ) load $(cut -d' ' -f1-3 /proc/loadavg)" > "${out}/run_info.txt"
t0=$(date +%s.%N)
case ${method} in
  G)
    "${TIME[@]}" timeout -s INT 7200 "${RKO_WS:-${REPO}}/install/rko_lio/lib/rko_lio/offline_node" --ros-args \
      --params-file "$(rv g_params)" -p bag_path:="${bag}" -p lidar_topic:="${LIDAR}" -p imu_topic:="${IMU}" \
      -p results_dir:="${out}" > "${out}/stdout.log" 2>&1 8>&-
    rc=$?
    cp "${out}/front_0/front_tum_0.txt" "${out}/traj_imu.tum" 2>/dev/null
    ;;
  rko_upstream)
    "${PY}" - "${rig}" ${REPO}/bench_report/data/blind_v7_2026-11-08/rko_upstream_default.yaml "${out}/config.yaml" <<'PY'
import sys
import yaml
rig = yaml.safe_load(open(sys.argv[1]))
cfg = yaml.safe_load(open(sys.argv[2]))
cfg['extrinsic_imu2base_quat_xyzw_xyz'] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
cfg['extrinsic_lidar2base_quat_xyzw_xyz'] = [float(v) for v in rig['lidar_to_imu_quat_xyzw_xyz']]
yaml.safe_dump(cfg, open(sys.argv[3], 'w'), sort_keys=False)
PY
    (cd "${out}" && "${TIME[@]}" timeout -s INT 7200 /home/user/venvs/rko_upstream/bin/rko_lio -d rosbag \
      --imu "${IMU}" --lidar "${LIDAR}" -c "${out}/config.yaml" -o "${out}/results" -n run "${bag}" \
      > "${out}/stdout.log" 2>&1 8>&-)
    rc=$?
    f=$(find "${out}/results" -name '*.txt' -newer "${out}/config.yaml" 2>/dev/null | head -1)
    [[ -n ${f} ]] && grep -v '^#' "${f}" > "${out}/traj_imu.tum"
    ;;
  bievr)
    BW=/home/user/ws/bievr; SRC=/home/user/rivals/BIEVR-LIO
    "${PY}" - "${rig}" "${SRC}/config/params.yaml" "${out}" <<'PY'
import sys
import numpy as np
import yaml
sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_to_ros2 import quat_to_R  # noqa: E402
rig = yaml.safe_load(open(sys.argv[1]))
q = rig['lidar_to_imu_quat_xyzw_xyz']
sensor = {'topics': {'pointcloud': rig['lidar_topic'], 'imu': rig['imu_topic']},
          'calibration': {'translation': [float(v) for v in q[4:]],
                          'rotation': [float(v) for v in np.asarray(quat_to_R(q[:4])).reshape(-1)]},
          'lidar': {'min_range_m': 0.5, 'max_range_m': 100}}
yaml.safe_dump(sensor, open(f'{sys.argv[3]}/sensor.yaml', 'w'), sort_keys=False)
params = yaml.safe_load(open(sys.argv[2]))
params['debug']['trajectory_path'] = f'{sys.argv[3]}/run_tum.txt'
params['debug']['dashboard'] = False
yaml.safe_dump(params, open(f'{sys.argv[3]}/params.yaml', 'w'), sort_keys=False)
PY
    set +u; source "${BW}/install/setup.bash"; set -u
    "${TIME[@]}" timeout -s INT 7200 "${BW}/install/bievr_lio_ros2/lib/bievr_lio_ros2/process_bag" \
      --sensor_config_file "${out}/sensor.yaml" --params_file "${out}/params.yaml" --bag "${bag}" \
      > "${out}/stdout.log" 2>&1 8>&-
    rc=$?
    [[ -f ${out}/run_tum.txt ]] && grep -v '^#' "${out}/run_tum.txt" > "${out}/traj_imu.tum"
    ;;
  fastlio)
    FW=/home/user/ws/fastlio; SRC=/home/user/rivals/FAST_LIO_ROS2
    # the default DDS transports lost 28 % of the LiDAR scans on exp04 (best-effort 2 MB messages); shared memory only
    export FASTRTPS_DEFAULT_PROFILES_FILE=${REPO}/bench_report/data/blind_v7_2026-11-08/fastdds_shm.xml
    "${PY}" - "${rig}" "${SRC}/config/$(rv fastlio_config)" "${out}" <<'PY'
import sys
import numpy as np
import yaml
sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_to_ros2 import quat_to_R  # noqa: E402
rig = yaml.safe_load(open(sys.argv[1]))
q = rig['lidar_to_imu_quat_xyzw_xyz']
cfg = yaml.safe_load(open(sys.argv[2]))
p = cfg['/**']['ros__parameters']
p['common']['lid_topic'] = rig['lidar_topic']
p['common']['imu_topic'] = rig['imu_topic']
p['preprocess'].update(rig['fastlio_preprocess'])
p['mapping']['extrinsic_T'] = [float(v) for v in q[4:]]
p['mapping']['extrinsic_R'] = [float(v) for v in np.asarray(quat_to_R(q[:4])).reshape(-1)]
p['pcd_save']['pcd_save_en'] = False
p['publish'].update({'path_en': False, 'scan_publish_en': False, 'dense_publish_en': False,
                     'scan_bodyframe_pub_en': False})
yaml.safe_dump(cfg, open(f'{sys.argv[3]}/fastlio.yaml', 'w'), sort_keys=False)
PY
    set +u; source "${FW}/install/setup.bash"; set -u
    "${TIME[@]}" "${FW}/install/fast_lio/lib/fast_lio/fastlio_mapping" --ros-args --params-file "${out}/fastlio.yaml" \
      > "${out}/stdout.log" 2>&1 8>&- &
    node=$!
    ros2 bag record -o "${out}/odom_bag" /Odometry > "${out}/record.log" 2>&1 8>&- &
    rec=$!
    sleep 5
    ros2 bag play "${bag}" --rate 1.0 > "${out}/play.log" 2>&1 8>&-
    sleep 5
    # GNU time ignores SIGINT itself: the node (its child) gets it; the recorder stops cleanly on SIGTERM
    pkill -INT -P ${node} 2>/dev/null; kill -TERM ${rec} 2>/dev/null
    wait ${node}; rc=$?
    wait ${rec} 2>/dev/null
    "${PY}" - "${out}/odom_bag" "${out}/traj_imu.tum" <<'PY'
import sys
import rclpy.serialization
import rosbag2_py
from nav_msgs.msg import Odometry
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1]), rosbag2_py.ConverterOptions('', ''))
with open(sys.argv[2], 'w') as f:
    while reader.has_next():
        topic, data, _ = reader.read_next()
        m = rclpy.serialization.deserialize_message(data, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        f.write(f'{t:.9f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n')
PY
    rm -rf "${out}/odom_bag"
    ;;
  glim)
    GI=/home/user/rivals/glim_install; GW=/home/user/rivals/glim_build/ws
    cfg="${out}/glim_config"
    cp -a "${GW}/src/glim/config" "${cfg}"
    for f in config.json config_ros.json config_logging.json config_sensors.json; do
      cp "${REPO}/configs/glim/hilti2022_cpu/${f}" "${cfg}/${f}"
    done
    "${PY}" - "${rig}" "${cfg}" "${out}" <<'PY'
import json
import sys
import numpy as np
import yaml
sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_to_ros2 import quat_to_R, R_to_quat  # noqa: E402
rig = yaml.safe_load(open(sys.argv[1]))
cfg, out = sys.argv[2], sys.argv[3]
q = rig['lidar_to_imu_quat_xyzw_xyz']
R = np.asarray(quat_to_R(q[:4]))
t = np.asarray(q[4:], dtype=float)
Ri, ti = R.T, -R.T @ t  # GLIM's T_lidar_imu: the IMU pose in the LiDAR frame
qi = R_to_quat(Ri)
ros = json.load(open(f'{cfg}/config_ros.json'))
ros['glim_ros']['imu_topic'] = rig['imu_topic']
ros['glim_ros']['points_topic'] = rig['lidar_topic']
json.dump(ros, open(f'{cfg}/config_ros.json', 'w'), indent=2)
sen = json.load(open(f'{cfg}/config_sensors.json'))
sen['sensors']['T_lidar_imu'] = [float(v) for v in ti] + [float(v) for v in qi]
json.dump(sen, open(f'{cfg}/config_sensors.json', 'w'), indent=2)
log = json.load(open(f'{cfg}/config_logging.json'))
text = json.dumps(log).replace('/out/logs', f'{out}/logs')
open(f'{cfg}/config_logging.json', 'w').write(text)
PY
    set +u; source "${GW}/install/setup.bash"; set -u
    LD_LIBRARY_PATH=${GI}/lib:${LD_LIBRARY_PATH:-} "${TIME[@]}" timeout -s INT 7200 \
      ros2 run glim_ros glim_rosbag "${bag}" --ros-args -p config_path:="${cfg}" -p auto_quit:=true \
      -p dump_path:="${out}/dump" > "${out}/stdout.log" 2>&1 8>&-
    rc=$?
    [[ -f ${out}/dump/traj_imu.txt ]] && cp "${out}/dump/traj_imu.txt" "${out}/traj_imu.tum"
    rm -rf "${out}/dump"
    ;;
  *) echo "unknown method ${method}"; exit 2 ;;
esac
t1=$(date +%s.%N)
flock -u 8
echo "${rc}" > "${out}/exit.txt"
echo "wall_seconds $(python3 -c "print(f'{${t1}-${t0}:.1f}')")" >> "${out}/run_info.txt"
if [[ -s ${out}/traj_imu.tum ]]; then
  "${PY}" - "${rig}" "${out}/traj_imu.tum" "${out}/traj.tum" <<'PY'
import sys
import numpy as np
import yaml
sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_to_ros2 import R_to_quat, matrix  # noqa: E402
rig = yaml.safe_load(open(sys.argv[1]))
q_t = rig.get('imu_to_prism_quat_xyzw_xyz')
est = np.loadtxt(sys.argv[2], comments='#', ndmin=2)
with open(sys.argv[3], 'w') as f:
    for row in est:
        if q_t is None:
            f.write(' '.join(f'{v:.9f}' for v in row[:8]) + '\n')
            continue
        T = matrix(row[1:4], row[4:8]) @ matrix(q_t[4:], q_t[:4])
        q = R_to_quat(T[:3, :3])
        f.write(f'{row[0]:.9f} {T[0, 3]:.6f} {T[1, 3]:.6f} {T[2, 3]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n')
PY
fi
rm -rf "${out}/roslog" "${out}/ros_home"
echo "${method} exit ${rc} $(grep -m1 wall_seconds "${out}/run_info.txt") maxrss_kb $(awk -F': ' '/Maximum resident/ {print $2}' "${out}/time.txt" 2>/dev/null) poses $(wc -l < "${out}/traj.tum" 2>/dev/null || echo 0)"
