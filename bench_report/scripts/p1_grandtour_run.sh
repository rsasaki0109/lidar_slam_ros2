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
# IMU=<group> (default livox_imu) takes another IMU of the mission as the base frame (a rig input, not part of G); its
# data are downloaded and converted into <data>/bag_<group>. BAG_IMUS=a,b writes those IMUs into one bag
# (<data>/bag_a+b) so that several IMU choices share one conversion; IMU must be one of them. LIDAR=hesai_points takes
# the Hesai XT-32 instead of the Livox MID-360 (hesai_points+livox_points merges both), RANGE_SCALE=s scales the points
# (rig-input tests), SECONDARY_RANGE_SCALE=s the second LiDAR's points of a merge only (the MID-360 range calibration
# 0.9985 of chapter 32), SECONDARY_MAX_RANGE=r keeps only its points within r m of it. DATA_ROOT (default /home/user/data/grandtour) holds the downloaded and converted data.
# Outputs go to /home/user/bench/grandtour/<label>/<short_name>/.
set -euo pipefail
folder=$1; short=$2; label=$3; ws=${4:-/home/user/ws/rko_g12}
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
D=${DATA_ROOT:-/home/user/data/grandtour}/${short}; O=/home/user/bench/grandtour/${label}/${short}
IMU=${IMU:-livox_imu}; BAG_IMUS=${BAG_IMUS:-${IMU}}; LIDAR=${LIDAR:-livox_points}; RANGE_SCALE=${RANGE_SCALE:-1}
SECONDARY_RANGE_SCALE=${SECONDARY_RANGE_SCALE:-1}; SECONDARY_MAX_RANGE=${SECONDARY_MAX_RANGE:-0}
SUFFIX=; [[ ${BAG_IMUS} == livox_imu ]] || SUFFIX=_${BAG_IMUS//,/+}
[[ ${LIDAR} == livox_points ]] || SUFFIX=${SUFFIX}_${LIDAR}; [[ ${RANGE_SCALE} == 1 ]] || SUFFIX=${SUFFIX}_rs${RANGE_SCALE}
[[ ${SECONDARY_RANGE_SCALE} == 1 ]] || SUFFIX=${SUFFIX}_srs${SECONDARY_RANGE_SCALE}
[[ ${SECONDARY_MAX_RANGE} == 0 ]] || SUFFIX=${SUFFIX}_smr${SECONDARY_MAX_RANGE}
BAG=${D}/bag${SUFFIX}
IMU_TOPIC=/livox/imu; [[ ${IMU} == livox_imu ]] || IMU_TOPIC=/${IMU}
LIDAR_TOPIC=/livox/points; [[ ${LIDAR} == livox_points ]] || LIDAR_TOPIC=/${LIDAR}; [[ ${LIDAR} != *+* ]] || LIDAR_TOPIC=/merged_points
PRISM="0.3088378015503127 -0.038312997211241984 -0.2349441159711169 -0.005724388155828644 -0.0013879431347362404 -0.004742721620119043 0.9999714053840392"
U=https://huggingface.co/datasets/leggedrobotics/grand_tour_dataset/resolve/main/${folder}/data
# fetch <group>: download <group>.tar and extract it into ${D}/z/<group>, retrying; the directory appears only when the
# whole archive arrived and is marked .complete (a thinned or partial copy has no marker and is fetched again)
fetch() {
  local f=$1 k
  [[ -f ${D}/z/${f}/.complete ]] && return 0
  for k in 1 2 3 4 5 6; do
    rm -rf "${D}/z.part" "${D}/z/${f}"; mkdir -p "${D}/z.part" "${D}/z"
    if curl -sS -L --fail "${U}/${f}.tar" | tar xf - -C "${D}/z.part"; then
      mv "${D}/z.part/${f}" "${D}/z/${f}"; rm -rf "${D}/z.part"; touch "${D}/z/${f}/.complete"; return 0
    fi
    # Hugging Face answers 429 when it rate-limits; wait 1, 2, 4, 8, 15, 15 min
    echo "download of ${f} failed (attempt ${k})" >&2; sleep $(( k < 5 ? 60 * 2 ** (k - 1) : 900 ))
  done
  return 1
}
mkdir -p "${D}/z" "${O}"
set +u; source /opt/ros/jazzy/setup.bash; set -u
if [[ ! -d ${BAG} ]]; then
  for f in ${LIDAR//+/ } ${BAG_IMUS//,/ }; do
    # streamed into tar, so that a large tar file and its extracted copy are never on the disk together
    fetch "${f}"
  done
  # written as .part and renamed when complete, so that a failed conversion is never taken for a bag
  rm -rf "${BAG}.part"
  CONSUME=--consume  # a merged cloud consumes both groups, each read forward in time
  "${PY}" "${S}/p1_grandtour_to_ros2.py" --imu "${BAG_IMUS}" --lidar "${LIDAR}" --range-scale "${RANGE_SCALE}" \
    --secondary-range-scale "${SECONDARY_RANGE_SCALE}" --secondary-max-range "${SECONDARY_MAX_RANGE}" \
    ${CONSUME} "${D}/z" "${BAG}.part" | tee "${D}/convert${SUFFIX}.log"
  mv "${BAG}.part" "${BAG}"
  # the bag holds the points now; keep only the zarr metadata (the extrinsics), so that a long Hesai mission (CON-3:
  # 4.4 GB of points plus its bag) leaves room on the disk; the .complete markers go too, so later conversions fetch again
  find "${D}/z" -type f ! -name '.z*' -delete 2>/dev/null || true
fi
# the inputs of this run, for later steps on the same data (p3_grandtour_backend.sh)
echo "${BAG} ${LIDAR_TOPIC} ${IMU_TOPIC}" > "${O}/inputs.txt"
# shellcheck disable=SC2086
"${PY}" "${S}/p1_grandtour_to_ros2.py" --imu "${IMU}" --lidar "${LIDAR}" --extrinsics "${D}/z" ${PRISM} > "${O}/extrinsics.txt"
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
RUN_TIMEOUT=7200 bash /home/user/scripts/run_front.sh "${label}_${short}" "${ws}" "${O}/params.yaml" "${BAG}" "${LIDAR_TOPIC}" \
  "${IMU_TOPIC}" "${O}/run" /home/user/bench/grandtour/runs.csv > /dev/null
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
