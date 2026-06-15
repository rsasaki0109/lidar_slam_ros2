#!/usr/bin/env bash
# Launch the closed-loop 3DGS camera sensor-sim ROS 2 node (Phase 2 of the
# 3DGS-as-sim2real track). Subscribes to the ego pose, renders a LiDAR-primed
# 3DGS scene from the matching camera viewpoint with a resident GPU renderer,
# and publishes sensor_msgs/Image + CameraInfo for a real-image-trained stack
# (e.g. Autoware perception) to consume in the loop.
#
# Requires: a sourced ROS 2 workspace (rclpy, cv_bridge) and a CUDA GPU with
# torch + gsplat. PLY and TRANSFORMS must be a matched pair. The ego pose must
# be expressed in the same world frame the 3DGS model was built in; if not, set
# ALIGN (16 row-major floats, model_world<-pose_world). EXTRINSIC is the static
# base_link<-camera_optical transform (OpenCV optical frame).
#
# Usage:
#   PLY=output/koide_3dgs_firstlight/gsplat/pc_sh1_15k.ply \
#   TRANSFORMS=output/koide_3dgs_firstlight/gsplat/transforms.json \
#   POSE_TOPIC=/localization/kinematic_state \
#   bash scripts/run_sensor_sim_node.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

PLY="${PLY:?set PLY to the trained 3DGS .ply}"
TRANSFORMS="${TRANSFORMS:?set TRANSFORMS to the matched transforms.json}"
SCALE="${SCALE:-0.5}"
POSE_TOPIC="${POSE_TOPIC:-/localization/kinematic_state}"
POSE_TYPE="${POSE_TYPE:-odometry}"   # or pose_stamped
IMAGE_TOPIC="${IMAGE_TOPIC:-/sensor_sim/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/sensor_sim/camera_info}"
FRAME_ID="${FRAME_ID:-camera_optical}"

python3 tools/gaussian_splatting/sensor_sim_node.py --ros-args \
  -p ply:="${PLY}" \
  -p transforms:="${TRANSFORMS}" \
  -p scale:="${SCALE}" \
  -p pose_topic:="${POSE_TOPIC}" \
  -p pose_type:="${POSE_TYPE}" \
  -p image_topic:="${IMAGE_TOPIC}" \
  -p camera_info_topic:="${CAMERA_INFO_TOPIC}" \
  -p frame_id:="${FRAME_ID}"
