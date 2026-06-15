#!/usr/bin/env bash
# Train a PPO policy in the 3DGS DriveEnv (Phase 4 RL substrate). The closed-loop
# 3DGS sim is wrapped as a Gymnasium env (tools/gaussian_splatting/drive_env.py);
# this trains a stable-baselines3 agent on the low-dim `state` observation so the
# RL loop validates in seconds on CPU. The ego (planar unicycle) must reach a
# goal while staying inside the valid-viewpoint corridor (Phase 0).
#
# Corridor anchors come from a recorded SLAM trajectory (TRAJ, TUM, recentred)
# or a synthetic arc when TRAJ is empty. The `camera` observation (the 3DGS
# render, the real sim2real signal) plugs into the same env via a
# GaussianRenderer render_fn -- pixel-space training is the GPU-heavy next step.
#
# Requires: gymnasium + stable-baselines3 + torch.
#
# Usage:
#   bash scripts/run_drive_rl.sh                     # synthetic arc corridor
#   TRAJ=output/rtkslam_stadtgarten_seq2_run/traj_raw.tum \
#     STEPS=80000 bash scripts/run_drive_rl.sh       # real trajectory corridor
#   CAMERA=1 PLY=output/stadt_3dgs/gsplat/point_cloud.ply \
#     TRANSFORMS=output/stadt_3dgs/gsplat/transforms.json \
#     STEPS=40000 bash scripts/run_drive_rl.sh       # pixel-observation (GPU)
#   DETECT=1 PLY=output/assets/truck_scene.ply STEPS=40000 \
#     bash scripts/run_drive_rl.sh                    # active perception (GPU+YOLO)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}/tools/gaussian_splatting"

TRAJ="${TRAJ:-}"
STEPS="${STEPS:-60000}"
EVAL_EPISODES="${EVAL_EPISODES:-30}"
SAVE="${SAVE:-}"
CAMERA="${CAMERA:-}"
DETECT="${DETECT:-}"
PLY="${PLY:-}"
TRANSFORMS="${TRANSFORMS:-}"
RENDER_SIZE="${RENDER_SIZE:-84}"

ARGS=(--steps "${STEPS}" --eval-episodes "${EVAL_EPISODES}")
[[ -n "${TRAJ}" ]] && ARGS+=(--traj "${REPO_ROOT}/${TRAJ}")
[[ -n "${SAVE}" ]] && ARGS+=(--save "${SAVE}")
if [[ -n "${DETECT}" ]]; then
  ARGS+=(--detect-target --ply "${REPO_ROOT}/${PLY}" --render-size "${RENDER_SIZE}")
elif [[ -n "${CAMERA}" ]]; then
  ARGS+=(--camera --ply "${REPO_ROOT}/${PLY}" \
         --transforms "${REPO_ROOT}/${TRANSFORMS}" --render-size "${RENDER_SIZE}")
fi

python3 -u train_drive_policy.py "${ARGS[@]}"
