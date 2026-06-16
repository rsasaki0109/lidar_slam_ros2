#!/usr/bin/env python3
"""Train a PPO policy in the 3DGS DriveEnv (state obs) to validate the RL loop.

Phase 4 demonstration: the closed-loop 3DGS sim is wrapped as a Gymnasium env
(``drive_env.py``); this trains a stable-baselines3 PPO agent on the low-dim
``state`` observation so a policy converges in seconds on CPU, proving the env is
learnable end to end. The agent must drive the unicycle ego to the goal while
staying inside the valid-viewpoint corridor.

The ``camera`` observation (the 3DGS render, the real sim2real signal) plugs into
the same env via a ``GaussianRenderer`` ``render_fn``; training on pixels is the
GPU-heavy next step and is intentionally out of this quick validation.

Corridor anchors come from a recorded SLAM trajectory (``--traj`` TUM, recentred)
or a synthetic arc; the goal is the far end of the corridor.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

import drive_env


def load_traj_xy(path: Path) -> np.ndarray:
    """Read the x,y columns of a TUM trajectory (cols 2,3) as (N, 2)."""
    rows = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        rows.append([float(parts[1]), float(parts[2])])
    return np.asarray(rows, dtype=float)


def synthetic_arc(n: int = 40, radius: float = 12.0,
                  sweep_deg: float = 80.0) -> np.ndarray:
    """A gentle arc corridor centred on the origin start (deterministic)."""
    ang = np.radians(np.linspace(0.0, sweep_deg, n))
    xy = np.stack([radius * np.sin(ang), radius * (1.0 - np.cos(ang))], axis=1)
    return xy


def synthetic_straight(length: float = 16.0, n: int = 33) -> np.ndarray:
    """A straight corridor along +x (deterministic); used for the actor task."""
    return np.stack([np.linspace(0.0, length, n), np.zeros(n)], axis=1)


def recenter_corridor(xy: np.ndarray) -> tuple:
    """Translate so the path starts at the origin; heading toward the 2nd point."""
    xy = np.asarray(xy, dtype=float)
    xy = xy - xy[0]
    heading = float(np.arctan2(xy[1, 1] - xy[0, 1], xy[1, 0] - xy[0, 0]))
    return xy, heading


def evaluate(env, policy, episodes: int) -> tuple:
    """Mean return, goal-success rate, and collision rate over greedy episodes."""
    returns, goals, collisions = [], 0, 0
    for _ in range(episodes):
        obs, _ = env.reset()
        done, total = False, 0.0
        while not done:
            action = (env.action_space.sample() if policy is None
                      else policy.predict(obs, deterministic=True)[0])
            obs, r, term, trunc, info = env.step(action)
            total += r
            done = term or trunc
        returns.append(total)
        goals += int(info.get('reason') == 'goal')
        collisions += int(info.get('reason') == 'collision')
    return float(np.mean(returns)), goals / episodes, collisions / episodes


def build_camera_env(args):
    """Build a pixel-observation DriveEnv: the agent sees the 3DGS render itself.

    Corridor and camera come from a recorded trajectory (``transforms.json``): the
    ground plane / world-up / travel axis are recovered from the camera poses, the
    ego drives in that plane, and each step rasterises the resident 3DGS model from
    the ego camera. This closes the sim2real loop -- the policy learns from exactly
    the image the closed-loop sensor-sim publishes.
    """
    import scene_camera as sc
    from gaussian_renderer import GaussianRenderer

    v_max, dt = 1.5, 0.2
    c2w = sc.load_cam_c2w(args.transforms)
    frame = sc.derive_ground_frame(c2w)
    anchors = sc.corridor_xy(c2w, frame)
    renderer = GaussianRenderer(args.ply)
    h = w = int(args.render_size)
    render_fn = sc.make_scene_render_fn(renderer, frame, fx=args.fx,
                                        width=w, height=h)
    start = anchors[0]
    # heading from the overall travel direction (adjacent anchors jitter in y)
    heading = float(np.arctan2(anchors[-1, 1] - anchors[0, 1],
                               anchors[-1, 0] - anchors[0, 0]))
    return drive_env.make_drive_env(
        anchors, anchors[-1], start_pose=(float(start[0]), float(start[1]),
                                          heading),
        max_dev=args.max_dev, hard_dev=args.hard_dev, dt=dt, v_max=v_max,
        omega_max=0.5, max_steps=args.max_steps, goal_tol=1.0,
        obs_mode='camera', render_fn=render_fn, render_size=(h, w))


def build_env(args):
    """Construct the DriveEnv from a trajectory, a straight corridor, or an arc.

    The actor (avoidance) task uses a straight corridor with a pedestrian
    crossing the centreline timed to a full-speed run -- so a naive fast policy
    collides and the agent must yield (slow, let it pass) to reach the goal.
    """
    if args.camera:
        return build_camera_env(args)
    v_max, dt = 2.0, 0.2
    if args.traj:
        xy = load_traj_xy(Path(args.traj))
        step = max(1, len(xy) // 40)
        xy = xy[::step]
    elif args.actor or args.straight:
        xy = synthetic_straight()
    else:
        xy = synthetic_arc()
    anchors, heading = recenter_corridor(xy)
    actor_fn = None
    if args.actor:
        mid = anchors[len(anchors) // 2]
        # the crossing is centred on the path exactly when a full-speed runner
        # arrives (arrival_step = mid_x / (v_max*dt)); cross spans 2x that.
        arrival = max(1, int(round(float(mid[0]) / (v_max * dt))))
        actor_fn = (lambda s, mx=float(mid[0]), my=float(mid[1]):
                    drive_env.crossing_actor(s, x=mx, y0=my - 3.0, y1=my + 3.0,
                                             cross_steps=2 * arrival))
    return drive_env.make_drive_env(
        anchors, anchors[-1], start_pose=(0.0, 0.0, heading),
        max_dev=args.max_dev, hard_dev=args.hard_dev, dt=dt, v_max=v_max,
        omega_max=1.0, max_steps=args.max_steps, goal_tol=1.0, obs_mode='state',
        actor_fn=actor_fn, actor_radius=1.0, yield_dist=3.0)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--traj', default='', help='TUM trajectory for corridor anchors')
    p.add_argument('--steps', type=int, default=60000, help='PPO training steps')
    p.add_argument('--eval-episodes', type=int, default=20)
    p.add_argument('--max-dev', type=float, default=1.0)
    p.add_argument('--hard-dev', type=float, default=3.0)
    p.add_argument('--max-steps', type=int, default=200)
    p.add_argument('--actor', action='store_true',
                   help='add a pedestrian crossing the corridor (avoidance task)')
    p.add_argument('--straight', action='store_true',
                   help='use a straight corridor (implied by --actor)')
    p.add_argument('--camera', action='store_true',
                   help='pixel observation: learn from the 3DGS render (GPU)')
    p.add_argument('--ply', default='', help='3DGS model .ply (camera obs)')
    p.add_argument('--transforms', default='',
                   help='transforms.json for corridor/camera (camera obs)')
    p.add_argument('--render-size', type=int, default=84,
                   help='square render resolution for camera obs')
    p.add_argument('--fx', type=float, default=60.0,
                   help='focal length px for camera obs')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--save', default='', help='optional path to save the policy')
    args = p.parse_args(argv)

    from stable_baselines3 import PPO

    if args.camera and not (args.ply and args.transforms):
        p.error('--camera requires --ply and --transforms')

    env = build_env(args)
    base_ret, base_succ, base_col = evaluate(env, None, args.eval_episodes)
    print(f'random policy: mean return {base_ret:.2f}, success {base_succ:.0%}, '
          f'collision {base_col:.0%}')

    policy = 'CnnPolicy' if args.camera else 'MlpPolicy'
    model = PPO(policy, env, seed=args.seed, verbose=0)
    model.learn(total_timesteps=args.steps)
    ret, succ, col = evaluate(env, model, args.eval_episodes)
    print(f'PPO ({args.steps} steps): mean return {ret:.2f}, success {succ:.0%}, '
          f'collision {col:.0%}')
    if args.save:
        model.save(args.save)
        print(f'saved policy to {args.save}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
