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


def recenter_corridor(xy: np.ndarray) -> tuple:
    """Translate so the path starts at the origin; heading toward the 2nd point."""
    xy = np.asarray(xy, dtype=float)
    xy = xy - xy[0]
    heading = float(np.arctan2(xy[1, 1] - xy[0, 1], xy[1, 0] - xy[0, 0]))
    return xy, heading


def evaluate(env, policy, episodes: int) -> tuple:
    """Mean return and goal-success rate over greedy episodes."""
    returns, goals = [], 0
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
    return float(np.mean(returns)), goals / episodes


def build_env(args):
    """Construct the DriveEnv from a trajectory or synthetic arc."""
    if args.traj:
        xy = load_traj_xy(Path(args.traj))
        step = max(1, len(xy) // 40)
        xy = xy[::step]
    else:
        xy = synthetic_arc()
    anchors, heading = recenter_corridor(xy)
    return drive_env.make_drive_env(
        anchors, anchors[-1], start_pose=(0.0, 0.0, heading),
        max_dev=args.max_dev, hard_dev=args.hard_dev, dt=0.2, v_max=2.0,
        omega_max=1.0, max_steps=args.max_steps, goal_tol=1.0, obs_mode='state')


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--traj', default='', help='TUM trajectory for corridor anchors')
    p.add_argument('--steps', type=int, default=60000, help='PPO training steps')
    p.add_argument('--eval-episodes', type=int, default=20)
    p.add_argument('--max-dev', type=float, default=1.0)
    p.add_argument('--hard-dev', type=float, default=3.0)
    p.add_argument('--max-steps', type=int, default=200)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--save', default='', help='optional path to save the policy')
    args = p.parse_args(argv)

    from stable_baselines3 import PPO

    env = build_env(args)
    base_ret, base_succ = evaluate(env, None, args.eval_episodes)
    print(f'random policy: mean return {base_ret:.2f}, success {base_succ:.0%}')

    model = PPO('MlpPolicy', env, seed=args.seed, verbose=0)
    model.learn(total_timesteps=args.steps)
    ret, succ = evaluate(env, model, args.eval_episodes)
    print(f'PPO ({args.steps} steps): mean return {ret:.2f}, success {succ:.0%}')
    if args.save:
        model.save(args.save)
        print(f'saved policy to {args.save}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
